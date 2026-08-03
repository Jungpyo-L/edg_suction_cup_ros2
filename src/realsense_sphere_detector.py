#!/usr/bin/env python3

# Filters the Intel RealSense point cloud down to the test-sphere workspace and
# publishes both the filtered cloud (filtered_points) and the apex of the sphere
# (sphere_apex), which sits directly above the sphere center.

import argparse
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros


def voxel_downsample(points, leaf_size):
    """Average the points inside each leaf_size cube. Returns an (M, 3) array."""
    if leaf_size <= 0.0 or points.shape[0] == 0:
        return points

    keys = np.floor(points / leaf_size).astype(np.int64)
    _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    # NumPy 2.0 returned a column vector here for axis=0; flatten so the
    # scatter-add below indexes rows rather than broadcasting silently.
    inverse = inverse.reshape(-1)

    sums = np.zeros((counts.shape[0], 3), dtype=np.float64)
    np.add.at(sums, inverse, points)
    return sums / counts[:, None]


def remove_statistical_outliers(points, num_neighbors, std_ratio):
    """Drop points whose mean distance to their k nearest neighbors is an outlier."""
    if num_neighbors <= 0 or points.shape[0] <= num_neighbors:
        return points

    tree = cKDTree(points)
    # The first neighbor of every point is itself, so ask for one extra and skip it.
    distances, _ = tree.query(points, k=num_neighbors + 1)
    mean_dist = distances[:, 1:].mean(axis=1)

    threshold = mean_dist.mean() + std_ratio * mean_dist.std()
    return points[mean_dist <= threshold]


class RealSenseSphereDetector(Node):
    def __init__(self):
        super().__init__("realsense_sphere_detector")

        self.declare_parameter("input_topic", "/camera/camera/depth/color/points")
        self.declare_parameter("output_topic", "filtered_points")
        self.declare_parameter("apex_topic", "sphere_apex")
        # Frame the cloud is transformed into before filtering. Everything below
        # (crop box, "highest point") is expressed in this frame.
        self.declare_parameter("target_frame", "base_link")
        # Workspace crop box in target_frame, in meters. Defaults leave x and y
        # unbounded and filter on height alone: the sphere apex is the topmost
        # point in the scene, so only z needs constraining. Keep z_max snug above
        # the sphere or the robot arm becomes the highest thing in the cloud.
        # z_min is below the base origin because the table sits lower than the
        # robot base; z_max only needs to sit above the sphere and below the arm.
        self.declare_parameter("crop_min", [-100.0, -100.0, -1.0])
        self.declare_parameter("crop_max", [100.0, 100.0, 1.0])
        self.declare_parameter("voxel_leaf", 0.002)
        self.declare_parameter("outlier_neighbors", 12)
        self.declare_parameter("outlier_std_ratio", 1.0)
        # Points within this distance of the highest point are averaged into the
        # apex estimate, which rejects single-pixel depth spikes.
        self.declare_parameter("apex_band", 0.003)
        self.declare_parameter("min_apex_points", 5)
        # Number of frames the apex is median-filtered over before publishing.
        self.declare_parameter("apex_history", 15)
        self.declare_parameter("min_points", 50)

        self.target_frame = self.get_parameter("target_frame").value
        self.crop_min = np.asarray(self.get_parameter("crop_min").value, dtype=np.float64)
        self.crop_max = np.asarray(self.get_parameter("crop_max").value, dtype=np.float64)
        self.voxel_leaf = float(self.get_parameter("voxel_leaf").value)
        self.outlier_neighbors = int(self.get_parameter("outlier_neighbors").value)
        self.outlier_std_ratio = float(self.get_parameter("outlier_std_ratio").value)
        self.apex_band = float(self.get_parameter("apex_band").value)
        self.min_apex_points = int(self.get_parameter("min_apex_points").value)
        self.min_points = int(self.get_parameter("min_points").value)

        self.apex_history = deque(maxlen=int(self.get_parameter("apex_history").value))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # The RealSense wrapper publishes best-effort, so the subscription must
        # be best-effort too or it receives nothing.
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # The filtered cloud is published reliably instead: it is a low-rate debug
        # output, and reliable is what `ros2 topic echo` and RViz default to.
        filtered_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Transient local so an experiment script started later still receives the
        # last apex estimate immediately.
        apex_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.cloud_pub = self.create_publisher(
            PointCloud2, self.get_parameter("output_topic").value, filtered_qos
        )
        self.apex_pub = self.create_publisher(
            PointStamped, self.get_parameter("apex_topic").value, apex_qos
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.get_parameter("input_topic").value,
            self.cloud_callback,
            sensor_qos,
        )

        self.get_logger().info(
            "Filtering %s -> %s in frame %s"
            % (
                self.get_parameter("input_topic").value,
                self.get_parameter("output_topic").value,
                self.target_frame,
            )
        )

    def lookup_cloud_transform(self, source_frame):
        """Return (3, 3) rotation and (3,) translation from source_frame to target_frame."""
        transform = self.tf_buffer.lookup_transform(self.target_frame, source_frame, Time())
        t = transform.transform.translation
        q = transform.transform.rotation
        rotation = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        return rotation, np.array([t.x, t.y, t.z])

    def cloud_callback(self, msg):
        points = point_cloud2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ).astype(np.float64)
        if points.shape[0] == 0:
            return

        if msg.header.frame_id != self.target_frame:
            try:
                rotation, translation = self.lookup_cloud_transform(msg.header.frame_id)
            except tf2_ros.TransformException as exc:
                # Throttled rather than one-shot: the extrinsic often arrives a
                # moment after the first cloud, and a stale startup warning is
                # worse than no warning at all.
                self.get_logger().warn(
                    "No transform %s -> %s yet (%s). Publish the camera extrinsic "
                    "before expecting filtered_points."
                    % (msg.header.frame_id, self.target_frame, exc),
                    throttle_duration_sec=5.0,
                )
                return
            points = points @ rotation.T + translation

        inside = np.all((points >= self.crop_min) & (points <= self.crop_max), axis=1)
        points = points[inside]
        if points.shape[0] < self.min_points:
            return

        points = voxel_downsample(points, self.voxel_leaf)
        points = remove_statistical_outliers(
            points, self.outlier_neighbors, self.outlier_std_ratio
        )
        if points.shape[0] < self.min_points:
            return

        header = msg.header
        header.frame_id = self.target_frame
        self.cloud_pub.publish(
            point_cloud2.create_cloud_xyz32(header, points.astype(np.float32))
        )
        self.get_logger().info(
            "filtered_points: %d points in %s" % (points.shape[0], self.target_frame),
            throttle_duration_sec=5.0,
        )

        apex = self.estimate_apex(points)
        if apex is None:
            return

        self.apex_history.append(apex)
        smoothed = np.median(np.asarray(self.apex_history), axis=0)

        apex_msg = PointStamped()
        apex_msg.header = header
        apex_msg.point.x = float(smoothed[0])
        apex_msg.point.y = float(smoothed[1])
        apex_msg.point.z = float(smoothed[2])
        self.apex_pub.publish(apex_msg)

    def estimate_apex(self, points):
        """Centroid of the points sitting in the top apex_band of the cloud."""
        z_max = points[:, 2].max()
        top = points[points[:, 2] >= z_max - self.apex_band]
        if top.shape[0] < self.min_apex_points:
            return None

        apex = top.mean(axis=0)
        # The band centroid sits slightly below the true top, so keep the measured
        # peak height and only take x, y from the centroid.
        apex[2] = z_max
        return apex


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ros-args", nargs="*", help=argparse.SUPPRESS)
    parser.parse_known_args()

    rclpy.init()
    node = RealSenseSphereDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
