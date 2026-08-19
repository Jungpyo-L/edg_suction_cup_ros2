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
    def __init__(self, sphere_radius=None):
        super().__init__("realsense_sphere_detector")

        self.declare_parameter("input_topic", "/camera/camera/depth/color/points")
        self.declare_parameter("output_topic", "filtered_points")
        self.declare_parameter("apex_topic", "sphere_apex")
        # Frame the cloud is transformed into before filtering. Everything below
        # (crop box, "highest point") is expressed in this frame.
        #
        # "base", not ROS's "base_link": the apex is consumed by rtde_helper,
        # which sends poses to the UR controller, and the controller works in
        # "base". The two differ by 180 deg about Z, so publishing in base_link
        # sends the arm to the mirrored point on the far side of the workspace.
        self.declare_parameter("target_frame", "base")
        # Workspace crop box in target_frame, in meters. These are the values
        # found by hand on the bench: x unbounded, y trimmed to the table around
        # the sphere, z from the table surface up to just under the arm.
        #
        # z_max is the one that matters. Keep it snug above the sphere - set it
        # too high and the robot arm becomes the highest thing in the cloud, and
        # the apex silently jumps to the arm while still looking plausible.
        self.declare_parameter("crop_min", [-100.0, -0.15, 0.0])
        self.declare_parameter("crop_max", [100.0, 0.08, 0.08])
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
        # Radius of the sphere being probed, in meters. When greater than zero
        # the apex comes from a least-squares fit of a sphere of this radius
        # instead of the top-band centroid. Set it to the same value passed as
        # --radius to the sweep. Zero keeps the old behavior.
        self.declare_parameter("sphere_radius", 0.0)
        self.declare_parameter("min_fit_points", 100)
        self.declare_parameter("fit_iterations", 20)
        self.declare_parameter("fit_tolerance", 1e-6)
        # Fit residual above which the cloud is judged not to be this sphere.
        self.declare_parameter("max_fit_rms", 0.005)

        self.target_frame = self.get_parameter("target_frame").value
        self.crop_min = np.asarray(self.get_parameter("crop_min").value, dtype=np.float64)
        self.crop_max = np.asarray(self.get_parameter("crop_max").value, dtype=np.float64)
        self.voxel_leaf = float(self.get_parameter("voxel_leaf").value)
        self.outlier_neighbors = int(self.get_parameter("outlier_neighbors").value)
        self.outlier_std_ratio = float(self.get_parameter("outlier_std_ratio").value)
        self.apex_band = float(self.get_parameter("apex_band").value)
        self.min_apex_points = int(self.get_parameter("min_apex_points").value)
        self.min_points = int(self.get_parameter("min_points").value)
        # --sphere-radius on the command line wins over the ROS parameter, so the
        # radius can be given the same way the sweep takes it.
        self.sphere_radius = (
            float(sphere_radius) if sphere_radius is not None
            else float(self.get_parameter("sphere_radius").value)
        )
        self.min_fit_points = int(self.get_parameter("min_fit_points").value)
        self.fit_iterations = int(self.get_parameter("fit_iterations").value)
        self.fit_tolerance = float(self.get_parameter("fit_tolerance").value)
        self.max_fit_rms = float(self.get_parameter("max_fit_rms").value)

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
        """Apex of the sphere, by fit when a radius is known and by band otherwise."""
        if self.sphere_radius > 0.0:
            apex = self.estimate_apex_by_fit(points)
            if apex is not None:
                return apex
            # Fall through to the band method rather than dropping the frame, so
            # a fit that fails on one bad frame does not stall the topic.
            self.get_logger().warn(
                "Sphere fit did not converge; falling back to the top-band "
                "centroid for this frame.",
                throttle_duration_sec=5.0,
            )
        return self.estimate_apex_by_band(points)

    def estimate_apex_by_band(self, points):
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

    def estimate_apex_by_fit(self, points):
        """Fit a sphere of known radius to the surface, then take its top.

        Better conditioned than the band centroid, which averages over a cap
        nearly 24 mm across on a 25 mm sphere and therefore drifts with the
        camera's asymmetric view of it. Here every surface point constrains the
        centre, and fixing the radius means one-sided coverage still pins it
        down. It also removes the upward bias of taking a maximum of noisy z.
        """
        if points.shape[0] < self.min_fit_points:
            return None

        radius = self.sphere_radius
        # Start from the band estimate's centre, one radius below the top.
        seed = self.estimate_apex_by_band(points)
        if seed is None:
            return None
        centre = np.array([seed[0], seed[1], seed[2] - radius], dtype=np.float64)

        for _ in range(self.fit_iterations):
            offsets = points - centre
            distances = np.linalg.norm(offsets, axis=1)
            # A point exactly at the centre has no defined direction; drop it.
            valid = distances > 1e-9
            if valid.sum() < self.min_fit_points:
                return None
            offsets = offsets[valid]
            distances = distances[valid]

            # Gauss-Newton on residual r_i = |p_i - c| - R. The Jacobian row is
            # the unit vector from the point to the centre.
            residuals = distances - radius
            jacobian = -offsets / distances[:, None]
            step, *_ = np.linalg.lstsq(jacobian, -residuals, rcond=None)
            centre = centre + step
            if np.linalg.norm(step) < self.fit_tolerance:
                break
        else:
            return None

        # Reject a fit the data does not actually support: a cloud that is not a
        # sphere of this radius will converge somewhere with a large residual.
        rms = float(np.sqrt(np.mean(
            (np.linalg.norm(points - centre, axis=1) - radius) ** 2
        )))
        if rms > self.max_fit_rms:
            self.get_logger().warn(
                "Sphere fit RMS %.4f m exceeds max_fit_rms %.4f; is sphere_radius "
                "correct and the crop box clean?" % (rms, self.max_fit_rms),
                throttle_duration_sec=5.0,
            )
            return None

        self.get_logger().info(
            "sphere fit: centre %s, RMS %.4f m over %d points"
            % (np.round(centre, 4), rms, points.shape[0]),
            throttle_duration_sec=5.0,
        )
        return np.array([centre[0], centre[1], centre[2] + radius])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ros-args", nargs="*", help=argparse.SUPPRESS)
    parser.add_argument(
        "--sphere-radius",
        type=float,
        default=None,
        help="radius of the sphere being probed (m). Given, the apex comes from "
        "a least-squares sphere fit instead of the top-band centroid. Pass the "
        "same value used for the sweep's --radius",
    )
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = RealSenseSphereDetector(sphere_radius=args.sphere_radius)
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
