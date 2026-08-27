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


def fit_sphere_fixed_radius(points, radius, centre, iterations, tolerance):
    """Gauss-Newton for the centre of a sphere of known radius.

    Returns (centre, converged). Residual is |p - c| - R and the Jacobian row is
    the unit vector from the point to the centre. Hitting the iteration cap is
    not failure: the caller judges the answer by its residuals, not by whether
    the step reached an arbitrary floor.
    """
    centre = np.asarray(centre, dtype=np.float64).copy()
    converged = False
    for _ in range(iterations):
        offsets = points - centre
        distances = np.linalg.norm(offsets, axis=1)
        keep = distances > 1e-9
        if keep.sum() < 4:
            return centre, False
        offsets, distances = offsets[keep], distances[keep]

        step, *_ = np.linalg.lstsq(-offsets / distances[:, None],
                                   -(distances - radius), rcond=None)
        centre = centre + step
        if np.linalg.norm(step) < tolerance:
            converged = True
            break
    return centre, converged


def fit_sphere_robust(points, radius, seed, band, iterations, tolerance,
                      trim_rounds=4, min_inliers=60):
    """Fixed-radius sphere fit that ignores points which are not on the sphere.

    Alternates fitting with re-selecting the inliers - the points lying within
    `band` of the current surface. Table edges, the mount and the arm fall out
    after a round or two, so the centre is driven by sphere points only. Without
    this every stray point pulls the centre, which is the dominant error when
    the crop box is not perfectly clean.

    Returns (centre, inliers, rms, ok) where rms is over the inliers alone -
    pooling it with outliers hides exactly the contamination worth catching.
    """
    centre = np.asarray(seed, dtype=np.float64)
    inliers = np.ones(points.shape[0], dtype=bool)

    for _ in range(trim_rounds):
        subset = points[inliers]
        if subset.shape[0] < min_inliers:
            return centre, inliers, float("inf"), False

        centre, _ = fit_sphere_fixed_radius(subset, radius, centre,
                                            iterations, tolerance)
        residuals = np.abs(np.linalg.norm(points - centre, axis=1) - radius)
        updated = residuals <= band
        if updated.sum() < min_inliers:
            return centre, inliers, float("inf"), False
        if np.array_equal(updated, inliers):
            inliers = updated
            break
        inliers = updated

    residuals = np.linalg.norm(points[inliers] - centre, axis=1) - radius
    rms = float(np.sqrt(np.mean(residuals ** 2)))
    return centre, inliers, rms, True


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
        # Inlier residual above which the fit is judged not to be this sphere.
        # 0.005 sat on the D435's own depth noise, so good frames failed it;
        # this is a gate on the fit, not a measure of the camera.
        self.declare_parameter("max_fit_rms", 0.010)
        # Points within this distance of the fitted surface count as sphere
        # points. Roughly 2x the depth noise: wide enough to keep real surface,
        # tight enough to shed the table and the mount.
        self.declare_parameter("fit_inlier_band", 0.008)
        # Reject the frame when the sphere accounts for less than this fraction
        # of the cropped cloud - that means the crop is holding something else,
        # or sphere_radius is wrong. 0.6 was chosen offline: it still accepts a
        # cloud that is 40% table (67% inliers) but rejects a 40 mm sphere being
        # fitted as 25 mm (53%), which 0.5 let through.
        self.declare_parameter("min_inlier_fraction", 0.6)
        # Clouds pooled before fitting. The sphere and camera are both static
        # while the apex is read, so pooling lets independent depth noise
        # average out before the fit runs. Measured offline: 10 frames cuts the
        # centre error by about 15% and costs ~12 ms a callback; 30 frames gains
        # almost nothing more and costs ~47 ms, which is slower than the camera.
        # 1 disables pooling.
        self.declare_parameter("accumulate_frames", 10)

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
        self.fit_inlier_band = float(self.get_parameter("fit_inlier_band").value)
        self.min_inlier_fraction = float(self.get_parameter("min_inlier_fraction").value)
        self.accumulate_frames = max(1, int(self.get_parameter("accumulate_frames").value))
        self.cloud_history = deque(maxlen=self.accumulate_frames)

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

        # Pool recent clouds before fitting. Depth noise is independent frame to
        # frame, so N frames shrink its contribution to the centre by ~sqrt(N),
        # which is where most of the apex x/y wander came from. The sphere and
        # camera are both static while this runs, so pooling is valid; the
        # deque ages frames out on its own if the scene changes.
        self.cloud_history.append(points)
        pooled = (np.vstack(self.cloud_history)
                  if self.accumulate_frames > 1 else points)
        # Deliberately NOT re-voxelised. Averaging the pool back into voxel_leaf
        # cells throws away exactly the independent samples pooling is there to
        # collect - offline it made the pooled estimate worse than a single
        # frame (0.67 mm vs 0.36 mm at 30 frames), while leaving the pool raw
        # improved it to 0.29 mm.

        apex = self.estimate_apex(pooled)
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
        if self.sphere_radius <= 0.0:
            return self.estimate_apex_by_band(points)

        # No band fallback here. When a radius is known and the fit rejects the
        # cloud, the cloud is wrong - and the band centroid on a wrong cloud is
        # wrong too, just without saying so. Dropping the frame is honest, and
        # the apex topic is latched transient-local so consumers keep the last
        # good value.
        return self.estimate_apex_by_fit(points)

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

        The fit is trimmed rather than plain least squares: any point that is
        not on the sphere would otherwise pull the centre in proportion to how
        far off it is.
        """
        if points.shape[0] < self.min_fit_points:
            return None

        seed = self.estimate_apex_by_band(points)
        if seed is None:
            return None
        seed_centre = np.array([seed[0], seed[1], seed[2] - self.sphere_radius])

        centre, inliers, rms, ok = fit_sphere_robust(
            points,
            self.sphere_radius,
            seed_centre,
            self.fit_inlier_band,
            self.fit_iterations,
            self.fit_tolerance,
            min_inliers=self.min_fit_points,
        )
        if not ok:
            self.get_logger().warn(
                "Sphere fit failed: fewer than %d points remained on the surface."
                % self.min_fit_points,
                throttle_duration_sec=5.0,
            )
            return None

        fraction = float(inliers.sum()) / float(points.shape[0])
        if fraction < self.min_inlier_fraction:
            self.get_logger().warn(
                "Only %.0f%% of the cloud lies on a %.3f m sphere (need %.0f%%); "
                "the crop box is holding something else."
                % (100.0 * fraction, self.sphere_radius,
                   100.0 * self.min_inlier_fraction),
                throttle_duration_sec=5.0,
            )
            return None
        if rms > self.max_fit_rms:
            self.get_logger().warn(
                "Sphere fit inlier RMS %.4f m exceeds max_fit_rms %.4f; is "
                "sphere_radius correct?" % (rms, self.max_fit_rms),
                throttle_duration_sec=5.0,
            )
            return None

        self.get_logger().info(
            "sphere fit: centre %s, inlier RMS %.4f m over %d/%d points"
            % (np.round(centre, 4), rms, int(inliers.sum()), points.shape[0]),
            throttle_duration_sec=5.0,
        )
        return np.array([centre[0], centre[1], centre[2] + self.sphere_radius])


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
