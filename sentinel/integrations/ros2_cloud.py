"""Optional ROS 2 PointCloud2 -> local-map XYZ capture for ATLAS.

Run in a ROS 2 Python environment. No ROS dependency is imported at module load,
so the coordinate math can be tested on machines without ROS. This is an input
bridge, not a flight controller, SLAM implementation, or deployed integration.
"""

import argparse
import csv
import time
from pathlib import Path

import numpy as np


def transform_xyz(points, translation, quaternion):
    """Apply a ROS xyzw quaternion and translation to Nx3 points.

    The caller must supply a transform from the source frame to the desired
    target frame. Rejecting invalid quaternions avoids silently corrupting a map.
    """
    points = np.asarray(points, dtype=float)
    translation = np.asarray(translation, dtype=float)
    quaternion = np.asarray(quaternion, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must be Nx3")
    if translation.shape != (3,) or quaternion.shape != (4,):
        raise ValueError("translation and xyzw quaternion must have 3 and 4 values")
    if not (np.isfinite(points).all() and np.isfinite(translation).all()
            and np.isfinite(quaternion).all()):
        raise ValueError("non-finite transform or point")
    norm = np.linalg.norm(quaternion)
    if norm < 1e-9:
        raise ValueError("zero-length quaternion")
    x, y, z, w = quaternion / norm
    rotation = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])
    return points @ rotation.T + translation


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", required=True, help="sensor_msgs/PointCloud2 topic")
    parser.add_argument("--frame", default="map", help="target Cartesian frame (default: map)")
    parser.add_argument("--out", type=Path, required=True, help="output XYZ CSV file")
    parser.add_argument("--clouds", type=int, default=1, help="number of valid clouds to capture")
    parser.add_argument("--max-points", type=int, default=200000, help="total output point cap")
    parser.add_argument("--timeout", type=float, default=30., help="capture deadline in seconds")
    args = parser.parse_args(argv)
    if args.clouds < 1 or args.max_points < 1 or args.timeout <= 0:
        parser.error("--clouds, --max-points and --timeout must be positive")
    try:
        import rclpy
        from rclpy.duration import Duration
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.time import Time
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py import point_cloud2
        from tf2_ros import Buffer, TransformException, TransformListener
    except ImportError:
        parser.exit(1, "ROS 2 Python packages are unavailable. Run in a sourced ROS 2 environment.\n")

    class CloudCapture(Node):
        def __init__(self):
            super().__init__("sentinel_cloud_capture")
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.count = 0
            self.points = 0
            self.done = False
            args.out.parent.mkdir(parents=True, exist_ok=True)
            self.file = args.out.open("w", newline="", encoding="utf-8")
            self.writer = csv.writer(self.file)
            self.writer.writerow(("x", "y", "z"))
            self.subscription = self.create_subscription(
                PointCloud2, args.topic, self.receive, qos_profile_sensor_data)
            self.get_logger().info(f"Listening to {args.topic}; transforming to {args.frame}")

        def receive(self, message):
            if self.done:
                return
            source = message.header.frame_id
            if not source:
                self.get_logger().warn("Skipped cloud with no frame_id")
                return
            if source != args.frame:
                try:
                    tf = self.tf_buffer.lookup_transform(
                        args.frame, source, Time.from_msg(message.header.stamp),
                        timeout=Duration(seconds=0.2))
                except TransformException as exc:
                    self.get_logger().warn(f"Skipped cloud: transform unavailable: {exc}")
                    return
                t = tf.transform.translation
                q = tf.transform.rotation
                translation = (t.x, t.y, t.z)
                quaternion = (q.x, q.y, q.z, q.w)
            else:
                translation = (0., 0., 0.)
                quaternion = (0., 0., 0., 1.)
            remaining = args.max_points - self.points
            if remaining <= 0:
                self.done = True
                return
            rows = []
            for point in point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True):
                rows.append((float(point[0]), float(point[1]), float(point[2])))
                if len(rows) >= remaining:
                    break
            if not rows:
                self.get_logger().warn("Skipped empty cloud")
                return
            mapped = transform_xyz(rows, translation, quaternion)
            self.writer.writerows(mapped.tolist())
            self.file.flush()
            self.points += len(rows)
            self.count += 1
            self.get_logger().info(f"Captured {self.count} cloud(s), {self.points} points")
            self.done = self.count >= args.clouds or self.points >= args.max_points

        def close(self):
            self.file.close()

    rclpy.init()
    node = None
    try:
        node = CloudCapture()
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and not node.done and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
        if node.count == 0:
            raise SystemExit("No valid clouds captured. Check the topic and TF map transform.")
        print(f"Saved {node.points} points in {args.frame} frame to {args.out}"
              + (" (partial: capture timed out)" if not node.done else ""))
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
