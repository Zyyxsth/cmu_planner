#!/usr/bin/env python3
"""
Publish a static map point cloud repeatedly on /registered_scan.

This allows route-planner bringup without a Unity depth/render pipeline when we
only need a fixed whitebox environment for early planner validation.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2


def read_ascii_ply_points(path: Path) -> list[tuple[float, float, float, float]]:
    lines = path.read_text(encoding="ascii").splitlines()
    vertex_count = 0
    data_start = None
    for idx, line in enumerate(lines):
        if line.startswith("element vertex "):
            vertex_count = int(line.split()[-1])
        if line.strip() == "end_header":
            data_start = idx + 1
            break
    if data_start is None:
        raise ValueError(f"Invalid PLY header: {path}")
    points: list[tuple[float, float, float, float]] = []
    for line in lines[data_start : data_start + vertex_count]:
        if not line.strip():
            continue
        x_str, y_str, z_str = line.split()[:3]
        points.append((float(x_str), float(y_str), float(z_str), 0.0))
    return points


class StaticRegisteredScanPublisher(Node):
    def __init__(self, ply_path: Path, topic: str, frame_id: str, rate_hz: float) -> None:
        super().__init__("static_registered_scan_publisher")
        self._points = read_ascii_ply_points(ply_path)
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(PointCloud2, topic, qos)
        self._topic = topic
        self._frame_id = frame_id
        self._msg = self._build_msg()
        self.create_timer(1.0 / rate_hz, self._publish)
        self.get_logger().info(
            f"Publishing {len(self._points)} static points from {ply_path} to {topic} at {rate_hz:.1f} Hz"
        )

    def _build_msg(self) -> PointCloud2:
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        header = Header(frame_id=self._frame_id)
        return point_cloud2.create_cloud(header, fields, self._points)

    def _publish(self) -> None:
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._msg)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish a static PLY as /registered_scan.")
    parser.add_argument(
        "--ply",
        default="src/base_autonomy/vehicle_simulator/mesh/whitebox_stair_test/map.ply",
        help="ASCII PLY file to publish.",
    )
    parser.add_argument("--topic", default="/registered_scan", help="Output point cloud topic.")
    parser.add_argument("--frame-id", default="map", help="Point cloud frame id.")
    parser.add_argument("--rate", type=float, default=5.0, help="Publish rate in Hz.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = StaticRegisteredScanPublisher(Path(args.ply), args.topic, args.frame_id, args.rate)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
