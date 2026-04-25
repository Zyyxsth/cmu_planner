#!/usr/bin/env python3
"""
Publish a small synthetic terrain map for vehicleSimulator height following.

This does not replace Gazebo lidar or planner-facing terrain maps. It only gives
the kinematic vehicleSimulator an unambiguous local floor height in whitebox
multi-level scenes where several traversable surfaces can share the same XY.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


HEIGHT_KINDS = {"floor", "floor_2", "mid_landing", "stair_step", "single_step", "ramp"}


class WhiteboxHeightMap:
    def __init__(self, metadata_path: Path) -> None:
        metadata = json.loads(metadata_path.read_text(encoding="ascii"))
        self.objects = [
            obj
            for obj in metadata["objects"]
            if str(obj.get("terrain_kind", "")) in HEIGHT_KINDS
        ]

    def _height_for_object(self, obj: dict[str, object], x: float) -> float:
        terrain_kind = str(obj.get("terrain_kind", ""))
        if terrain_kind == "ramp":
            x_min = float(obj["x_min"])
            x_max = float(obj["x_max"])
            z_min = float(obj["z_min"])
            z_max = float(obj["z_max"])
            ratio = (x - x_min) / max(x_max - x_min, 1e-6)
            return z_min + max(0.0, min(1.0, ratio)) * (z_max - z_min)
        return float(obj["z_max"])

    def _candidates_at(self, x: float, y: float, margin: float) -> list[float]:
        candidates: list[float] = []
        for obj in self.objects:
            x_min = float(obj["x_min"])
            x_max = float(obj["x_max"])
            y_min = float(obj["y_min"])
            y_max = float(obj["y_max"])
            if not (x_min - margin <= x <= x_max + margin):
                continue
            if not (y_min - margin <= y <= y_max + margin):
                continue
            candidates.append(self._height_for_object(obj, x))
        return candidates

    def surface_height_at(
        self,
        x: float,
        y: float,
        previous_height: Optional[float],
        edge_margin: float,
        max_height_step: float,
    ) -> float:
        exact_candidates = self._candidates_at(x, y, margin=0.0)
        candidates = exact_candidates if exact_candidates else self._candidates_at(x, y, margin=edge_margin)
        if not candidates:
            return previous_height if previous_height is not None else 0.0
        if previous_height is None:
            return min(candidates)
        continuous_candidates = [
            height for height in candidates if abs(height - previous_height) <= max_height_step
        ]
        if not continuous_candidates:
            return previous_height
        return min(continuous_candidates, key=lambda height: abs(height - previous_height))


class WhiteboxVehicleTerrainPublisher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("whitebox_vehicle_terrain_map_publisher")
        self.args = args
        self.height_map = WhiteboxHeightMap(Path(args.metadata))
        self.last_odom: Optional[Odometry] = None
        self.surface_height: Optional[float] = None
        self.publisher = self.create_publisher(PointCloud2, args.output_topic, 2)
        self.create_subscription(Odometry, args.odom_topic, self._odom_callback, 10)
        self.create_timer(1.0 / args.rate, self._publish)
        self.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

    def _odom_callback(self, msg: Odometry) -> None:
        self.last_odom = msg

    def _publish(self) -> None:
        if self.last_odom is None:
            return
        pos = self.last_odom.pose.pose.position
        self.surface_height = self.height_map.surface_height_at(
            float(pos.x),
            float(pos.y),
            self.surface_height,
            self.args.edge_margin,
            self.args.max_height_step,
        )
        points: list[tuple[float, float, float, float]] = []
        steps = int(math.ceil(self.args.radius / self.args.resolution))
        for x_idx in range(-steps, steps + 1):
            for y_idx in range(-steps, steps + 1):
                x = float(pos.x) + x_idx * self.args.resolution
                y = float(pos.y) + y_idx * self.args.resolution
                if math.hypot(x - float(pos.x), y - float(pos.y)) > self.args.radius:
                    continue
                points.append((x, y, self.surface_height, 0.0))

        header = self.last_odom.header
        header.frame_id = self.args.frame_id
        self.publisher.publish(point_cloud2.create_cloud(header, self.fields, points))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish whitebox terrain heights for vehicleSimulator.")
    parser.add_argument("--metadata", required=True)
    parser.add_argument("--odom-topic", default="/state_estimation")
    parser.add_argument("--output-topic", default="/whitebox_vehicle_terrain_map")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--radius", type=float, default=1.20)
    parser.add_argument("--resolution", type=float, default=0.20)
    parser.add_argument("--edge-margin", type=float, default=0.25)
    parser.add_argument("--max-height-step", type=float, default=0.35)
    parser.add_argument("--rate", type=float, default=20.0)
    return parser.parse_args()


def main() -> None:
    rclpy.init()
    node = WhiteboxVehicleTerrainPublisher(parse_args())
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
