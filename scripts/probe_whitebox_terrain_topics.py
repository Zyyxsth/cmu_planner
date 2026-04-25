#!/usr/bin/env python3
"""
Drive the Gazebo whitebox scene through fixed probe goals and sample 2.5D topics.

Start the simulation first:

  ./system_simulation_with_route_planner.sh --gazebo

Then run:

  python3 scripts/probe_whitebox_terrain_topics.py

The script publishes RViz-equivalent route goals on /goal_point by default,
waits until /state_estimation is close to each probe goal, then writes per-topic
terrain statistics under logs/whitebox_terrain_probe/<timestamp>/.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from statistics import mean
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import PointCloud2

from analyze_whitebox_terrain_map import (
    Point,
    Region,
    TerrainClassification,
    assess_direction,
    classify_terrain_points,
    cloud_xy_bounds,
    load_regions,
    read_cloud_points,
    region_overlaps_cloud,
    summarize,
    summarize_bounds,
    stair_traversal_decision,
)


@dataclass(frozen=True)
class ProbeGoal:
    name: str
    x: float
    y: float
    z: float
    note: str


@dataclass
class TopicSample:
    topic: str
    frame_id: str
    stamp_sec: float
    points: list[Point]
    robot_yaw: Optional[float]


def yaw_from_odom(odom: Optional[Odometry]) -> Optional[float]:
    if odom is None:
        return None
    q = odom.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def load_objects(metadata_path: Path) -> dict[str, dict[str, float]]:
    metadata = json.loads(metadata_path.read_text(encoding="ascii"))
    return {obj["name"]: obj for obj in metadata["objects"]}


def center(obj: dict[str, float]) -> tuple[float, float]:
    return (float(obj["x_min"]) + float(obj["x_max"])) / 2.0, (float(obj["y_min"]) + float(obj["y_max"])) / 2.0


def default_probe_goals(objects: dict[str, dict[str, float]]) -> list[ProbeGoal]:
    goals = [ProbeGoal("flat_center", 0.0, 0.0, 0.0, "baseline flat floor near spawn")]

    for name in (
        "ramp_south_2m_18cm",
        "ramp_west_3m_25cm",
        "ramp_southwest_3m_45cm",
        "ramp_northeast_4m_45cm",
    ):
        obj = objects[name]
        x, y = center(obj)
        goals.append(ProbeGoal(name, x, y, float(obj["z_max"]), "ramp center"))

    for name in (
        "single_step_10cm_north",
        "single_step_10cm_east",
        "single_step_15cm_west",
        "single_step_08cm_southwest",
    ):
        obj = objects[name]
        x, y = center(obj)
        goals.append(ProbeGoal(f"{name}_approach", x, y, 0.0, "single-step approach/near-field probe"))

    stair_groups = {
        "stair_south_right": (1.2, -7.6),
        "stair_north_left": (-1.2, 8.6),
        "stair_east_mid": (6.8, 4.4),
    }
    available_stair_groups = {
        str(obj.get("group_name"))
        for obj in objects.values()
        if obj.get("terrain_kind") == "stair_step" and obj.get("group_name") is not None
    }
    for name, (x, y) in stair_groups.items():
        if name in available_stair_groups:
            goals.append(ProbeGoal(f"{name}_entry", x, y, 0.0, "stair entry, not top-of-stair traversal"))

    floor_2 = objects.get("floor_2_south_landing")
    mid_landing = objects.get("mid_landing_realistic")
    if floor_2 is not None and mid_landing is not None:
        floor_2_x, floor_2_y = center(floor_2)
        mid_x, mid_y = center(mid_landing)
        goals.extend(
            [
                ProbeGoal(
                    "two_floor_stair_preentry",
                    -4.00,
                    -1.80,
                    0.0,
                    "staging point aligned with the realistic lower stair flight",
                ),
                ProbeGoal(
                    "two_floor_stair_lower",
                    -2.00,
                    -1.80,
                    0.60,
                    "front-entry probe on the lower realistic stair flight",
                ),
                ProbeGoal(
                    "two_floor_mid_landing",
                    mid_x,
                    mid_y,
                    float(mid_landing["z_max"]),
                    "middle landing between the two stair flights",
                ),
                ProbeGoal(
                    "two_floor_stair_upper",
                    0.60,
                    -0.50,
                    1.65,
                    "entry onto the upper realistic stair flight after the 90-degree turn",
                ),
                ProbeGoal(
                    "two_floor_stair_top",
                    0.60,
                    1.75,
                    float(floor_2["z_max"]),
                    "top step before the realistic second-floor landing",
                ),
                ProbeGoal(
                    "two_floor_goal",
                    floor_2_x,
                    floor_2_y,
                    float(floor_2["z_max"]),
                    "second-floor landing goal reached through the split realistic stair",
                ),
            ]
        )
    elif floor_2 is not None:
        goals.extend(
            [
                ProbeGoal(
                    "two_floor_stair_preentry",
                    1.20,
                    -7.60,
                    0.0,
                    "staging point aligned with stair_south_right lower entry",
                ),
                ProbeGoal(
                    "two_floor_stair_lower",
                    2.45,
                    -7.60,
                    0.10,
                    "front-entry probe on the lower stair run",
                ),
                ProbeGoal(
                    "two_floor_stair_top",
                    3.75,
                    -7.60,
                    0.60,
                    "top step before the second-floor landing",
                ),
                ProbeGoal(
                    "two_floor_goal",
                    (float(floor_2["x_min"]) + float(floor_2["x_max"])) / 2.0,
                    (float(floor_2["y_min"]) + float(floor_2["y_max"])) / 2.0,
                    float(floor_2["z_max"]),
                    "second-floor landing goal reached through stair_south_right",
                ),
            ]
        )

    return goals


def parse_probe_filter(raw: str, goals: list[ProbeGoal]) -> list[ProbeGoal]:
    if not raw:
        return goals
    requested = {item.strip() for item in raw.split(",") if item.strip()}
    selected = [goal for goal in goals if goal.name in requested]
    missing = sorted(requested - {goal.name for goal in selected})
    if missing:
        raise ValueError(f"Unknown probe name(s): {', '.join(missing)}")
    return selected


def distance_xy(odom: Optional[Odometry], goal: ProbeGoal) -> float:
    if odom is None:
        return math.inf
    pos = odom.pose.pose.position
    return math.hypot(pos.x - goal.x, pos.y - goal.y)


def odom_pose_dict(odom: Optional[Odometry]) -> Optional[dict[str, float]]:
    if odom is None:
        return None
    pos = odom.pose.pose.position
    return {"x": float(pos.x), "y": float(pos.y), "z": float(pos.z)}


def sample_region_rows(probe: ProbeGoal, sample: TopicSample, regions: list[Region]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    bounds = cloud_xy_bounds(sample.points)
    for region in regions:
        region_points = [
            point
            for point in sample.points
            if region.x_min <= point[0] <= region.x_max and region.y_min <= point[1] <= region.y_max
        ]
        z_values = [point[2] for point in region_points]
        intensity_values = [point[3] for point in region_points if point[3] is not None]
        if region_points:
            coverage = "observed"
        elif region_overlaps_cloud(region, bounds):
            coverage = "xy_overlaps_but_no_points"
        else:
            coverage = "outside_current_cloud_xy"
        classification = classify_terrain_points(region_points, region) if region_points else TerrainClassification("unobserved", coverage)
        direction = assess_direction(region, sample.robot_yaw)
        traversal_decision = stair_traversal_decision(region, classification, direction)
        rows.append(
            {
                "probe": probe.name,
                "topic": sample.topic,
                "region_kind": region.kind,
                "region": region.name,
                "count": len(region_points),
                "z_stats": summarize(z_values),
                "intensity_stats": summarize(intensity_values),
                "coverage": coverage,
                "terrain_class": classification.label,
                "traversal_axis": direction.axis,
                "entry_side": direction.entry_side,
                "approach_alignment": direction.alignment,
                "approach_class": direction.approach_class,
                "direction_confidence": direction.confidence,
                "stair_traversal_decision": traversal_decision,
                "class_reason": classification.reason,
            }
        )
    return rows


def sample_overview(sample: TopicSample) -> dict[str, object]:
    intensity_values = [point[3] for point in sample.points if point[3] is not None]
    return {
        "topic": sample.topic,
        "frame_id": sample.frame_id,
        "stamp_sec": sample.stamp_sec,
        "point_count": len(sample.points),
        "bounds": summarize_bounds(sample.points),
        "intensity_stats": summarize(intensity_values),
    }


class WhiteboxProbeNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("whitebox_terrain_probe")
        self.args = args
        self.last_odom: Optional[Odometry] = None
        self.last_clouds: dict[str, PointCloud2] = {}

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.goal_pub = self.create_publisher(PointStamped, args.goal_topic, 5)
        self.joy_pub = self.create_publisher(Joy, args.joy_topic, 5) if args.publish_joy else None
        self.direct_waypoint_pub = (
            self.create_publisher(PointStamped, args.direct_waypoint_topic, 5)
            if args.publish_waypoint_too
            else None
        )
        self.create_subscription(Odometry, args.odom_topic, self._odom_callback, 10)
        for topic in args.topics:
            self.create_subscription(PointCloud2, topic, lambda msg, topic=topic: self._cloud_callback(topic, msg), qos)

    def _odom_callback(self, msg: Odometry) -> None:
        self.last_odom = msg

    def _cloud_callback(self, topic: str, msg: PointCloud2) -> None:
        self.last_clouds[topic] = msg

    def publish_goal(self, goal: ProbeGoal) -> None:
        msg = PointStamped()
        msg.header.frame_id = self.args.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = goal.x
        msg.point.y = goal.y
        msg.point.z = goal.z
        self.goal_pub.publish(msg)
        if self.direct_waypoint_pub is not None:
            self.direct_waypoint_pub.publish(msg)

    def publish_joy_enable(self) -> None:
        if self.joy_pub is None:
            return
        joy = Joy()
        joy.header.frame_id = self.args.frame_id
        joy.header.stamp = self.get_clock().now().to_msg()
        joy.axes = [0.0] * 6
        joy.buttons = [0] * 12
        joy.axes[2] = -1.0
        joy.axes[3] = 0.0
        joy.axes[4] = self.args.joy_forward_axis
        joy.axes[5] = 1.0
        self.joy_pub.publish(joy)

    def publish_drive_inputs(self, goal: ProbeGoal) -> None:
        self.publish_goal(goal)
        self.publish_joy_enable()

    def wait_for_initial_data(self) -> bool:
        deadline = time.monotonic() + self.args.startup_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_odom is not None and all(topic in self.last_clouds for topic in self.args.topics):
                return True
        return False

    def goal_z_error(self, goal: ProbeGoal) -> Optional[float]:
        if self.last_odom is None:
            return None
        expected_z = goal.z + self.args.vehicle_height
        return abs(float(self.last_odom.pose.pose.position.z) - expected_z)

    def run_probe(self, goal: ProbeGoal) -> tuple[str, float, Optional[float], Optional[dict[str, float]], list[TopicSample]]:
        start = time.monotonic()
        last_goal_publish = 0.0
        reached = False
        min_distance = math.inf
        min_z_error: Optional[float] = None

        while rclpy.ok() and time.monotonic() - start < self.args.goal_timeout:
            now = time.monotonic()
            if now - last_goal_publish >= 1.0 / self.args.goal_rate:
                self.publish_drive_inputs(goal)
                last_goal_publish = now
            else:
                self.publish_joy_enable()
            rclpy.spin_once(self, timeout_sec=0.1)
            current_distance = distance_xy(self.last_odom, goal)
            min_distance = min(min_distance, current_distance)
            z_error = self.goal_z_error(goal)
            if z_error is not None:
                min_z_error = z_error if min_z_error is None else min(min_z_error, z_error)
            reached_z = True
            if self.args.require_goal_z:
                reached_z = z_error is not None and z_error <= self.args.z_tolerance
            if current_distance <= self.args.reach_radius and reached_z:
                reached = True
                break

        settle_deadline = time.monotonic() + self.args.settle_time
        last_goal_publish = 0.0
        while rclpy.ok() and time.monotonic() < settle_deadline:
            now = time.monotonic()
            if now - last_goal_publish >= 1.0 / self.args.goal_rate:
                self.publish_drive_inputs(goal)
                last_goal_publish = now
            else:
                self.publish_joy_enable()
            rclpy.spin_once(self, timeout_sec=0.1)

        samples: list[TopicSample] = []
        robot_yaw = yaw_from_odom(self.last_odom)
        for topic in self.args.topics:
            msg = self.last_clouds.get(topic)
            if msg is None:
                continue
            stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            samples.append(
                TopicSample(
                    topic=topic,
                    frame_id=msg.header.frame_id,
                    stamp_sec=stamp_sec,
                    points=read_cloud_points(msg),
                    robot_yaw=robot_yaw,
                )
            )

        status = "reached" if reached else "timeout"
        return status, min_distance, min_z_error, odom_pose_dict(self.last_odom), samples


def write_outputs(
    output_dir: Path,
    probe_rows: list[dict[str, object]],
    overview_rows: list[dict[str, object]],
    summary: dict[str, object],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="ascii")

    with (output_dir / "topic_overview.csv").open("w", newline="", encoding="ascii") as csv_file:
        fieldnames = [
            "probe",
            "status",
            "min_distance",
            "final_odom_z",
            "expected_odom_z",
            "min_z_error",
            "topic",
            "frame_id",
            "stamp_sec",
            "point_count",
            "bounds",
            "intensity_stats",
        ]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(overview_rows)

    with (output_dir / "region_stats.csv").open("w", newline="", encoding="ascii") as csv_file:
        fieldnames = [
            "probe",
            "status",
            "min_distance",
            "final_odom_z",
            "expected_odom_z",
            "min_z_error",
            "topic",
            "region_kind",
            "region",
            "count",
            "z_stats",
            "intensity_stats",
            "coverage",
            "terrain_class",
            "traversal_axis",
            "entry_side",
            "approach_alignment",
            "approach_class",
            "direction_confidence",
            "stair_traversal_decision",
            "class_reason",
        ]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(probe_rows)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Probe Gazebo whitebox terrain topics at fixed route goals.")
    parser.add_argument("--metadata", default="src/base_autonomy/vehicle_simulator/mesh/whitebox_stair_test/whitebox_stair_test.json")
    parser.add_argument("--goal-topic", default="/goal_point", help="Route-planner goal topic. RViz goalpoint uses /goal_point.")
    parser.add_argument("--direct-waypoint-topic", default="/way_point", help="Direct local-planner waypoint topic.")
    parser.add_argument("--publish-waypoint-too", action="store_true", help="Also publish each goal directly to /way_point.")
    parser.add_argument("--joy-topic", default="/joy")
    parser.add_argument("--no-publish-joy", dest="publish_joy", action="store_false", help="Do not publish autonomy-enable /joy messages.")
    parser.set_defaults(publish_joy=True)
    parser.add_argument("--joy-forward-axis", type=float, default=0.8, help="Synthetic joystick forward axis for autonomy speed.")
    parser.add_argument("--odom-topic", default="/state_estimation")
    parser.add_argument("--topics", nargs="+", default=["/registered_scan", "/terrain_map", "/terrain_map_ext"])
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--probe", default="", help="Comma-separated probe names. Empty runs all default probes.")
    parser.add_argument("--margin", type=float, default=0.05)
    parser.add_argument("--reach-radius", type=float, default=0.75)
    parser.add_argument("--startup-timeout", type=float, default=20.0)
    parser.add_argument("--goal-timeout", type=float, default=45.0)
    parser.add_argument("--settle-time", type=float, default=2.0)
    parser.add_argument("--goal-rate", type=float, default=2.0)
    parser.add_argument("--vehicle-height", type=float, default=0.75)
    parser.add_argument("--z-tolerance", type=float, default=0.35)
    parser.add_argument("--require-goal-z", action="store_true", help="Require odom z to match goal.z + vehicle-height.")
    parser.add_argument("--output-dir", default="")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    metadata_path = Path(args.metadata)
    objects = load_objects(metadata_path)
    regions = load_regions(metadata_path, args.margin)
    goals = parse_probe_filter(args.probe, default_probe_goals(objects))
    output_dir = Path(args.output_dir) if args.output_dir else Path("logs/whitebox_terrain_probe") / datetime.now().strftime("%Y%m%d_%H%M%S")

    rclpy.init()
    node = WhiteboxProbeNode(args)
    try:
        if not node.wait_for_initial_data():
            node.get_logger().error("Timed out waiting for odometry and cloud topics. Is the Gazebo simulation running?")
            sys.exit(1)

        overview_rows: list[dict[str, object]] = []
        region_rows: list[dict[str, object]] = []
        summary: dict[str, object] = {
            "metadata": str(metadata_path),
            "goal_topic": args.goal_topic,
            "publish_waypoint_too": args.publish_waypoint_too,
            "publish_joy": args.publish_joy,
            "joy_topic": args.joy_topic,
            "joy_forward_axis": args.joy_forward_axis,
            "odom_topic": args.odom_topic,
            "vehicle_height": args.vehicle_height,
            "require_goal_z": args.require_goal_z,
            "z_tolerance": args.z_tolerance,
            "topics": args.topics,
            "output_dir": str(output_dir),
            "probes": [],
        }

        for goal in goals:
            node.get_logger().info(f"Probe {goal.name}: goal=({goal.x:.2f}, {goal.y:.2f}, {goal.z:.2f}) {goal.note}")
            status, min_distance, min_z_error, final_odom, samples = node.run_probe(goal)
            final_z = final_odom["z"] if final_odom is not None else None
            expected_z = goal.z + args.vehicle_height
            final_z_text = f"{final_z:.2f}m" if final_z is not None else "n/a"
            z_error_text = f"{min_z_error:.2f}m" if min_z_error is not None else "n/a"
            node.get_logger().info(
                f"Probe {goal.name}: status={status}, min_distance={min_distance:.2f}m, "
                f"final_odom_z={final_z_text}, expected_odom_z={expected_z:.2f}m, min_z_error={z_error_text}"
            )
            summary["probes"].append(
                {
                    "name": goal.name,
                    "goal": {"x": goal.x, "y": goal.y, "z": goal.z},
                    "note": goal.note,
                    "status": status,
                    "min_distance": min_distance,
                    "expected_odom_z": expected_z,
                    "min_z_error": min_z_error,
                    "final_odom": final_odom,
                }
            )
            for sample in samples:
                overview = sample_overview(sample)
                overview.update(
                    {
                        "probe": goal.name,
                        "status": status,
                        "min_distance": min_distance,
                        "final_odom_z": final_z,
                        "expected_odom_z": expected_z,
                        "min_z_error": min_z_error,
                    }
                )
                overview_rows.append(overview)
                for row in sample_region_rows(goal, sample, regions):
                    row.update({
                        "status": status,
                        "min_distance": min_distance,
                        "final_odom_z": final_z,
                        "expected_odom_z": expected_z,
                        "min_z_error": min_z_error,
                    })
                    region_rows.append(row)

        write_outputs(output_dir, region_rows, overview_rows, summary)
        print(f"Wrote probe outputs to {output_dir}")
        print(f"- {output_dir / 'summary.json'}")
        print(f"- {output_dir / 'topic_overview.csv'}")
        print(f"- {output_dir / 'region_stats.csv'}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
