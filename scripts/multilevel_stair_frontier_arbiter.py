#!/usr/bin/env python3
"""Arbitrate normal TARE exploration waypoints with known stair frontiers.

This node is a Gazebo-only multi-level exploration prototype. It keeps the
original TARE planner unchanged by remapping TARE's waypoint output to a raw
topic, then publishes the final /way_point after adding a known stair connector
as a competing frontier candidate.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header, Int8, String
from geometry_msgs.msg import TwistStamped
from visualization_msgs.msg import Marker


@dataclass
class Goal:
    name: str
    x: float
    y: float
    z: float


@dataclass
class StairConnector:
    name: str
    lower_entry: Goal
    upper_entry: Goal
    up_route: list[Goal]

    @property
    def length(self) -> float:
        points = [self.lower_entry] + self.up_route
        return sum(
            math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
            for a, b in zip(points, points[1:])
        )


def yaw_from_odom(odom: Odometry) -> float:
    q = odom.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def goal_from_dict(data: dict, default_name: str, vehicle_height: float) -> Goal:
    return Goal(
        str(data.get("name", default_name)),
        float(data["x"]),
        float(data["y"]),
        float(data["z"]) + vehicle_height,
    )


def office_connector_from_metadata(data: dict, vehicle_height: float) -> StairConnector:
    lower = data["lower_entry"]
    upper = data["upper_entry"]
    lower_goal = Goal("stair_lower_entry", float(lower[0]), float(lower[1]), float(lower[2]) + vehicle_height)
    upper_goal = Goal("stair_upper_entry", float(upper[0]), float(upper[1]), float(upper[2]) + vehicle_height)
    if data.get("up_route"):
        route = [goal_from_dict(goal, f"stair_step_{index:02d}", vehicle_height) for index, goal in enumerate(data["up_route"], 1)]
    else:
        step_count = max(1, int(data.get("step_count", 20)))
        route = []
        for index in range(1, step_count + 1):
            ratio = index / step_count
            route.append(
                Goal(
                    f"stair_step_{index:02d}",
                    lower_goal.x + (upper_goal.x - lower_goal.x) * ratio,
                    lower_goal.y + (upper_goal.y - lower_goal.y) * ratio,
                    lower_goal.z + (upper_goal.z - lower_goal.z) * ratio,
                )
            )
    if not route or math.dist((route[-1].x, route[-1].y, route[-1].z), (upper_goal.x, upper_goal.y, upper_goal.z)) > 0.05:
        route.append(upper_goal)
    return StairConnector(str(data.get("name", "office_stair_connector")), lower_goal, upper_goal, route)


def whitebox_connector_from_metadata(data: dict, vehicle_height: float) -> StairConnector:
    lower = goal_from_dict(data["lower_entry"], "stair_lower_entry", vehicle_height)
    upper = goal_from_dict(data["upper_entry"], "stair_upper_entry", vehicle_height)
    up_route = [goal_from_dict(goal, f"stair_step_{i:02d}", vehicle_height) for i, goal in enumerate(data["route_up"], 1)]
    if not up_route or math.dist((up_route[-1].x, up_route[-1].y, up_route[-1].z), (upper.x, upper.y, upper.z)) > 0.05:
        up_route.append(upper)
    return StairConnector(str(data.get("name", "stair_connector")), lower, upper, up_route)


def load_stair_connectors(metadata_path: Path, vehicle_height: float) -> list[StairConnector]:
    with metadata_path.open("r", encoding="utf-8") as handle:
        metadata = json.load(handle)
    connectors: list[StairConnector] = []
    for item in metadata.get("connectors", []):
        if not isinstance(item, dict):
            continue
        if item.get("connector_kind") == "stair" and "route_up" in item:
            connectors.append(whitebox_connector_from_metadata(item, vehicle_height))
        elif "lower_entry" in item and "upper_entry" in item:
            connectors.append(office_connector_from_metadata(item, vehicle_height))
    return connectors


class MultilevelStairFrontierArbiter(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("multilevel_stair_frontier_arbiter")
        self.args = args
        self.connectors = load_stair_connectors(Path(args.metadata), args.vehicle_height)
        self.connector = self.connectors[0] if self.connectors else None
        self.last_odom: Optional[Odometry] = None
        self.initial_home: Optional[Goal] = None
        self.floor1_raw_waypoint: Optional[PointStamped] = None
        self.floor2_raw_waypoint: Optional[PointStamped] = None
        self.floor1_exploration_finished = False
        self.floor2_exploration_finished = False
        self.current_floor = 1
        self.state = "PASS_THROUGH"
        self.route: list[Goal] = []
        self.route_index = 0
        self.traversed_up = False
        self.traversed_down = False
        self.floor2_start_requested = False
        self.floor2_safe_zone_scan_count = 0
        self.floor2_safe_zone_scanned = False
        self.stair_discovered = False
        self.invalid_raw_waypoint_warned = False
        self.floor1_input_counts = {"state": 0, "scan": 0, "terrain": 0, "terrain_ext": 0}
        self.floor2_input_counts = {"state": 0, "scan": 0, "terrain": 0, "terrain_ext": 0}
        self.last_debug_summary_time = self.get_clock().now()
        self.debug_log_path = self._make_debug_log_path(Path(args.debug_log_dir))
        self.debug_log = self.debug_log_path.open("a", encoding="utf-8")
        self.last_output_stamp = self.get_clock().now()
        self._write_debug_event(
            "run_start",
            pid=os.getpid(),
            metadata=args.metadata,
            log_path=str(self.debug_log_path),
            floor1_inputs={
                "state": args.floor1_state_estimation_at_scan_topic,
                "scan": args.floor1_registered_scan_topic,
                "terrain": args.floor1_terrain_map_topic,
                "terrain_ext": args.floor1_terrain_map_ext_topic,
            },
            floor2_inputs={
                "state": args.floor2_state_estimation_at_scan_topic,
                "scan": args.floor2_registered_scan_topic,
                "terrain": args.floor2_terrain_map_topic,
                "terrain_ext": args.floor2_terrain_map_ext_topic,
            },
        )
        self.get_logger().info(f"Writing multilevel debug log to {self.debug_log_path}")

        self.waypoint_pub = self.create_publisher(PointStamped, args.output_waypoint_topic, 5)
        self.pose_override_pub = self.create_publisher(PoseStamped, args.pose_override_topic, 5)
        self.stop_pub = self.create_publisher(Int8, args.stop_topic, 5)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, args.cmd_vel_topic, 5)
        self.frontier_cloud_pub = self.create_publisher(PointCloud2, args.stair_frontier_topic, 2)
        self.connector_path_pub = self.create_publisher(RosPath, args.connector_path_topic, 2)
        self.state_pub = self.create_publisher(String, args.state_topic, 2)
        self.floor2_start_pub = self.create_publisher(Bool, args.floor2_start_request_topic, 5)
        self.floor1_state_scan_pub = self.create_publisher(Odometry, args.floor1_state_estimation_at_scan_topic, 10)
        self.floor1_scan_pub = self.create_publisher(PointCloud2, args.floor1_registered_scan_topic, 5)
        self.floor1_terrain_pub = self.create_publisher(PointCloud2, args.floor1_terrain_map_topic, 5)
        self.floor1_terrain_ext_pub = self.create_publisher(PointCloud2, args.floor1_terrain_map_ext_topic, 5)
        self.floor2_state_scan_pub = self.create_publisher(Odometry, args.floor2_state_estimation_at_scan_topic, 10)
        self.floor2_scan_pub = self.create_publisher(PointCloud2, args.floor2_registered_scan_topic, 5)
        self.floor2_terrain_pub = self.create_publisher(PointCloud2, args.floor2_terrain_map_topic, 5)
        self.floor2_terrain_ext_pub = self.create_publisher(PointCloud2, args.floor2_terrain_map_ext_topic, 5)
        self._setup_active_floor_visualization_relays()

        floor1_raw_topic = args.raw_waypoint_topic or args.floor1_raw_waypoint_topic
        floor1_finish_topic = args.exploration_finish_topic or args.floor1_exploration_finish_topic
        self.create_subscription(PointStamped, floor1_raw_topic, self._floor1_raw_waypoint_callback, 10)
        self.create_subscription(PointStamped, args.floor2_raw_waypoint_topic, self._floor2_raw_waypoint_callback, 10)
        self.create_subscription(Odometry, args.odom_topic, self._odom_callback, 20)
        self.create_subscription(Odometry, args.state_estimation_at_scan_topic, self._state_scan_callback, 20)
        self.create_subscription(PointCloud2, args.registered_scan_topic, self._registered_scan_callback, 5)
        self.create_subscription(PointCloud2, args.terrain_map_topic, self._terrain_map_callback, 5)
        self.create_subscription(PointCloud2, args.terrain_map_ext_topic, self._terrain_map_ext_callback, 5)
        self.create_subscription(Bool, floor1_finish_topic, self._floor1_exploration_finish_callback, 5)
        self.create_subscription(Bool, args.floor2_exploration_finish_topic, self._floor2_exploration_finish_callback, 5)
        self.create_timer(1.0 / args.publish_rate, self._tick)
        self.create_timer(1.0, self._publish_visualization)

        if self.connector is None:
            self.get_logger().warn("No stair connector metadata found; multilevel arbiter will pass TARE waypoints through.")
            self._write_debug_event("connector_missing")
        else:
            self.get_logger().info(
                f"Loaded stair connector {self.connector.name}: "
                f"lower=({self.connector.lower_entry.x:.2f}, {self.connector.lower_entry.y:.2f}, {self.connector.lower_entry.z:.2f}) "
                f"upper=({self.connector.upper_entry.x:.2f}, {self.connector.upper_entry.y:.2f}, {self.connector.upper_entry.z:.2f})."
            )
            self._write_debug_event(
                "connector_loaded",
                name=self.connector.name,
                lower=self._goal_to_dict(self.connector.lower_entry),
                upper=self._goal_to_dict(self.connector.upper_entry),
                route_len=len(self.connector.up_route),
            )
            if self.args.known_stair_global:
                self.get_logger().info("Stair connector is globally known to the robot at startup.")
            else:
                self.get_logger().info(
                    "Stair connector is hidden from the robot until the robot is within "
                    f"{self.args.stair_discovery_radius:.1f}m of the lower stair entry."
                )

    def destroy_node(self) -> bool:
        try:
            self._write_debug_event("run_end")
            self.debug_log.close()
        except Exception:
            pass
        return super().destroy_node()

    def _make_debug_log_path(self, debug_log_dir: Path) -> Path:
        debug_log_dir = debug_log_dir.expanduser()
        debug_log_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        return debug_log_dir / f"multilevel_arbiter_{stamp}_{os.getpid()}.jsonl"

    def _write_debug_event(self, event: str, **fields) -> None:
        if not hasattr(self, "debug_log"):
            return
        now = self.get_clock().now()
        xyz = self._robot_xyz()
        record = {
            "event": event,
            "ros_time": round(now.nanoseconds / 1e9, 6),
            "wall_time": round(time.time(), 6),
            "state": self.state,
            "floor": self.current_floor,
            "robot": self._xyz_to_dict(xyz),
            "floor1_done": self.floor1_exploration_finished,
            "floor2_done": self.floor2_exploration_finished,
            "traversed_up": self.traversed_up,
            "traversed_down": self.traversed_down,
        }
        record.update(fields)
        self.debug_log.write(json.dumps(record, sort_keys=True) + "\n")
        self.debug_log.flush()

    def _xyz_to_dict(self, xyz: Optional[tuple[float, float, float]]) -> Optional[dict[str, float]]:
        if xyz is None:
            return None
        return {"x": round(xyz[0], 3), "y": round(xyz[1], 3), "z": round(xyz[2], 3)}

    def _goal_to_dict(self, goal: Goal) -> dict[str, object]:
        return {"name": goal.name, "x": round(goal.x, 3), "y": round(goal.y, 3), "z": round(goal.z, 3)}

    def _point_to_dict(self, msg: Optional[PointStamped]) -> Optional[dict[str, object]]:
        if msg is None:
            return None
        point = msg.point
        return {
            "x": float(point.x),
            "y": float(point.y),
            "z": float(point.z),
            "finite": self._point_is_finite(point),
        }

    def _setup_active_floor_visualization_relays(self) -> None:
        self.cloud_relay_pubs: dict[str, object] = {}
        self.marker_relay_pubs: dict[str, object] = {}
        self.path_relay_pubs: dict[str, object] = {}
        for topic in self.args.relay_cloud_topics:
            self.cloud_relay_pubs[topic] = self.create_publisher(PointCloud2, f"/{topic}", 2)
            self.create_subscription(PointCloud2, f"/floor1/{topic}", lambda msg, t=topic: self._relay_cloud(1, t, msg), 2)
            self.create_subscription(PointCloud2, f"/floor2/{topic}", lambda msg, t=topic: self._relay_cloud(2, t, msg), 2)
        for topic in self.args.relay_marker_topics:
            self.marker_relay_pubs[topic] = self.create_publisher(Marker, f"/{topic}", 2)
            self.create_subscription(Marker, f"/floor1/{topic}", lambda msg, t=topic: self._relay_marker(1, t, msg), 2)
            self.create_subscription(Marker, f"/floor2/{topic}", lambda msg, t=topic: self._relay_marker(2, t, msg), 2)
        for topic in self.args.relay_path_topics:
            self.path_relay_pubs[topic] = self.create_publisher(RosPath, f"/{topic}", 2)
            self.create_subscription(RosPath, f"/floor1/{topic}", lambda msg, t=topic: self._relay_path(1, t, msg), 2)
            self.create_subscription(RosPath, f"/floor2/{topic}", lambda msg, t=topic: self._relay_path(2, t, msg), 2)

    def _relay_cloud(self, floor: int, topic: str, msg: PointCloud2) -> None:
        if floor == self.current_floor:
            self.cloud_relay_pubs[topic].publish(msg)

    def _relay_marker(self, floor: int, topic: str, msg: Marker) -> None:
        if floor == self.current_floor:
            self.marker_relay_pubs[topic].publish(msg)

    def _relay_path(self, floor: int, topic: str, msg: RosPath) -> None:
        if floor == self.current_floor:
            self.path_relay_pubs[topic].publish(msg)

    def _odom_callback(self, msg: Odometry) -> None:
        self.last_odom = msg
        if self.initial_home is None:
            pos = msg.pose.pose.position
            self.initial_home = Goal("global_initial_home", float(pos.x), float(pos.y), float(pos.z))
            self.get_logger().info(
                f"Recorded global initial home at "
                f"({self.initial_home.x:.2f}, {self.initial_home.y:.2f}, {self.initial_home.z:.2f})."
            )
            self._write_debug_event("initial_home_recorded", home=self._goal_to_dict(self.initial_home))

    def _floor1_raw_waypoint_callback(self, msg: PointStamped) -> None:
        self.floor1_raw_waypoint = msg
        if not self._point_is_finite(msg.point):
            self._write_debug_event("raw_waypoint_nonfinite", source_floor=1, waypoint=self._point_to_dict(msg))

    def _floor2_raw_waypoint_callback(self, msg: PointStamped) -> None:
        self.floor2_raw_waypoint = msg
        if not self._point_is_finite(msg.point):
            self._write_debug_event("raw_waypoint_nonfinite", source_floor=2, waypoint=self._point_to_dict(msg))

    def _floor1_exploration_finish_callback(self, msg: Bool) -> None:
        finished = bool(msg.data)
        if finished != self.floor1_exploration_finished:
            self._write_debug_event("exploration_finish_changed", source_floor=1, value=finished)
        self.floor1_exploration_finished = finished

    def _floor2_exploration_finish_callback(self, msg: Bool) -> None:
        finished = bool(msg.data)
        if finished != self.floor2_exploration_finished:
            self._write_debug_event("exploration_finish_changed", source_floor=2, value=finished)
        self.floor2_exploration_finished = finished

    def _floor2_inputs_active(self) -> bool:
        return self.current_floor == 2 and self.floor2_start_requested and not self.traversed_down

    def _floor1_inputs_active(self) -> bool:
        return self.current_floor == 1 and self.state == "PASS_THROUGH"

    def _state_scan_callback(self, msg: Odometry) -> None:
        if self._floor1_inputs_active():
            self.floor1_input_counts["state"] += 1
            self.floor1_state_scan_pub.publish(msg)
        if self._floor2_inputs_active():
            self.floor2_input_counts["state"] += 1
            self.floor2_state_scan_pub.publish(msg)

    def _registered_scan_callback(self, msg: PointCloud2) -> None:
        if self._floor1_inputs_active():
            self.floor1_input_counts["scan"] += 1
            self.floor1_scan_pub.publish(msg)
        if self._floor2_inputs_active():
            self.floor2_input_counts["scan"] += 1
            self.floor2_scan_pub.publish(msg)
            self._update_floor2_safe_zone_scan_progress()

    def _terrain_map_callback(self, msg: PointCloud2) -> None:
        if self._floor1_inputs_active():
            self.floor1_input_counts["terrain"] += 1
            self.floor1_terrain_pub.publish(msg)
        if self._floor2_inputs_active():
            self.floor2_input_counts["terrain"] += 1
            self.floor2_terrain_pub.publish(msg)

    def _terrain_map_ext_callback(self, msg: PointCloud2) -> None:
        if self._floor1_inputs_active():
            self.floor1_input_counts["terrain_ext"] += 1
            self.floor1_terrain_ext_pub.publish(msg)
        if self._floor2_inputs_active():
            self.floor2_input_counts["terrain_ext"] += 1
            self.floor2_terrain_ext_pub.publish(msg)

    def _robot_xyz(self) -> Optional[tuple[float, float, float]]:
        if self.last_odom is None:
            return None
        pos = self.last_odom.pose.pose.position
        return float(pos.x), float(pos.y), float(pos.z)

    def _robot_on_upper_floor(self) -> bool:
        xyz = self._robot_xyz()
        if xyz is None or self.connector is None:
            return False
        return xyz[2] >= self.connector.upper_entry.z - self.args.floor_z_tolerance

    def _floor2_distance_to_upper_entry(self) -> float:
        if self.connector is None:
            return float("inf")
        return self._distance_xy_to_goal(self.connector.upper_entry)

    def _update_floor2_safe_zone_scan_progress(self) -> None:
        if (
            self.floor2_safe_zone_scanned
            or self.connector is None
            or self.current_floor != 2
            or self.state != "PASS_THROUGH"
            or self.floor2_exploration_finished
            or not self._robot_on_upper_floor()
        ):
            return
        if self._floor2_distance_to_upper_entry() < self.args.floor2_safe_scan_radius:
            return
        self.floor2_safe_zone_scan_count += 1
        if self.floor2_safe_zone_scan_count >= self.args.floor2_safe_scan_min_scans:
            self.floor2_safe_zone_scanned = True
            self.get_logger().info(
                "Floor2 stair safety zone has been scanned from outside "
                f"{self.args.floor2_safe_scan_radius:.1f}m of the upper stair entry."
            )
            self._write_debug_event(
                "floor2_safe_zone_scanned",
                scan_count=self.floor2_safe_zone_scan_count,
                distance_to_upper=round(self._floor2_distance_to_upper_entry(), 3),
            )

    def _distance_xy_to_goal(self, goal: Goal) -> float:
        xyz = self._robot_xyz()
        if xyz is None:
            return float("inf")
        return math.hypot(xyz[0] - goal.x, xyz[1] - goal.y)

    def _raw_waypoint_score(self) -> float:
        xyz = self._robot_xyz()
        raw_waypoint = self._active_raw_waypoint()
        if xyz is None or raw_waypoint is None:
            return float("inf")
        point = raw_waypoint.point
        if not self._point_is_finite(point):
            return float("inf")
        return math.hypot(float(point.x) - xyz[0], float(point.y) - xyz[1])

    def _active_raw_waypoint(self) -> Optional[PointStamped]:
        if self.current_floor == 2:
            return self.floor2_raw_waypoint
        return self.floor1_raw_waypoint

    def _point_is_finite(self, point) -> bool:
        return math.isfinite(float(point.x)) and math.isfinite(float(point.y)) and math.isfinite(float(point.z))

    def _active_exploration_finished(self) -> bool:
        return self.floor2_exploration_finished if self.current_floor == 2 else self.floor1_exploration_finished

    def _stair_score(self) -> float:
        if self.connector is None:
            return float("inf")
        return (
            self._distance_xy_to_goal(self.connector.lower_entry)
            + self.args.connector_cost_weight * self.connector.length
            - self.args.stair_frontier_bonus
        )

    def _stair_candidate_active(self) -> bool:
        if self.connector is None or self.current_floor != 1 or self.traversed_up or self._robot_on_upper_floor():
            return False
        if self.args.known_stair_global:
            return True
        return self.stair_discovered

    def _stair_should_win(self) -> bool:
        if not self._stair_candidate_active():
            return False
        if self._active_exploration_finished() and self.args.force_stair_when_floor_complete:
            return True
        if self._active_raw_waypoint() is None:
            return False
        if self._distance_xy_to_goal(self.connector.lower_entry) <= self.args.stair_commit_radius:
            return True
        return self._stair_score() < self._raw_waypoint_score()

    def _start_stair_route(self) -> None:
        if self.connector is None:
            return
        self._write_debug_event(
            "state_transition",
            from_state=self.state,
            to_state="TO_STAIR_ENTRY",
            reason="stair_selected",
            stair_score=round(self._stair_score(), 3),
            raw_score=round(self._raw_waypoint_score(), 3),
            raw_waypoint=self._point_to_dict(self._active_raw_waypoint()),
        )
        self.state = "TO_STAIR_ENTRY"
        self.route = [self.connector.lower_entry]
        self.route_index = 0
        self.get_logger().info(
            f"Selected stair frontier {self.connector.name}: "
            f"stair_score={self._stair_score():.2f}, raw_score={self._raw_waypoint_score():.2f}."
        )

    def _goal_reached(self, goal: Goal, radius: float, z_tolerance: float) -> bool:
        xyz = self._robot_xyz()
        if xyz is None:
            return False
        return math.hypot(xyz[0] - goal.x, xyz[1] - goal.y) <= radius and abs(xyz[2] - goal.z) <= z_tolerance

    def _publish_waypoint_goal(self, goal: Goal) -> None:
        msg = PointStamped()
        msg.header.frame_id = self.args.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = goal.x
        msg.point.y = goal.y
        msg.point.z = goal.z
        self.waypoint_pub.publish(msg)

    def _publish_raw_waypoint(self) -> None:
        raw_waypoint = self._active_raw_waypoint()
        if raw_waypoint is None:
            return
        if not self._point_is_finite(raw_waypoint.point):
            if not self.invalid_raw_waypoint_warned:
                self.get_logger().warn(
                    f"Dropping non-finite floor{self.current_floor} raw waypoint; waiting for TARE to recover."
                )
                self._write_debug_event(
                    "drop_nonfinite_raw_waypoint",
                    source_floor=self.current_floor,
                    waypoint=self._point_to_dict(raw_waypoint),
                )
                self.invalid_raw_waypoint_warned = True
            self._publish_stop()
            return
        self.invalid_raw_waypoint_warned = False
        self._release_path_follower()
        msg = PointStamped()
        msg.header = raw_waypoint.header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point = raw_waypoint.point
        self.waypoint_pub.publish(msg)

    def _publish_stop(self) -> None:
        msg = Int8()
        msg.data = 2
        self.stop_pub.publish(msg)
        cmd = TwistStamped()
        cmd.header.frame_id = self.args.frame_id
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.cmd_vel_pub.publish(cmd)

    def _release_path_follower(self) -> None:
        msg = Int8()
        msg.data = 0
        self.stop_pub.publish(msg)

    def _publish_pose_override_toward(self, goal: Goal) -> None:
        xyz = self._robot_xyz()
        if xyz is None or self.last_odom is None:
            return
        dx = goal.x - xyz[0]
        dy = goal.y - xyz[1]
        dz = goal.z - xyz[2]
        xy_dist = math.hypot(dx, dy)
        max_step = self.args.connector_speed / self.args.publish_rate
        if xy_dist > max_step:
            ratio = max_step / xy_dist
            next_x = xyz[0] + dx * ratio
            next_y = xyz[1] + dy * ratio
            next_z = xyz[2] + dz * ratio
            yaw = math.atan2(dy, dx)
        else:
            next_x = goal.x
            next_y = goal.y
            if abs(dz) > max_step:
                next_z = xyz[2] + math.copysign(max_step, dz)
            else:
                next_z = goal.z
            yaw = math.atan2(dy, dx) if xy_dist > 1e-3 else yaw_from_odom(self.last_odom)

        msg = PoseStamped()
        msg.header.frame_id = self.args.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = next_x
        msg.pose.position.y = next_y
        msg.pose.position.z = next_z
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        self.pose_override_pub.publish(msg)

    def _advance_route(self) -> None:
        self.route_index += 1
        if self.route_index < len(self.route):
            return
        if self.state == "TO_STAIR_ENTRY":
            assert self.connector is not None
            self._write_debug_event(
                "state_transition",
                from_state=self.state,
                to_state="STAIR_EXEC_UP",
                reason="lower_entry_reached",
                route_len=len(self.connector.up_route),
            )
            self.state = "STAIR_EXEC_UP"
            self.route = self.connector.up_route
            self.route_index = 0
            self.get_logger().info(f"Reached stair entry; executing {len(self.route)} connector waypoints.")
            return
        if self.state == "STAIR_EXEC_UP":
            self._write_debug_event(
                "state_transition",
                from_state=self.state,
                to_state="PASS_THROUGH",
                reason="upper_exit_reached",
                new_floor=2,
            )
            self.state = "PASS_THROUGH"
            self.route = []
            self.route_index = 0
            self.current_floor = 2
            self.traversed_up = True
            self.floor2_exploration_finished = False
            self._release_path_follower()
            self._request_floor2_tare_start()
            self.get_logger().info("Reached upper stair exit; starting floor2 TARE and switching active exploration floor.")
            return
        if self.state == "STAIR_EXEC_DOWN":
            self._write_debug_event(
                "state_transition",
                from_state=self.state,
                to_state="PASS_THROUGH",
                reason="lower_exit_reached",
                new_floor=1,
            )
            self.state = "PASS_THROUGH"
            self.route = []
            self.route_index = 0
            self.current_floor = 1
            self.traversed_down = True
            self._release_path_follower()
            self.get_logger().info("Reached lower stair exit; resuming floor1 TARE exploration/return-home.")
            return

    def _request_floor2_tare_start(self) -> None:
        if self.floor2_start_requested:
            return
        self.floor2_start_requested = True
        self._write_debug_event("floor2_start_requested")
        msg = Bool()
        msg.data = True
        self.floor2_start_pub.publish(msg)

    def _start_down_route(self) -> None:
        if self.connector is None:
            return
        route = list(reversed(self.connector.up_route))
        route = self._trim_route_to_nearest_ahead(route)
        if not route or math.dist((route[-1].x, route[-1].y, route[-1].z), (self.connector.lower_entry.x, self.connector.lower_entry.y, self.connector.lower_entry.z)) > 0.05:
            route.append(self.connector.lower_entry)
        self._write_debug_event(
            "state_transition",
            from_state=self.state,
            to_state="STAIR_EXEC_DOWN",
            reason="floor2_finished_at_upper_entry",
            route_len=len(route),
            floor2_waypoint=self._point_to_dict(self.floor2_raw_waypoint),
        )
        self.state = "STAIR_EXEC_DOWN"
        self.route = route
        self.route_index = 0
        self.get_logger().info(f"Floor2 exploration finished; executing {len(self.route)} stair waypoints down.")

    def _trim_route_to_nearest_ahead(self, route: list[Goal]) -> list[Goal]:
        xyz = self._robot_xyz()
        if xyz is None or not route:
            return route
        best_index = min(
            range(len(route)),
            key=lambda index: math.dist((xyz[0], xyz[1], xyz[2]), (route[index].x, route[index].y, route[index].z)),
        )
        if best_index == 0:
            return route
        self._write_debug_event(
            "route_trimmed_to_nearest",
            original_len=len(route),
            trimmed_len=len(route) - best_index,
            skipped=best_index,
            nearest_goal=self._goal_to_dict(route[best_index]),
        )
        return route[best_index:]

    def _start_to_upper_stair_entry(self) -> None:
        if self.connector is None:
            return
        self._write_debug_event(
            "state_transition",
            from_state=self.state,
            to_state="TO_UPPER_STAIR_ENTRY",
            reason="floor2_finished_away_from_upper_entry",
            distance_to_upper=round(self._floor2_distance_to_upper_entry(), 3),
            floor2_waypoint=self._point_to_dict(self.floor2_raw_waypoint),
        )
        self.state = "TO_UPPER_STAIR_ENTRY"
        self.route = [self.connector.upper_entry]
        self.route_index = 0
        self.get_logger().info("Floor2 exploration finished; returning to upper stair entry before descending.")

    def _tick(self) -> None:
        self._publish_state()
        self._maybe_log_debug_summary()
        if self.connector is None:
            self._publish_raw_waypoint()
            return
        self._update_stair_discovery()

        if self.state == "PASS_THROUGH":
            if self.current_floor == 2:
                self._publish_floor2_start_request_if_needed()
                if self.floor2_exploration_finished:
                    if self._floor2_distance_to_upper_entry() <= self.args.floor2_direct_descent_radius:
                        self._start_down_route()
                    else:
                        self._start_to_upper_stair_entry()
                else:
                    self._publish_raw_waypoint()
                    return
            elif self._stair_should_win():
                self._start_stair_route()
            else:
                self._publish_raw_waypoint()
                return

        if self.state == "COMPLETE":
            self._publish_stop()
            return

        if not self.route or self.route_index >= len(self.route):
            return

        goal = self.route[self.route_index]
        if self.state == "TO_STAIR_ENTRY":
            self._release_path_follower()
            self._publish_waypoint_goal(goal)
            if self._goal_reached(goal, self.args.lower_entry_reach_radius, self.args.entry_z_tolerance):
                self._advance_route()
        elif self.state == "TO_UPPER_STAIR_ENTRY":
            self._release_path_follower()
            self._publish_waypoint_goal(goal)
            if self._goal_reached(goal, self.args.entry_reach_radius, self.args.entry_z_tolerance):
                self._start_down_route()
        elif self.state in ("STAIR_EXEC_UP", "STAIR_EXEC_DOWN"):
            self._publish_stop()
            self._publish_pose_override_toward(goal)
            if self._goal_reached(goal, self.args.connector_reach_radius, self.args.connector_z_tolerance):
                self._write_debug_event(
                    "route_waypoint_reached",
                    route_state=self.state,
                    route_index=self.route_index + 1,
                    route_len=len(self.route),
                    goal=self._goal_to_dict(goal),
                )
                self.get_logger().info(f"Reached stair connector waypoint {self.route_index + 1}/{len(self.route)}: {goal.name}")
                self._advance_route()

    def _publish_floor2_start_request_if_needed(self) -> None:
        if not self.floor2_start_requested or self.floor2_raw_waypoint is not None:
            return
        self._write_debug_event("floor2_start_request_republished")
        msg = Bool()
        msg.data = True
        self.floor2_start_pub.publish(msg)

    def _maybe_log_debug_summary(self) -> None:
        now = self.get_clock().now()
        if (now - self.last_debug_summary_time).nanoseconds < int(self.args.debug_summary_period * 1e9):
            return
        self.last_debug_summary_time = now
        self._write_debug_event(
            "summary",
            floor1_input_counts=dict(self.floor1_input_counts),
            floor2_input_counts=dict(self.floor2_input_counts),
            floor1_waypoint=self._point_to_dict(self.floor1_raw_waypoint),
            floor2_waypoint=self._point_to_dict(self.floor2_raw_waypoint),
            stair_candidate_active=self._stair_candidate_active(),
            stair_discovered=self.stair_discovered,
            floor2_safe_zone_scanned=self.floor2_safe_zone_scanned,
            floor2_safe_zone_scan_count=self.floor2_safe_zone_scan_count,
        )

    def _publish_state(self) -> None:
        msg = String()
        discovery = "discovered" if self._stair_candidate_active() else "undiscovered"
        msg.data = (
            f"{self.state}:floor={self.current_floor}:{discovery}:"
            f"floor1_done={self.floor1_exploration_finished}:floor2_done={self.floor2_exploration_finished}"
        )
        self.state_pub.publish(msg)

    def _update_stair_discovery(self) -> None:
        if self.connector is None or self.args.known_stair_global or self.stair_discovered:
            return
        dist = self._distance_xy_to_goal(self.connector.lower_entry)
        if dist <= self.args.stair_discovery_radius:
            self.stair_discovered = True
            self._write_debug_event("stair_discovered", distance_to_entry=round(dist, 3))
            self.get_logger().info(
                f"Discovered stair connector {self.connector.name} from lidar-range proxy: "
                f"distance_to_entry={dist:.2f}m."
            )

    def _publish_visualization(self) -> None:
        if self.connector is None:
            return
        now = self.get_clock().now().to_msg()
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        header = Header(frame_id=self.args.frame_id, stamp=now)
        if not self.args.show_undiscovered_stair_debug and not self._stair_candidate_active():
            return
        intensity = 2.0 if self._stair_candidate_active() else 0.15
        cloud = point_cloud2.create_cloud(
            header,
            fields,
            [
                (self.connector.lower_entry.x, self.connector.lower_entry.y, self.connector.lower_entry.z, intensity),
                (self.connector.upper_entry.x, self.connector.upper_entry.y, self.connector.upper_entry.z, 1.5),
            ],
        )
        self.frontier_cloud_pub.publish(cloud)

        path = RosPath()
        path.header = header
        path_goals = [self.connector.lower_entry] + self.connector.up_route
        if self.initial_home is not None:
            path_goals.append(self.initial_home)
        for goal in path_goals:
            pose = PoseStamped()
            pose.header = header
            pose.pose.position.x = goal.x
            pose.pose.position.y = goal.y
            pose.pose.position.z = goal.z
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.connector_path_pub.publish(path)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Known-stair frontier arbiter for Gazebo multi-level exploration.")
    parser.add_argument("--metadata", required=True)
    parser.add_argument("--vehicle-height", type=float, default=0.75)
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--raw-waypoint-topic", default=None)
    parser.add_argument("--floor1-raw-waypoint-topic", default="/floor1/tare/way_point_raw")
    parser.add_argument("--floor2-raw-waypoint-topic", default="/floor2/tare/way_point_raw")
    parser.add_argument("--output-waypoint-topic", default="/way_point")
    parser.add_argument("--exploration-finish-topic", default=None)
    parser.add_argument("--floor1-exploration-finish-topic", default="/floor1/tare/exploration_finish_raw")
    parser.add_argument("--floor2-exploration-finish-topic", default="/floor2/tare/exploration_finish_raw")
    parser.add_argument("--odom-topic", default="/state_estimation")
    parser.add_argument("--state-estimation-at-scan-topic", default="/state_estimation_at_scan")
    parser.add_argument("--registered-scan-topic", default="/registered_scan")
    parser.add_argument("--terrain-map-topic", default="/terrain_map")
    parser.add_argument("--terrain-map-ext-topic", default="/terrain_map_ext")
    parser.add_argument("--floor1-state-estimation-at-scan-topic", default="/floor1/state_estimation_at_scan")
    parser.add_argument("--floor1-registered-scan-topic", default="/floor1/registered_scan")
    parser.add_argument("--floor1-terrain-map-topic", default="/floor1/terrain_map")
    parser.add_argument("--floor1-terrain-map-ext-topic", default="/floor1/terrain_map_ext")
    parser.add_argument("--floor2-state-estimation-at-scan-topic", default="/floor2/state_estimation_at_scan")
    parser.add_argument("--floor2-registered-scan-topic", default="/floor2/registered_scan")
    parser.add_argument("--floor2-terrain-map-topic", default="/floor2/terrain_map")
    parser.add_argument("--floor2-terrain-map-ext-topic", default="/floor2/terrain_map_ext")
    parser.add_argument("--pose-override-topic", default="/whitebox_vehicle_pose_override")
    parser.add_argument("--stop-topic", default="/stop")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--floor2-start-request-topic", default="/multilevel/start_floor2_tare")
    parser.add_argument("--stair-frontier-topic", default="/multilevel/stair_frontier_cloud")
    parser.add_argument("--connector-path-topic", default="/multilevel/stair_connector_path")
    parser.add_argument("--state-topic", default="/multilevel/stair_frontier_state")
    parser.add_argument(
        "--relay-cloud-topics",
        nargs="*",
        default=[
            "viewpoint_vis_cloud",
            "selected_viewpoint_vis_cloud",
            "keypose_graph_cloud",
            "exploration_path_cloud",
            "keypose_cloud",
        ],
    )
    parser.add_argument(
        "--relay-marker-topics",
        nargs="*",
        default=[
            "keypose_graph_edge_marker",
            "keypose_graph_node_marker",
        ],
    )
    parser.add_argument(
        "--relay-path-topics",
        nargs="*",
        default=[
            "global_path",
            "local_path",
            "exploration_path",
        ],
    )
    parser.add_argument("--publish-rate", type=float, default=10.0)
    parser.add_argument("--connector-speed", type=float, default=0.45)
    parser.add_argument("--known-stair-global", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--stair-discovery-radius", type=float, default=8.0)
    parser.add_argument("--show-undiscovered-stair-debug", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--stair-commit-radius", type=float, default=2.0)
    parser.add_argument("--stair-frontier-bonus", type=float, default=6.0)
    parser.add_argument("--connector-cost-weight", type=float, default=0.35)
    parser.add_argument("--force-stair-when-floor-complete", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--entry-reach-radius", type=float, default=0.6)
    parser.add_argument("--lower-entry-reach-radius", type=float, default=1.0)
    parser.add_argument("--floor2-direct-descent-radius", type=float, default=1.6)
    parser.add_argument("--entry-z-tolerance", type=float, default=0.45)
    parser.add_argument("--connector-reach-radius", type=float, default=0.25)
    parser.add_argument("--connector-z-tolerance", type=float, default=0.18)
    parser.add_argument("--floor-z-tolerance", type=float, default=0.45)
    parser.add_argument("--floor2-safe-scan-radius", type=float, default=2.5)
    parser.add_argument("--floor2-safe-scan-min-scans", type=int, default=8)
    parser.add_argument("--floor2-safe-finish-radius", type=float, default=1.6)
    parser.add_argument("--debug-log-dir", default="~/.ros/multilevel_debug")
    parser.add_argument("--debug-summary-period", type=float, default=5.0)
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    rclpy.init()
    node = MultilevelStairFrontierArbiter(args)
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
