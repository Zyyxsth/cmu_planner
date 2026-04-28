#!/usr/bin/env python3
"""
Route multi-floor whitebox goals through known stair connectors.

This is a simulation-only 2.5D connector prototype. It keeps FAR Planner in
charge of normal floor travel by consuming user /goal_point commands and
publishing routed floor goals to FAR on a separate topic. Only the known stair
connector segment is executed through /whitebox_vehicle_pose_override, which is
the placeholder for the future stair/RL low-level controller.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int8


@dataclass(frozen=True)
class RoutedGoal:
    name: str
    x: float
    y: float
    z: float
    odom_z: bool = False


STATE_IDLE = "IDLE"
STATE_PASSTHROUGH = "PASSTHROUGH"
STATE_TO_ENTRY = "TO_ENTRY"
STATE_STAIR_EXEC = "STAIR_EXEC"
STATE_TO_FINAL = "TO_FINAL"
STATE_POST_STAIR_EXEC_FALLBACK = "POST_STAIR_EXEC_FALLBACK"


def center(obj: dict[str, object]) -> tuple[float, float]:
    return (
        (float(obj["x_min"]) + float(obj["x_max"])) / 2.0,
        (float(obj["y_min"]) + float(obj["y_max"])) / 2.0,
    )


class WhiteboxStairRoute:
    def __init__(self, metadata_path: Path, vehicle_height: float) -> None:
        metadata = json.loads(metadata_path.read_text(encoding="ascii"))
        objects = {str(obj["name"]): obj for obj in metadata["objects"]}
        self.vehicle_height = vehicle_height
        self.floor2 = objects.get("floor_2_south_landing")
        self.stair_connector = self._select_stair_connector(metadata.get("connectors", []))
        self.lower_steps = self._sorted_steps(objects, "stair_realistic_lower")
        self.upper_steps = self._sorted_steps(objects, "stair_realistic_upper")
        self.mid_landing = objects.get("mid_landing_realistic")

    @staticmethod
    def _select_stair_connector(connectors: object) -> Optional[dict[str, object]]:
        if not isinstance(connectors, list):
            return None
        for connector in connectors:
            if isinstance(connector, dict) and connector.get("connector_kind") == "stair":
                return connector
        return None

    @staticmethod
    def _sorted_steps(objects: dict[str, dict[str, object]], group_name: str) -> list[dict[str, object]]:
        steps = [
            obj
            for obj in objects.values()
            if obj.get("terrain_kind") == "stair_step" and obj.get("group_name") == group_name
        ]
        return sorted(steps, key=lambda obj: int(obj.get("step_index", 0)))

    def has_realistic_stair(self) -> bool:
        return bool(self.floor2 and (self.stair_connector or (self.mid_landing and self.lower_steps and self.upper_steps)))

    def is_second_floor_goal(self, goal: PointStamped) -> bool:
        goal_surface_z = self.surface_goal_z(float(goal.point.z))
        if self.floor2 is None:
            return goal_surface_z >= 2.0
        return goal_surface_z >= float(self.floor2["z_max"]) - 0.35

    def surface_goal_z(self, raw_z: float) -> float:
        # RViz floor2 mode publishes odom/sensor height; probe scripts use surface z.
        if self.floor2 is None:
            return raw_z
        floor2_z = float(self.floor2["z_max"])
        if raw_z > floor2_z + 0.35:
            return raw_z - self.vehicle_height
        return raw_z

    def stair_preentry_goal(self) -> RoutedGoal:
        goal = self._connector_goal("lower_entry_goal")
        if goal is not None:
            return goal
        return RoutedGoal("stair_preentry", -4.0, -1.8, 0.0)

    def floor2_entry_goal(self) -> RoutedGoal:
        goal = self._connector_goal("upper_entry_goal")
        if goal is not None:
            return goal
        if self.floor2 is None or not self.upper_steps:
            return RoutedGoal("floor2_entry", 0.0, 0.0, 0.0)

        top_x, top_y = center(self.upper_steps[-1])
        floor_x_min = float(self.floor2["x_min"])
        floor_x_max = float(self.floor2["x_max"])
        floor_y_min = float(self.floor2["y_min"])
        floor_y_max = float(self.floor2["y_max"])
        floor_z = float(self.floor2["z_max"])

        start_x = max(floor_x_min + 0.6, min(floor_x_max - 0.6, top_x))
        start_y = max(floor_y_min + 0.45, min(floor_y_max - 0.6, top_y + 0.55))
        return RoutedGoal("floor2_entry", start_x, start_y, floor_z)

    def stair_connector_up_route(self) -> list[RoutedGoal]:
        route = self._connector_route("up_route")
        if route:
            return route

        route: list[RoutedGoal] = []
        route.extend(self._step_goals(self.lower_steps))

        assert self.mid_landing is not None
        x, y = center(self.mid_landing)
        route.append(RoutedGoal("mid_landing_realistic", x, y, float(self.mid_landing["z_max"])))

        route.extend(self._step_goals(self.upper_steps))
        route.append(self.floor2_entry_goal())
        return route

    def stair_connector_down_route(self) -> list[RoutedGoal]:
        route = self._connector_route("down_route")
        if route:
            return route

        route: list[RoutedGoal] = []
        route.extend(reversed(self._step_goals(self.upper_steps)))

        assert self.mid_landing is not None
        x, y = center(self.mid_landing)
        route.append(RoutedGoal("mid_landing_realistic", x, y, float(self.mid_landing["z_max"])))

        route.extend(reversed(self._step_goals(self.lower_steps)))
        route.append(self.stair_preentry_goal())
        return route

    def build_floor2_far_route(self, final_goal: RoutedGoal, spacing: float) -> list[RoutedGoal]:
        if self.floor2 is None or not self.upper_steps:
            return [final_goal]

        floor_x_min = float(self.floor2["x_min"])
        floor_x_max = float(self.floor2["x_max"])
        floor_y_min = float(self.floor2["y_min"])
        floor_y_max = float(self.floor2["y_max"])
        floor_z = float(self.floor2["z_max"])

        entry_goal = self.floor2_entry_goal()
        start_x = entry_goal.x
        start_y = entry_goal.y
        end_x = max(floor_x_min + 0.6, min(floor_x_max - 0.6, final_goal.x))
        end_y = max(floor_y_min + 0.45, min(floor_y_max - 0.6, final_goal.y))

        goals = [entry_goal]
        dx = end_x - start_x
        dy = end_y - start_y
        dist = math.hypot(dx, dy)
        if dist > 1e-3:
            steps = max(1, int(math.ceil(dist / max(0.5, spacing))))
            for idx in range(1, steps + 1):
                ratio = idx / steps
                name = "final_goal" if idx == steps else f"floor2_waypoint_{idx:02d}"
                goals.append(RoutedGoal(name, start_x + dx * ratio, start_y + dy * ratio, floor_z))

        if abs(final_goal.x - end_x) > 1e-3 or abs(final_goal.y - end_y) > 1e-3:
            goals.append(RoutedGoal("final_goal", final_goal.x, final_goal.y, final_goal.z))
        return goals

    @staticmethod
    def _step_goals(steps: list[dict[str, object]]) -> list[RoutedGoal]:
        # Two 0.15m steps keep the synthetic vehicle height transition within
        # the terrain-map continuity gate while avoiding over-dense subgoals.
        selected: list[dict[str, object]] = []
        for idx, step in enumerate(steps):
            if idx % 2 == 1 or idx == len(steps) - 1:
                selected.append(step)
        goals = []
        for step in selected:
            x, y = center(step)
            goals.append(RoutedGoal(str(step["name"]), x, y, float(step["z_max"])))
        return goals

    def _connector_goal(self, key: str) -> Optional[RoutedGoal]:
        if self.stair_connector is None:
            return None
        raw_goal = self.stair_connector.get(key)
        if not isinstance(raw_goal, dict):
            return None
        return self._goal_from_dict(raw_goal)

    def _connector_route(self, key: str) -> list[RoutedGoal]:
        if self.stair_connector is None:
            return []
        raw_route = self.stair_connector.get(key)
        if not isinstance(raw_route, list):
            return []
        route = []
        for raw_goal in raw_route:
            if isinstance(raw_goal, dict):
                route.append(self._goal_from_dict(raw_goal))
        return route

    @staticmethod
    def _goal_from_dict(raw_goal: dict[str, object]) -> RoutedGoal:
        return RoutedGoal(
            str(raw_goal["name"]),
            float(raw_goal["x"]),
            float(raw_goal["y"]),
            float(raw_goal["z"]),
        )


class WhiteboxStairGoalRouter(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("whitebox_stair_goal_router")
        self.args = args
        self.route = WhiteboxStairRoute(Path(args.metadata), args.vehicle_height)
        self.last_odom: Optional[Odometry] = None
        self.active_route: list[RoutedGoal] = []
        self.active_index = 0
        self.state = STATE_IDLE
        self.pending_final_goal: Optional[RoutedGoal] = None
        self.post_stair_far_route: list[RoutedGoal] = []
        self.stair_connector_route: list[RoutedGoal] = []
        self.raw_goal_key: Optional[tuple[int, int, int]] = None
        self.last_publish_time = self.get_clock().now()
        self.segment_start_time = self.get_clock().now()
        self.publisher = self.create_publisher(PointStamped, args.output_topic, 5)
        self.pose_override_pub = self.create_publisher(PoseStamped, args.pose_override_topic, 5)
        self.stop_pub = self.create_publisher(Int8, args.stop_topic, 5)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, args.cmd_vel_topic, 5)
        self.create_subscription(PointStamped, args.input_topic, self._goal_callback, 10)
        self.create_subscription(Odometry, args.odom_topic, self._odom_callback, 10)
        self.create_timer(1.0 / args.publish_rate, self._tick)
        if not self.route.has_realistic_stair():
            self.get_logger().warn("Realistic stair metadata not found; router will pass goals through.")

    def _odom_callback(self, msg: Odometry) -> None:
        self.last_odom = msg

    def _goal_callback(self, msg: PointStamped) -> None:
        goal_key = (
            round(float(msg.point.x) * 100),
            round(float(msg.point.y) * 100),
            round(float(msg.point.z) * 100),
        )
        if goal_key == self.raw_goal_key:
            return
        self.raw_goal_key = goal_key

        target_floor = 2 if self.route.is_second_floor_goal(msg) else 1
        current_floor = self._current_floor()

        if current_floor is None:
            if target_floor == 2 and self.route.has_realistic_stair():
                self.get_logger().warn("Current floor is unknown; assuming floor 1 for floor2 stair routing.")
                self._start_upstairs_route(msg)
                return
            self._start_passthrough(self._goal_from_msg(f"floor{target_floor}_passthrough", msg))
            self.get_logger().info("Current floor is unknown; passing goal directly to FAR.")
            return

        if current_floor == target_floor:
            self._start_passthrough(self._goal_from_msg(f"floor{target_floor}_passthrough", msg))
            self.get_logger().info(f"Robot is already on floor {current_floor}; passing goal directly to FAR.")
            return

        if current_floor == 1 and target_floor == 2:
            self._start_upstairs_route(msg)
            return

        if current_floor == 2 and target_floor == 1:
            self._start_downstairs_route(msg)
            return

        self._start_passthrough(self._goal_from_msg("passthrough", msg))

    def _start_upstairs_route(self, msg: PointStamped) -> None:
        if not self.route.has_realistic_stair():
            self._start_passthrough(self._goal_from_msg("floor2_passthrough", msg))
            return

        self.pending_final_goal = RoutedGoal(
            "final_goal",
            float(msg.point.x),
            float(msg.point.y),
            self.route.surface_goal_z(float(msg.point.z)),
        )
        self.post_stair_far_route = self.route.build_floor2_far_route(
            self.pending_final_goal, self.args.floor2_far_spacing
        )
        self.stair_connector_route = self.route.stair_connector_up_route()
        if self.post_stair_far_route and self.post_stair_far_route[0].name == "floor2_entry":
            # The final stair-to-platform transition belongs to the stair
            # controller handoff. FAR resumes after this point.
            self.post_stair_far_route.pop(0)
        self._start_segment(STATE_TO_ENTRY, [self.route.stair_preentry_goal()], force_publish=True)
        self.get_logger().info(
            "Routing floor1->floor2 goal: FAR to stair_preentry, stair executor for "
            f"{len(self.stair_connector_route)} connector goals, FAR through "
            f"{len(self.post_stair_far_route)} upstairs goals."
        )

    def _start_downstairs_route(self, msg: PointStamped) -> None:
        if not self.route.has_realistic_stair():
            self._start_passthrough(self._goal_from_msg("floor1_passthrough", msg))
            return

        self.pending_final_goal = self._goal_from_msg("final_goal", msg)
        self.post_stair_far_route = [self.pending_final_goal]
        self.stair_connector_route = self.route.stair_connector_down_route()
        self._start_segment(STATE_TO_ENTRY, [self.route.floor2_entry_goal()], force_publish=True)
        self.get_logger().info(
            "Routing floor2->floor1 goal: FAR to floor2_entry, stair executor for "
            f"{len(self.stair_connector_route)} connector goals, FAR to final floor1 goal."
        )

    def _goal_from_msg(self, name: str, msg: PointStamped) -> RoutedGoal:
        raw_z = float(msg.point.z)
        surface_z = self.route.surface_goal_z(raw_z)
        # RViz floor modes publish odom/sensor height; probe scripts can publish
        # surface height for second-floor probes. Preserve the source encoding.
        uses_odom_z = True
        if self.route.is_second_floor_goal(msg):
            uses_odom_z = abs(raw_z - surface_z) > 1e-3
        return RoutedGoal(name, float(msg.point.x), float(msg.point.y), raw_z if uses_odom_z else surface_z, uses_odom_z)

    def _current_floor(self) -> Optional[int]:
        if self._robot_is_on_second_floor():
            return 2
        if self.last_odom is not None:
            return 1
        return None

    def _robot_is_on_second_floor(self) -> bool:
        if self.last_odom is None or self.route.floor2 is None:
            return False
        odom_z = float(self.last_odom.pose.pose.position.z)
        floor2_odom_z = float(self.route.floor2["z_max"]) + self.args.vehicle_height
        return odom_z >= floor2_odom_z - self.args.current_floor_z_tolerance

    def _start_passthrough(self, goal: RoutedGoal) -> None:
        self.pending_final_goal = None
        self.post_stair_far_route = []
        self.stair_connector_route = []
        self._start_segment(STATE_PASSTHROUGH, [goal], force_publish=True)
        self.get_logger().info(f"Passing goal through: ({goal.x:.2f}, {goal.y:.2f}, {goal.z:.2f}).")

    def _start_segment(self, state: str, route: list[RoutedGoal], force_publish: bool = False) -> None:
        self.state = state
        self.active_route = route
        self.active_index = 0
        self.segment_start_time = self.get_clock().now()
        if route:
            self.get_logger().info(f"Starting segment {state}: first goal {route[0].name}.")
        if self._far_controls_active_segment():
            self._publish_path_follower_stop(False)
            self._publish_active_goal(force=force_publish)
        elif self._using_stair_executor():
            self._publish_path_follower_stop(True)

    def _goal_reached(self, goal: RoutedGoal) -> bool:
        if self.last_odom is None:
            return False
        pos = self.last_odom.pose.pose.position
        xy_error = math.hypot(float(pos.x) - goal.x, float(pos.y) - goal.y)
        z_error = abs(float(pos.z) - self._goal_odom_z(goal))
        is_final_goal = self.state in (STATE_PASSTHROUGH, STATE_TO_FINAL)
        reach_radius = self.args.final_reach_radius if is_final_goal else self.args.connector_reach_radius
        z_tolerance = self.args.final_z_tolerance if is_final_goal else self.args.connector_z_tolerance
        if self.state == STATE_STAIR_EXEC and self.active_index == len(self.active_route) - 1:
            reach_radius = min(reach_radius, self.args.stair_exit_reach_radius)
            z_tolerance = min(z_tolerance, self.args.stair_exit_z_tolerance)
        return xy_error <= reach_radius and z_error <= z_tolerance

    def _goal_odom_z(self, goal: RoutedGoal) -> float:
        if goal.odom_z:
            return goal.z
        return goal.z + self.args.vehicle_height

    def _publish_active_goal(self, force: bool = False) -> None:
        if not self.active_route or self.active_index >= len(self.active_route):
            return
        now = self.get_clock().now()
        if not force:
            elapsed = (now - self.last_publish_time).nanoseconds * 1e-9
            if elapsed < 1.0 / self.args.goal_publish_rate:
                return
        self.last_publish_time = now

        goal = self.active_route[self.active_index]
        msg = PointStamped()
        msg.header.frame_id = self.args.frame_id
        msg.header.stamp = now.to_msg()
        msg.point.x = goal.x
        msg.point.y = goal.y
        msg.point.z = self._goal_odom_z(goal)
        self.publisher.publish(msg)

    def _far_controls_active_segment(self) -> bool:
        return self.state in (STATE_PASSTHROUGH, STATE_TO_ENTRY, STATE_TO_FINAL)

    def _using_stair_executor(self) -> bool:
        return self.state in (STATE_STAIR_EXEC, STATE_POST_STAIR_EXEC_FALLBACK)

    def _publish_path_follower_stop(self, stop: bool) -> None:
        msg = Int8()
        msg.data = 2 if stop else 0
        self.stop_pub.publish(msg)
        if stop:
            self._publish_zero_cmd_vel()

    def _publish_zero_cmd_vel(self) -> None:
        msg = TwistStamped()
        msg.header.frame_id = self.args.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_vel_pub.publish(msg)

    def _publish_connector_pose_override(self, goal: RoutedGoal) -> None:
        if self.last_odom is None:
            return
        pos = self.last_odom.pose.pose.position
        dx = goal.x - float(pos.x)
        dy = goal.y - float(pos.y)
        target_z = self._goal_odom_z(goal)
        dz = target_z - float(pos.z)
        xy_dist = math.hypot(dx, dy)
        max_step = self.args.connector_speed / self.args.publish_rate
        if xy_dist > max_step:
            ratio = max_step / xy_dist
            next_x = float(pos.x) + dx * ratio
            next_y = float(pos.y) + dy * ratio
            next_z = float(pos.z) + dz * ratio
            yaw = math.atan2(dy, dx)
        else:
            next_x = goal.x
            next_y = goal.y
            if abs(dz) > max_step:
                next_z = float(pos.z) + math.copysign(max_step, dz)
            else:
                next_z = target_z
            yaw = math.atan2(dy, dx) if xy_dist > 1e-3 else yaw_from_odom(self.last_odom)

        msg = PoseStamped()
        msg.header.frame_id = self.args.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = next_x
        msg.pose.position.y = next_y
        msg.pose.position.z = next_z
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_override_pub.publish(msg)

    def _tick(self) -> None:
        if not self.active_route or self.active_index >= len(self.active_route):
            return
        goal = self.active_route[self.active_index]
        self._maybe_fallback_final_segment(goal)
        if self._using_stair_executor():
            self._publish_path_follower_stop(True)
            self._publish_connector_pose_override(goal)
        if self._goal_reached(goal):
            self.get_logger().info(f"Reached {self.state} goal {self.active_index + 1}/{len(self.active_route)}: {goal.name}")
            self.active_index += 1
            if self.active_index >= len(self.active_route):
                self._advance_segment()
                return
            self.segment_start_time = self.get_clock().now()
            if self._far_controls_active_segment():
                self._publish_active_goal(force=True)
            return
        if self._far_controls_active_segment():
            self._publish_active_goal()

    def _advance_segment(self) -> None:
        if self.state == STATE_TO_ENTRY:
            if self.stair_connector_route:
                self._start_segment(STATE_STAIR_EXEC, self.stair_connector_route)
                return
            if self.post_stair_far_route:
                self._start_segment(STATE_TO_FINAL, self.post_stair_far_route, force_publish=True)
                return
        elif self.state == STATE_STAIR_EXEC:
            if self.post_stair_far_route:
                self._start_segment(STATE_TO_FINAL, self.post_stair_far_route, force_publish=True)
                return
        elif self.state == STATE_POST_STAIR_EXEC_FALLBACK:
            self.get_logger().info("Completed post-stair fallback executor segment.")
        elif self.state in (STATE_TO_FINAL, STATE_PASSTHROUGH):
            self.get_logger().info(f"Completed route segment sequence in state {self.state}.")

        self.state = STATE_IDLE
        self.active_route = []
        self.active_index = 0
        self.pending_final_goal = None
        self.post_stair_far_route = []
        self.stair_connector_route = []
        self._publish_path_follower_stop(True)

    def _maybe_fallback_final_segment(self, goal: RoutedGoal) -> None:
        if self.state != STATE_TO_FINAL or self.args.final_fallback_timeout <= 0.0:
            return
        if self._goal_reached(goal):
            return
        elapsed = (self.get_clock().now() - self.segment_start_time).nanoseconds * 1e-9
        if elapsed < self.args.final_fallback_timeout:
            return
        self.get_logger().warn(
            "FAR has not connected the current post-stair subgoal within "
            f"{self.args.final_fallback_timeout:.1f}s; switching the remaining post-stair route "
            "to temporary POST_STAIR_EXEC_FALLBACK. "
            "This keeps the whitebox demo moving but marks the FAR planning gap explicitly."
        )
        self._start_segment(STATE_POST_STAIR_EXEC_FALLBACK, self.active_route[self.active_index:])


def yaw_from_odom(odom: Odometry) -> float:
    q = odom.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Route whitebox cross-floor goals through stair connectors.")
    parser.add_argument("--metadata", required=True)
    parser.add_argument("--input-topic", default="/goal_point")
    parser.add_argument("--output-topic", default="/routed_goal_point")
    parser.add_argument("--pose-override-topic", default="/whitebox_vehicle_pose_override")
    parser.add_argument("--stop-topic", default="/stop")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--odom-topic", default="/state_estimation")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--vehicle-height", type=float, default=0.75)
    parser.add_argument("--connector-speed", type=float, default=0.55)
    parser.add_argument("--connector-reach-radius", type=float, default=0.85)
    parser.add_argument("--connector-z-tolerance", type=float, default=0.55)
    parser.add_argument("--stair-exit-reach-radius", type=float, default=0.20)
    parser.add_argument("--stair-exit-z-tolerance", type=float, default=0.12)
    parser.add_argument("--floor2-far-spacing", type=float, default=1.25)
    parser.add_argument("--current-floor-z-tolerance", type=float, default=0.25)
    parser.add_argument("--final-reach-radius", type=float, default=0.60)
    parser.add_argument("--final-z-tolerance", type=float, default=0.35)
    parser.add_argument("--final-fallback-timeout", type=float, default=8.0)
    parser.add_argument("--publish-rate", type=float, default=20.0)
    parser.add_argument("--goal-publish-rate", type=float, default=2.0)
    return parser.parse_args()


def main() -> None:
    rclpy.init()
    node = WhiteboxStairGoalRouter(parse_args())
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
