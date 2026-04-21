#!/usr/bin/env python3
"""
Record robot odometry while teleoperating and fit a 4-corner boundary.

Typical workflow:
  1. Start the script.
  2. Teleoperate the robot around the desired exploration area.
  3. Press Ctrl+C to stop recording.
  4. The script saves:
     - a raw XY trajectory CSV
     - a rectangular boundary PLY compatible with navigationBoundary

The rectangle is computed as the minimum-area oriented bounding box of the
recorded odometry path, then expanded by an optional padding margin.
"""

import argparse
import csv
import math
import os
import signal
import sys
from typing import Iterable, List, Sequence, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

Point2D = Tuple[float, float]


def distance_2d(p1: Point2D, p2: Point2D) -> float:
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def cross(o: Point2D, a: Point2D, b: Point2D) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def convex_hull(points: Sequence[Point2D]) -> List[Point2D]:
    pts = sorted(set(points))
    if len(pts) <= 1:
        return list(pts)

    lower: List[Point2D] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    upper: List[Point2D] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    return lower[:-1] + upper[:-1]


def rotate_point(point: Point2D, angle: float) -> Point2D:
    c = math.cos(angle)
    s = math.sin(angle)
    return (point[0] * c - point[1] * s, point[0] * s + point[1] * c)


def unrotate_point(point: Point2D, angle: float) -> Point2D:
    return rotate_point(point, -angle)


def expand_rectangle(corners: Sequence[Point2D], padding: float) -> List[Point2D]:
    if padding <= 0.0:
        return list(corners)

    p0, p1, p2, p3 = corners
    axis_x = (p1[0] - p0[0], p1[1] - p0[1])
    axis_y = (p3[0] - p0[0], p3[1] - p0[1])
    len_x = math.hypot(axis_x[0], axis_x[1])
    len_y = math.hypot(axis_y[0], axis_y[1])
    if len_x < 1e-6 or len_y < 1e-6:
        return list(corners)

    ux = (axis_x[0] / len_x, axis_x[1] / len_x)
    uy = (axis_y[0] / len_y, axis_y[1] / len_y)

    return [
        (p0[0] - padding * ux[0] - padding * uy[0], p0[1] - padding * ux[1] - padding * uy[1]),
        (p1[0] + padding * ux[0] - padding * uy[0], p1[1] + padding * ux[1] - padding * uy[1]),
        (p2[0] + padding * ux[0] + padding * uy[0], p2[1] + padding * ux[1] + padding * uy[1]),
        (p3[0] - padding * ux[0] + padding * uy[0], p3[1] - padding * ux[1] + padding * uy[1]),
    ]


def minimum_area_rectangle(points: Sequence[Point2D], padding: float) -> List[Point2D]:
    if len(points) < 3:
        raise ValueError("At least 3 unique points are required to fit a rectangle.")

    hull = convex_hull(points)
    if len(hull) < 3:
        raise ValueError("Convex hull is degenerate. Please drive a full loop with turns.")

    best_area = float("inf")
    best_rect: List[Point2D] = []

    for i in range(len(hull)):
        p1 = hull[i]
        p2 = hull[(i + 1) % len(hull)]
        edge_angle = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        rotated = [rotate_point(p, -edge_angle) for p in hull]
        xs = [p[0] for p in rotated]
        ys = [p[1] for p in rotated]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        area = (max_x - min_x) * (max_y - min_y)
        if area < best_area:
            best_area = area
            rect_rot = [
                (min_x, min_y),
                (max_x, min_y),
                (max_x, max_y),
                (min_x, max_y),
            ]
            best_rect = [unrotate_point(p, -edge_angle) for p in rect_rot]

    return expand_rectangle(best_rect, padding)


def polygon_area(points: Sequence[Point2D]) -> float:
    area = 0.0
    for i in range(len(points)):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % len(points)]
        area += x1 * y2 - x2 * y1
    return abs(area) * 0.5


def save_boundary_ply(points: Sequence[Point2D], output_file: str) -> None:
    closed_points = list(points)
    if closed_points[0] != closed_points[-1]:
        closed_points.append(closed_points[0])

    with open(output_file, "w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(closed_points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for x, y in closed_points:
            f.write(f"{x:.3f}\t{y:.3f}\t0.0\n")


def save_csv(points_xyz: Iterable[Tuple[float, float, float]], output_file: str) -> None:
    with open(output_file, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "z"])
        writer.writerows(points_xyz)


class BoundaryRecorder(Node):
    def __init__(self, topic: str, min_sample_distance: float):
        super().__init__("boundary_odom_recorder")
        self.min_sample_distance = min_sample_distance
        self.raw_points_xyz: List[Tuple[float, float, float]] = []
        self.xy_points: List[Point2D] = []
        self._last_xy: Point2D | None = None
        self.create_subscription(Odometry, topic, self.odom_callback, 50)
        self.get_logger().info(
            f"Recording boundary from odom topic: {topic} "
            f"(min_sample_distance={min_sample_distance:.2f} m)"
        )

    def odom_callback(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        z = float(msg.pose.pose.position.z)
        cur = (x, y)

        if self._last_xy is None or distance_2d(cur, self._last_xy) >= self.min_sample_distance:
            self.xy_points.append(cur)
            self.raw_points_xyz.append((x, y, z))
            self._last_xy = cur
            self.get_logger().info(
                f"sample #{len(self.xy_points):03d}: x={x:.3f}, y={y:.3f}, z={z:.3f}"
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record odom while teleoperating and generate a rectangular exploration boundary."
    )
    parser.add_argument(
        "--topic",
        default="/odin1/odometry",
        help="Odometry topic to record. Default: /odin1/odometry",
    )
    parser.add_argument(
        "--output-ply",
        default="src/exploration_planner/tare_planner/data/boundary_from_odom.ply",
        help="Output boundary PLY path.",
    )
    parser.add_argument(
        "--output-csv",
        default="src/exploration_planner/tare_planner/data/boundary_from_odom.csv",
        help="Output raw recorded odom CSV path.",
    )
    parser.add_argument(
        "--min-sample-distance",
        type=float,
        default=0.10,
        help="Only keep odom samples that move at least this far in XY. Default: 0.10 m",
    )
    parser.add_argument(
        "--padding",
        type=float,
        default=0.20,
        help="Expand the fitted rectangle outward by this margin in meters. Default: 0.20 m",
    )
    return parser.parse_args()


def ensure_parent_dir(path: str) -> None:
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)


def main() -> int:
    args = parse_args()
    rclpy.init()
    recorder = BoundaryRecorder(args.topic, args.min_sample_distance)

    stop_requested = False

    def _handle_stop(signum, frame) -> None:
        nonlocal stop_requested
        stop_requested = True
        recorder.get_logger().info("Stop requested, finishing boundary fitting...")

    signal.signal(signal.SIGINT, _handle_stop)
    signal.signal(signal.SIGTERM, _handle_stop)

    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(recorder, timeout_sec=0.2)
    finally:
        ensure_parent_dir(args.output_ply)
        ensure_parent_dir(args.output_csv)

        if len(recorder.xy_points) < 4:
            recorder.get_logger().error(
                f"Only recorded {len(recorder.xy_points)} valid points. "
                "Please drive a larger closed loop."
            )
            recorder.destroy_node()
            rclpy.shutdown()
            return 1

        try:
            corners = minimum_area_rectangle(recorder.xy_points, args.padding)
        except ValueError as exc:
            recorder.get_logger().error(str(exc))
            recorder.destroy_node()
            rclpy.shutdown()
            return 1

        save_csv(recorder.raw_points_xyz, args.output_csv)
        save_boundary_ply(corners, args.output_ply)

        print("\nBoundary generation finished.")
        print(f"Raw odom CSV: {os.path.abspath(args.output_csv)}")
        print(f"Boundary PLY: {os.path.abspath(args.output_ply)}")
        print("Fitted rectangle corners (map frame):")
        for idx, (x, y) in enumerate(corners, start=1):
            print(f"  corner{idx}: x={x:.3f}, y={y:.3f}")
        print(f"Boundary area: {polygon_area(corners):.2f} m^2")
        print("\nTo use this boundary for exploration, point the exploration launch to this PLY file.")

        recorder.destroy_node()
        rclpy.shutdown()
        return 0


if __name__ == "__main__":
    sys.exit(main())
