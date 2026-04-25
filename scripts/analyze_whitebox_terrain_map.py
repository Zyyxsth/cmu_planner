#!/usr/bin/env python3
"""
Analyze terrain-map height/intensity statistics over known whitebox regions.

Check generated global geometry without ROS:

  python3 scripts/analyze_whitebox_terrain_map.py \
      --ply src/base_autonomy/vehicle_simulator/mesh/whitebox_stair_test/map.ply

Run while the Gazebo whitebox simulation is publishing /terrain_map:

  python3 scripts/analyze_whitebox_terrain_map.py --topic /terrain_map

The script prints per-region point counts and z/intensity ranges so we can
verify the current 2.5D terrain representation before changing planner logic.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from statistics import mean
from typing import Optional


@dataclass(frozen=True)
class Region:
    name: str
    kind: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    traversal_axis: Optional[tuple[float, float]] = None
    entry_side: Optional[str] = None
    group_name: Optional[str] = None


@dataclass(frozen=True)
class TerrainClassification:
    label: str
    reason: str


@dataclass(frozen=True)
class DirectionAssessment:
    axis: str
    entry_side: str
    alignment: str
    approach_class: str
    confidence: str


Point = tuple[float, float, float, Optional[float]]
STAIR_FRONT_ALIGNMENT_THRESHOLD = 0.80
STAIR_SIDE_ALIGNMENT_THRESHOLD = 0.35


def normalize_axis(axis: object) -> tuple[float, float] | None:
    if not isinstance(axis, list) and not isinstance(axis, tuple):
        return None
    if len(axis) != 2:
        return None
    axis_x = float(axis[0])
    axis_y = float(axis[1])
    norm = math.hypot(axis_x, axis_y)
    if norm <= 1e-6:
        return None
    return axis_x / norm, axis_y / norm


def format_axis(axis: tuple[float, float] | None) -> str:
    if axis is None:
        return "n/a"
    return f"({axis[0]:.3f};{axis[1]:.3f})"


def assess_direction(region: Region, heading_yaw: float | None = None) -> DirectionAssessment:
    axis = region.traversal_axis
    if axis is None:
        return DirectionAssessment("n/a", region.entry_side or "n/a", "n/a", "no_direction_metadata", "0.00")

    if heading_yaw is None:
        return DirectionAssessment(format_axis(axis), region.entry_side or "n/a", "n/a", "direction_metadata_only", "1.00")

    heading = (math.cos(heading_yaw), math.sin(heading_yaw))
    alignment = heading[0] * axis[0] + heading[1] * axis[1]
    if alignment >= STAIR_FRONT_ALIGNMENT_THRESHOLD:
        approach_class = "front_ascent_aligned"
    elif alignment <= -STAIR_FRONT_ALIGNMENT_THRESHOLD:
        approach_class = "reverse_or_descent_aligned"
    elif abs(alignment) <= STAIR_SIDE_ALIGNMENT_THRESHOLD:
        approach_class = "side_view_blocking"
    else:
        approach_class = "oblique_uncertain"
    return DirectionAssessment(
        format_axis(axis),
        region.entry_side or "n/a",
        f"{alignment:.3f}",
        approach_class,
        f"{abs(alignment):.2f}",
    )


def stair_traversal_decision(region: Region, classification: TerrainClassification, direction: DirectionAssessment) -> str:
    is_stair_region = region.kind in {"stair", "staircase"} or classification.label == "stair_like"
    if not is_stair_region:
        return "not_stair"
    if direction.approach_class == "front_ascent_aligned":
        return "front_entry_candidate"
    if direction.approach_class == "side_view_blocking":
        return "side_blocked"
    if direction.approach_class == "reverse_or_descent_aligned":
        return "reverse_or_descent_unsupported"
    if direction.approach_class == "oblique_uncertain":
        return "oblique_uncertain"
    return "direction_unknown"


def load_regions(metadata_path: Path, margin: float, include_stair_groups: bool = True) -> list[Region]:
    metadata = json.loads(metadata_path.read_text(encoding="ascii"))
    regions: list[Region] = []
    stair_groups: dict[str, list[dict[str, object]]] = {}
    for obj in metadata["objects"]:
        name = obj["name"]
        terrain_kind = str(obj.get("terrain_kind", ""))
        if terrain_kind == "floor" or name == "floor":
            continue
        if terrain_kind == "floor_2":
            kind = "floor_2"
        elif terrain_kind == "mid_landing":
            kind = "landing"
        elif terrain_kind == "ramp" or name.startswith("ramp_"):
            kind = "ramp"
        elif terrain_kind == "single_step" or name.startswith("single_step_"):
            kind = "single_step"
        elif terrain_kind == "stair_step" or name.startswith("stair_"):
            kind = "stair"
            prefix, suffix = name.rsplit("_", 1)
            if suffix.isdigit():
                stair_groups.setdefault(prefix, []).append(obj)
        else:
            kind = "other"
        regions.append(
            Region(
                name=name,
                kind=kind,
                x_min=float(obj["x_min"]) - margin,
                x_max=float(obj["x_max"]) + margin,
                y_min=float(obj["y_min"]) - margin,
                y_max=float(obj["y_max"]) + margin,
                traversal_axis=normalize_axis(obj.get("traversal_axis")),
                entry_side=obj.get("entry_side"),
                group_name=obj.get("group_name"),
            )
        )
    if include_stair_groups:
        for prefix, objects in sorted(stair_groups.items()):
            if len(objects) < 2:
                continue
            regions.append(
                Region(
                    name=f"{prefix}_all",
                    kind="staircase",
                    x_min=min(float(obj["x_min"]) for obj in objects) - margin,
                    x_max=max(float(obj["x_max"]) for obj in objects) + margin,
                    y_min=min(float(obj["y_min"]) for obj in objects) - margin,
                    y_max=max(float(obj["y_max"]) for obj in objects) + margin,
                    traversal_axis=normalize_axis(objects[0].get("traversal_axis")),
                    entry_side=objects[0].get("entry_side"),
                    group_name=prefix,
                )
            )
    return regions


def percentile(values: list[float], ratio: float) -> float:
    if not values:
        return 0.0
    values_sorted = sorted(values)
    idx = int(round((len(values_sorted) - 1) * ratio))
    return values_sorted[max(0, min(idx, len(values_sorted) - 1))]


def summarize(values: list[float]) -> str:
    if not values:
        return "n/a"
    return (
        f"min={min(values):.3f} p50={percentile(values, 0.50):.3f} "
        f"p90={percentile(values, 0.90):.3f} max={max(values):.3f} "
        f"mean={mean(values):.3f}"
    )


def median(values: list[float]) -> float:
    return percentile(values, 0.50)


def grid_height_jumps(points: list[Point], cell_size: float = 0.20) -> tuple[float, float]:
    if len(points) < 2:
        return 0.0, 0.0

    cells: dict[tuple[int, int], list[float]] = {}
    for x, y, z, _intensity in points:
        cell = (int(round(x / cell_size)), int(round(y / cell_size)))
        cells.setdefault(cell, []).append(z)

    cell_heights = {cell: median(z_values) for cell, z_values in cells.items() if z_values}
    jumps: list[float] = []
    for cell, height in cell_heights.items():
        x_idx, y_idx = cell
        for neighbor in ((x_idx + 1, y_idx), (x_idx, y_idx + 1)):
            if neighbor in cell_heights:
                jumps.append(abs(height - cell_heights[neighbor]))

    if not jumps:
        return 0.0, 0.0
    return percentile(jumps, 0.90), max(jumps)


def profile_height_jumps(points: list[Point], region: Region | None = None, cell_size: float = 0.20) -> tuple[float, float]:
    if len(points) < 2:
        return 0.0, 0.0

    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    if region is not None:
        axis_is_x = (region.x_max - region.x_min) >= (region.y_max - region.y_min)
        axis_min = region.x_min if axis_is_x else region.y_min
        axis_max = region.x_max if axis_is_x else region.y_max
    else:
        axis_is_x = (max(xs) - min(xs)) >= (max(ys) - min(ys))
        axis_min = min(xs) if axis_is_x else min(ys)
        axis_max = max(xs) if axis_is_x else max(ys)

    span = max(axis_max - axis_min, cell_size)
    bin_count = max(3, min(40, int(round(span / cell_size))))
    bins: list[list[float]] = [[] for _ in range(bin_count)]
    for x, y, z, _intensity in points:
        axis_value = x if axis_is_x else y
        bin_id = int((axis_value - axis_min) / span * bin_count)
        bin_id = max(0, min(bin_count - 1, bin_id))
        bins[bin_id].append(z)

    profile = [median(z_values) for z_values in bins if len(z_values) >= 3]
    jumps = [abs(profile[idx + 1] - profile[idx]) for idx in range(len(profile) - 1)]
    if not jumps:
        return 0.0, 0.0
    return percentile(jumps, 0.90), max(jumps)


def classify_terrain_points(points: list[Point], region: Region | None = None) -> TerrainClassification:
    if len(points) < 20:
        return TerrainClassification("insufficient_data", f"count={len(points)}")

    z_values = [point[2] for point in points]
    intensity_values = [point[3] for point in points if point[3] is not None]
    z_p10 = percentile(z_values, 0.10)
    z_p90 = percentile(z_values, 0.90)
    z_span = z_p90 - z_p10
    z_max = max(z_values)
    intensity_p90 = percentile(intensity_values, 0.90) if intensity_values else 0.0
    intensity_max = max(intensity_values) if intensity_values else 0.0
    jump_p90, jump_max = grid_height_jumps(points)
    profile_jump_p90, profile_jump_max = profile_height_jumps(points, region)
    reason = (
        f"z_span_p10_p90={z_span:.3f};z_max={z_max:.3f};"
        f"intensity_p90={intensity_p90:.3f};intensity_max={intensity_max:.3f};"
        f"profile_jump_p90={profile_jump_p90:.3f};profile_jump_max={profile_jump_max:.3f};"
        f"grid_jump_p90={jump_p90:.3f};grid_jump_max={jump_max:.3f}"
    )

    if region is not None:
        prior_reason = f"{reason};whitebox_prior={region.kind}"
        if region.kind == "ramp":
            return TerrainClassification("ramp_like_traversable", prior_reason)
        if region.kind == "single_step":
            return TerrainClassification("step_like", prior_reason)
        if region.kind == "stair":
            return TerrainClassification("step_like", prior_reason)
        if region.kind == "staircase":
            return TerrainClassification("stair_like", prior_reason)
        if region.kind in {"floor_2", "landing"}:
            return TerrainClassification("flat", prior_reason)

    if z_span < 0.06 and intensity_p90 < 0.05 and profile_jump_max < 0.06:
        return TerrainClassification("flat", reason)
    if z_span >= 0.22 and profile_jump_max >= 0.055:
        return TerrainClassification("stair_like", reason)
    if 0.06 <= z_span < 0.22 and profile_jump_max >= 0.055:
        return TerrainClassification("step_like", reason)
    if z_span >= 0.06 and profile_jump_max < 0.055 and intensity_p90 <= 0.20:
        return TerrainClassification("ramp_like_traversable", reason)
    if intensity_p90 > 0.15 or intensity_max > 0.25:
        return TerrainClassification("obstacle_like", reason)
    return TerrainClassification("rough_or_uncertain", reason)


def summarize_bounds(points: list[Point]) -> str:
    if not points:
        return "n/a"
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    zs = [point[2] for point in points]
    return (
        f"x=[{min(xs):.2f},{max(xs):.2f}] "
        f"y=[{min(ys):.2f},{max(ys):.2f}] "
        f"z=[{min(zs):.2f},{max(zs):.2f}]"
    )


def cloud_xy_bounds(points: list[Point]) -> tuple[float, float, float, float] | None:
    if not points:
        return None
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return min(xs), max(xs), min(ys), max(ys)


def region_overlaps_cloud(region: Region, bounds: tuple[float, float, float, float] | None) -> bool:
    if bounds is None:
        return False
    x_min, x_max, y_min, y_max = bounds
    return not (region.x_max < x_min or region.x_min > x_max or region.y_max < y_min or region.y_min > y_max)


def read_cloud_points(msg: object) -> list[Point]:
    from sensor_msgs_py import point_cloud2

    points: list[Point] = []
    field_names = {field.name for field in msg.fields}
    required_fields = ("x", "y", "z")
    if not all(field in field_names for field in required_fields):
        raise ValueError(f"PointCloud2 is missing required x/y/z fields: {sorted(field_names)}")

    if "intensity" in field_names:
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            points.append((float(point[0]), float(point[1]), float(point[2]), float(point[3])))
    else:
        for point in point_cloud2.read_points(msg, field_names=required_fields, skip_nans=True):
            points.append((float(point[0]), float(point[1]), float(point[2]), None))
    return points


def read_ascii_ply_points(ply_path: Path) -> list[Point]:
    vertex_count: int | None = None
    properties: list[str] = []
    points: list[Point] = []

    with ply_path.open("r", encoding="ascii") as ply_file:
        first_line = ply_file.readline().strip()
        if first_line != "ply":
            raise ValueError(f"{ply_path} is not a PLY file")

        for line in ply_file:
            line = line.strip()
            if line == "end_header":
                break
            if line.startswith("format ") and line != "format ascii 1.0":
                raise ValueError(f"{ply_path} must be ascii PLY")
            if line.startswith("element vertex "):
                vertex_count = int(line.split()[-1])
            elif line.startswith("property "):
                properties.append(line.split()[-1])
        else:
            raise ValueError(f"{ply_path} has no end_header")

        if vertex_count is None:
            raise ValueError(f"{ply_path} has no vertex count")

        try:
            x_idx = properties.index("x")
            y_idx = properties.index("y")
            z_idx = properties.index("z")
        except ValueError as exc:
            raise ValueError(f"{ply_path} must contain x/y/z properties") from exc

        intensity_idx = properties.index("intensity") if "intensity" in properties else None
        for _ in range(vertex_count):
            parts = ply_file.readline().split()
            if len(parts) < len(properties):
                continue
            intensity = float(parts[intensity_idx]) if intensity_idx is not None else None
            points.append((float(parts[x_idx]), float(parts[y_idx]), float(parts[z_idx]), intensity))

    return points


def print_analysis(source: str, points: list[Point], regions: list[Region], heading_yaw: float | None = None) -> None:
    bounds = cloud_xy_bounds(points)
    print(f"source={source}")
    print(f"total_points={len(points)}")
    print(f"bounds={summarize_bounds(points)}")
    print("")
    print(
        "kind,name,count,z_stats,intensity_stats,coverage,terrain_class,"
        "traversal_axis,entry_side,approach_alignment,approach_class,"
        "direction_confidence,stair_traversal_decision,class_reason"
    )

    for region in regions:
        region_points = [
            point
            for point in points
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
        direction = assess_direction(region, heading_yaw)
        traversal_decision = stair_traversal_decision(region, classification, direction)
        print(
            f"{region.kind},{region.name},{len(region_points)},"
            f"{summarize(z_values)},{summarize(intensity_values)},{coverage},"
            f"{classification.label},{direction.axis},{direction.entry_side},"
            f"{direction.alignment},{direction.approach_class},{direction.confidence},"
            f"{traversal_decision},{classification.reason}"
        )

    print("")
    print("Notes:")
    print("- --ply checks the generated global scene geometry; it usually has no intensity field.")
    print("- --topic /terrain_map checks the robot's current local 2.5D terrain-analysis output.")
    print("- outside_current_cloud_xy means the robot has not observed that terrain region in this cloud.")


def run_topic_mode(topic: str, regions: list[Region], timeout: float) -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import PointCloud2

    class TerrainMapAnalyzer(Node):
        def __init__(self) -> None:
            super().__init__("whitebox_terrain_map_analyzer")
            self.received = False
            qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
            )
            self.create_subscription(PointCloud2, topic, self._cloud_callback, qos)
            self.get_logger().info(f"Waiting for one PointCloud2 message on {topic}")

        def _cloud_callback(self, msg: PointCloud2) -> None:
            if self.received:
                return
            self.received = True
            points = read_cloud_points(msg)
            source = f"topic:{topic} frame={msg.header.frame_id} stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
            print_analysis(source, points, regions)

    rclpy.init()
    node = TerrainMapAnalyzer()
    deadline = time.monotonic() + timeout
    try:
        while rclpy.ok() and not node.received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not node.received:
            node.get_logger().error("Timed out waiting for terrain map.")
            sys.exit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze whitebox terrain-map 2.5D expression.")
    parser.add_argument("--topic", default="/terrain_map", help="PointCloud2 topic to analyze.")
    parser.add_argument("--ply", help="Analyze an ASCII PLY file instead of subscribing to ROS.")
    parser.add_argument(
        "--metadata",
        default="src/base_autonomy/vehicle_simulator/mesh/whitebox_stair_test/whitebox_stair_test.json",
        help="Generated whitebox metadata JSON.",
    )
    parser.add_argument("--margin", type=float, default=0.05, help="XY margin around each region in meters.")
    parser.add_argument("--timeout", type=float, default=10.0, help="Seconds to wait for one cloud.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    regions = load_regions(Path(args.metadata), args.margin)
    if args.ply:
        ply_path = Path(args.ply)
        print_analysis(f"ply:{ply_path}", read_ascii_ply_points(ply_path), regions)
        return
    run_topic_mode(args.topic, regions, args.timeout)


if __name__ == "__main__":
    main()
