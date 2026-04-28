#!/usr/bin/env python3
"""
Generate a minimal whitebox stair test scene.

Outputs:
- a Unity-importable Wavefront OBJ mesh
- a sampled ASCII PLY point cloud map for ROS-side visualization/debug

The default realistic scene contains:
- a 36m x 36m floor
- multiple ramps with different rises
- multiple single steps
- a split two-flight staircase with a mid landing
- a 3.0m second-floor landing connected by 20 x 0.15m steps
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional


@dataclass(frozen=True)
class BoxSpec:
    name: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float
    terrain_kind: str = "box"
    traversal_axis: Optional[tuple[float, float]] = None
    entry_side: Optional[str] = None
    group_name: Optional[str] = None
    step_index: Optional[int] = None
    step_count: Optional[int] = None
    rise_m: Optional[float] = None
    tread_m: Optional[float] = None
    width_m: Optional[float] = None


@dataclass(frozen=True)
class RampSpec:
    name: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float
    terrain_kind: str = "ramp"
    traversal_axis: tuple[float, float] = (1.0, 0.0)
    entry_side: str = "x_min"


def frange(start: float, stop: float, step: float) -> list[float]:
    values: list[float] = []
    cur = start
    eps = step * 0.5
    while cur <= stop + eps:
        values.append(round(cur, 6))
        cur += step
    return values


def format_vertex(vertex: tuple[float, float, float]) -> str:
    return f"v {vertex[0]:.6f} {vertex[1]:.6f} {vertex[2]:.6f}\n"


def format_normal(normal: tuple[float, float, float]) -> str:
    return f"vn {normal[0]:.6f} {normal[1]:.6f} {normal[2]:.6f}\n"


def compute_face_normal(
    vertices: list[tuple[float, float, float]], face: tuple[int, int, int, int]
) -> tuple[float, float, float]:
    a = vertices[face[0] - 1]
    b = vertices[face[1] - 1]
    c = vertices[face[2] - 1]
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    norm = math.sqrt(nx * nx + ny * ny + nz * nz)
    if norm <= 1e-9:
        return (0.0, 0.0, 1.0)
    return (nx / norm, ny / norm, nz / norm)


def write_box(lines: list[str], vertex_offset: int, normal_offset: int, spec: BoxSpec) -> tuple[int, int]:
    x0, x1 = spec.x_min, spec.x_max
    y0, y1 = spec.y_min, spec.y_max
    z0, z1 = spec.z_min, spec.z_max

    vertices = [
        (x0, y0, z0),
        (x1, y0, z0),
        (x1, y1, z0),
        (x0, y1, z0),
        (x0, y0, z1),
        (x1, y0, z1),
        (x1, y1, z1),
        (x0, y1, z1),
    ]
    faces = [
        (1, 2, 3, 4),  # bottom
        (5, 8, 7, 6),  # top
        (1, 5, 6, 2),
        (2, 6, 7, 3),
        (3, 7, 8, 4),
        (4, 8, 5, 1),
    ]

    lines.append(f"o {spec.name}\n")
    if spec.terrain_kind == "floor":
        lines.append("usemtl floor\n")
    elif spec.terrain_kind in {"floor_2", "mid_landing"}:
        lines.append("usemtl upper_floor\n")
    elif spec.name.startswith("single_step"):
        lines.append("usemtl step\n")
    elif spec.name.startswith("stair_"):
        lines.append("usemtl stair\n")
    else:
        lines.append("usemtl obstacle\n")
    for vertex in vertices:
        lines.append(format_vertex(vertex))
    normals = [compute_face_normal(vertices, face) for face in faces]
    for normal in normals:
        lines.append(format_normal(normal))
    for face_index, face in enumerate(faces):
        a, b, c, d = (vertex_offset + idx for idx in face)
        n = normal_offset + face_index + 1
        lines.append(f"f {a}//{n} {b}//{n} {c}//{n} {d}//{n}\n")
    return vertex_offset + len(vertices), normal_offset + len(normals)


def write_ramp(
    lines: list[str], vertex_offset: int, normal_offset: int, spec: RampSpec
) -> tuple[int, int]:
    x0, x1 = spec.x_min, spec.x_max
    y0, y1 = spec.y_min, spec.y_max
    z0, z1 = spec.z_min, spec.z_max

    vertices = [
        (x0, y0, z0),
        (x1, y0, z0),
        (x1, y1, z0),
        (x0, y1, z0),
        (x0, y0, z0),
        (x1, y0, z1),
        (x1, y1, z1),
        (x0, y1, z0),
    ]
    faces = [
        (1, 2, 3, 4),  # bottom
        (5, 8, 7, 6),  # top slope
        (1, 5, 6, 2),
        (2, 6, 7, 3),
        (3, 7, 8, 4),
        (4, 8, 5, 1),
    ]

    lines.append(f"o {spec.name}\n")
    lines.append("usemtl accent\n")
    for vertex in vertices:
        lines.append(format_vertex(vertex))
    normals = [compute_face_normal(vertices, face) for face in faces]
    for normal in normals:
        lines.append(format_normal(normal))
    for face_index, face in enumerate(faces):
        a, b, c, d = (vertex_offset + idx for idx in face)
        n = normal_offset + face_index + 1
        lines.append(f"f {a}//{n} {b}//{n} {c}//{n} {d}//{n}\n")
    return vertex_offset + len(vertices), normal_offset + len(normals)


def add_x_staircase(
    boxes: list[BoxSpec],
    name: str,
    start_x: float,
    y_min: float,
    yaw_sign: int,
    step_count: int,
    stair_rise: float,
    stair_tread: float,
    width: float,
    base_z: float = 0.0,
    global_step_offset: int = 0,
    total_step_count: Optional[int] = None,
) -> None:
    traversal_axis = (float(yaw_sign), 0.0)
    entry_side = "x_min" if yaw_sign > 0 else "x_max"
    total_steps = total_step_count or step_count
    for idx in range(step_count):
        x0 = start_x + yaw_sign * stair_tread * idx
        x1 = start_x + yaw_sign * stair_tread * (idx + 1)
        global_step_index = global_step_offset + idx + 1
        boxes.append(
            BoxSpec(
                name=f"{name}_{idx + 1:02d}",
                x_min=min(x0, x1),
                x_max=max(x0, x1),
                y_min=y_min,
                y_max=y_min + width,
                z_min=base_z,
                z_max=base_z + stair_rise * (idx + 1),
                terrain_kind="stair_step",
                traversal_axis=traversal_axis,
                entry_side=entry_side,
                group_name=name,
                step_index=global_step_index,
                step_count=total_steps,
                rise_m=stair_rise,
                tread_m=stair_tread,
                width_m=width,
            )
        )


def add_y_staircase(
    boxes: list[BoxSpec],
    name: str,
    x_min: float,
    start_y: float,
    yaw_sign: int,
    step_count: int,
    stair_rise: float,
    stair_tread: float,
    width: float,
    base_z: float = 0.0,
    global_step_offset: int = 0,
    total_step_count: Optional[int] = None,
) -> None:
    traversal_axis = (0.0, float(yaw_sign))
    entry_side = "y_min" if yaw_sign > 0 else "y_max"
    total_steps = total_step_count or step_count
    for idx in range(step_count):
        y0 = start_y + yaw_sign * stair_tread * idx
        y1 = start_y + yaw_sign * stair_tread * (idx + 1)
        global_step_index = global_step_offset + idx + 1
        boxes.append(
            BoxSpec(
                name=f"{name}_{idx + 1:02d}",
                x_min=x_min,
                x_max=x_min + width,
                y_min=min(y0, y1),
                y_max=max(y0, y1),
                z_min=base_z,
                z_max=base_z + stair_rise * (idx + 1),
                terrain_kind="stair_step",
                traversal_axis=traversal_axis,
                entry_side=entry_side,
                group_name=name,
                step_index=global_step_index,
                step_count=total_steps,
                rise_m=stair_rise,
                tread_m=stair_tread,
                width_m=width,
            )
        )


def add_compact_test_features(boxes: list[BoxSpec], include_debug_stairs: bool = True) -> None:
    step_layout = [
        ("single_step_10cm_north", -0.60, 0.60, 5.80, 7.00, 0.10),
        ("single_step_10cm_east", 6.20, 7.40, -0.60, 0.60, 0.10),
        ("single_step_15cm_west", -7.40, -6.20, -0.60, 0.60, 0.15),
        ("single_step_08cm_southwest", -7.20, -6.00, -7.20, -6.00, 0.08),
    ]
    for name, x_min, x_max, y_min, y_max, height in step_layout:
        boxes.append(
            BoxSpec(
                name=name,
                x_min=x_min,
                x_max=x_max,
                y_min=y_min,
                y_max=y_max,
                z_min=0.0,
                z_max=height,
                terrain_kind="single_step",
            )
        )

    if include_debug_stairs:
        add_x_staircase(boxes, "stair_south_right", 2.20, -8.20, 1, 6, 0.10, 0.28, 1.20)
        add_x_staircase(boxes, "stair_north_left", -2.20, 8.00, -1, 6, 0.10, 0.28, 1.20)
        add_x_staircase(boxes, "stair_east_mid", 7.80, 3.80, 1, 6, 0.10, 0.28, 1.20)

        boxes.append(
            BoxSpec(
                name="floor_2_south_landing",
                x_min=3.88,
                x_max=9.40,
                y_min=-9.20,
                y_max=-5.60,
                z_min=0.55,
                z_max=0.60,
                terrain_kind="floor_2",
                traversal_axis=(1.0, 0.0),
                entry_side="x_min",
            )
        )


def build_common_ramps() -> list[RampSpec]:
    return [
        RampSpec(
            name="ramp_southwest_3m_45cm",
            x_min=-10.40,
            x_max=-7.40,
            y_min=-8.40,
            y_max=-7.20,
            z_min=0.0,
            z_max=0.45,
        ),
        RampSpec(
            name="ramp_west_3m_25cm",
            x_min=-10.40,
            x_max=-7.40,
            y_min=2.40,
            y_max=3.60,
            z_min=0.0,
            z_max=0.25,
        ),
        RampSpec(
            name="ramp_northeast_4m_45cm",
            x_min=5.20,
            x_max=9.20,
            y_min=7.20,
            y_max=8.40,
            z_min=0.0,
            z_max=0.45,
        ),
        RampSpec(
            name="ramp_south_2m_18cm",
            x_min=-1.00,
            x_max=1.00,
            y_min=-9.20,
            y_max=-8.00,
            z_min=0.0,
            z_max=0.18,
        ),
    ]


def build_compact_scene() -> tuple[list[BoxSpec], list[RampSpec]]:
    floor_half_extent = 12.0
    boxes: list[BoxSpec] = [
        BoxSpec(
            name="floor",
            x_min=-floor_half_extent,
            x_max=floor_half_extent,
            y_min=-floor_half_extent,
            y_max=floor_half_extent,
            z_min=-0.10,
            z_max=0.0,
            terrain_kind="floor",
        ),
    ]
    add_compact_test_features(boxes)
    return boxes, build_common_ramps()


def build_realistic_scene() -> tuple[list[BoxSpec], list[RampSpec]]:
    floor_half_extent = 18.0
    floor_z_min = -0.10
    floor_z_max = 0.0
    floor_cut_x_min = -3.20
    floor_cut_x_max = 8.00
    floor_cut_y_min = -2.40
    floor_cut_y_max = 9.00
    boxes: list[BoxSpec] = [
        BoxSpec(
            name="floor_west",
            x_min=-floor_half_extent,
            x_max=floor_cut_x_min,
            y_min=-floor_half_extent,
            y_max=floor_half_extent,
            z_min=floor_z_min,
            z_max=floor_z_max,
            terrain_kind="floor",
        ),
        BoxSpec(
            name="floor_east",
            x_min=floor_cut_x_max,
            x_max=floor_half_extent,
            y_min=-floor_half_extent,
            y_max=floor_half_extent,
            z_min=floor_z_min,
            z_max=floor_z_max,
            terrain_kind="floor",
        ),
        BoxSpec(
            name="floor_south",
            x_min=floor_cut_x_min,
            x_max=floor_cut_x_max,
            y_min=-floor_half_extent,
            y_max=floor_cut_y_min,
            z_min=floor_z_min,
            z_max=floor_z_max,
            terrain_kind="floor",
        ),
        BoxSpec(
            name="floor_north",
            x_min=floor_cut_x_min,
            x_max=floor_cut_x_max,
            y_min=floor_cut_y_max,
            y_max=floor_half_extent,
            z_min=floor_z_min,
            z_max=floor_z_max,
            terrain_kind="floor",
        ),
    ]
    add_compact_test_features(boxes, include_debug_stairs=False)

    stair_rise = 0.15
    stair_tread = 0.28
    stair_width = 1.20
    run_steps = 10
    total_steps = run_steps * 2
    first_run_start_x = -3.20
    first_run_y_min = -2.40
    mid_landing_z = stair_rise * run_steps
    second_floor_z = stair_rise * total_steps

    add_x_staircase(
        boxes,
        "stair_realistic_lower",
        first_run_start_x,
        first_run_y_min,
        1,
        run_steps,
        stair_rise,
        stair_tread,
        stair_width,
        base_z=0.0,
        global_step_offset=0,
        total_step_count=total_steps,
    )

    first_run_top_x = first_run_start_x + stair_tread * run_steps
    mid_landing_x_max = first_run_top_x + 1.60
    mid_landing_y_max = -0.80
    boxes.append(
        BoxSpec(
            name="mid_landing_realistic",
            x_min=first_run_top_x,
            x_max=mid_landing_x_max,
            y_min=first_run_y_min,
            y_max=mid_landing_y_max,
            z_min=mid_landing_z - 0.05,
            z_max=mid_landing_z,
            terrain_kind="mid_landing",
            traversal_axis=(0.0, 1.0),
            entry_side="y_min",
        )
    )

    second_run_x_min = mid_landing_x_max - stair_width
    add_y_staircase(
        boxes,
        "stair_realistic_upper",
        second_run_x_min,
        mid_landing_y_max,
        1,
        run_steps,
        stair_rise,
        stair_tread,
        stair_width,
        base_z=mid_landing_z,
        global_step_offset=run_steps,
        total_step_count=total_steps,
    )

    second_run_top_y = mid_landing_y_max + stair_tread * run_steps
    boxes.append(
        BoxSpec(
            name="floor_2_south_landing",
            x_min=second_run_x_min,
            x_max=8.00,
            y_min=second_run_top_y,
            y_max=9.00,
            z_min=second_floor_z - 0.05,
            z_max=second_floor_z,
            terrain_kind="floor_2",
            traversal_axis=(0.0, 1.0),
            entry_side="y_min",
        )
    )

    return boxes, build_common_ramps()


def build_scene(profile: str = "realistic") -> tuple[list[BoxSpec], list[RampSpec]]:
    if profile == "compact":
        return build_compact_scene()
    if profile == "realistic":
        return build_realistic_scene()
    raise ValueError(f"Unknown scene profile: {profile}")


def scene_floor_size(profile: str) -> list[float]:
    return [24.0, 24.0] if profile == "compact" else [36.0, 36.0]


def scene_profile_summary(profile: str) -> dict[str, object]:
    if profile == "compact":
        return {
            "name": "compact",
            "floor_2_height_m": 0.60,
            "staircase_type": "single-flight debug stair",
            "realistic_stair": False,
        }
    return {
        "name": "realistic",
        "floor_2_height_m": 3.00,
        "staircase_type": "split two-flight stair with mid landing",
        "realistic_stair": True,
        "rise_m": 0.15,
        "tread_m": 0.28,
        "steps_total": 20,
        "steps_per_flight": 10,
        "mid_landing_height_m": 1.50,
    }


def count_stair_groups(boxes: list[BoxSpec]) -> int:
    return len({spec.group_name for spec in boxes if spec.terrain_kind == "stair_step" and spec.group_name})


def max_stair_step_count(boxes: list[BoxSpec]) -> int:
    return max((spec.step_count or 0 for spec in boxes if spec.terrain_kind == "stair_step"), default=0)


def canonical_stair_params(boxes: list[BoxSpec]) -> dict[str, float]:
    stair_specs = [spec for spec in boxes if spec.terrain_kind == "stair_step"]
    if not stair_specs:
        return {"rise_m": 0.0, "tread_m": 0.0, "width_m": 0.0}
    return {
        "rise_m": max(float(spec.rise_m or 0.0) for spec in stair_specs),
        "tread_m": max(float(spec.tread_m or 0.0) for spec in stair_specs),
        "width_m": max(float(spec.width_m or 0.0) for spec in stair_specs),
    }


def add_box_points(points: set[tuple[float, float, float]], spec: BoxSpec, step: float) -> None:
    xs = frange(spec.x_min, spec.x_max, step)
    ys = frange(spec.y_min, spec.y_max, step)
    zs = frange(spec.z_min, spec.z_max, step)

    for x in xs:
        for y in ys:
            points.add((x, y, round(spec.z_max, 6)))

    for x in xs:
        for z in zs:
            points.add((x, round(spec.y_min, 6), z))
            points.add((x, round(spec.y_max, 6), z))

    for y in ys:
        for z in zs:
            points.add((round(spec.x_min, 6), y, z))
            points.add((round(spec.x_max, 6), y, z))


def add_ramp_points(points: set[tuple[float, float, float]], spec: RampSpec, step: float) -> None:
    xs = frange(spec.x_min, spec.x_max, step)
    ys = frange(spec.y_min, spec.y_max, step)
    zs = frange(spec.z_min, spec.z_max, step)
    x_span = spec.x_max - spec.x_min

    for x in xs:
        ratio = (x - spec.x_min) / x_span if x_span > 0 else 0.0
        z = round(spec.z_min + ratio * (spec.z_max - spec.z_min), 6)
        for y in ys:
            points.add((x, y, z))

    for x in xs:
        ratio = (x - spec.x_min) / x_span if x_span > 0 else 0.0
        top_z = round(spec.z_min + ratio * (spec.z_max - spec.z_min), 6)
        for z in zs:
            if z <= top_z + 1e-6:
                points.add((x, round(spec.y_min, 6), z))
                points.add((x, round(spec.y_max, 6), z))

    for y in ys:
        for z in frange(spec.z_min, spec.z_max, step):
            points.add((round(spec.x_min, 6), y, z))
        for z in frange(spec.z_min, spec.z_max, step):
            points.add((round(spec.x_max, 6), y, z))


def write_ascii_ply(points: list[tuple[float, float, float]], output_path: Path) -> None:
    lines = [
        "ply\n",
        "format ascii 1.0\n",
        f"element vertex {len(points)}\n",
        "property float x\n",
        "property float y\n",
        "property float z\n",
        "end_header\n",
    ]
    for x, y, z in points:
        lines.append(f"{x:.6f} {y:.6f} {z:.6f}\n")
    output_path.write_text("".join(lines), encoding="ascii")


def metadata_dict(spec: BoxSpec | RampSpec) -> dict[str, object]:
    data = asdict(spec)
    return {key: value for key, value in data.items() if value is not None}


def spec_center(spec: BoxSpec | RampSpec) -> tuple[float, float]:
    return (float(spec.x_min) + float(spec.x_max)) / 2.0, (float(spec.y_min) + float(spec.y_max)) / 2.0


def route_goal(name: str, x: float, y: float, z: float) -> dict[str, object]:
    return {"name": name, "x": x, "y": y, "z": z}


def selected_stair_step_goals(steps: list[BoxSpec]) -> list[dict[str, object]]:
    goals: list[dict[str, object]] = []
    for idx, step in enumerate(steps):
        if idx % 2 == 1 or idx == len(steps) - 1:
            x, y = spec_center(step)
            goals.append(route_goal(step.name, x, y, float(step.z_max)))
    return goals


def floor2_entry_goal(floor2: BoxSpec, upper_steps: list[BoxSpec]) -> dict[str, object]:
    top_x, top_y = spec_center(upper_steps[-1])
    start_x = max(float(floor2.x_min) + 0.6, min(float(floor2.x_max) - 0.6, top_x))
    start_y = max(float(floor2.y_min) + 0.45, min(float(floor2.y_max) - 0.6, top_y + 0.55))
    return route_goal("floor2_entry", start_x, start_y, float(floor2.z_max))


def build_connectors(boxes: list[BoxSpec], profile: str) -> list[dict[str, object]]:
    if profile != "realistic":
        return []

    floor2 = next((spec for spec in boxes if spec.name == "floor_2_south_landing"), None)
    mid_landing = next((spec for spec in boxes if spec.name == "mid_landing_realistic"), None)
    lower_steps = sorted(
        [spec for spec in boxes if spec.group_name == "stair_realistic_lower"],
        key=lambda spec: int(spec.step_index or 0),
    )
    upper_steps = sorted(
        [spec for spec in boxes if spec.group_name == "stair_realistic_upper"],
        key=lambda spec: int(spec.step_index or 0),
    )
    if floor2 is None or mid_landing is None or not lower_steps or not upper_steps:
        return []

    mid_x, mid_y = spec_center(mid_landing)
    entry_goal = route_goal("stair_preentry", -4.0, -1.8, 0.0)
    exit_goal = floor2_entry_goal(floor2, upper_steps)
    up_route = (
        selected_stair_step_goals(lower_steps)
        + [route_goal("mid_landing_realistic", mid_x, mid_y, float(mid_landing.z_max))]
        + selected_stair_step_goals(upper_steps)
        + [exit_goal]
    )
    down_route = list(reversed(up_route[:-1])) + [entry_goal]

    return [
        {
            "name": "south_split_stair_connector",
            "connector_kind": "stair",
            "lower_floor": "floor_1",
            "upper_floor": "floor_2_south_landing",
            "lower_entry_goal": entry_goal,
            "upper_entry_goal": exit_goal,
            "allowed_directions": ["up", "down"],
            "up_route": up_route,
            "down_route": down_route,
            "controller": "stair_policy_placeholder",
            "notes": "Simulation whitebox connector; replace controller with RL stair policy on hardware.",
        }
    ]


def write_map_ply(output_path: Path, step: float = 0.05, profile: str = "realistic") -> int:
    boxes, ramps = build_scene(profile)
    points: set[tuple[float, float, float]] = set()
    for spec in boxes:
        add_box_points(points, spec, step)
    for spec in ramps:
        add_ramp_points(points, spec, step)
    ordered_points = sorted(points)
    write_ascii_ply(ordered_points, output_path)
    return len(ordered_points)


def write_obj(output_path: Path, profile: str = "realistic") -> dict[str, object]:
    boxes, ramps = build_scene(profile)
    step_count = len([spec for spec in boxes if spec.name.startswith("single_step_")])
    floor_2_specs = [spec for spec in boxes if spec.terrain_kind == "floor_2"]
    stair_params = canonical_stair_params(boxes)
    lines = [
        "# Minimal whitebox stair test scene for cmu_planner\n",
        "# Units: meters\n",
        f"mtllib {output_path.with_suffix('.mtl').name}\n",
    ]
    vertex_offset = 0
    normal_offset = 0
    for spec in boxes:
        vertex_offset, normal_offset = write_box(lines, vertex_offset, normal_offset, spec)
    for spec in ramps:
        vertex_offset, normal_offset = write_ramp(lines, vertex_offset, normal_offset, spec)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("".join(lines), encoding="ascii")
    mtl_lines = [
        "newmtl floor\n",
        "Ka 0.72 0.74 0.78\n",
        "Kd 0.72 0.74 0.78\n",
        "Ks 0.04 0.04 0.04\n",
        "Ns 12.0\n",
        "\n",
        "newmtl upper_floor\n",
        "Ka 0.52 0.68 0.74\n",
        "Kd 0.52 0.68 0.74\n",
        "Ks 0.05 0.05 0.05\n",
        "Ns 14.0\n",
        "\n",
        "newmtl obstacle\n",
        "Ka 0.78 0.28 0.18\n",
        "Kd 0.78 0.28 0.18\n",
        "Ks 0.08 0.08 0.08\n",
        "Ns 18.0\n",
        "\n",
        "newmtl step\n",
        "Ka 0.24 0.48 0.84\n",
        "Kd 0.24 0.48 0.84\n",
        "Ks 0.10 0.10 0.10\n",
        "Ns 20.0\n",
        "\n",
        "newmtl stair\n",
        "Ka 0.20 0.66 0.44\n",
        "Kd 0.20 0.66 0.44\n",
        "Ks 0.10 0.10 0.10\n",
        "Ns 20.0\n",
        "\n",
        "newmtl accent\n",
        "Ka 0.85 0.60 0.18\n",
        "Kd 0.85 0.60 0.18\n",
        "Ks 0.10 0.10 0.10\n",
        "Ns 20.0\n",
    ]
    output_path.with_suffix(".mtl").write_text("".join(mtl_lines), encoding="ascii")

    metadata = {
        "scene_type": "whitebox_2_5d_terrain",
        "profile": profile,
        "profile_summary": scene_profile_summary(profile),
        "floor_size_m": scene_floor_size(profile),
        "terrain_features": {
            "floor_2": {
                "count": len(floor_2_specs),
                "height_m": max((float(spec.z_max) for spec in floor_2_specs), default=0.0),
                "connected_by": "stair_realistic_upper" if profile == "realistic" else "stair_south_right",
            },
            "ramp": {
                "count": len(ramps),
                "examples": [
                    {"length_m": 3.0, "rise_m": 0.45, "width_m": 1.2},
                    {"length_m": 3.0, "rise_m": 0.25, "width_m": 1.2},
                    {"length_m": 4.0, "rise_m": 0.45, "width_m": 1.2},
                ],
            },
            "single_step": {
                "count": step_count,
                "heights_m": [0.08, 0.10, 0.15],
                "width_m": 1.2,
            },
            "stairs": {
                "flight_count": count_stair_groups(boxes),
                "staircase_count": count_stair_groups(boxes),
                "total_steps": scene_profile_summary(profile).get("steps_total", max_stair_step_count(boxes)),
                "steps_per_flight": scene_profile_summary(profile).get(
                    "steps_per_flight", max_stair_step_count(boxes)
                ),
                "steps_per_staircase": max_stair_step_count(boxes),
                "rise_m": stair_params["rise_m"],
                "tread_m": stair_params["tread_m"],
                "width_m": stair_params["width_m"],
            },
        },
        "objects": [metadata_dict(spec) for spec in boxes] + [metadata_dict(spec) for spec in ramps],
        "connectors": build_connectors(boxes, profile),
    }
    metadata_path = output_path.with_suffix(".json")
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="ascii")
    return metadata


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a minimal whitebox stair test scene OBJ for Unity import."
    )
    parser.add_argument(
        "-o",
        "--output",
        default="src/base_autonomy/vehicle_simulator/mesh/whitebox_stair_test/whitebox_stair_test.obj",
        help="Output OBJ path.",
    )
    parser.add_argument(
        "--profile",
        choices=("realistic", "compact"),
        default="realistic",
        help="Scene profile. realistic is a 3m split stair; compact is the previous 0.6m debug scene.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_path = Path(args.output)
    metadata = write_obj(output_path, args.profile)
    map_ply_path = output_path.with_name("map.ply")
    point_count = write_map_ply(map_ply_path, profile=args.profile)
    print(f"Scene OBJ written to: {output_path}")
    print(f"Scene metadata written to: {output_path.with_suffix('.json')}")
    print(f"Scene PLY map written to: {map_ply_path}")
    print("Summary:")
    print(f"  profile: {metadata['profile']}")
    print(f"  floor: {metadata['floor_size_m'][0]}m x {metadata['floor_size_m'][1]}m")
    print(
        "  floor_2: "
        f"{metadata['terrain_features']['floor_2']['count']} landing at "
        f"{metadata['terrain_features']['floor_2']['height_m']}m"
    )
    print(f"  ramps: {metadata['terrain_features']['ramp']['count']} fixed ramps")
    print(f"  single steps: {metadata['terrain_features']['single_step']['count']} fixed steps")
    print(
        "  stair flights: "
        f"{metadata['terrain_features']['stairs']['flight_count']} flights, "
        f"{metadata['terrain_features']['stairs']['total_steps']} total steps"
    )
    if metadata["profile"] == "realistic":
        print("  realistic stair: 20 steps, 0.15m rise, 0.28m tread, mid landing at 1.5m")
    print(f"  sampled map points: {point_count}")


if __name__ == "__main__":
    main()
