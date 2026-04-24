#!/usr/bin/env python3
"""
Generate a minimal whitebox stair test scene.

Outputs:
- a Unity-importable Wavefront OBJ mesh
- a sampled ASCII PLY point cloud map for ROS-side visualization/debug

The scene contains:
- a 24m x 24m floor
- multiple ramps with different rises
- multiple single steps
- multiple 6-step staircases with 0.10m rise and 0.28m tread depth
- a minimal second-floor landing connected to one staircase
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
    if spec.name == "floor":
        lines.append("usemtl floor\n")
    elif spec.terrain_kind == "floor_2":
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


def build_scene() -> tuple[list[BoxSpec], list[RampSpec]]:
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

    def add_staircase(name: str, start_x: float, y_min: float, yaw_sign: int = 1) -> None:
        stair_tread = 0.28
        stair_rise = 0.10
        width = 1.20
        traversal_axis = (float(yaw_sign), 0.0)
        entry_side = "x_min" if yaw_sign > 0 else "x_max"
        for idx in range(6):
            x0 = start_x + yaw_sign * stair_tread * idx
            x1 = start_x + yaw_sign * stair_tread * (idx + 1)
            boxes.append(
                BoxSpec(
                    name=f"{name}_{idx + 1:02d}",
                    x_min=min(x0, x1),
                    x_max=max(x0, x1),
                    y_min=y_min,
                    y_max=y_min + width,
                    z_min=0.0,
                    z_max=stair_rise * (idx + 1),
                    terrain_kind="stair_step",
                    traversal_axis=traversal_axis,
                    entry_side=entry_side,
                    group_name=name,
                    step_index=idx + 1,
                    step_count=6,
                    rise_m=stair_rise,
                    tread_m=stair_tread,
                    width_m=width,
                )
            )

    add_staircase("stair_south_right", 2.20, -8.20, 1)
    add_staircase("stair_north_left", -2.20, 8.00, -1)
    add_staircase("stair_east_mid", 7.80, 3.80, 1)

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

    ramps: list[RampSpec] = [
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
    return boxes, ramps


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


def write_map_ply(output_path: Path, step: float = 0.05) -> int:
    boxes, ramps = build_scene()
    points: set[tuple[float, float, float]] = set()
    for spec in boxes:
        add_box_points(points, spec, step)
    for spec in ramps:
        add_ramp_points(points, spec, step)
    ordered_points = sorted(points)
    write_ascii_ply(ordered_points, output_path)
    return len(ordered_points)


def write_obj(output_path: Path) -> dict[str, object]:
    boxes, ramps = build_scene()
    step_count = len([spec for spec in boxes if spec.name.startswith("single_step_")])
    staircase_count = len(
        {
            "_".join(spec.name.split("_")[:-1])
            for spec in boxes
            if spec.name.startswith("stair_")
        }
    )
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
        "floor_size_m": [24.0, 24.0],
        "terrain_features": {
            "floor_2": {
                "count": len([spec for spec in boxes if spec.terrain_kind == "floor_2"]),
                "height_m": 0.60,
                "connected_by": "stair_south_right",
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
                "staircase_count": staircase_count,
                "steps_per_staircase": 6,
                "rise_m": 0.10,
                "tread_m": 0.28,
                "width_m": 1.2,
            },
        },
        "objects": [metadata_dict(spec) for spec in boxes] + [metadata_dict(spec) for spec in ramps],
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
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_path = Path(args.output)
    metadata = write_obj(output_path)
    map_ply_path = output_path.with_name("map.ply")
    point_count = write_map_ply(map_ply_path)
    print(f"Scene OBJ written to: {output_path}")
    print(f"Scene metadata written to: {output_path.with_suffix('.json')}")
    print(f"Scene PLY map written to: {map_ply_path}")
    print("Summary:")
    print(f"  floor: {metadata['floor_size_m'][0]}m x {metadata['floor_size_m'][1]}m")
    print(
        "  floor_2: "
        f"{metadata['terrain_features']['floor_2']['count']} landing at "
        f"{metadata['terrain_features']['floor_2']['height_m']}m"
    )
    print(f"  ramps: {metadata['terrain_features']['ramp']['count']} fixed ramps")
    print(f"  single steps: {metadata['terrain_features']['single_step']['count']} fixed steps")
    print(
        "  staircases: "
        f"{metadata['terrain_features']['stairs']['staircase_count']} x "
        f"{metadata['terrain_features']['stairs']['steps_per_staircase']} steps"
    )
    print(f"  sampled map points: {point_count}")


if __name__ == "__main__":
    main()
