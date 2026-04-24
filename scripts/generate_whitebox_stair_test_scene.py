#!/usr/bin/env python3
"""
Generate a minimal whitebox stair test scene.

Outputs:
- a Unity-importable Wavefront OBJ mesh
- a sampled ASCII PLY point cloud map for ROS-side visualization/debug

The scene contains:
- a 12m x 12m floor
- a 3m ramp that rises 0.45m
- a single 0.10m step
- a 6-step staircase with 0.10m rise and 0.28m tread depth
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass(frozen=True)
class BoxSpec:
    name: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float


@dataclass(frozen=True)
class RampSpec:
    name: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float


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
    lines.append("usemtl floor\n" if spec.name == "floor" else "usemtl obstacle\n")
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
    floor_half_extent = 16.0
    boxes: list[BoxSpec] = [
        BoxSpec(
            name="floor",
            x_min=-floor_half_extent,
            x_max=floor_half_extent,
            y_min=-floor_half_extent,
            y_max=floor_half_extent,
            z_min=-0.10,
            z_max=0.0,
        ),
    ]

    obstacle_layout = [
        (-12.8, -10.8, -11.8, -10.2, 0.80),
        (-9.8, -8.0, -7.5, -5.7, 0.95),
        (-13.2, -11.6, -2.6, -0.8, 0.70),
        (-10.4, -8.6, 1.2, 2.8, 1.10),
        (-12.6, -10.4, 6.2, 8.0, 0.85),
        (-9.0, -7.2, 10.0, 11.8, 0.65),
        (-6.2, -4.4, -12.2, -10.4, 1.00),
        (-3.0, -1.2, -8.2, -6.0, 0.75),
        (-6.8, -4.8, -3.2, -1.6, 0.55),
        (-6.0, -4.2, 2.4, 4.0, 1.15),
        (-3.4, -1.4, 6.2, 8.4, 0.70),
        (-6.4, -4.4, 10.4, 12.2, 0.90),
        (1.6, 3.6, -11.8, -10.0, 0.75),
        (4.8, 6.8, -8.0, -6.2, 1.05),
        (1.4, 3.2, -3.0, -1.4, 0.60),
        (5.4, 7.6, 1.8, 3.8, 1.20),
        (1.2, 3.0, 6.4, 8.2, 0.85),
        (4.6, 6.4, 10.2, 12.0, 0.70),
        (9.2, 11.4, -12.0, -10.0, 0.95),
        (11.8, 13.8, -7.4, -5.2, 0.60),
        (8.8, 10.8, -2.8, -0.8, 1.10),
        (11.2, 13.6, 1.0, 2.8, 0.75),
        (8.6, 10.6, 6.0, 8.0, 0.90),
        (11.0, 13.2, 10.0, 12.2, 1.00),
    ]

    obstacles: list[BoxSpec] = [
        BoxSpec(
            name=f"box_obstacle_{idx + 1:02d}",
            x_min=x_min,
            x_max=x_max,
            y_min=y_min,
            y_max=y_max,
            z_min=0.0,
            z_max=z_max,
        )
        for idx, (x_min, x_max, y_min, y_max, z_max) in enumerate(obstacle_layout)
    ]

    boxes.extend(obstacles)
    ramps: list[RampSpec] = []
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
        "newmtl obstacle\n",
        "Ka 0.78 0.28 0.18\n",
        "Kd 0.78 0.28 0.18\n",
        "Ks 0.08 0.08 0.08\n",
        "Ns 18.0\n",
        "\n",
        "newmtl accent\n",
        "Ka 0.85 0.60 0.18\n",
        "Kd 0.85 0.60 0.18\n",
        "Ks 0.10 0.10 0.10\n",
        "Ns 20.0\n",
    ]
    output_path.with_suffix(".mtl").write_text("".join(mtl_lines), encoding="ascii")

    metadata = {
        "scene_type": "flat_obstacle_arena",
        "floor_size_m": [32.0, 32.0],
        "spawn_keepout_m": {
            "x": [-4.0, 4.0],
            "y": [-4.0, 4.0],
        },
        "obstacle_count": len([spec for spec in boxes if spec.name.startswith("box_obstacle_")]),
        "objects": [asdict(spec) for spec in boxes] + [asdict(spec) for spec in ramps],
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
    print(f"  obstacles: {metadata['obstacle_count']} fixed box obstacles")
    print(
        "  spawn keepout: "
        f"x in [{metadata['spawn_keepout_m']['x'][0]}, {metadata['spawn_keepout_m']['x'][1]}], "
        f"y in [{metadata['spawn_keepout_m']['y'][0]}, {metadata['spawn_keepout_m']['y'][1]}]"
    )
    print(f"  sampled map points: {point_count}")


if __name__ == "__main__":
    main()
