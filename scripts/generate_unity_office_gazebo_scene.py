#!/usr/bin/env python3
"""
Convert the packaged Unity office metadata into a simplified Gazebo whitebox scene.

The Unity build does not expose a Gazebo-loadable visual mesh. This script uses
AssetList.csv + Dimensions.csv + Categories.csv to generate axis-aligned boxes
that approximate walls, floors, and large furniture. By default it also creates
a shifted second-floor copy plus an exterior stair connector for multi-floor
preview without visually overlapping the two floors.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable


LOD_RE = re.compile(r"_LOD(\d+)$")
INCLUDED_CLASSES = {
    "wall",
    "door",
    "desk",
    "chair",
    "table",
    "cabinet",
    "shelves",
    "sofa",
    "counter",
    "sink",
    "toilet",
    "refridgerator",
    "television",
    "otherfurniture",
}
STRUCTURAL_NAME_HINTS = (
    "Block",
    "BuildGlass",
    "Column",
    "Wall",
    "Door",
    "Frame",
)
DEFAULT_SECOND_FLOOR_HEIGHT = 3.0
DEFAULT_STAIR_STEP_HEIGHT = 0.15
DEFAULT_STAIR_TREAD = 0.30
DEFAULT_STAIR_WIDTH = 2.4
DEFAULT_STAIR_FLIGHT_WIDTH = 1.2
DEFAULT_STAIR_LANDING_DEPTH = 1.4
DEFAULT_STAIR_RAIL_HEIGHT = 0.55
DEFAULT_STAIR_RAIL_THICKNESS = 0.12
DEFAULT_PERIMETER_WALL_HEIGHT = 3.0
DEFAULT_PERIMETER_WALL_THICKNESS = 0.24


@dataclass(frozen=True)
class BoxSpec:
    name: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float
    terrain_kind: str
    source_asset: str | None = None
    source_class: str | None = None


def strip_lod(name: str) -> tuple[str, int | None]:
    match = LOD_RE.search(name)
    if not match:
        return name, None
    return name[: match.start()], int(match.group(1))


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def parse_float(value: str) -> float | None:
    if value == "_" or value == "":
        return None
    return float(value)


def quat_rotate(qw: float, qx: float, qy: float, qz: float, point: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = point
    # q * p * q^-1, written out to keep this script dependency-free.
    ix = qw * x + qy * z - qz * y
    iy = qw * y + qz * x - qx * z
    iz = qw * z + qx * y - qy * x
    iw = -qx * x - qy * y - qz * z
    return (
        ix * qw + iw * -qx + iy * -qz - iz * -qy,
        iy * qw + iw * -qy + iz * -qx - ix * -qz,
        iz * qw + iw * -qz + ix * -qy - iy * -qx,
    )


def transformed_aabb(asset: dict[str, str], dims: dict[str, str]) -> tuple[float, float, float, float, float, float] | None:
    min_x = parse_float(dims["min_x"])
    max_x = parse_float(dims["max_x"])
    min_y = parse_float(dims["min_y"])
    max_y = parse_float(dims["max_y"])
    min_z = parse_float(dims["min_z"])
    max_z = parse_float(dims["max_z"])
    if None in (min_x, max_x, min_y, max_y, min_z, max_z):
        return None

    qw = float(asset["qw"])
    qx = float(asset["qx"])
    qy = float(asset["qy"])
    qz = float(asset["qz"])
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm <= 1e-9:
        qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
    else:
        qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm

    px = float(asset["px"])
    py = float(asset["py"])
    pz = float(asset["pz"])
    points = []
    for x in (min_x, max_x):
        for y in (min_y, max_y):
            for z in (min_z, max_z):
                rx, ry, rz = quat_rotate(qw, qx, qy, qz, (x, y, z))
                points.append((px + rx, py + ry, pz + rz))
    xs, ys, zs = zip(*points)
    return min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)


def should_include(asset_name: str, category: str, size: tuple[float, float, float], z_min: float, z_max: float) -> bool:
    if z_max < 0.08:
        return False
    sx, sy, sz = size
    if max(sx, sy) < 0.22 and sz < 0.35:
        return False
    if category in INCLUDED_CLASSES:
        return True
    if any(hint in asset_name for hint in STRUCTURAL_NAME_HINTS):
        return True
    return False


def terrain_kind_for(asset_name: str, category: str, size: tuple[float, float, float]) -> str:
    sx, sy, sz = size
    if category in {"wall", "door"} or any(hint in asset_name for hint in ("Block", "BuildGlass", "Wall", "Door")):
        return "wall"
    if "Column" in asset_name:
        return "support"
    if category in {"desk", "chair", "table", "cabinet", "shelves", "sofa", "counter", "sink", "toilet", "refridgerator", "television", "otherfurniture"}:
        return "furniture"
    if sz > 2.0 and min(sx, sy) < 0.7:
        return "wall"
    return "obstacle"


def is_glass_room_seal_candidate(spec: BoxSpec) -> bool:
    source = spec.source_asset or ""
    if spec.terrain_kind != "wall":
        return False
    return any(
        hint in source
        for hint in (
            "Glass",
            "DoorGlass",
            "BuildBlockDoor",
            "BlockGlass",
        )
    )


def thicken_seal_candidate(spec: BoxSpec, thickness: float = 0.24) -> BoxSpec:
    x_min, x_max = spec.x_min, spec.x_max
    y_min, y_max = spec.y_min, spec.y_max
    sx = x_max - x_min
    sy = y_max - y_min
    if sx <= sy:
        center = (x_min + x_max) * 0.5
        x_min = center - thickness * 0.5
        x_max = center + thickness * 0.5
        y_min -= 0.08
        y_max += 0.08
    else:
        center = (y_min + y_max) * 0.5
        y_min = center - thickness * 0.5
        y_max = center + thickness * 0.5
        x_min -= 0.08
        x_max += 0.08
    return BoxSpec(
        name=f"{spec.name}_sealed",
        x_min=x_min,
        x_max=x_max,
        y_min=y_min,
        y_max=y_max,
        z_min=0.0,
        z_max=max(3.0, spec.z_max),
        terrain_kind=spec.terrain_kind,
        source_asset=spec.source_asset,
        source_class=spec.source_class,
    )


def add_collinear_seal_gaps(boxes: list[BoxSpec], max_gap: float = 1.35, thickness: float = 0.24) -> list[BoxSpec]:
    candidates = [box for box in boxes if is_glass_room_seal_candidate(box)]
    additions: list[BoxSpec] = []

    def add_gap_fillers(grouped: dict[float, list[BoxSpec]], axis: str) -> None:
        nonlocal additions
        for key, group in grouped.items():
            if len(group) < 2:
                continue
            if axis == "x":
                group = sorted(group, key=lambda item: item.y_min)
            else:
                group = sorted(group, key=lambda item: item.x_min)
            prev = group[0]
            for cur in group[1:]:
                if axis == "x":
                    gap = cur.y_min - prev.y_max
                    if 0.01 < gap <= max_gap:
                        additions.append(
                            BoxSpec(
                                name=f"office_glass_gap_seal_{len(additions) + 1:03d}",
                                x_min=key - thickness * 0.5,
                                x_max=key + thickness * 0.5,
                                y_min=prev.y_max,
                                y_max=cur.y_min,
                                z_min=0.0,
                                z_max=max(3.0, prev.z_max, cur.z_max),
                                terrain_kind="wall",
                                source_asset="generated_glass_gap_seal",
                                source_class="wall",
                            )
                        )
                    if cur.y_max > prev.y_max:
                        prev = cur
                else:
                    gap = cur.x_min - prev.x_max
                    if 0.01 < gap <= max_gap:
                        additions.append(
                            BoxSpec(
                                name=f"office_glass_gap_seal_{len(additions) + 1:03d}",
                                x_min=prev.x_max,
                                x_max=cur.x_min,
                                y_min=key - thickness * 0.5,
                                y_max=key + thickness * 0.5,
                                z_min=0.0,
                                z_max=max(3.0, prev.z_max, cur.z_max),
                                terrain_kind="wall",
                                source_asset="generated_glass_gap_seal",
                                source_class="wall",
                            )
                        )
                    if cur.x_max > prev.x_max:
                        prev = cur

    vertical: dict[float, list[BoxSpec]] = {}
    horizontal: dict[float, list[BoxSpec]] = {}
    for spec in candidates:
        sx = spec.x_max - spec.x_min
        sy = spec.y_max - spec.y_min
        if sx <= sy:
            vertical.setdefault(round(((spec.x_min + spec.x_max) * 0.5) / 0.25) * 0.25, []).append(spec)
        else:
            horizontal.setdefault(round(((spec.y_min + spec.y_max) * 0.5) / 0.25) * 0.25, []).append(spec)
    add_gap_fillers(vertical, "x")
    add_gap_fillers(horizontal, "y")
    return additions


def seal_glass_rooms_for_lidar(boxes: list[BoxSpec]) -> list[BoxSpec]:
    sealed: list[BoxSpec] = []
    for spec in boxes:
        if is_glass_room_seal_candidate(spec):
            sealed.append(thicken_seal_candidate(spec))
        else:
            sealed.append(spec)
    sealed.extend(add_collinear_seal_gaps(sealed))
    return sealed


def fit_floor_to_wall_bounds(boxes: list[BoxSpec]) -> list[BoxSpec]:
    if not boxes:
        return boxes
    floor = boxes[0]
    structural_walls = [
        spec
        for spec in boxes[1:]
        if spec.terrain_kind == "wall"
        and "RedFrame" not in (spec.source_asset or "")
        and spec.z_max - spec.z_min >= 2.0
        and max(spec.x_max - spec.x_min, spec.y_max - spec.y_min) >= 1.5
    ]
    if not structural_walls:
        return boxes
    vertical_walls = [
        spec
        for spec in structural_walls
        if spec.x_max - spec.x_min <= 0.8 and spec.y_max - spec.y_min >= 1.5
    ]
    horizontal_walls = [
        spec
        for spec in structural_walls
        if spec.y_max - spec.y_min <= 0.8 and spec.x_max - spec.x_min >= 1.5
    ]
    x_walls = vertical_walls or structural_walls
    y_walls = horizontal_walls or structural_walls
    fitted_floor = BoxSpec(
        name=floor.name,
        x_min=min(spec.x_min for spec in x_walls),
        x_max=max(spec.x_max for spec in x_walls),
        y_min=min(spec.y_min for spec in y_walls),
        y_max=max(spec.y_max for spec in y_walls),
        z_min=floor.z_min,
        z_max=floor.z_max,
        terrain_kind=floor.terrain_kind,
        source_asset="wall_fitted_traversable_area",
        source_class=floor.source_class,
    )
    return [fitted_floor] + boxes[1:]


def offset_box(spec: BoxSpec, name: str, dx: float, dy: float, dz: float, terrain_kind: str | None = None) -> BoxSpec:
    return BoxSpec(
        name=name,
        x_min=spec.x_min + dx,
        x_max=spec.x_max + dx,
        y_min=spec.y_min + dy,
        y_max=spec.y_max + dy,
        z_min=spec.z_min + dz,
        z_max=spec.z_max + dz,
        terrain_kind=terrain_kind or spec.terrain_kind,
        source_asset=spec.source_asset,
        source_class=spec.source_class,
    )


def overlaps_xy(spec: BoxSpec, x_min: float, x_max: float, y_min: float, y_max: float) -> bool:
    return spec.x_min < x_max and spec.x_max > x_min and spec.y_min < y_max and spec.y_max > y_min


def subtract_xy_rect(spec: BoxSpec, zone: tuple[float, float, float, float], min_size: float = 0.05) -> list[BoxSpec]:
    if not overlaps_xy(spec, *zone):
        return [spec]
    zx0, zx1, zy0, zy1 = zone
    ix0 = max(spec.x_min, zx0)
    ix1 = min(spec.x_max, zx1)
    iy0 = max(spec.y_min, zy0)
    iy1 = min(spec.y_max, zy1)
    pieces: list[BoxSpec] = []

    def add_piece(name_suffix: str, x0: float, x1: float, y0: float, y1: float) -> None:
        if x1 - x0 < min_size or y1 - y0 < min_size:
            return
        pieces.append(
            BoxSpec(
                name=f"{spec.name}_{name_suffix}",
                x_min=x0,
                x_max=x1,
                y_min=y0,
                y_max=y1,
                z_min=spec.z_min,
                z_max=spec.z_max,
                terrain_kind=spec.terrain_kind,
                source_asset=spec.source_asset,
                source_class=spec.source_class,
            )
        )

    add_piece("left", spec.x_min, ix0, spec.y_min, spec.y_max)
    add_piece("right", ix1, spec.x_max, spec.y_min, spec.y_max)
    add_piece("front", ix0, ix1, spec.y_min, iy0)
    add_piece("back", ix0, ix1, iy1, spec.y_max)
    return pieces


def carve_connector_entry(
    boxes: list[BoxSpec],
    zones: list[tuple[float, float, float, float]],
) -> tuple[list[BoxSpec], int, int]:
    cleared: list[BoxSpec] = []
    removed = 0
    modified = 0
    protected = {"floor", "floor_2", "stair_step"}
    for spec in boxes:
        if spec.terrain_kind in protected:
            cleared.append(spec)
            continue
        pieces = [spec]
        touched = False
        for zone in zones:
            next_pieces: list[BoxSpec] = []
            for piece in pieces:
                if overlaps_xy(piece, *zone):
                    touched = True
                    next_pieces.extend(subtract_xy_rect(piece, zone))
                else:
                    next_pieces.append(piece)
            pieces = next_pieces
        if not pieces:
            removed += 1
        else:
            if touched:
                modified += 1
            cleared.extend(pieces)
    return cleared, removed, modified


def add_perimeter_walls(
    boxes: list[BoxSpec],
    floor: BoxSpec,
    prefix: str,
    z_min: float,
    z_max: float,
    north_opening: tuple[float, float] | None = None,
    south_opening: tuple[float, float] | None = None,
) -> list[BoxSpec]:
    thickness = DEFAULT_PERIMETER_WALL_THICKNESS
    walls: list[BoxSpec] = [
        BoxSpec(
            name=f"{prefix}_west_perimeter_wall",
            x_min=floor.x_min - thickness,
            x_max=floor.x_min,
            y_min=floor.y_min,
            y_max=floor.y_max,
            z_min=z_min,
            z_max=z_max,
            terrain_kind="wall",
            source_asset="generated_perimeter_wall",
            source_class="wall",
        ),
        BoxSpec(
            name=f"{prefix}_east_perimeter_wall",
            x_min=floor.x_max,
            x_max=floor.x_max + thickness,
            y_min=floor.y_min,
            y_max=floor.y_max,
            z_min=z_min,
            z_max=z_max,
            terrain_kind="wall",
            source_asset="generated_perimeter_wall",
            source_class="wall",
        ),
    ]

    def add_horizontal_segments(name: str, y_min: float, y_max: float, opening: tuple[float, float] | None) -> None:
        if opening is None:
            segments = [(floor.x_min, floor.x_max)]
        else:
            open_min = max(floor.x_min, opening[0])
            open_max = min(floor.x_max, opening[1])
            segments = [(floor.x_min, open_min), (open_max, floor.x_max)]
        for index, (x_min, x_max) in enumerate(segments, 1):
            if x_max - x_min <= 0.05:
                continue
            walls.append(
                BoxSpec(
                    name=f"{prefix}_{name}_perimeter_wall_{index}",
                    x_min=x_min,
                    x_max=x_max,
                    y_min=y_min,
                    y_max=y_max,
                    z_min=z_min,
                    z_max=z_max,
                    terrain_kind="wall",
                    source_asset="generated_perimeter_wall",
                    source_class="wall",
                )
            )

    add_horizontal_segments("south", floor.y_min - thickness, floor.y_min, south_opening)
    add_horizontal_segments("north", floor.y_max, floor.y_max + thickness, north_opening)
    return boxes + walls


def add_shifted_second_floor(
    boxes: list[BoxSpec],
    height: float = DEFAULT_SECOND_FLOOR_HEIGHT,
) -> tuple[list[BoxSpec], dict[str, object], list[dict[str, object]]]:
    floor = boxes[0]
    x_offset = 0.0
    y_offset = 0.0
    stair_count = int(round(height / DEFAULT_STAIR_STEP_HEIGHT))
    step_height = height / stair_count
    steps_per_flight = stair_count // 2
    flight_rise = height * 0.5
    flight_length = steps_per_flight * DEFAULT_STAIR_TREAD

    upper_boxes: list[BoxSpec] = []
    for spec in boxes:
        kind = "floor_2" if spec.terrain_kind == "floor" else spec.terrain_kind
        upper_boxes.append(offset_box(spec, f"floor2_{spec.name}", x_offset, y_offset, height, kind))

    upper_floor = upper_boxes[0]
    stair_x_center = max(floor.x_min + 4.0, min(floor.x_max - 4.0, floor.x_min + 0.35 * (floor.x_max - floor.x_min)))
    flight_gap = 0.55
    lower_flight_x_center = stair_x_center - (DEFAULT_STAIR_FLIGHT_WIDTH + flight_gap) * 0.5
    upper_flight_x_center = stair_x_center + (DEFAULT_STAIR_FLIGHT_WIDTH + flight_gap) * 0.5
    lower_flight_x_min = lower_flight_x_center - DEFAULT_STAIR_FLIGHT_WIDTH * 0.5
    lower_flight_x_max = lower_flight_x_center + DEFAULT_STAIR_FLIGHT_WIDTH * 0.5
    upper_flight_x_min = upper_flight_x_center - DEFAULT_STAIR_FLIGHT_WIDTH * 0.5
    upper_flight_x_max = upper_flight_x_center + DEFAULT_STAIR_FLIGHT_WIDTH * 0.5
    stair_x_min = lower_flight_x_min
    stair_x_max = upper_flight_x_max
    stair_y_start = floor.y_max
    landing_y_min = stair_y_start + flight_length
    landing_y_max = landing_y_min + DEFAULT_STAIR_LANDING_DEPTH
    upper_y_end = stair_y_start
    stair_y_end = landing_y_max
    clearance_margin = 0.12
    clearance_depth = 5.2
    clearance_x_min = stair_x_min - clearance_margin
    clearance_x_max = stair_x_max + clearance_margin
    lower_clearance = (clearance_x_min, clearance_x_max, floor.y_max - clearance_depth, stair_y_start + 0.35)
    upper_clearance = (clearance_x_min, clearance_x_max, floor.y_max - clearance_depth, stair_y_start + 0.35)
    boxes, lower_removed, lower_modified = carve_connector_entry(boxes, [lower_clearance])
    upper_boxes, upper_removed, upper_modified = carve_connector_entry(upper_boxes, [upper_clearance])
    lower_stair_opening = (clearance_x_min, clearance_x_max)
    upper_stair_opening = (clearance_x_min, clearance_x_max)
    boxes = add_perimeter_walls(
        boxes,
        floor,
        "office_floor1",
        0.0,
        DEFAULT_PERIMETER_WALL_HEIGHT,
        north_opening=lower_stair_opening,
    )
    upper_boxes = add_perimeter_walls(
        upper_boxes,
        upper_floor,
        "office_floor2",
        height,
        height + DEFAULT_PERIMETER_WALL_HEIGHT,
        north_opening=upper_stair_opening,
    )

    connector_boxes: list[BoxSpec] = []
    up_route: list[dict[str, float | str]] = []
    for index in range(steps_per_flight):
        y0 = stair_y_start + index * DEFAULT_STAIR_TREAD
        y1 = y0 + DEFAULT_STAIR_TREAD
        tread_height = (index + 1) * step_height
        connector_boxes.append(
            BoxSpec(
                name=f"office_switchback_stair_lower_step_{index + 1:02d}",
                x_min=lower_flight_x_min,
                x_max=lower_flight_x_max,
                y_min=y0,
                y_max=y1,
                z_min=0.0,
                z_max=tread_height,
                terrain_kind="stair_step",
                source_asset="generated_second_floor_connector",
                source_class="stairs",
            )
        )
        up_route.append(
            {
                "name": f"office_switchback_lower_step_{index + 1:02d}",
                "x": lower_flight_x_center,
                "y": (y0 + y1) * 0.5,
                "z": tread_height,
            }
        )
    connector_boxes.append(
        BoxSpec(
            name="office_switchback_stair_mid_landing",
            x_min=stair_x_min,
            x_max=stair_x_max,
            y_min=landing_y_min,
            y_max=landing_y_max,
            z_min=0.0,
            z_max=flight_rise,
            terrain_kind="stair_step",
            source_asset="generated_second_floor_connector",
            source_class="stairs",
        )
    )
    up_route.append(
        {
            "name": "office_switchback_mid_landing",
            "x": stair_x_center,
            "y": (landing_y_min + landing_y_max) * 0.5,
            "z": flight_rise,
        }
    )
    for index in range(steps_per_flight):
        y0 = landing_y_min - (index + 1) * DEFAULT_STAIR_TREAD
        y1 = landing_y_min - index * DEFAULT_STAIR_TREAD
        tread_height = flight_rise + (index + 1) * step_height
        connector_boxes.append(
            BoxSpec(
                name=f"office_switchback_stair_upper_step_{index + 1:02d}",
                x_min=upper_flight_x_min,
                x_max=upper_flight_x_max,
                y_min=y0,
                y_max=y1,
                z_min=0.0,
                z_max=tread_height,
                terrain_kind="stair_step",
                source_asset="generated_second_floor_connector",
                source_class="stairs",
            )
        )
        up_route.append(
            {
                "name": f"office_switchback_upper_step_{index + 1:02d}",
                "x": upper_flight_x_center,
                "y": (y0 + y1) * 0.5,
                "z": tread_height,
            }
        )
    rail_specs = [
        ("lower_left", lower_flight_x_min - DEFAULT_STAIR_RAIL_THICKNESS, lower_flight_x_min, stair_y_start, landing_y_min, 0.0, flight_rise + DEFAULT_STAIR_RAIL_HEIGHT),
        ("upper_right", upper_flight_x_max, upper_flight_x_max + DEFAULT_STAIR_RAIL_THICKNESS, upper_y_end, landing_y_min, flight_rise, height + DEFAULT_STAIR_RAIL_HEIGHT),
        ("landing_north", stair_x_min - DEFAULT_STAIR_RAIL_THICKNESS, stair_x_max + DEFAULT_STAIR_RAIL_THICKNESS, landing_y_max, landing_y_max + DEFAULT_STAIR_RAIL_THICKNESS, flight_rise, flight_rise + DEFAULT_STAIR_RAIL_HEIGHT),
    ]
    for name, x_min, x_max, y_min, y_max, z_min, z_max in rail_specs:
        connector_boxes.append(
            BoxSpec(
                name=f"office_switchback_stair_rail_{name}",
                x_min=x_min,
                x_max=x_max,
                y_min=y_min,
                y_max=y_max,
                z_min=z_min,
                z_max=z_max,
                terrain_kind="stair_rail",
                source_asset="generated_second_floor_connector",
                source_class="railing",
            )
        )
    lower_entry = [lower_flight_x_center, stair_y_start, 0.0]
    upper_entry = [upper_flight_x_center, upper_y_end, height]
    up_route.append({"name": "office_floor2_entry", "x": upper_entry[0], "y": upper_entry[1], "z": upper_entry[2]})
    feature_metadata = {
        "floor_2": {
            "count": 1,
            "height_m": height,
            "xy_offset_m": [x_offset, y_offset],
            "connected_by": "office_second_floor_staircase",
            "alignment": "stacked_floor_with_original_north_stair_location",
        },
        "stairs": {
            "flight_count": 2,
            "staircase_count": 1,
            "total_steps": stair_count,
            "steps_per_flight": steps_per_flight,
            "step_height_m": step_height,
            "tread_depth_m": DEFAULT_STAIR_TREAD,
            "width_m": DEFAULT_STAIR_FLIGHT_WIDTH,
            "landing_depth_m": DEFAULT_STAIR_LANDING_DEPTH,
            "low_rail_height_m": DEFAULT_STAIR_RAIL_HEIGHT,
            "low_rail_thickness_m": DEFAULT_STAIR_RAIL_THICKNESS,
        },
        "connector_clearance": {
            "lower_zone_xy": list(lower_clearance),
            "upper_zone_xy": list(upper_clearance),
            "removed_lower_blockers": lower_removed,
            "removed_upper_blockers": upper_removed,
            "modified_lower_blockers": lower_modified,
            "modified_upper_blockers": upper_modified,
        },
    }
    connectors = [
        {
            "name": "office_second_floor_staircase",
            "type": "exterior_switchback_stair_connector",
            "lower_entry": lower_entry,
            "upper_entry": upper_entry,
            "direction": "switchback",
            "step_count": stair_count,
            "steps_per_flight": steps_per_flight,
            "step_height_m": step_height,
            "tread_depth_m": DEFAULT_STAIR_TREAD,
            "up_route": up_route,
        }
    ]
    return boxes + connector_boxes + upper_boxes, feature_metadata, connectors


def load_traversable_bounds(path: Path) -> tuple[float, float, float, float]:
    xs: list[float] = []
    ys: list[float] = []
    with path.open("r", encoding="ascii", errors="ignore") as handle:
        for line in handle:
            if line.strip() == "end_header":
                break
        for line in handle:
            parts = line.split()
            if len(parts) < 3:
                continue
            xs.append(float(parts[0]))
            ys.append(float(parts[1]))
    if not xs:
        raise RuntimeError(f"No points found in {path}")
    return min(xs), max(xs), min(ys), max(ys)


def build_boxes(unity_dir: Path, two_floor: bool = True) -> tuple[list[BoxSpec], dict[str, object], list[dict[str, object]]]:
    env_dir = unity_dir / "environment"
    assets = read_csv(env_dir / "AssetList.csv")
    dims_by_name = {row["name"]: row for row in read_csv(env_dir / "Dimensions.csv")}
    categories = {row["name"]: row.get("nyu40class", "") for row in read_csv(env_dir / "Categories.csv")}

    trav_x_min, trav_x_max, trav_y_min, trav_y_max = load_traversable_bounds(unity_dir / "traversable_area.ply")
    boxes: list[BoxSpec] = [
        BoxSpec(
            name="office_floor",
            x_min=math.floor(trav_x_min) - 0.5,
            x_max=math.ceil(trav_x_max) + 0.5,
            y_min=math.floor(trav_y_min) - 0.5,
            y_max=math.ceil(trav_y_max) + 0.5,
            z_min=-0.08,
            z_max=0.0,
            terrain_kind="floor",
            source_asset="traversable_area",
            source_class="floor",
        )
    ]
    seen: set[tuple[str, str, str, str, str, str, str, str]] = set()
    asset_index = 0
    for asset in assets:
        asset_name = asset["name"]
        base_name, lod = strip_lod(asset_name)
        if lod not in (None, 0):
            continue
        dims = dims_by_name.get(asset_name)
        if dims is None:
            dims = dims_by_name.get(base_name)
        if dims is None:
            continue
        category = categories.get(asset_name, categories.get(base_name, ""))
        aabb = transformed_aabb(asset, dims)
        if aabb is None:
            continue
        x_min, x_max, y_min, y_max, z_min, z_max = aabb
        size = (x_max - x_min, y_max - y_min, z_max - z_min)
        dedup_key = (
            base_name,
            f"{float(asset['px']):.3f}",
            f"{float(asset['py']):.3f}",
            f"{float(asset['pz']):.3f}",
            f"{float(asset['qw']):.3f}",
            f"{float(asset['qx']):.3f}",
            f"{float(asset['qy']):.3f}",
            f"{float(asset['qz']):.3f}",
        )
        if dedup_key in seen:
            continue
        seen.add(dedup_key)
        if not should_include(asset_name, category, size, z_min, z_max):
            continue

        # Keep high ceiling decorations out of robot lidar/collision tests for now.
        if z_min > 3.2 and category not in {"wall", "door"}:
            continue
        z_min = max(0.0, z_min)
        if z_max <= z_min + 0.05:
            continue
        asset_index += 1
        terrain_kind = terrain_kind_for(asset_name, category, size)
        boxes.append(
            BoxSpec(
                name=f"office_{terrain_kind}_{asset_index:04d}",
                x_min=x_min,
                x_max=x_max,
                y_min=y_min,
                y_max=y_max,
                z_min=z_min,
                z_max=z_max,
                terrain_kind=terrain_kind,
                source_asset=asset_name,
                source_class=category,
            )
        )
    boxes = fit_floor_to_wall_bounds(seal_glass_rooms_for_lidar(boxes))
    if two_floor:
        return add_shifted_second_floor(boxes)
    return (
        boxes,
        {
            "floor_2": {"count": 0, "height_m": 0.0, "xy_offset_m": [0.0, 0.0], "connected_by": ""},
            "stairs": {"flight_count": 0, "staircase_count": 0, "total_steps": 0},
        },
        [],
    )


def format_vertex(vertex: tuple[float, float, float]) -> str:
    return f"v {vertex[0]:.6f} {vertex[1]:.6f} {vertex[2]:.6f}\n"


def write_box(lines: list[str], vertex_offset: int, spec: BoxSpec) -> int:
    x0, x1 = spec.x_min, spec.x_max
    y0, y1 = spec.y_min, spec.y_max
    z0, z1 = spec.z_min, spec.z_max
    faces = [
        (((x0, y0, z0), (x0, y1, z0), (x1, y1, z0), (x1, y0, z0)), (0.0, 0.0, -1.0)),
        (((x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)), (0.0, 0.0, 1.0)),
        (((x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)), (0.0, -1.0, 0.0)),
        (((x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)), (1.0, 0.0, 0.0)),
        (((x1, y1, z0), (x0, y1, z0), (x0, y1, z1), (x1, y1, z1)), (0.0, 1.0, 0.0)),
        (((x0, y1, z0), (x0, y0, z0), (x0, y0, z1), (x0, y1, z1)), (-1.0, 0.0, 0.0)),
    ]
    material = {
        "floor": "floor",
        "floor_2": "floor_2",
        "wall": "wall",
        "support": "support",
        "furniture": "furniture",
        "stair_step": "stair",
        "stair_rail": "wall",
    }.get(spec.terrain_kind, "obstacle")
    lines.append(f"o {spec.name}\n")
    lines.append(f"usemtl {material}\n")
    triangles: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = []
    for quad, normal in faces:
        a, b, c, d = quad
        triangles.extend(((a, normal), (b, normal), (c, normal), (a, normal), (c, normal), (d, normal)))
    for vertex, _normal in triangles:
        lines.append(format_vertex(vertex))
    for _vertex, normal in triangles:
        lines.append(f"vn {normal[0]:.6f} {normal[1]:.6f} {normal[2]:.6f}\n")
    for index in range(0, len(triangles), 3):
        a = vertex_offset + index + 1
        b = vertex_offset + index + 2
        c = vertex_offset + index + 3
        lines.append(f"f {a}//{a} {b}//{b} {c}//{c}\n")
    return vertex_offset + len(triangles)


def frange(start: float, stop: float, step: float) -> Iterable[float]:
    cur = start
    while cur <= stop + step * 0.5:
        yield round(cur, 6)
        cur += step


def sample_box(points: set[tuple[float, float, float]], spec: BoxSpec, step: float) -> None:
    xs = list(frange(spec.x_min, spec.x_max, step))
    ys = list(frange(spec.y_min, spec.y_max, step))
    zs = list(frange(spec.z_min, spec.z_max, step))
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


def write_ply(path: Path, boxes: list[BoxSpec], step: float) -> int:
    points: set[tuple[float, float, float]] = set()
    for spec in boxes:
        sample_box(points, spec, step)
    lines = [
        "ply\n",
        "format ascii 1.0\n",
        f"element vertex {len(points)}\n",
        "property float x\n",
        "property float y\n",
        "property float z\n",
        "end_header\n",
    ]
    for x, y, z in sorted(points):
        lines.append(f"{x:.6f} {y:.6f} {z:.6f}\n")
    path.write_text("".join(lines), encoding="ascii")
    return len(points)


def write_boundary(path: Path, boxes: list[BoxSpec], margin: float = 0.7) -> None:
    floors = [box for box in boxes if box.terrain_kind in {"floor", "floor_2"}]
    if not floors:
        floors = [boxes[0]]
    x0 = min(box.x_min for box in floors) + margin
    x1 = max(box.x_max for box in floors) - margin
    y0 = min(box.y_min for box in floors) + margin
    y1 = max(box.y_max for box in floors) - margin
    stair_steps = [box for box in boxes if box.terrain_kind == "stair_step"]
    if stair_steps:
        stair_x0 = max(x0, min(box.x_min for box in stair_steps) + margin * 0.25)
        stair_x1 = min(x1, max(box.x_max for box in stair_steps) - margin * 0.25)
        stair_y1 = max(box.y_max for box in stair_steps) - margin
    else:
        stair_x0 = stair_x1 = stair_y1 = y1
    if stair_steps and stair_y1 > y1 and stair_x1 - stair_x0 > 0.2:
        main_y1 = min(y1, y0 + 0.50 * (y1 - y0))
        transition_y = min(y1, y0 + 0.73 * (y1 - y0))
        corridor_x1 = min(x1, stair_x1 + 1.25)
        notch_x = min(x1, max(corridor_x1 + 5.0, x0 + 0.52 * (x1 - x0)))
        vertices = [
            (x0, y0, 0.0),
            (x1, y0, 0.0),
            (x1, main_y1, 0.0),
            (notch_x, main_y1, 0.0),
            (notch_x, transition_y, 0.0),
            (corridor_x1, transition_y, 0.0),
            (corridor_x1, y1, 0.0),
            (stair_x1, y1, 0.0),
            (stair_x1, stair_y1, 0.0),
            (stair_x0, stair_y1, 0.0),
            (stair_x0, y1, 0.0),
            (x0, y1, 0.0),
            (x0, y0, 0.0),
        ]
    else:
        vertices = [
            (x0, y0, 0.0),
            (x1, y0, 0.0),
            (x1, y1, 0.0),
            (x0, y1, 0.0),
            (x0, y0, 0.0),
        ]
    path.write_text(
        "\n".join(
            [
                "ply",
                "format ascii 1.0",
                f"element vertex {len(vertices)}",
                "property float x",
                "property float y",
                "property float z",
                "end_header",
            ]
            + [f"{x:.3f} {y:.3f} {z:.1f}" for x, y, z in vertices]
        )
        + "\n",
        encoding="ascii",
    )


def write_scene(output_path: Path, unity_dir: Path, map_step: float, two_floor: bool = True) -> dict[str, object]:
    boxes, terrain_features, connectors = build_boxes(unity_dir, two_floor=two_floor)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Simplified Gazebo office scene generated from Unity metadata\n",
        "# Units: meters\n",
        f"mtllib {output_path.with_suffix('.mtl').name}\n",
    ]
    vertex_offset = 0
    for spec in boxes:
        vertex_offset = write_box(lines, vertex_offset, spec)
    output_path.write_text("".join(lines), encoding="ascii")
    output_path.with_suffix(".mtl").write_text(
        "\n".join(
            [
                "newmtl floor",
                "Ka 0.55 0.57 0.55",
                "Kd 0.55 0.57 0.55",
                "d 1.0",
                "Tr 0.0",
                "",
                "newmtl floor_2",
                "Ka 0.40 0.58 0.72",
                "Kd 0.40 0.58 0.72",
                "d 1.0",
                "Tr 0.0",
                "",
                "newmtl stair",
                "Ka 0.76 0.50 0.26",
                "Kd 0.76 0.50 0.26",
                "d 1.0",
                "Tr 0.0",
                "",
                "newmtl wall",
                "Ka 0.68 0.66 0.59",
                "Kd 0.68 0.66 0.59",
                "d 1.0",
                "Tr 0.0",
                "",
                "newmtl furniture",
                "Ka 0.54 0.36 0.22",
                "Kd 0.54 0.36 0.22",
                "d 1.0",
                "Tr 0.0",
                "",
                "newmtl support",
                "Ka 0.32 0.34 0.34",
                "Kd 0.32 0.34 0.34",
                "d 1.0",
                "Tr 0.0",
                "",
                "newmtl obstacle",
                "Ka 0.72 0.30 0.20",
                "Kd 0.72 0.30 0.20",
                "d 1.0",
                "Tr 0.0",
                "",
            ]
        ),
        encoding="ascii",
    )
    point_count = write_ply(output_path.with_name("map.ply"), boxes, map_step)
    write_boundary(output_path.with_name("boundary.ply"), boxes)
    metadata = {
        "scene_type": "unity_office_box_approximation_two_floor" if two_floor else "unity_office_box_approximation",
        "source": str(unity_dir),
        "floor_size_m": [boxes[0].x_max - boxes[0].x_min, boxes[0].y_max - boxes[0].y_min],
        "terrain_features": terrain_features,
        "objects": [asdict(spec) for spec in boxes],
        "connectors": connectors,
        "sampled_map_points": point_count,
    }
    output_path.with_suffix(".json").write_text(json.dumps(metadata, indent=2), encoding="ascii")
    return metadata


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate a simplified Gazebo office scene from Unity metadata.")
    parser.add_argument(
        "-o",
        "--output",
        default="src/base_autonomy/vehicle_simulator/mesh/unity_office_gazebo/unity_office_gazebo.obj",
        help="Output OBJ path.",
    )
    parser.add_argument(
        "--unity-dir",
        default="src/base_autonomy/vehicle_simulator/mesh/unity",
        help="Unity mesh metadata directory.",
    )
    parser.add_argument("--map-step", type=float, default=0.20, help="Point sampling step for generated map.ply.")
    parser.add_argument(
        "--single-floor",
        action="store_true",
        help="Generate only the original single-floor office approximation.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    metadata = write_scene(Path(args.output), Path(args.unity_dir), args.map_step, two_floor=not args.single_floor)
    print(f"Office OBJ written to: {args.output}")
    print(f"Office metadata written to: {Path(args.output).with_suffix('.json')}")
    print(f"Office PLY map written to: {Path(args.output).with_name('map.ply')}")
    print(f"Office boundary written to: {Path(args.output).with_name('boundary.ply')}")
    print(f"Objects: {len(metadata['objects'])}")
    print(f"Sampled map points: {metadata['sampled_map_points']}")
    print(f"Floor size: {metadata['floor_size_m'][0]:.1f}m x {metadata['floor_size_m'][1]:.1f}m")
    floor_2 = metadata["terrain_features"]["floor_2"]
    stairs = metadata["terrain_features"]["stairs"]
    print(f"Second floor count: {floor_2['count']}")
    if floor_2["count"]:
        print(f"Second floor offset: {floor_2['xy_offset_m']} at z={floor_2['height_m']:.2f}m")
        print(f"Stairs: {stairs['total_steps']} steps, rise={stairs['step_height_m']:.3f}m, tread={stairs['tread_depth_m']:.3f}m")


if __name__ == "__main__":
    main()
