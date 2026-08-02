#!/usr/bin/env python3
"""Export ROS 2 bags into CSV files with a units row.

Each selected topic is written to a separate CSV file:
- row 1: column names
- row 2: units
- row 3+: data values
"""

from __future__ import annotations

import argparse
import array
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export ROS 2 bag topics to CSV with units on row 2.")
    parser.add_argument("bag", type=Path, help="Path to bag directory or bag file (.db3/.mcap)")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("csv_export"),
        help="Directory where CSV files are written (default: ./csv_export)",
    )
    parser.add_argument(
        "--topics",
        nargs="*",
        default=None,
        help="Optional topic allowlist. If omitted, export all topics.",
    )
    parser.add_argument(
        "--storage-id",
        default=None,
        help="Bag storage plugin ID (e.g., sqlite3, mcap). Auto-detected when omitted.",
    )
    parser.add_argument(
        "--units-json",
        type=Path,
        default=None,
        help=(
            "Optional JSON file for unit overrides. Supports either: "
            "{\"field\":\"unit\"} or {\"/topic\":{\"field\":\"unit\"}}"
        ),
    )
    return parser.parse_args()


def detect_storage_id(bag_path: Path) -> str:
    if bag_path.is_file():
        suffix = bag_path.suffix.lower()
        if suffix == ".db3":
            return "sqlite3"
        if suffix == ".mcap":
            return "mcap"

    if bag_path.is_dir():
        if any(p.suffix == ".db3" for p in bag_path.glob("*.db3")):
            return "sqlite3"
        if any(p.suffix == ".mcap" for p in bag_path.glob("*.mcap")):
            return "mcap"

    raise ValueError(
        "Unable to detect storage ID. Pass --storage-id explicitly (e.g., sqlite3 or mcap)."
    )


def sanitize_topic_name(topic: str) -> str:
    return topic.strip("/").replace("/", "__") or "root"


def primitive_value(value: Any) -> bool:
    return isinstance(value, (str, bool, int, float, bytes, bytearray))


def normalize_scalar(value: Any) -> Any:
    if isinstance(value, bytes):
        return value.hex()
    if isinstance(value, bytearray):
        return bytes(value).hex()
    if isinstance(value, float) and (math.isnan(value) or math.isinf(value)):
        return str(value)
    return value


def flatten_ros_message(msg: Any, prefix: str = "") -> dict[str, Any]:
    out: dict[str, Any] = {}

    if hasattr(msg, "get_fields_and_field_types"):
        for field_name in msg.get_fields_and_field_types().keys():
            value = getattr(msg, field_name)
            path = f"{prefix}.{field_name}" if prefix else field_name
            out.update(flatten_ros_message(value, path))
        return out

    if primitive_value(msg):
        out[prefix] = normalize_scalar(msg)
        return out

    if isinstance(msg, array.array):
        out[prefix] = json.dumps(list(msg))
        return out

    if isinstance(msg, (list, tuple)):
        if all(primitive_value(v) for v in msg):
            out[prefix] = json.dumps([normalize_scalar(v) for v in msg])
        else:
            out[prefix] = json.dumps([str(v) for v in msg])
        return out

    out[prefix] = str(msg)
    return out


def load_unit_overrides(path: Path | None) -> dict[str, Any]:
    if path is None:
        return {}
    return json.loads(path.read_text())


def guess_unit(topic: str, field: str, overrides: dict[str, Any]) -> str:
    topic_overrides = overrides.get(topic, {}) if isinstance(overrides.get(topic, {}), dict) else {}
    if field in topic_overrides:
        return str(topic_overrides[field])
    if field in overrides and not isinstance(overrides[field], dict):
        return str(overrides[field])

    field_lower = field.lower()

    if field == "time_ns":
        return "ns"
    if field == "time_s":
        return "s"

    rules = [
        (("temperature", "temp"), "°C"),
        (("pressure", "fluid_pressure"), "Pa"),
        (("depth",), "m"),
        (("voltage",), "V"),
        (("current",), "A"),
        (("torque",), "N·m"),
        (("position",), "rad"),
        (("velocity", "angular_velocity"), "rad/s"),
        (("linear_acceleration", "accel"), "m/s²"),
        (("latitude", "longitude"), "deg"),
        (("altitude",), "m"),
        (("orientation", "quaternion"), "unitless"),
    ]
    for keys, unit in rules:
        if any(k in field_lower for k in keys):
            return unit

    return ""


def write_topic_csv(
    topic: str,
    rows: list[dict[str, Any]],
    output_dir: Path,
    unit_overrides: dict[str, Any],
) -> Path:
    all_columns: list[str] = []
    seen = set()
    for row in rows:
        for key in row.keys():
            if key not in seen:
                seen.add(key)
                all_columns.append(key)

    all_columns = [c for c in all_columns if c not in {"time_ns", "time_s"}]
    columns = ["time_ns", "time_s", *all_columns]

    output_dir.mkdir(parents=True, exist_ok=True)
    out_file = output_dir / f"{sanitize_topic_name(topic)}.csv"

    with out_file.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(columns)
        writer.writerow([guess_unit(topic, c, unit_overrides) for c in columns])
        for row in rows:
            writer.writerow([row.get(c, "") for c in columns])

    return out_file


def main() -> None:
    args = parse_args()
    bag_path = args.bag
    storage_id = args.storage_id or detect_storage_id(bag_path)
    unit_overrides = load_unit_overrides(args.units_json)
    topic_filter = set(args.topics) if args.topics else None

    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topics_and_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics_and_types}

    rows_by_topic: dict[str, list[dict[str, Any]]] = defaultdict(list)
    msg_class_cache: dict[str, Any] = {}

    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        if topic_filter and topic_name not in topic_filter:
            continue

        msg_type = type_map.get(topic_name)
        if msg_type is None:
            continue

        if msg_type not in msg_class_cache:
            msg_class_cache[msg_type] = get_message(msg_type)

        msg = deserialize_message(data, msg_class_cache[msg_type])
        row = flatten_ros_message(msg)
        row["time_ns"] = timestamp
        row["time_s"] = timestamp / 1e9
        rows_by_topic[topic_name].append(row)

    if not rows_by_topic:
        print("No messages found for requested topics.")
        return

    output_files = []
    for topic, rows in sorted(rows_by_topic.items()):
        output_files.append(write_topic_csv(topic, rows, args.output_dir, unit_overrides))

    print("Wrote CSV files:")
    for path in output_files:
        print(f"  - {path}")


if __name__ == "__main__":
    main()
