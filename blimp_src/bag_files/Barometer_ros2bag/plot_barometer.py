#!/usr/bin/env python3
"""
Plot barometer height from a ROS 2 rosbag2 SQLite bag.

Defaults:
- bag path: this script's directory (Barometer_ros2bag)
- topic: /barometer_data
- field: height
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys


def _resolve_bag_uri(path: str) -> str:
    path = os.path.abspath(path)
    if os.path.isfile(path) and path.endswith(".db3"):
        return os.path.dirname(path)
    return path


def _metadata_path(bag_uri: str) -> str:
    return os.path.join(bag_uri, "metadata.yaml")


def _detect_metadata_format(path: str) -> str:
    if not os.path.exists(path):
        return "missing"
    if os.path.getsize(path) == 0:
        return "empty"
    first_key = ""
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            first_key = stripped.split(":", 1)[0].strip()
            break
    if not first_key:
        return "empty"
    if first_key == "rosbag2_bagfile_information":
        return "new"
    if first_key in {
        "version",
        "storage_identifier",
        "duration",
        "starting_time",
        "message_count",
        "topics_with_message_count",
        "compression_format",
        "compression_mode",
        "relative_file_paths",
        "files",
    }:
        return "old"
    return "unknown"


def _wrap_metadata_with_root(path: str) -> None:
    backup_path = path + ".bak"
    if not os.path.exists(backup_path):
        shutil.copy2(path, backup_path)
    with open(path, "r", encoding="utf-8") as handle:
        original = handle.read()
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("rosbag2_bagfile_information:\n")
        for line in original.splitlines():
            handle.write(f"  {line}\n")


def _try_reindex(bag_uri: str) -> bool:
    try:
        subprocess.run(
            ["ros2", "bag", "reindex", bag_uri],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        return True
    except FileNotFoundError:
        print(
            "ros2 CLI not found. Source your ROS 2 environment and retry.",
            file=sys.stderr,
        )
        return False
    except subprocess.CalledProcessError as exc:
        print("ros2 bag reindex failed:", file=sys.stderr)
        if exc.stdout:
            print(exc.stdout, file=sys.stderr)
        if exc.stderr:
            print(exc.stderr, file=sys.stderr)
        return False


def _parse_args() -> argparse.Namespace:
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(
        description="Plot barometer data from a ROS 2 rosbag2 SQLite bag."
    )
    parser.add_argument(
        "--bag",
        default=here,
        help="Path to bag directory (or a .db3 file). Defaults to this script's folder.",
    )
    parser.add_argument(
        "--topic",
        default="/barometer_data",
        help="Topic to plot (default: /barometer_data).",
    )
    parser.add_argument(
        "--field",
        default="height",
        help="Message field to plot (default: height).",
    )
    parser.add_argument(
        "--out",
        default="",
        help="Output image path. If set, saves the plot instead of showing it.",
    )
    parser.add_argument(
        "--list-topics",
        action="store_true",
        help="List topics in the bag and exit.",
    )
    parser.add_argument(
        "--reindex",
        action="store_true",
        help="Run `ros2 bag reindex` if metadata is missing or empty.",
    )
    parser.add_argument(
        "--fix-metadata",
        action="store_true",
        help="Wrap legacy metadata.yaml format under rosbag2_bagfile_information.",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    bag_uri = _resolve_bag_uri(args.bag)
    if not os.path.exists(bag_uri):
        print(f"Bag path not found: {bag_uri}", file=sys.stderr)
        return 2
    metadata_path = _metadata_path(bag_uri)
    metadata_state = _detect_metadata_format(metadata_path)
    if metadata_state in {"missing", "empty"}:
        msg = (
            f"metadata.yaml is {metadata_state} in {bag_uri}. "
            "Run `ros2 bag reindex <bag>` or pass --reindex."
        )
        if args.reindex and _try_reindex(bag_uri):
            metadata_state = _detect_metadata_format(metadata_path)
        else:
            print(msg, file=sys.stderr)
            return 2
    if metadata_state == "old":
        if args.fix_metadata:
            _wrap_metadata_with_root(metadata_path)
            metadata_state = _detect_metadata_format(metadata_path)
        else:
            print(
                "metadata.yaml appears to use the legacy format. "
                "Run with --fix-metadata to wrap it.",
                file=sys.stderr,
            )
            return 2
    if metadata_state == "unknown":
        print(
            "metadata.yaml format is not recognized. Try `ros2 bag reindex <bag>`.",
            file=sys.stderr,
        )
        return 2

    try:
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except Exception as exc:
        print(
            "Missing ROS 2 Python dependencies (rosbag2_py, rclpy, rosidl_runtime_py). "
            "Run this script in a sourced ROS 2 environment.",
            file=sys.stderr,
        )
        print(f"Import error: {exc}", file=sys.stderr)
        return 3

    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_uri, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )

    try:
        reader.open(storage_options, converter_options)
    except Exception as exc:
        print(f"Failed to open bag at {bag_uri}: {exc}", file=sys.stderr)
        print(
            "If the error mentions parsing the info file, run with --fix-metadata "
            "or `ros2 bag reindex <bag>`.",
            file=sys.stderr,
        )
        return 4

    topic_types = reader.get_all_topics_and_types()
    if not topic_types:
        print("No topics found in bag.", file=sys.stderr)
        return 5

    type_map = {t.name: t.type for t in topic_types}

    if args.list_topics:
        for name, typ in type_map.items():
            print(f"{name}  ({typ})")
        return 0

    topic = args.topic
    if topic not in type_map:
        if len(type_map) == 1 and args.topic == "/barometer_data":
            topic = next(iter(type_map))
            print(f"Topic '{args.topic}' not found; using '{topic}' instead.")
        else:
            print(
                f"Topic '{args.topic}' not found. Available topics: {', '.join(type_map)}",
                file=sys.stderr,
            )
            return 6

    msg_type_str = type_map[topic]
    msg_type = get_message(msg_type_str)

    times_s = []
    values = []
    t0 = None

    while reader.has_next():
        t_topic, data, t = reader.read_next()
        if t_topic != topic:
            continue
        if t0 is None:
            t0 = t
        msg = deserialize_message(data, msg_type)
        if not hasattr(msg, args.field):
            print(
                f"Message type {msg_type_str} does not have field '{args.field}'.",
                file=sys.stderr,
            )
            return 7
        times_s.append((t - t0) * 1e-9)
        values.append(getattr(msg, args.field))

    if not values:
        print(f"No messages found on topic '{topic}'.", file=sys.stderr)
        return 8

    if args.out:
        import matplotlib

        matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    plt.figure(figsize=(9, 4.5))
    plt.plot(times_s, values, linewidth=1.5)
    plt.xlabel("Time (s)")
    plt.ylabel(args.field)
    plt.title(f"{topic} ({msg_type_str})")
    plt.grid(True, alpha=0.3)

    if args.out:
        plt.savefig(args.out, dpi=150, bbox_inches="tight")
        print(f"Saved plot to {args.out}")
    else:
        plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
