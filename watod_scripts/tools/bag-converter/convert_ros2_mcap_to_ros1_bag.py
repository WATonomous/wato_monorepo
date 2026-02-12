#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

from rosbags.convert import ConverterError, convert
from rosbags.typesys import Stores, get_typestore


def _get_store(name: str) -> Stores:
    """Return a Stores enum value if available, else fall back.

    Rosbags may not ship a dedicated typestore for the newest ROS2 distro in
    older releases. For our conversion use-case (mostly standard msgs like
    sensor_msgs), Humble/Foxy typestores are sufficient fallbacks.
    """

    normalized = name.strip().lower()

    def has(attr: str) -> bool:
        return hasattr(Stores, attr)

    if normalized in {"ros2_default", "default", "auto"}:
        for candidate in ("ROS2_JAZZY", "ROS2_IRON", "ROS2_HUMBLE", "ROS2_FOXY"):
            if has(candidate):
                return getattr(Stores, candidate)
        if has("EMPTY"):
            return getattr(Stores, "EMPTY")
        # Very old rosbags versions should still have at least one store.
        return next(iter(Stores))

    mapping: dict[str, str] = {
        "ros2_jazzy": "ROS2_JAZZY",
        "ros2_iron": "ROS2_IRON",
        "ros2_humble": "ROS2_HUMBLE",
        "ros2_foxy": "ROS2_FOXY",
        "empty": "EMPTY",
    }
    attr = mapping.get(normalized)
    if attr and has(attr):
        return getattr(Stores, attr)

    if normalized == "ros2_jazzy" and has("ROS2_HUMBLE"):
        return getattr(Stores, "ROS2_HUMBLE")

    raise SystemExit(
        f"Unknown --src-typestore '{name}'. Expected one of: {', '.join(sorted(mapping.keys() | {'ros2_default'}))}"
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert a ROS2 MCAP (rosbag2) file into a ROS1 rosbag1 .bag, "
            "filtering to selected topics/msgtypes to avoid non-default message types."
        )
    )

    parser.add_argument(
        "--src",
        required=True,
        help=(
            "Source MCAP path. Recommended to pass a path under /bags, e.g. "
            "--src /bags/recording_..._0-002.mcap"
        ),
    )
    parser.add_argument(
        "--dst",
        required=True,
        help=(
            "Destination ROS1 bag path (must end with .bag). Recommended under /bags, e.g. "
            "--dst /bags/calib_inputs.bag"
        ),
    )

    parser.add_argument(
        "--include-topic",
        action="append",
        default=[],
        help=(
            "Topic to include (repeatable). Example: --include-topic /novatel/oem7/imu/data_raw "
            "--include-topic /lidar_top/velodyne_points"
        ),
    )
    parser.add_argument(
        "--include-msgtype",
        action="append",
        default=[],
        help=(
            "Message type to include (repeatable). Use ROS2 style names, e.g. sensor_msgs/msg/Imu. "
            "Topics and msgtypes can be combined; include filters are OR-ed."
        ),
    )

    parser.add_argument(
        "--start",
        type=float,
        default=None,
        help=(
            "Optional start time offset in seconds from the beginning of the bag. "
            "Not implemented (kept for future parity); use an MCAP chunk that already contains the desired window."
        ),
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help=(
            "Optional duration in seconds. Not implemented; use a smaller MCAP chunk for now."
        ),
    )

    parser.add_argument(
        "--src-typestore",
        default="ros2_default",
        help=(
            "Typestore to use if the source bag lacks message definitions. "
            "Values: ros2_default (auto), ros2_jazzy, ros2_iron, ros2_humble, ros2_foxy, empty. "
            "If your rosbags version doesn't include Jazzy, ros2_default will fall back to Humble/Foxy."
        ),
    )

    return parser.parse_args()


def main() -> int:
    args = _parse_args()

    src = Path(args.src)
    dst = Path(args.dst)

    if dst.suffix != ".bag":
        raise SystemExit("--dst must end with .bag (ROS1 rosbag1 output)")

    if not src.exists():
        raise SystemExit(f"Source path not found: {src}")

    dst.parent.mkdir(parents=True, exist_ok=True)

    if args.start is not None or args.duration is not None:
        raise SystemExit(
            "--start/--duration not implemented. "
            "For now, pick a single MCAP file that already contains the time window you want."
        )

    if not args.include_topic and not args.include_msgtype:
        raise SystemExit(
            "Refusing to convert without include filters. "
            "Pass at least one --include-topic or --include-msgtype to avoid non-default message types."
        )

    default_typestore = get_typestore(_get_store(args.src_typestore))

    try:
        convert(
            srcs=[src],
            dst=dst,
            dst_storage="mcap",
            dst_version=9,
            compress=None,
            compress_mode="file",
            default_typestore=default_typestore,
            typestore=None,
            exclude_topics=[],
            include_topics=args.include_topic,
            exclude_msgtypes=[],
            include_msgtypes=args.include_msgtype,
        )
    except ConverterError as exc:
        raise SystemExit(str(exc)) from exc

    print(f"Wrote ROS1 bag: {dst}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
