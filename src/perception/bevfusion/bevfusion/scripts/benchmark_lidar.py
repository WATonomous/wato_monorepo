#!/usr/bin/env python3
# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Check inputs, run alternating lidar trials, and summarize saved measurements.

`run` starts ROS nodes and replay; `prepare-models` initializes engines without
replay. `check` reads bag metadata and samples without loading a GPU model.
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

CAMERAS = [f"camera_pano_{name}" for name in ("nn", "ne", "nw", "ss", "sw", "se")]
LIDAR = "/lidar_cc/velodyne_points"
TOPICS = [LIDAR, "/tf", "/tf_static"] + [
    f"/{camera}/{suffix}"
    for camera in CAMERAS
    for suffix in ("image_rect_compressed", "camera_info")
]
STAGES = [
    "image_ms",
    "lidar_ms",
    "cpu_fp16_ms",
    "infer_ms",
    "postproc_ms",
    "total_ms",
    "ready_to_publish_ms",
]


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def save_json(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n")


def command_output(command):
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    return result.stdout.strip() if result.returncode == 0 else result.stderr.strip()


def inspect_inputs(bag, model_dir):
    import cv2
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import yaml

    bag = bag.resolve(strict=True)
    metadata = yaml.safe_load((bag / "metadata.yaml").read_text())[
        "rosbag2_bagfile_information"
    ]
    counts = {
        item["topic_metadata"]["name"]: item["message_count"]
        for item in metadata["topics_with_message_count"]
    }
    missing = [name for name in TOPICS if not counts.get(name)]
    if missing:
        raise ValueError(f"Bag is missing required input topics: {missing}")
    for relative in metadata["relative_file_paths"]:
        if not (bag / relative).is_file():
            raise ValueError(f"Bag file is missing: {relative}")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=TOPICS))
    types = {
        item.name: get_message(item.type)
        for item in reader.get_all_topics_and_types()
        if item.name in TOPICS
    }
    samples = {}
    first_record_ns = None
    lidar_origin = None
    while reader.has_next() and len(samples) < len(TOPICS):
        topic, payload, recorded_ns = reader.read_next()
        if first_record_ns is None:
            first_record_ns = recorded_ns
        if recorded_ns - first_record_ns > 5_000_000_000:
            break
        if topic in samples:
            continue
        message = deserialize_message(payload, types[topic])
        if topic == LIDAR:
            fields = {field.name: field for field in message.fields}
            for name in ("x", "y", "z", "intensity"):
                if (
                    name not in fields
                    or fields[name].datatype != 7
                    or fields[name].count != 1
                ):
                    raise ValueError(
                        f"CUDA backend requires scalar FLOAT32 {name}: {fields}"
                    )
            lidar_origin = stamp_ns(message.header.stamp)
            samples[topic] = {
                "frame": message.header.frame_id,
                "points": message.width * message.height,
                "point_step": message.point_step,
                "row_step": message.row_step,
                "fields": {name: field.offset for name, field in fields.items()},
            }
        elif topic.endswith("image_rect_compressed"):
            import numpy as np

            image = cv2.imdecode(
                np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR
            )
            if image is None or tuple(image.shape) != (1024, 1280, 3):
                raise ValueError(
                    f"Unexpected JPEG dimensions on {topic}: {None if image is None else image.shape}"
                )
            expected_frame = topic.split("/")[1]
            if message.header.frame_id != expected_frame:
                raise ValueError(
                    f"{topic}: expected frame {expected_frame}, got {message.header.frame_id}"
                )
            samples[topic] = {
                "frame": message.header.frame_id,
                "shape": list(image.shape),
            }
        elif topic.endswith("camera_info"):
            if (
                message.header.frame_id != topic.split("/")[1]
                or message.k[0] <= 0
                or message.k[4] <= 0
            ):
                raise ValueError(f"Invalid calibration on {topic}")
            samples[topic] = {
                "frame": message.header.frame_id,
                "width": message.width,
                "height": message.height,
            }
        else:
            samples[topic] = [
                [transform.header.frame_id, transform.child_frame_id]
                for transform in message.transforms
            ]
    if len(samples) != len(TOPICS) or lidar_origin is None:
        raise ValueError(
            f"Could not sample all required topics in first 5 bag seconds: {set(TOPICS) - samples.keys()}"
        )
    model_files = ["lidar.backbone.xyz.onnx"] + [
        f"build/{name}.plan"
        for name in ("camera.backbone", "camera.vtransform", "fuser", "head.bbox")
    ]
    return {
        "bag": str(bag),
        "bag_duration_s": metadata["duration"]["nanoseconds"] / 1e9,
        "bag_counts": {name: counts[name] for name in TOPICS},
        "lidar_origin_ns": lidar_origin,
        "samples": samples,
        "model_dir": str(model_dir.resolve()),
        "missing_models": [
            name for name in model_files if not (model_dir / name).is_file()
        ],
        "gpu": command_output(
            [
                "nvidia-smi",
                "--query-gpu=name,compute_cap,driver_version,memory.used,utilization.gpu",
                "--format=csv",
            ]
        ),
        "tensor_rt": command_output(
            ["dpkg-query", "-W", "libnvinfer10", "libnvinfer11", "libnvinfer-dev"]
        ),
    }


def stop_process(process):
    if process is None or process.poll() is not None:
        return
    for sig, timeout in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=timeout)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            continue


def process_snapshot(root_pid):
    """Aggregate the ROS launch process tree, including camera decode worker threads."""
    processes = {}
    for path in Path("/proc").glob("[0-9]*/stat"):
        try:
            fields = path.read_text().rsplit(")", 1)[1].split()
            processes[int(path.parent.name)] = (
                int(fields[1]),
                int(fields[11]) + int(fields[12]),
                int(fields[21]),
            )
        except (OSError, ValueError, IndexError):
            continue
    selected = {root_pid}
    while True:
        children = {
            pid for pid, (parent, _, _) in processes.items() if parent in selected
        }
        if children.issubset(selected):
            break
        selected |= children
    ticks = sum(processes[pid][1] for pid in selected if pid in processes)
    rss = sum(processes[pid][2] for pid in selected if pid in processes) * os.sysconf(
        "SC_PAGE_SIZE"
    )
    return ticks / os.sysconf("SC_CLK_TCK"), rss / (1024 * 1024)


def wait_for_active(node, launch_process, timeout=120):
    from lifecycle_msgs.srv import GetState

    client = node.create_client(GetState, "/bevfusion_node/get_state")
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            if launch_process.poll() is not None:
                raise RuntimeError(
                    "Launch exited before BEVFusion activated; inspect launch.log"
                )
            if client.wait_for_service(timeout_sec=0.5):
                future = client.call_async(GetState.Request())
                import rclpy

                rclpy.spin_until_future_complete(node, future, timeout_sec=1)
                if (
                    future.done()
                    and future.result()
                    and future.result().current_state.id == 3
                ):
                    if any(
                        name == "multi_camera_sync"
                        for name, _ in node.get_node_names_and_namespaces()
                    ):
                        return
            time.sleep(0.1)
        raise RuntimeError("Timed out waiting for active BEVFusion and camera_sync")
    finally:
        node.destroy_client(client)


def trial(args, directory, backend, observer):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosidl_runtime_py.convert import message_to_ordereddict
    from vision_msgs.msg import Detection3DArray

    directory.mkdir()
    launch_command = [
        "ros2",
        "launch",
        "bevfusion",
        "benchmark_lidar.launch.py",
        f"backend:={backend}",
        f"model_dir:={args.model_dir.resolve()}",
        f"csv_path:={directory / 'timings.csv'}",
        f"config_file:={args.config.resolve()}",
    ]
    if args.nsys:
        launch_command = [
            "nsys",
            "profile",
            "--trace=cuda,nvtx,osrt",
            "--sample=none",
            "--cpuctxsw=none",
            "--force-overwrite=true",
            f"--output={directory / 'trace'}",
        ] + launch_command
    replay_command = [
        "ros2",
        "bag",
        "play",
        str(args.bag.resolve()),
        "--clock",
        "100",
        "--rate",
        str(args.rate),
        "--delay",
        "2",
        "--playback-duration",
        str(args.warmup + args.duration),
        "--disable-keyboard-controls",
        "--qos-profile-overrides-path",
        str(directory / "qos.yaml"),
        "--topics",
        *TOPICS,
    ]
    # Reliable replay satisfies both reliable camera_info and best-effort lidar subscribers.
    import yaml

    (directory / "qos.yaml").write_text(
        yaml.safe_dump(
            {
                topic: {
                    "history": "keep_last",
                    "depth": 10,
                    "reliability": "reliable",
                    "durability": "transient_local"
                    if topic == "/tf_static"
                    else "volatile",
                }
                for topic in TOPICS
            }
        )
    )
    save_json(
        directory / "commands.json",
        {"launch": launch_command, "replay": replay_command},
    )
    launched = replay = gpu = None
    subscription = None
    try:
        with (
            (directory / "launch.log").open("w") as launch_log,
            (directory / "replay.log").open("w") as replay_log,
            (directory / "detections.jsonl").open("w") as detections,
            (directory / "resources.csv").open("w") as resources,
            (directory / "gpu.csv").open("w") as gpu_log,
        ):

            def receive(message):
                detections.write(
                    json.dumps(
                        {
                            "observed_ns": time.monotonic_ns(),
                            "message": message_to_ordereddict(message),
                        }
                    )
                    + "\n"
                )

            subscription = observer.create_subscription(
                Detection3DArray,
                "/benchmark/detections",
                receive,
                QoSProfile(
                    depth=100,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                ),
            )
            launched = subprocess.Popen(
                launch_command,
                stdout=launch_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            wait_for_active(observer, launched)
            gpu = subprocess.Popen(
                [
                    "nvidia-smi",
                    "--query-gpu=timestamp,index,utilization.gpu,utilization.memory,memory.used,power.draw",
                    "--format=csv,nounits",
                    "--loop=1",
                ],
                stdout=gpu_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            replay = subprocess.Popen(
                replay_command,
                stdout=replay_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            writer = csv.writer(resources)
            writer.writerow(
                [
                    "steady_ns",
                    "ros_cpu_seconds",
                    "ros_rss_mib",
                    "player_cpu_seconds",
                    "player_rss_mib",
                ]
            )
            next_sample = 0
            drain_until = None
            deadline = time.monotonic() + (args.warmup + args.duration) / args.rate + 90
            while time.monotonic() < deadline:
                rclpy.spin_once(observer, timeout_sec=0.05)
                now = time.monotonic()
                if launched.poll() is not None:
                    raise RuntimeError(
                        "ROS launch exited during replay; inspect launch.log"
                    )
                if now >= next_sample:
                    writer.writerow(
                        [
                            time.monotonic_ns(),
                            *process_snapshot(launched.pid),
                            *process_snapshot(replay.pid),
                        ]
                    )
                    next_sample = now + 1
                if replay.poll() is not None:
                    if replay.returncode != 0:
                        raise RuntimeError(
                            f"Bag player exited with code {replay.returncode}; inspect replay.log"
                        )
                    if drain_until is None:
                        drain_until = now + 5
                    if now >= drain_until:
                        break
            else:
                raise RuntimeError("Replay timed out")
    finally:
        stop_process(replay)
        stop_process(launched)
        stop_process(gpu)
        if subscription is not None:
            observer.destroy_subscription(subscription)
    rows = list(csv.DictReader((directory / "timings.csv").open()))
    if not rows or any(
        row["backend"] != backend
        or row["camera_input"] != "3"
        or not row["camera_stamps"]
        for row in rows
    ):
        raise RuntimeError(
            "Missing measurements, unexpected backend, or unexpected camera path; inspect launch.log"
        )


def prepare_models(args):
    """Explicit GPU preparation: build/load engines without playing any recorded input."""
    import rclpy

    args.output.mkdir(parents=True, exist_ok=False)
    launched = None
    rclpy.init()
    observer = rclpy.create_node("lidar_model_preparation")
    try:
        with (args.output / "prepare.log").open("w") as log:
            launched = subprocess.Popen(
                [
                    "ros2",
                    "launch",
                    "bevfusion",
                    "benchmark_lidar.launch.py",
                    "backend:=cpu",
                    f"model_dir:={args.model_dir.resolve()}",
                    f"csv_path:={args.output.resolve() / 'unused.csv'}",
                    f"config_file:={args.config.resolve()}",
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            wait_for_active(observer, launched, timeout=900)
        print(
            "Engine generation and node initialization passed. No bag replay was started."
        )
    finally:
        stop_process(launched)
        observer.destroy_node()
        rclpy.shutdown()


def percentile(values, fraction):
    values = sorted(values)
    if not values:
        return None
    index = fraction * (len(values) - 1)
    low = math.floor(index)
    return values[low] + (values[math.ceil(index)] - values[low]) * (index - low)


def summarize_rows(rows, receipts, start_ns, end_ns, rate):
    rows = [row for row in rows if start_ns <= int(row["lidar_stamp_ns"]) < end_ns]
    ready = {
        int(row["lidar_stamp_ns"])
        for row in receipts
        if start_ns <= int(row["lidar_stamp_ns"]) < end_ns
    }
    done = {int(row["lidar_stamp_ns"]) for row in rows}
    for row in rows:
        row["ready_to_publish_ms"] = (
            int(row["completed_ns"]) - int(row["subscriber_ready_ns"])
        ) / 1e6
    return {
        "frames": len(rows),
        "received_unique_scans": len(ready),
        "received_without_completed_output": len(ready - done),
        "duplicate_outputs": len(rows) - len(done),
        "output_hz_over_completed_span": (
            (len(rows) - 1)
            / ((int(rows[-1]["completed_ns"]) - int(rows[0]["completed_ns"])) / 1e9)
            if len(rows) > 1
            and int(rows[-1]["completed_ns"]) > int(rows[0]["completed_ns"])
            else None
        ),
        "output_hz_per_source_window": len(rows) / ((end_ns - start_ns) / 1e9),
        "output_hz_per_wall_window": len(rows) / ((end_ns - start_ns) / 1e9 / rate),
        "stages_ms": {
            stage: {
                name: percentile([float(row[stage]) for row in rows], quantile)
                for name, quantile in (("p50", 0.5), ("p95", 0.95), ("p99", 0.99))
            }
            for stage in STAGES
        },
    }


def summarize_resources(resources, start_ns, end_ns):
    samples = [row for row in resources if start_ns <= int(row["steady_ns"]) <= end_ns]
    cpu = []
    for previous, current in zip(samples, samples[1:]):
        elapsed = (int(current["steady_ns"]) - int(previous["steady_ns"])) / 1e9
        used = float(current["ros_cpu_seconds"]) - float(previous["ros_cpu_seconds"])
        if elapsed > 0 and used >= 0:
            cpu.append(used / elapsed * 100)
    return {
        "ros_tree_cpu_percent_p50": percentile(cpu, 0.5),
        "ros_tree_cpu_percent_p95": percentile(cpu, 0.95),
        "ros_tree_rss_mib_max": max(
            (float(row["ros_rss_mib"]) for row in samples), default=None
        ),
        "cpu_percent_definition": "100% = one CPU core; includes BEVFusion and camera_sync; excludes bag player/observer",
    }


def report(output):
    manifest = json.loads((output / "manifest.json").read_text())
    start = manifest["lidar_origin_ns"] + int(manifest["warmup"] * 1e9)
    end = start + int(manifest["duration"] * 1e9)
    summaries = {}
    keys = {}
    for directory in sorted(output.glob("[0-9][0-9]-*")):
        with (directory / "timings.csv").open() as stream:
            rows = list(csv.DictReader(stream))
        with (directory / "timings.csv.receipts.csv").open() as stream:
            receipts = list(csv.DictReader(stream))
        summaries[directory.name] = summarize_rows(
            rows, receipts, start, end, manifest["rate"]
        )
        measured = [row for row in rows if start <= int(row["lidar_stamp_ns"]) < end]
        if measured:
            with (directory / "resources.csv").open() as stream:
                summaries[directory.name]["resources"] = summarize_resources(
                    list(csv.DictReader(stream)),
                    int(measured[0]["callback_start_ns"]),
                    int(measured[-1]["completed_ns"]),
                )
        keys[directory.name] = {
            (row["lidar_stamp_ns"], row["camera_stamps"])
            for row in rows
            if start <= int(row["lidar_stamp_ns"]) < end
        }
    pairs = {}
    names = list(keys)
    for left, right in zip(names[::2], names[1::2]):
        pairs[f"{left}/{right}"] = {
            "identical_sensor_sets": len(keys[left] & keys[right])
        }
    result = {
        "trials": summaries,
        "pairs": pairs,
        "notes": [
            "infer_ms includes CPU FP16, host staging/upload on CPU backend, and model inference; cpu_fp16_ms is a subset.",
            "ready_to_publish_ms starts AFTER NITROS adaptation/upload and includes sync wait; it is not end-to-end latency.",
            "Receipt counts exclude DDS loss before the subscriber. Output Hz is a window-normalized rate, including drain.",
            "GPU telemetry is device-wide; analyze resource time series over the measured window and use an idle GPU.",
            "Identical sensor sets only establish comparable inputs. Detections JSONL still requires numerical comparison.",
            "Nsight trials include profiling overhead and must be kept separate from throughput trials.",
        ],
    }
    save_json(output / "summary.json", result)
    print(json.dumps(result, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for command in ("check", "prepare-models", "run"):
        sub = commands.add_parser(command)
        sub.add_argument("--bag", type=Path, required=True)
        sub.add_argument("--model-dir", type=Path, required=True)
        if command != "check":
            sub.add_argument("--output", type=Path, required=True)
            sub.add_argument("--config", type=Path)
        if command == "run":
            sub.add_argument("--warmup", type=float, default=30)
            sub.add_argument("--duration", type=float, default=180)
            sub.add_argument("--rate", type=float, default=1)
            sub.add_argument("--repeats", type=int, default=3)
            sub.add_argument("--nsys", action="store_true")
    sub = commands.add_parser("report")
    sub.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.command == "report":
        report(args.output)
        return
    inputs = inspect_inputs(args.bag, args.model_dir)
    if args.command == "check":
        print(json.dumps(inputs, indent=2))
        if inputs["missing_models"]:
            raise SystemExit(
                "Input inspection passed; required deployment model files listed above are missing."
            )
        return
    if inputs["missing_models"] and args.command != "prepare-models":
        raise SystemExit(
            f"Prepare compatible engines before replay: {inputs['missing_models']}"
        )
    from ament_index_python.packages import get_package_share_directory

    if args.config is None:
        args.config = (
            Path(get_package_share_directory("bevfusion")) / "config/params.yaml"
        )
    if args.command == "prepare-models":
        required = [
            f"{name}.onnx"
            for name in (
                "camera.backbone",
                "camera.vtransform",
                "fuser",
                "head.bbox",
                "lidar.backbone.xyz",
            )
        ]
        missing = [name for name in required if not (args.model_dir / name).is_file()]
        if missing:
            raise SystemExit(f"Missing source models: {missing}")
        prepare_models(args)
        return
    if (
        not all(
            math.isfinite(value) for value in (args.warmup, args.duration, args.rate)
        )
        or args.warmup < 0
        or args.duration <= 0
        or args.rate <= 0
        or args.repeats < 1
    ):
        parser.error(
            "warmup must be >= 0; duration, rate, and repeats must be positive"
        )
    if args.warmup + args.duration > inputs["bag_duration_s"] - 2:
        parser.error(
            "Requested replay window exceeds bag duration (allow 2 seconds of margin)"
        )
    if args.nsys and not shutil.which("nsys"):
        parser.error("nsys is not installed")
    # A new directory is mandatory so a repeated command cannot overwrite an earlier result.
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = inputs | {
        key: value
        for key, value in vars(args).items()
        if isinstance(value, (str, float, int, bool))
    }
    manifest["config_yaml"] = args.config.read_text()
    manifest["ros_domain_id"] = os.environ.get("ROS_DOMAIN_ID", "0")
    manifest["rmw_implementation"] = os.environ.get("RMW_IMPLEMENTATION", "default")
    build_info = os.environ.get("BEVFUSION_BENCHMARK_BUILD_INFO")
    if build_info:
        manifest["build_info"] = json.loads(Path(build_info).read_text())
    save_json(args.output / "manifest.json", manifest)
    import rclpy

    rclpy.init()
    observer = rclpy.create_node("lidar_benchmark_observer")
    try:
        if any(
            name in ("bevfusion_node", "multi_camera_sync")
            for name, _ in observer.get_node_names_and_namespaces()
        ):
            raise RuntimeError(
                "Existing perception nodes detected; use a separate ROS_DOMAIN_ID for the benchmark"
            )
        for repetition in range(args.repeats):
            order = ("cpu", "cuda") if repetition % 2 == 0 else ("cuda", "cpu")
            for offset, backend in enumerate(order):
                directory = args.output / f"{repetition * 2 + offset + 1:02d}-{backend}"
                print(f"Starting {directory.name}", flush=True)
                trial(args, directory, backend, observer)
    finally:
        observer.destroy_node()
        rclpy.shutdown()
    report(args.output)


if __name__ == "__main__":
    main()
