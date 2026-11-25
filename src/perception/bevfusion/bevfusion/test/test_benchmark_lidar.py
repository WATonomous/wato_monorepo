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

"""Offline checks for benchmark windowing and metric definitions; no ROS/GPU required."""

from contextlib import redirect_stdout
import csv
import importlib.util
import io
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

SPEC = importlib.util.spec_from_file_location(
    "benchmark_lidar", Path(__file__).parents[1] / "scripts/benchmark_lidar.py"
)
benchmark = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(benchmark)


class BenchmarkTests(unittest.TestCase):
    def row(self, stamp, completed):
        return {
            "lidar_stamp_ns": str(stamp),
            "subscriber_ready_ns": str(completed - 20_000_000),
            "completed_ns": str(completed),
        } | dict.fromkeys(benchmark.STAGES[:-1], "5.0")

    def test_sensor_window_excludes_warmup_and_end_boundary(self):
        rows = [self.row(stamp, stamp) for stamp in (99, 100, 150, 199, 200)]
        receipts = [
            {"lidar_stamp_ns": str(stamp)} for stamp in (99, 100, 125, 150, 199, 200)
        ]
        summary = benchmark.summarize_rows(rows, receipts, 100, 200, 1)
        self.assertEqual(summary["frames"], 3)
        self.assertEqual(summary["received_unique_scans"], 4)
        self.assertEqual(summary["received_without_completed_output"], 1)
        self.assertEqual(summary["stages_ms"]["ready_to_publish_ms"]["p95"], 20)

    def test_duplicate_outputs_do_not_hide_missing_scans(self):
        rows = [self.row(100, 10_000_000), self.row(100, 20_000_000)]
        receipts = [{"lidar_stamp_ns": "100"}, {"lidar_stamp_ns": "101"}]
        summary = benchmark.summarize_rows(rows, receipts, 100, 200, 2)
        self.assertEqual(summary["duplicate_outputs"], 1)
        self.assertEqual(summary["received_without_completed_output"], 1)
        self.assertEqual(summary["output_hz_over_completed_span"], 100)

    def test_empty_window_does_not_manufacture_zero_latencies(self):
        summary = benchmark.summarize_rows([], [], 100, 200, 1)
        self.assertIsNone(summary["stages_ms"]["total_ms"]["p50"])
        self.assertIsNone(summary["output_hz_over_completed_span"])

    def test_cpu_utilization_can_exceed_one_core(self):
        rows = [
            {"steady_ns": "1000000000", "ros_cpu_seconds": "1.0", "ros_rss_mib": "512"},
            {"steady_ns": "2000000000", "ros_cpu_seconds": "3.0", "ros_rss_mib": "600"},
            {
                "steady_ns": "3000000000",
                "ros_cpu_seconds": "10.0",
                "ros_rss_mib": "900",
            },
        ]
        summary = benchmark.summarize_resources(rows, 1_000_000_000, 2_000_000_000)
        self.assertEqual(summary["ros_tree_cpu_percent_p50"], 200)
        self.assertEqual(summary["ros_tree_rss_mib_max"], 600)

    def test_saved_report_matches_full_sensor_sets(self):
        with TemporaryDirectory() as temporary:
            root = Path(temporary)
            benchmark.save_json(
                root / "manifest.json",
                {
                    "lidar_origin_ns": 0,
                    "warmup": 1,
                    "duration": 1,
                    "rate": 1,
                },
            )
            for name, stamps in (("01-cpu", "1;2;3;4;5;6"), ("02-cuda", "1;2;3;4;5;7")):
                directory = root / name
                directory.mkdir()
                row = self.row(1_500_000_000, 2_000_000_000) | {
                    "callback_start_ns": "1900000000",
                    "camera_stamps": stamps,
                }
                with (directory / "timings.csv").open("w") as stream:
                    writer = csv.DictWriter(stream, fieldnames=row)
                    writer.writeheader()
                    writer.writerow(row)
                (directory / "timings.csv.receipts.csv").write_text(
                    "lidar_stamp_ns,subscriber_ready_ns\n1500000000,1980000000\n"
                )
                (directory / "resources.csv").write_text(
                    "steady_ns,ros_cpu_seconds,ros_rss_mib\n1950000000,1.0,512\n"
                )
            with redirect_stdout(io.StringIO()):
                benchmark.report(root)
            report = json.loads((root / "summary.json").read_text())
            self.assertEqual(report["trials"]["01-cpu"]["frames"], 1)
            self.assertEqual(
                report["pairs"]["01-cpu/02-cuda"]["identical_sensor_sets"], 0
            )


if __name__ == "__main__":
    unittest.main()
