#!/usr/bin/env python3
"""Measure BEVFusion end-to-end latency live.

Subscribes to the synchronized camera topic, the LiDAR topic bevfusion consumes,
and the BEVFusion 3D detection output, then reports:
  * e2e latency   : detection receive time - image header stamp (capture -> consumer)
  * pipeline lat. : detection receive time - image receive time (clock-offset free)
  * camera lat.   : image receive time - image header stamp
  * rates / periods for each stream
"""
import argparse
import json
import statistics
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from vision_msgs.msg import Detection3DArray
from sensor_msgs.msg import PointCloud2

try:
    from deep_msgs.msg import MultiImageCompressed
except ImportError:  # pragma: no cover - fall back if msg pkg missing
    MultiImageCompressed = None

NS = 1e6  # ns -> ms


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def qos(reliability, depth=50):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=reliability,
        durability=DurabilityPolicy.VOLATILE,
    )


def summarize(name, values, unit="ms"):
    if not values:
        return {"name": name, "count": 0}
    s = sorted(values)
    n = len(s)

    def pct(p):
        return s[min(n - 1, int(round(p / 100.0 * (n - 1))))]

    return {
        "name": name,
        "unit": unit,
        "count": n,
        "mean": statistics.fmean(s),
        "std": statistics.pstdev(s) if n > 1 else 0.0,
        "min": s[0],
        "p50": pct(50),
        "p90": pct(90),
        "p99": pct(99),
        "max": s[-1],
    }


class Bench(Node):
    def __init__(self, args):
        super().__init__("bevfusion_latency_bench")
        self.args = args
        self.img_recv = {}          # image header stamp_ns -> recv_ns
        self.img_order = []
        self.camera_lat = []        # img recv - img stamp
        self.img_stamps = []
        self.lidar_stamps = []
        self.lidar_lat = []
        self.det_records = []       # (stamp_ns, recv_ns, nboxes)
        self.warned_missing = 0

        best = qos(ReliabilityPolicy.BEST_EFFORT)
        rel = qos(ReliabilityPolicy.RELIABLE)

        if MultiImageCompressed is not None:
            self.create_subscription(
                MultiImageCompressed, args.image_topic, self.on_image, best)
        self.create_subscription(PointCloud2, args.lidar_topic, self.on_lidar, best)
        # BEVFusion publishes RELIABLE; a RELIABLE sub is compatible.
        self.create_subscription(
            Detection3DArray, args.det_topic, self.on_det, rel)

        self.get_logger().info(
            f"benchmarking for {args.duration}s\n"
            f"  images : {args.image_topic}\n"
            f"  lidar  : {args.lidar_topic}\n"
            f"  dets   : {args.det_topic}")

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def on_image(self, msg):
        recv = self.now_ns()
        s = stamp_ns(msg.header.stamp)
        self.img_recv[s] = recv
        self.img_order.append(s)
        self.img_stamps.append(s)
        self.camera_lat.append((recv - s) / NS)
        # bound memory
        while len(self.img_order) > 4000:
            self.img_recv.pop(self.img_order.pop(0), None)

    def on_lidar(self, msg):
        recv = self.now_ns()
        s = stamp_ns(msg.header.stamp)
        self.lidar_stamps.append(s)
        self.lidar_lat.append((recv - s) / NS)

    def on_det(self, msg):
        recv = self.now_ns()
        s = stamp_ns(msg.header.stamp)
        self.det_records.append((s, recv, len(msg.detections)))

    # ---------- reporting ----------
    def report(self):
        e2e = []
        pipeline = []
        matched = 0
        nboxes = []
        for s, recv, nb in self.det_records:
            e2e.append((recv - s) / NS)
            nboxes.append(nb)
            img_r = self.img_recv.get(s)
            if img_r is not None:
                matched += 1
                pipeline.append((recv - img_r) / NS)

        def periods(stamps):
            st = sorted(stamps)
            return [(b - a) / NS for a, b in zip(st, st[1:]) if b > a]

        det_stamps = [r[0] for r in self.det_records]
        det_recvs = [r[1] for r in self.det_records]

        # camera/lidar stamp skew for each detection frame
        skew = []
        ls = sorted(self.lidar_stamps)
        if ls:
            import bisect
            for s in det_stamps:
                i = bisect.bisect_left(ls, s)
                cands = [c for c in ls[max(0, i - 1): i + 1]]
                if cands:
                    skew.append(min(((c - s) / NS for c in cands), key=abs))

        out = {
            "duration_s": self.args.duration,
            "counts": {
                "images": len(self.img_stamps),
                "lidar": len(self.lidar_stamps),
                "detections": len(self.det_records),
                "detections_matched_to_image": matched,
            },
            "metrics": [
                summarize("end_to_end_latency (det_recv - image_stamp)", e2e),
                summarize("pipeline_latency (det_recv - image_recv)", pipeline),
                summarize("camera_latency (image_recv - image_stamp)", self.camera_lat),
                summarize("lidar_latency (lidar_recv - lidar_stamp)", self.lidar_lat),
                summarize("detection_output_period (by recv)", periods(det_recvs)),
                summarize("detection_stamp_period", periods(det_stamps)),
                summarize("image_period", periods(self.img_stamps)),
                summarize("lidar_period", periods(self.lidar_stamps)),
                summarize("cam_lidar_stamp_skew", skew),
            ],
            "boxes_per_frame": summarize("boxes_per_frame", [float(b) for b in nboxes], unit="boxes"),
        }
        return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--duration", type=float, default=60.0)
    p.add_argument("--image-topic", default="/multi_camera_sync/multi_image_compressed")
    p.add_argument("--lidar-topic", default="/lidar_cc/velodyne_points")
    p.add_argument("--det-topic", default="/perception/detections_3d_bev")
    p.add_argument("--json-out", default="")
    args = p.parse_args()

    rclpy.init()
    node = Bench(args)
    end = node.get_clock().now().nanoseconds + int(args.duration * 1e9)
    try:
        while rclpy.ok() and node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass

    rep = node.report()
    node.destroy_node()
    rclpy.shutdown()

    print("\n" + "=" * 78)
    print("BEVFusion live latency benchmark")
    print("=" * 78)
    c = rep["counts"]
    print(f"messages: images={c['images']}  lidar={c['lidar']}  "
          f"detections={c['detections']} (matched to an image: "
          f"{c['detections_matched_to_image']})")
    print("-" * 78)
    hdr = f"{'metric':<48}{'n':>5}{'mean':>9}{'p50':>9}{'p90':>9}{'max':>9}"
    print(hdr)
    for m in rep["metrics"] + [rep["boxes_per_frame"]]:
        if not m.get("count"):
            print(f"{m['name']:<48}{0:>5}{'-':>9}{'-':>9}{'-':>9}{'-':>9}")
            continue
        print(f"{m['name']:<48}{m['count']:>5}{m['mean']:>9.1f}"
              f"{m['p50']:>9.1f}{m['p90']:>9.1f}{m['max']:>9.1f}")
    print("=" * 78)

    if args.json_out:
        with open(args.json_out, "w") as f:
            json.dump(rep, f, indent=2)
        print(f"wrote {args.json_out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
