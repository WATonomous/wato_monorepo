"""One process owns simulation stepping. One invocation is one fresh episode."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import time

from .contract import Mapper, command_to_control, wrap
from .tools import connect, dump, jsonable


def main():
    # Imports are deferred so the contract and smoke tools work without ROS.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from ackermann_msgs.msg import AckermannDriveStamped
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from tf2_ros import TransformBroadcaster
    from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose

    p = argparse.ArgumentParser()
    p.add_argument("--socket", default="/run/simforge/env.sock")
    p.add_argument("--contract", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--steps", type=int, default=200)
    p.add_argument("--command-timeout", type=float, default=10)
    p.add_argument("--controller-kind", choices=("diagnostic", "wato"), default="diagnostic")
    p.add_argument("--publish-actors", action="store_true")
    p.add_argument("--base-frame", default="base_footprint")
    a, ros_args = p.parse_known_args()
    if a.steps < 1 or a.command_timeout <= 0:
        p.error("steps and command timeout must be positive")
    config = json.loads(Path(a.contract).read_text())
    mapper = Mapper(config)
    if a.controller_kind == "wato" and config.get("scenario_control_reviewed") is not True:
        p.error("Audit ego scripted actions and engine assistance before a WATO run")
    if a.controller_kind == "wato" and not a.publish_actors:
        p.error("This initial WATO behavior evaluation requires complete actor truth")
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    if any((out / name).exists() for name in ("trace.jsonl", "run.json", "contract.json")):
        raise FileExistsError("Choose an unused episode output directory")
    rclpy.init(args=ros_args)
    node = Node("simforge_bridge")
    # This node uses a monotonic wall deadline, not a timer driven by its own /clock.
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    clock_pub = node.create_publisher(Clock, "/clock", qos)
    odom_pub = node.create_publisher(Odometry, "/ego/odom", qos)
    pose_pub = node.create_publisher(PoseStamped, "/localization/pose", qos)
    tracks_pub = node.create_publisher(Detection3DArray, "/simforge/tracked_detections_3d", qos)
    transforms = TransformBroadcaster(node)
    expected_ns = None
    command = None
    def callback(msg):
        nonlocal command
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        if expected_ns is not None and stamp == expected_ns:
            command = msg
    subscription = node.create_subscription(AckermannDriveStamped, "/simforge/command", callback, qos)
    env = None
    status = {"kind": "ros_control_prototype", "controller_kind": a.controller_kind,
              "completed": False, "seed": a.seed,
              "actor_truth_enabled": a.publish_actors,
              "checks": ["command timestamps", "state mapping"],
              "limitations": ["adapter speed P controller is uncalibrated",
                              "no proof of full pipeline synchronization",
                              "not a vehicle dynamics validation"]}
    journal = (out / "trace.jsonl").open("w")
    previous = None
    try:
        env = connect(a.socket)
        dump(out / "hello.json", env.hello)
        dump(out / "contract.json", config)
        obs, info = env.reset(seed=a.seed)
        terminal, truncated = False, False
        for tick in range(a.steps+1):
            snapshot = jsonable({"observation": obs, "info": info})
            ego = mapper.ego(snapshot)
            actors = mapper.actors(snapshot) if a.publish_actors else []
            if previous is not None and ego.t <= previous.t:
                raise RuntimeError("Nonincreasing simulation time")
            yaw_rate = 0.0 if previous is None else wrap(ego.yaw-previous.yaw)/(ego.t-previous.t)
            row = {"tick": tick, "snapshot": snapshot, "ego": asdict(ego), "actors": actors, "events": info.get("events"),
                   "terminated": terminal, "truncated": truncated}
            expected_ns = round(ego.t*1_000_000_000)
            if expected_ns < 0:
                raise RuntimeError("ROS prototype expects nonnegative simulation time")
            sec, nanosec = divmod(expected_ns, 1_000_000_000)
            odom = Odometry()
            odom.header.frame_id, odom.child_frame_id = "map", a.base_frame
            odom.header.stamp.sec, odom.header.stamp.nanosec = sec, nanosec
            odom.pose.pose.position.x, odom.pose.pose.position.y = ego.x, ego.y
            odom.pose.pose.orientation.z = math.sin(ego.yaw/2)
            odom.pose.pose.orientation.w = math.cos(ego.yaw/2)
            odom.twist.twist.linear.x = ego.speed
            odom.twist.twist.angular.z = yaw_rate
            pose = PoseStamped()
            pose.header, pose.pose = odom.header, odom.pose.pose
            tf = TransformStamped()
            tf.header, tf.child_frame_id = odom.header, a.base_frame
            tf.transform.translation.x, tf.transform.translation.y = ego.x, ego.y
            tf.transform.rotation = odom.pose.pose.orientation
            detections = Detection3DArray()
            detections.header = odom.header
            for actor in actors:
                if actor["id"] == str(env.ego):
                    continue
                det = Detection3D()
                det.header, det.id = odom.header, actor["id"]
                det.bbox.center.position.x = actor["x"]
                det.bbox.center.position.y = actor["y"]
                det.bbox.center.position.z = actor["z"]
                det.bbox.center.orientation.z = math.sin(actor["yaw"]/2)
                det.bbox.center.orientation.w = math.cos(actor["yaw"]/2)
                det.bbox.size.x, det.bbox.size.y, det.bbox.size.z = actor["length"], actor["width"], actor["height"]
                labels = [actor["class"]]
                if actor["class"] == "traffic_light":
                    labels.append(actor["signal_state"])
                for label in labels:
                    result = ObjectHypothesisWithPose()
                    result.hypothesis.class_id, result.hypothesis.score = label, 1.0
                    result.pose.pose = det.bbox.center
                    det.results.append(result)
                detections.detections.append(det)
            command = None
            odom_pub.publish(odom)
            pose_pub.publish(pose)
            transforms.sendTransform(tf)
            if a.publish_actors:
                tracks_pub.publish(detections)
            clock = Clock()
            clock.clock = odom.header.stamp
            clock_pub.publish(clock)
            if terminal or truncated or tick == a.steps:
                journal.write(json.dumps(row, allow_nan=False)+"\n")
                journal.flush()
                status.update(completed=True, end="terminated" if terminal else "truncated" if truncated else "horizon")
                break
            deadline = time.monotonic()+a.command_timeout
            while command is None and rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=min(0.05, max(0, deadline-time.monotonic())))
            if command is None:
                raise TimeoutError(f"No command stamped {expected_ns} for tick {tick}")
            control = command_to_control(command.drive.speed, command.drive.steering_angle, ego.speed, config["control"])
            row.update(command={"speed": command.drive.speed, "steering": command.drive.steering_angle}, control=control)
            journal.write(json.dumps(row, allow_nan=False)+"\n")
            journal.flush()
            previous = ego
            obs, _, terminal, truncated, info = env.step(control)
    except Exception as exc:
        status["error"] = str(exc)
        raise
    finally:
        journal.close()
        if env is not None:
            env.close()
        dump(out / "run.json", status)
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
