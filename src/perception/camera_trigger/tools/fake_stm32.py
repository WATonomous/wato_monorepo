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
"""Fake STM32 camera trigger board: sends pulse reports over UDP.

Sends the same lines as the real board (docs/serial_protocol.md): one BOOT line, then
one TRG line per pulse, 20 pulses per second on the PPS grid of this computer's clock.
Lets camera_trigger_node be developed and tested without hardware. The fault options
inject the problems the node has to detect.

Examples:
    ros2 run camera_trigger fake_stm32.py
    ros2 run camera_trigger fake_stm32.py --udp 127.0.0.1:5800 --drop-seq 50
    python3 fake_stm32.py --pause 3 --pause-at 5 --verbose
"""

import argparse
import random
import socket
import sys
import time

NS_PER_S = 1_000_000_000
LATE_WARNING_MS = 20.0
NUM_STROBES = 12
ALL_STROBES = (1 << NUM_STROBES) - 1


def checksum(body: str) -> int:
    """XOR of all bytes of the body (NMEA-style)."""
    value = 0
    for byte in body.encode("ascii"):
        value ^= byte
    return value


def frame(body: str) -> bytes:
    """Builds a complete line: <body>*XX\\r\\n."""
    return f"{body}*{checksum(body):02X}\r\n".encode("ascii")


def garble(body: str, rng: random.Random) -> bytes:
    """A line with one corrupted character, so its checksum no longer matches."""
    i = rng.randrange(len(body))
    corrupted = body[:i] + chr(ord(body[i]) ^ 0x01) + body[i + 1 :]
    return f"{corrupted}*{checksum(body):02X}\r\n".encode("ascii")


def host_port(text: str) -> tuple:
    host, sep, port = text.rpartition(":")
    if not sep or not host:
        raise argparse.ArgumentTypeError(f"expected HOST:PORT, got {text!r}")
    return host, int(port)


def strobe_miss(text: str) -> tuple:
    cam, sep, rate = text.partition(":")
    if not sep:
        raise argparse.ArgumentTypeError(f"expected CAM:RATE, got {text!r}")
    cam_bit, miss_rate = int(cam), float(rate)
    if not 0 <= cam_bit < NUM_STROBES or not 0.0 <= miss_rate <= 1.0:
        raise argparse.ArgumentTypeError("CAM must be 0-11 and RATE 0-1")
    return cam_bit, miss_rate


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--udp",
        type=host_port,
        default=("127.0.0.1", 5800),
        metavar="HOST:PORT",
        help="where to send the reports (default 127.0.0.1:5800)",
    )
    parser.add_argument("--rate", type=int, default=20, help="pulses per second")
    parser.add_argument(
        "--latency",
        type=float,
        default=0.005,
        help="seconds between a pulse and its report (default 0.005)",
    )
    parser.add_argument(
        "--latency-jitter",
        type=float,
        default=0.001,
        help="random +/- spread added to --latency (default 0.001)",
    )
    parser.add_argument(
        "--fw-version", default="fake-1.0", help="version sent in the BOOT line"
    )
    parser.add_argument(
        "--duration", type=float, default=0.0, help="stop after S seconds (0 = run)"
    )
    parser.add_argument("--seed", type=int, default=None, help="random seed")
    parser.add_argument("--verbose", action="store_true", help="print every line")

    faults = parser.add_argument_group("faults")
    faults.add_argument(
        "--drop-seq",
        type=int,
        default=0,
        metavar="N",
        help="don't send every N-th pulse report (the pulse still happens)",
    )
    faults.add_argument(
        "--unlock-after",
        type=float,
        default=0.0,
        metavar="S",
        help="after S seconds, lose PPS: pps_locked=0 and pulses drift off the grid",
    )
    faults.add_argument(
        "--drift-ppm",
        type=float,
        default=200.0,
        help="pulse period error once unlocked (default 200 ppm)",
    )
    faults.add_argument(
        "--garble-rate",
        type=float,
        default=0.0,
        metavar="P",
        help="corrupt this fraction of lines (bad checksum)",
    )
    faults.add_argument(
        "--strobe-miss",
        type=strobe_miss,
        action="append",
        default=[],
        metavar="CAM:RATE",
        help="clear strobe bit CAM (0-11) on this fraction of pulses; repeatable",
    )
    faults.add_argument(
        "--no-strobe", action="store_true", help="send '---' (strobes not measured)"
    )
    faults.add_argument(
        "--reboot-after",
        type=float,
        default=0.0,
        metavar="S",
        help="after S seconds, reboot once: send BOOT, restart seq and pps_count at 0",
    )
    faults.add_argument(
        "--pause",
        type=float,
        default=0.0,
        metavar="S",
        help="stop sending for S seconds, like an unplugged cable (pulses continue)",
    )
    faults.add_argument(
        "--pause-at",
        type=float,
        default=5.0,
        metavar="T",
        help="when --pause starts, in seconds after start (default 5)",
    )
    faults.add_argument(
        "--clock-offset",
        type=float,
        default=0.0,
        metavar="S",
        help="pretend this computer's clock is S seconds ahead of GPS time",
    )
    faults.add_argument(
        "--duplicate-rate",
        type=float,
        default=0.0,
        metavar="P",
        help="send this fraction of lines twice",
    )
    args = parser.parse_args(argv)
    if not 1 <= args.rate <= 256:
        parser.error("--rate must be between 1 and 256")
    return args


class FakeBoard:
    def __init__(self, args):
        self.args = args
        self.rng = random.Random(args.seed)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.clock_offset_ns = round(args.clock_offset * NS_PER_S)
        self.sent = 0

    def gps_now_ns(self) -> int:
        """GPS time as seen from this computer, whose clock is off by --clock-offset."""
        return time.time_ns() - self.clock_offset_ns

    def first_pulse_ns(self) -> int:
        """The board starts on the PPS edge after next: a whole GPS second."""
        return (self.gps_now_ns() // NS_PER_S + 2) * NS_PER_S

    def send(self, line: bytes):
        try:
            self.sock.sendto(line, self.args.udp)
            self.sent += 1
        except OSError as error:  # nobody listening yet, network down, ...
            print(f"fake_stm32: send failed: {error}", file=sys.stderr)
        if self.args.verbose:
            print(line.decode("ascii", "replace").rstrip(), flush=True)

    def sleep_until(self, gps_ns: int):
        delay = (gps_ns - self.gps_now_ns()) / NS_PER_S
        if delay > 0:
            time.sleep(delay)
        # If this process was held up (busy machine, VM pause), its lines go out late and the
        # node rightly flags them. Say so, so it isn't mistaken for a node problem.
        late_ms = (self.gps_now_ns() - gps_ns) / 1e6
        if late_ms > LATE_WARNING_MS:
            print(
                f"fake_stm32: line sent {late_ms:.0f} ms late (this computer was busy)",
                file=sys.stderr,
                flush=True,
            )

    def strobe_field(self) -> str:
        if self.args.no_strobe:
            return "---"
        mask = ALL_STROBES
        for cam_bit, miss_rate in self.args.strobe_miss:
            if self.rng.random() < miss_rate:
                mask &= ~(1 << cam_bit)
        return f"{mask:03X}"

    def run(self):
        a = self.args
        start_s = time.monotonic()
        rate = a.rate
        nominal_period_ns = NS_PER_S // rate
        unlocked_period_ns = round(nominal_period_ns * (1 + a.drift_ppm * 1e-6))

        boot_gps_ns = self.first_pulse_ns()  # GPS time of pulse 0 since (re)boot
        k = 0  # pulses since (re)boot: seq = k, pulse_idx = k % rate, pps_count = k // rate
        locked = True
        rebooted = False
        pulse_ns = boot_gps_ns

        print(
            f"fake_stm32: sending to {a.udp[0]}:{a.udp[1]} at {rate} Hz "
            f"(clock offset {a.clock_offset:+.3f} s)",
            file=sys.stderr,
            flush=True,
        )
        self.send(frame(f"BOOT,1,{a.fw_version}"))

        while True:
            elapsed = time.monotonic() - start_s
            if a.duration and elapsed >= a.duration:
                break

            if a.reboot_after and not rebooted and elapsed >= a.reboot_after:
                rebooted = True
                self.send(frame(f"BOOT,1,{a.fw_version}"))
                boot_gps_ns = self.first_pulse_ns()
                k = 0
                pulse_ns = boot_gps_ns

            if a.unlock_after and locked and elapsed >= a.unlock_after:
                locked = False
                print("fake_stm32: PPS lost (unlocked)", file=sys.stderr, flush=True)

            # The time of this pulse, on the GPS clock.
            if locked:
                second, idx = divmod(k, rate)
                pulse_ns = boot_gps_ns + second * NS_PER_S + idx * NS_PER_S // rate
            elif k > 0:
                pulse_ns += unlocked_period_ns

            jitter = self.rng.uniform(-a.latency_jitter, a.latency_jitter)
            send_ns = pulse_ns + round((a.latency + jitter) * NS_PER_S)
            self.sleep_until(send_ns)

            seq = k & 0xFFFFFFFF
            pps_count = (k // rate) & 0xFFFFFFFF
            idx = k % rate
            body = (
                f"TRG,1,{seq},{pps_count},{idx},{1 if locked else 0},"
                f"{self.strobe_field()}"
            )
            k += 1

            in_pause = a.pause and a.pause_at <= elapsed < a.pause_at + a.pause
            dropped = a.drop_seq > 0 and seq % a.drop_seq == a.drop_seq - 1
            if in_pause or dropped:
                continue

            if self.rng.random() < a.garble_rate:
                line = garble(body, self.rng)
            else:
                line = frame(body)
            self.send(line)
            if self.rng.random() < a.duplicate_rate:
                self.send(line)


def main(argv=None) -> int:
    board = FakeBoard(parse_args(argv))
    try:
        board.run()
    except KeyboardInterrupt:
        pass
    print(f"fake_stm32: sent {board.sent} lines", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
