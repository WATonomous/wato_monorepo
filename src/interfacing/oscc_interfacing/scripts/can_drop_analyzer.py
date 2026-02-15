#!/usr/bin/env python3
"""Analyze CAN ID 0x2B0 (steering angle) for gaps/drops directly from SocketCAN."""

import socket
import struct
import time
import sys

CAN_ID = 0x2B0
CAN_FRAME_FMT = "=IB3x8s"  # can_id, can_dlc, pad, data
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else "can0"
    threshold_ms = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0

    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)

    # Filter for 0x2B0 only
    # struct can_filter: uint32_t can_id, uint32_t can_mask
    can_filter = struct.pack("=II", CAN_ID, 0x7FF)
    sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, can_filter)

    sock.bind((iface,))

    print(f"Listening for CAN ID 0x{CAN_ID:X} on {iface}")
    print(f"Gap threshold: {threshold_ms:.1f} ms")
    print(f"{'Frame#':>7}  {'dt_ms':>8}  {'angle_raw':>10}  {'delta':>8}  {'note'}")
    print("-" * 60)

    last_time = None
    last_raw = None
    count = 0
    gaps = []
    deltas_ms = []

    try:
        while True:
            data = sock.recv(CAN_FRAME_SIZE)
            now = time.monotonic()
            can_id, dlc, can_data = struct.unpack(CAN_FRAME_FMT, data)

            raw = struct.unpack_from("<h", can_data, 0)[0]

            if last_time is not None:
                dt_ms = (now - last_time) * 1000.0
                d_raw = raw - last_raw if last_raw is not None else 0
                deltas_ms.append(dt_ms)

                note = ""
                if dt_ms > threshold_ms:
                    note = f"<< GAP ({dt_ms:.1f} ms)"
                    gaps.append((count, dt_ms, d_raw))

                print(f"{count:7d}  {dt_ms:8.2f}  {raw:10d}  {d_raw:8d}  {note}")
            else:
                print(f"{count:7d}  {'---':>8}  {raw:10d}  {'---':>8}")

            last_time = now
            last_raw = raw
            count += 1

    except KeyboardInterrupt:
        print("\n" + "=" * 60)
        print(f"Total frames: {count}")
        if deltas_ms:
            avg = sum(deltas_ms) / len(deltas_ms)
            mn = min(deltas_ms)
            mx = max(deltas_ms)
            print(f"dt stats:  avg={avg:.2f} ms  min={mn:.2f} ms  max={mx:.2f} ms")
        print(f"Gaps (>{threshold_ms} ms): {len(gaps)}")
        for i, (frame, dt, d_raw) in enumerate(gaps):
            missed = round(dt / 10.0) - 1
            print(f"  [{i+1}] frame {frame}: {dt:.1f} ms gap, value jump {d_raw}, ~{missed} frames missed")


if __name__ == "__main__":
    main()
