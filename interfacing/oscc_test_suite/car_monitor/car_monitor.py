#!/usr/bin/env python3
"""
listen_can.py

Passive CAN bus listener for wheel speed, brake pressure, and steering wheel angle.

This script opens a CAN interface (defaults to socketcan can0) and starts three
threads that monitor incoming frames for the same arbitration IDs used by
`oscccan/canbus.py` in this repo. The main thread prints the latest values once
per 0.2s using carriage returns so the terminal is not flooded.

Usage: python3 listen_can.py [--channel can0] [--bustype socketcan_native]

Note: Requires python-can if you want to connect to a real CAN bus. For unit
tests or offline use, you can mock the `can` interface.
"""
import argparse
import can
import struct
import threading
import time
import sys

# The arbitration IDs match those in oscccan/canbus.py
BRAKE_PRESSURE_IDS = [0x220]
STEERING_ANGLE_IDS = [0x2B0]
WHEEL_SPEED_IDS = [0x4B0, 0x386]


class LatestValues:
    """Thread-safe container for the latest sensor values."""

    def __init__(self):
        self.lock = threading.Lock()
        self.brake = None
        self.steering = None
        self.wheels = None

    def update_brake(self, value):
        with self.lock:
            self.brake = value

    def update_steering(self, value):
        with self.lock:
            self.steering = value

    def update_wheels(self, value):
        with self.lock:
            self.wheels = value

    def snapshot(self):
        with self.lock:
            return self.brake, self.steering, self.wheels


def get_wheel_speed(data, offset):
    """Same unpacking logic as in oscccan.canbus.CanBus.get_wheel_speed"""
    byte1 = (data[offset + 1] & 0x0F) << 8
    byte0 = data[offset]
    value = int(str(byte1 | byte0), 10)
    return float(value) / 10.0


def brake_thread(bus, latest, vehicle='soul_ev', timeout=1.0):
    """Listen for brake pressure frames and update latest.brake"""
    while not stop_event.is_set():
        msg = bus.recv(timeout)
        if msg is None:
            continue
        if msg.arbitration_id not in BRAKE_PRESSURE_IDS:
            continue

        # Parse depending on vehicle type (keep same logic as canbus.py)
        try:
            if vehicle == 'kia_niro':
                byte1 = (msg.data[4] & 0x0F) << 8
                byte0 = msg.data[3]
                value = int(str(byte1 | byte0), 10) / 40.0
            else:
                byte1 = (msg.data[5] & 0x0F) << 8
                byte0 = msg.data[4]
                value = int(str(byte1 | byte0), 10) / 10.0
        except Exception:
            # Malformed frame
            continue

        latest.update_brake(value)


def steering_thread(bus, latest, timeout=1.0):
    """Listen for steering wheel angle frames and update latest.steering"""
    while not stop_event.is_set():
        msg = bus.recv(timeout)
        if msg is None:
            continue
        if msg.arbitration_id not in STEERING_ANGLE_IDS:
            continue

        try:
            value = -float(struct.unpack("=h", msg.data[:2])[0]) / 10.0
        except Exception:
            continue

        latest.update_steering(value)


def wheels_thread(bus, latest, timeout=1.0):
    """Listen for wheel speed frames and update latest.wheels as list"""
    while not stop_event.is_set():
        msg = bus.recv(timeout)
        if msg is None:
            continue
        if msg.arbitration_id not in WHEEL_SPEED_IDS:
            continue

        try:
            left_front = get_wheel_speed(msg.data, 0)
            right_front = get_wheel_speed(msg.data, 2)
            left_rear = get_wheel_speed(msg.data, 4)
            right_rear = get_wheel_speed(msg.data, 6)
            latest.update_wheels([left_front, right_front, left_rear, right_rear])
        except Exception:
            continue


def main():
    parser = argparse.ArgumentParser(description='Passive CAN listener')
    parser.add_argument('--bustype', default='socketcan_native')
    parser.add_argument('--channel', default='can0')
    parser.add_argument('--bitrate', default=500000, type=int)
    parser.add_argument('--vehicle', default='soul_ev', help='kia_niro or other')
    args = parser.parse_args()

    try:
        bus = can.interface.Bus(interface=args.bustype, channel=args.channel, bitrate=args.bitrate)
    except Exception as e:
        print('Unable to open CAN interface: {}'.format(e), file=sys.stderr)
        sys.exit(1)

    latest = LatestValues()

    # Start threads
    threads = [
        threading.Thread(target=brake_thread, args=(bus, latest, args.vehicle), daemon=True),
        threading.Thread(target=steering_thread, args=(bus, latest), daemon=True),
        threading.Thread(target=wheels_thread, args=(bus, latest), daemon=True),
    ]

    for t in threads:
        t.start()

    try:
        while not stop_event.is_set():
            brake, steering, wheels = latest.snapshot()
            # Build a single-line output and use carriage return to overwrite
            out = 'Brake: {brake:<6} | Steering: {steer:<6} | Wheels: {wheels}'
            out = out.format(
                brake=('{:.2f}'.format(brake) if brake is not None else 'None'),
                steer=('{:.2f}'.format(steering) if steering is not None else 'None'),
                wheels=((','.join('{:.1f}'.format(w) for w in wheels) if wheels is not None else 'None'))
            )

            # Print with carriage return and flush, keep cursor on same line
            print('\r' + out, end='')
            sys.stdout.flush()
            time.sleep(0.2)
    except KeyboardInterrupt:
        stop_event.set()
        print('\nStopping...')


if __name__ == '__main__':
    # Global stop event used by threads
    stop_event = threading.Event()
    main()
