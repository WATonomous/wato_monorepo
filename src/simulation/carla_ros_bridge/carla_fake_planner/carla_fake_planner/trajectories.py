"""Preset geometry and forward-only progress, with no ROS/CARLA dependency."""

import copy
import math

from carla_common.lanelet_track import finite


COMMON = {"speed_kph", "speed_profile", "horizon_m", "sample_spacing_m"}
PARAMETERS = {
    "follow_lane": COMMON,
    "snake": COMMON | {"length_m", "amplitude_m", "wavelength_m", "end_stop_distance_m"},
    "hard_brake": COMMON | {"length_m", "brake_start_m", "brake_distance_m", "end_stop_distance_m"},
}


def parameters(preset, overrides):
    generator = preset["generator"]
    if generator not in PARAMETERS:
        raise ValueError(f"unknown generator: {generator}")
    if not isinstance(overrides, dict):
        raise ValueError("parameters_yaml must contain a mapping")
    result = copy.deepcopy(preset["parameters"])
    result.update(overrides)
    unknown = set(result) - PARAMETERS[generator]
    if unknown:
        raise ValueError(f"unsupported parameters: {sorted(unknown)}")
    for name, value in result.items():
        if name != "speed_profile":
            result[name] = finite(value, name)
            if result[name] < 0:
                raise ValueError(f"{name} must be nonnegative")
    spacing = result.get("sample_spacing_m", 0.25)
    horizon = result.get("horizon_m", 50.0)
    if not 0.05 <= spacing <= 2.0 or not 5.0 <= horizon <= 100.0:
        raise ValueError("sample_spacing_m must be 0.05..2 and horizon_m 5..100")
    if horizon / spacing > 2000:
        raise ValueError("forward window exceeds 2000 samples")
    if generator != "follow_lane" and not 5.0 <= result.get("length_m", 80.0) <= 10000:
        raise ValueError("length_m must be 5..10000")
    if generator != "follow_lane" and not 3.0 <= result.get("end_stop_distance_m", 5.0) < result.get("length_m", 80.0):
        raise ValueError("end_stop_distance_m must be >=3 and less than length_m")
    if generator == "snake" and (result.get("wavelength_m", 20.0) < 5
                                  or result.get("amplitude_m", 0.8) > 3):
        raise ValueError("snake wavelength must be >=5m and amplitude <=3m")
    if generator == "hard_brake":
        end = result.get("brake_start_m", 40.0) + result.get("brake_distance_m", 2.0)
        if end >= result.get("length_m", 80.0) - 3.0:
            raise ValueError("brake transition must end at least 3m before path end")
    profile = result.get("speed_profile", [])
    if not isinstance(profile, list):
        raise ValueError("speed_profile must be a list of distance/speed knots")
    previous = -1.0
    for knot in profile:
        distance = finite(knot["distance_m"], "speed profile distance")
        speed = finite(knot["speed_kph"], "speed profile speed")
        if distance <= previous or distance < 0 or not 0 <= speed <= 50:
            raise ValueError("speed knots must have increasing nonnegative distances and speeds 0..50 km/h")
        previous = distance
        knot["distance_m"], knot["speed_kph"] = distance, speed
    if not 0 <= result.get("speed_kph", 20.0) <= 50:
        raise ValueError("speed_kph must be 0..50")
    return result


class Run:
    def __init__(self, track, pose, preset, overrides):
        self.generator = preset["generator"]
        self.params = parameters(preset, overrides)
        lane_id, self.start = track.nearest(*pose)
        self.path, self.closed = track.chain(lane_id)
        if self.generator == "follow_lane" and not self.closed:
            raise ValueError("continuous follow_lane requires a closed lane chain")
        self.length = math.inf if self.generator == "follow_lane" else self.params.get("length_m", 80.0)
        if not self.closed and self.start + self.length > self.path.length:
            raise ValueError("requested trajectory extends beyond the lane chain")
        self.progress = 0.0
        x, y, yaw = self.path.at(self.start, self.closed)
        self.initial_lateral = -(pose[0] - x) * math.sin(yaw) + (pose[1] - y) * math.cos(yaw)

    def advance(self, pose):
        """Project only onto the next short interval, never onto an old lap."""
        start = self.start + self.progress
        limit = min(start + 15.0, self.start + self.length)
        best = (math.inf, start)
        # At most two intervals at the lap seam; retain unwrapped progress.
        first_lap = int(start // self.path.length) if self.closed else 0
        last_lap = int(limit // self.path.length) if self.closed else 0
        for lap in range(first_lap, last_lap + 1):
            offset = lap * self.path.length
            distance, along, _ = self.path.project(pose[0], pose[1],
                                                  max(0.0, start - offset),
                                                  min(self.path.length, limit - offset))
            if distance < best[0]:
                best = (distance, offset + along)
        if best[0] > 8.0:
            raise ValueError("ego left the trajectory neighborhood; select again from its current pose")
        self.progress = max(self.progress, best[1] - self.start)

    def speed(self, distance):
        speed = self.params.get("speed_kph", 20.0)
        profile = self.params.get("speed_profile", [])
        if profile:
            speed = profile[-1]["speed_kph"]
            if distance <= profile[0]["distance_m"]:
                speed = profile[0]["speed_kph"]
            else:
                for a, b in zip(profile, profile[1:]):
                    if a["distance_m"] <= distance <= b["distance_m"]:
                        t = (distance - a["distance_m"]) / (b["distance_m"] - a["distance_m"])
                        speed = a["speed_kph"] + t * (b["speed_kph"] - a["speed_kph"])
                        break
        if self.generator == "hard_brake":
            start = self.params.get("brake_start_m", 40.0)
            transition = self.params.get("brake_distance_m", 2.0)
            if distance >= start:
                speed *= max(0.0, 1.0 - (distance - start) / transition) if transition else 0.0
        if distance >= self.length - self.params.get("end_stop_distance_m", 5.0):
            speed = 0.0
        return speed / 3.6

    def position(self, distance):
        x, y, yaw = self.path.at(self.start + distance, self.closed)
        if self.generator == "snake":
            wavelength = self.params.get("wavelength_m", 20.0)
            envelope = min(1.0, distance / 5.0, max(0.0, (self.length - distance) / 5.0))
            lateral = self.params.get("amplitude_m", 0.8) * math.sin(2 * math.pi * distance / wavelength) * envelope
            lateral += self.initial_lateral * max(0.0, 1.0 - distance / 5.0)
            x -= lateral * math.sin(yaw)
            y += lateral * math.cos(yaw)
        return x, y

    def samples(self, full=False):
        start = 0.0 if full else self.progress
        if start >= self.length:
            return []
        end = ((self.path.length if math.isinf(self.length) else self.length) if full else
               min(self.length, start + self.params.get("horizon_m", 50.0)))
        spacing = self.params.get("sample_spacing_m", 0.25)
        if full:
            spacing = max(spacing, (end - start) / 2000)
        count = max(2, math.ceil((end - start) / spacing) + 1)
        result = []
        for i in range(count):
            distance = min(start + i * spacing, end)
            x, y = self.position(distance)
            # Include the lateral derivative for snake yaw.
            a, b = self.position(max(0.0, distance - 0.02)), self.position(min(distance + 0.02, self.length))
            yaw = math.atan2(b[1] - a[1], b[0] - a[0])
            result.append((x, y, yaw, self.speed(distance)))
        return result
