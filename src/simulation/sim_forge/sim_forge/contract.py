"""Explicit field mapping. Never infer simulator coordinates or state array indices."""
from dataclasses import dataclass
import math


class ContractError(ValueError):
    pass


def at(document, pointer):
    if not isinstance(pointer, str) or not pointer.startswith("/"):
        raise ContractError(f"Configure a JSON pointer, got {pointer!r}")
    value = document
    try:
        for token in pointer[1:].split("/"):
            token = token.replace("~1", "/").replace("~0", "~")
            value = value[int(token)] if isinstance(value, (list, tuple)) else value[token]
    except (KeyError, IndexError, TypeError, ValueError) as exc:
        raise ContractError(f"Runtime field missing: {pointer}") from exc
    return value


def finite(value):
    if isinstance(value, bool):
        raise ContractError("Boolean is not a numeric state")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ContractError(f"Expected numeric state, got {value!r}") from exc
    if not math.isfinite(number):
        raise ContractError("Nonfinite state")
    return number


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Ego:
    t: float
    x: float
    y: float
    yaw: float
    speed: float


class Mapper:
    def __init__(self, config):
        if config.get("reviewed") is not True:
            raise ContractError("Capture probe output and review contract.json before bridging")
        self.c = config
        self.m = config["plane_matrix"]
        if not isinstance(self.m, list) or len(self.m) != 2 or any(len(row) != 2 for row in self.m):
            raise ContractError("plane_matrix must be a 2 by 2 orthogonal matrix")
        self.m = [[finite(v) for v in row] for row in self.m]
        a, b = self.m
        if not (math.isclose(sum(v*v for v in a), 1, abs_tol=1e-6)
                and math.isclose(sum(v*v for v in b), 1, abs_tol=1e-6)
                and math.isclose(sum(x*y for x, y in zip(a, b)), 0, abs_tol=1e-6)):
            raise ContractError("plane_matrix must preserve distances")

    def position(self, x, y):
        a, b = self.m
        o = self.c["map_offset_m"]
        return a[0]*x+a[1]*y+finite(o[0]), b[0]*x+b[1]*y+finite(o[1])

    def heading(self, raw):
        h = finite(raw)*finite(self.c["yaw_scale"])+finite(self.c["yaw_offset_rad"])
        a, b = self.m
        return math.atan2(b[0]*math.cos(h)+b[1]*math.sin(h),
                          a[0]*math.cos(h)+a[1]*math.sin(h))

    def ego(self, snapshot):
        fields = self.c["ego"]
        x, y = self.position(finite(at(snapshot, fields["x"])), finite(at(snapshot, fields["y"])))
        yaw = self.heading(at(snapshot, fields["yaw"]))
        # Offset from the runtime reference to the ROS base in vehicle FLU coordinates.
        dx, dy = map(finite, self.c["runtime_origin_to_base_m"])
        x += math.cos(yaw)*dx-math.sin(yaw)*dy
        y += math.sin(yaw)*dx+math.cos(yaw)*dy
        speed = finite(at(snapshot, fields["speed"]))*finite(self.c["speed_scale"])
        return Ego(finite(at(snapshot, self.c["time"])), x, y, yaw, speed)

    def actors(self, snapshot):
        spec = self.c.get("actors")
        if not spec or spec.get("reviewed") is not True:
            raise ContractError("Full actor truth is unconfigured; do not fabricate boxes from compact observations")
        actors = at(snapshot, spec["list"])
        if not isinstance(actors, list):
            raise ContractError("Actor truth must be a list")
        output = []
        seen_ids = set()
        for actor in actors:
            row = {key: at(actor, pointer) for key, pointer in spec["fields"].items()}
            row["x"], row["y"] = self.position(finite(row["x"]), finite(row["y"]))
            row["yaw"] = self.heading(row["yaw"])
            for field in ("z", "length", "width", "height"):
                row[field] = finite(row[field])
            if min(row[k] for k in ("length", "width", "height")) <= 0:
                raise ContractError("Actor dimensions must be positive")
            raw_class = str(row["class"])
            if raw_class not in spec["class_map"]:
                raise ContractError(f"Unmapped actor class {raw_class}")
            row["class"] = spec["class_map"][raw_class]
            if row["class"] == "traffic_light":
                if row.get("signal_state") not in ("red", "yellow", "green"):
                    raise ContractError("Traffic light lacks a supported signal state")
            row["id"] = str(row["id"])
            if not row["id"] or row["id"] in seen_ids:
                raise ContractError("Actor IDs must be nonempty and unique per snapshot")
            seen_ids.add(row["id"])
            output.append(row)
        return output


def command_to_control(speed, steering, ego_speed, config):
    if config.get("reviewed") is not True:
        raise ContractError("Review steering scale and sign with the actual runtime first")
    speed, steering, ego_speed = map(finite, (speed, steering, ego_speed))
    maximum = finite(config["max_speed_mps"])
    limit = finite(config["steer_limit_rad"])
    sign = finite(config["steer_sign"])
    kp = finite(config["speed_kp"])
    if maximum <= 0 or limit <= 0 or kp <= 0 or sign not in (-1, 1):
        raise ContractError("Invalid command conversion parameters")
    if speed < 0 or speed > maximum or abs(steering) > limit:
        raise ContractError("Command exceeds the reviewed forward driving envelope")
    effort = max(-1.0, min(1.0, kp*(speed-ego_speed)))
    throttle, brake = max(0.0, effort), max(0.0, -effort)
    if speed == 0:
        throttle, brake = 0.0, 1.0
    return [throttle, brake, sign*steering/limit]
