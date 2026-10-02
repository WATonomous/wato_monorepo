"""Declarative scenario/map bundles. All paths are local to the ROS container."""

from dataclasses import dataclass
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from carla_common.lanelet_track import LaneletTrack, finite, opendrive, validate_carla_map


@dataclass
class MapBundle:
    id: str
    data: dict
    root: Path
    xodr_path: Path | None = None

    @property
    def lanelet(self):
        return self.data["lanelet2"]

    @property
    def osm_path(self):
        return self.resolve(self.lanelet["path"])

    @property
    def origin(self):
        return self.lanelet["origin"]

    def resolve(self, path):
        path = Path(path)
        return path if path.is_absolute() else self.root / path

    def prepare(self):
        """Validate before the outgoing world is touched, and cache conversion."""
        if not self.osm_path.is_file():
            raise ValueError(f"missing Lanelet2 map: {self.osm_path}; run watod-map.sh first")
        osm = ET.parse(self.osm_path).getroot()
        if osm.tag != "osm" or not osm.findall("relation/tag[@k='type'][@v='lanelet']"):
            raise ValueError("map bundle requires Lanelet2 OSM relations")
        if self.lanelet.get("projector", "local_cartesian") != "local_cartesian":
            raise ValueError("simulation bundles currently require local_cartesian projection")
        for axis, limit in (("lat", 90), ("lon", 180)):
            if abs(finite(self.origin[axis], f"origin {axis}")) > limit:
                raise ValueError("invalid geographic origin")
        source = self.data["carla"]
        choices = [key for key in ("built_in_map", "xodr_path", "from_lanelet2") if source.get(key)]
        if len(choices) != 1:
            raise ValueError("specify exactly one CARLA map source")
        if source.get("from_lanelet2"):
            track = LaneletTrack(self.osm_path, self.origin["lat"], self.origin["lon"])
            # Include converter source in the digest so implementation changes invalidate caches.
            from carla_common import lanelet_track
            digest = hashlib.sha256(self.osm_path.read_bytes() + repr(self.origin).encode()
                                    + Path(lanelet_track.__file__).read_bytes()).hexdigest()[:20]
            self.xodr_path = self.root / ".cache" / f"{self.id}-{digest}.xodr"
            if not self.xodr_path.is_file():
                generated = opendrive(track)
                self.xodr_path.parent.mkdir(parents=True, exist_ok=True)
                temporary = self.xodr_path.with_suffix(".tmp")
                temporary.write_text(generated, encoding="utf-8")
                temporary.replace(self.xodr_path)
        elif source.get("xodr_path"):
            self.xodr_path = self.resolve(source["xodr_path"])
        if self.xodr_path:
            root = ET.parse(self.xodr_path).getroot()
            if root.tag != "OpenDRIVE" or not root.findall("road"):
                raise ValueError("XODR must contain OpenDRIVE roads")
            track = LaneletTrack(self.osm_path, self.origin["lat"], self.origin["lon"])
            validate_carla_map(self.xodr_path.read_text(), track)
        if "ego_spawn" in self.data:
            track = LaneletTrack(self.osm_path, self.origin["lat"], self.origin["lon"])
            spawn = self.data["ego_spawn"]
            lane = track.lanes[int(spawn["lanelet_id"])]
            if not 0 <= finite(spawn.get("distance_m", 0)) <= lane.center.length:
                raise ValueError("ego spawn distance is outside the lanelet")
            if not 0 < finite(spawn.get("z", 0.4)) <= 5:
                raise ValueError("ego spawn z must be 0..5m above the flat track")
        return self


class Registry:
    def __init__(self, path, maps_root):
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        if data.get("version") != 1:
            raise ValueError("scenario registry version must be 1")
        self.maps = {key: MapBundle(key, value, Path(maps_root)) for key, value in data["maps"].items()}
        self.scenarios = data["scenarios"]
        for scenario in self.scenarios.values():
            if scenario["map"] not in self.maps:
                raise ValueError(f"unknown map: {scenario['map']}")

    def scenario(self, name):
        # Retain existing Foxglove requests that use Python module paths.
        if name not in self.scenarios:
            matches = [key for key, value in self.scenarios.items() if value["module"] == name]
            if len(matches) != 1:
                raise ValueError(f"unknown scenario: {name}")
            name = matches[0]
        return name, self.scenarios[name], self.maps[self.scenarios[name]["map"]]
