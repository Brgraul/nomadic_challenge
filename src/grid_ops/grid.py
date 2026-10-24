"""Static grid network from the participant data package, and its pandapower model."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

DATA_DIR = Path(__file__).resolve().parents[2] / "data"

LonLat = tuple[float, float]


@dataclass(frozen=True)
class Node:
    id: str
    type: str
    name: str | None
    component: str
    position: LonLat


@dataclass(frozen=True)
class Segment:
    id: str
    component: str
    from_node: str
    to_node: str
    voltage_kv: float
    circuits: int | None
    length_km: float
    path: tuple[LonLat, ...]

    @property
    def midpoint(self) -> LonLat:
        if len(self.path) > 2:
            return self.path[len(self.path) // 2]
        (lon1, lat1), (lon2, lat2) = self.path[0], self.path[-1]
        return ((lon1 + lon2) / 2, (lat1 + lat2) / 2)


@dataclass(frozen=True)
class Area:
    id: str
    kind: str
    label: str
    polygon: tuple[LonLat, ...]


@dataclass(frozen=True)
class Report:
    id: str
    type: str
    description: str
    timestamp: str
    confidence: str
    area_id: str


@dataclass(frozen=True)
class Network:
    nodes: dict[str, Node]
    segments: dict[str, Segment]
    areas: dict[str, Area]
    reports: dict[str, Report]

    def segments_at(self, node_id: str) -> list[Segment]:
        return [s for s in self.segments.values() if node_id in (s.from_node, s.to_node)]


def _features(filename: str) -> list[dict]:
    return json.loads((DATA_DIR / filename).read_text())["features"]


def load_network() -> Network:
    nodes = {
        p["node_id"]: Node(
            id=p["node_id"],
            type=p["node_type"],
            name=p["name"],
            component=p["component_id"],
            position=(p["lon"], p["lat"]),
        )
        for p in (f["properties"] for f in _features("grid_nodes.geojson"))
    }
    segments = {
        f["properties"]["segment_id"]: Segment(
            id=f["properties"]["segment_id"],
            component=f["properties"]["component_id"],
            from_node=f["properties"]["from_node"],
            to_node=f["properties"]["to_node"],
            voltage_kv=f["properties"]["voltage_kv"],
            circuits=f["properties"]["circuits"],
            length_km=f["properties"]["length_km"],
            path=tuple((lon, lat) for lon, lat in f["geometry"]["coordinates"]),
        )
        for f in _features("grid_segments.geojson")
    }
    areas = {
        f["properties"]["context_id"]: Area(
            id=f["properties"]["context_id"],
            kind=f["properties"]["context_kind"],
            label=f["properties"]["label"],
            polygon=tuple((lon, lat) for lon, lat in f["geometry"]["coordinates"][0]),
        )
        for f in _features("scenario_context.geojson")
    }
    config = json.loads((DATA_DIR / "scenario_public_config.json").read_text())
    reports = {
        r["information_id"]: Report(
            id=r["information_id"],
            type=r["information_type"],
            description=r["description"],
            timestamp=r["timestamp_relative_to_storm"],
            confidence=r["confidence"],
            area_id=r["information_id"],
        )
        for r in config["public_context"]["reports"]
    }
    return Network(nodes=nodes, segments=segments, areas=areas, reports=reports)
