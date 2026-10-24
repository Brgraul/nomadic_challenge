"""Exact positions along mapped lines: offsets, projections, clipping and sampling.

Offsets are integer millimetres measured along the mapped geodesic path from `from_node`, so a
site's identity never depends on formatting a floating-point coordinate. The path length can
differ from the source `length_km` attribute by up to `LENGTH_ATTRIBUTE_TOLERANCE` (the source
uses another distance model); electrical parameters use the attribute, positions use the path.
"""

from __future__ import annotations

import bisect
import itertools
import math
from dataclasses import dataclass

from grid_ops.grid import Area, LonLat, Network, Segment

EARTH_RADIUS_KM = 6371.0
MM_PER_KM = 1_000_000

# Declared tolerance: linear lon/lat interpolation inside a path piece vs great-circle distance.
INTERPOLATION_TOLERANCE_KM = 0.001
LENGTH_ATTRIBUTE_TOLERANCE = 0.005
# Local sensors inspect up to this distance along the line either side of the stop, so a
# footprint spans 1 km and complete coverage needs stops no more than 1 km apart (2 km spacing
# would leave 1 km blind intervals).
INSPECTION_RANGE_KM = 0.5
INSPECTION_SPACING_KM = 1.0
# Landing candidates use a coarser, independent grid (augmented elsewhere by exact sites).
LANDING_SPACING_KM = 2.0


def haversine_km(a: LonLat, b: LonLat) -> float:
    lon1, lat1, lon2, lat2 = map(math.radians, (*a, *b))
    h = (
        math.sin((lat2 - lat1) / 2) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin((lon2 - lon1) / 2) ** 2
    )
    return 2 * EARTH_RADIUS_KM * math.asin(math.sqrt(h))


@dataclass(frozen=True, order=True)
class LinePoint:
    """A point on a mapped line, measured along its path from from_node."""

    segment_id: str
    offset_mm: int

    @property
    def offset_km(self) -> float:
        return self.offset_mm / MM_PER_KM


@dataclass(frozen=True)
class LandingSite:
    point: LinePoint
    circuit_id: str


@dataclass(frozen=True)
class Interval:
    """Closed sub-interval of one segment, in integer millimetres."""

    segment_id: str
    start_mm: int
    end_mm: int

    @property
    def length_km(self) -> float:
        return (self.end_mm - self.start_mm) / MM_PER_KM


@dataclass(frozen=True)
class Site:
    """A registered exact location. The id is derived from the integer offset, not coordinates."""

    id: str
    point: LinePoint
    position: LonLat


def site_id(point: LinePoint) -> str:
    return f"{point.segment_id}@{point.offset_mm}mm"


def union_length_mm(intervals: list[Interval]) -> int:
    """Total length of the union of intervals (per segment), without double counting."""
    total = 0
    by_segment: dict[str, list[Interval]] = {}
    for iv in intervals:
        by_segment.setdefault(iv.segment_id, []).append(iv)
    for ivs in by_segment.values():
        end = None
        for iv in sorted(ivs, key=lambda i: i.start_mm):
            if end is None or iv.start_mm > end:
                total += iv.end_mm - iv.start_mm
                end = iv.end_mm
            elif iv.end_mm > end:
                total += iv.end_mm - end
                end = iv.end_mm
    return total


class _Line:
    def __init__(self, segment: Segment) -> None:
        self.segment = segment
        self.vertices = segment.path
        self.cum_km = [0.0]
        for a, b in itertools.pairwise(self.vertices):
            self.cum_km.append(self.cum_km[-1] + haversine_km(a, b))
        self.cum_mm = [c * MM_PER_KM for c in self.cum_km]
        self.length_mm = round(self.cum_mm[-1])

    def position(self, offset_mm: int) -> LonLat:
        if not 0 <= offset_mm <= self.length_mm:
            raise ValueError(
                f"{self.segment.id}: offset {offset_mm} mm outside 0..{self.length_mm} mm"
            )
        if self.length_mm == 0:
            return self.vertices[0]
        i = min(max(bisect.bisect_right(self.cum_mm, offset_mm) - 1, 0), len(self.vertices) - 2)
        span = self.cum_mm[i + 1] - self.cum_mm[i]
        f = 0.0 if span == 0 else (offset_mm - self.cum_mm[i]) / span
        (lon1, lat1), (lon2, lat2) = self.vertices[i], self.vertices[i + 1]
        return (lon1 + (lon2 - lon1) * f, lat1 + (lat2 - lat1) * f)


class Geometry:
    """Position queries over a network. Pure and immutable after construction."""

    def __init__(self, network: Network) -> None:
        self.network = network
        self._lines = {sid: _Line(seg) for sid, seg in network.segments.items()}

    def length_mm(self, segment_id: str) -> int:
        return self._line(segment_id).length_mm

    def _line(self, segment_id: str) -> _Line:
        try:
            return self._lines[segment_id]
        except KeyError:
            raise ValueError(f"Unknown segment {segment_id}.") from None

    def point(self, segment_id: str, offset_km: float) -> LinePoint:
        line = self._line(segment_id)
        mm = round(offset_km * MM_PER_KM)
        if not 0 <= mm <= line.length_mm:
            raise ValueError(f"{segment_id}: offset {offset_km} km outside the line")
        return LinePoint(segment_id, mm)

    def position_of(self, point: LinePoint) -> LonLat:
        return self._line(point.segment_id).position(point.offset_mm)

    def site(self, point: LinePoint) -> Site:
        return Site(site_id(point), point, self.position_of(point))

    # -- projection -----------------------------------------------------------------------

    def nearest_point(self, segment_id: str, near: LonLat) -> LinePoint:
        """Exact projection of `near` onto the line, to the nearest millimetre."""
        line = self._line(segment_id)
        lat0 = math.radians(near[1])
        kx = math.cos(lat0) * math.radians(1) * EARTH_RADIUS_KM
        ky = math.radians(1) * EARTH_RADIUS_KM
        px, py = near[0] * kx, near[1] * ky
        best = (math.inf, 0.0)
        for i, (a, b) in enumerate(itertools.pairwise(line.vertices)):
            ax, ay, bx, by = a[0] * kx, a[1] * ky, b[0] * kx, b[1] * ky
            dx, dy = bx - ax, by - ay
            seg2 = dx * dx + dy * dy
            t = 0.0 if seg2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg2))
            d = math.hypot(px - (ax + t * dx), py - (ay + t * dy))
            if d < best[0]:
                best = (d, line.cum_mm[i] + t * (line.cum_mm[i + 1] - line.cum_mm[i]))
        return LinePoint(segment_id, min(max(round(best[1]), 0), line.length_mm))

    # -- intervals ------------------------------------------------------------------------

    def footprint(self, point: LinePoint, range_km: float = INSPECTION_RANGE_KM) -> Interval:
        """Nominal inspection footprint centred on the point, clipped to the segment."""
        half = round(range_km * MM_PER_KM)
        length = self.length_mm(point.segment_id)
        return Interval(
            point.segment_id, max(0, point.offset_mm - half), min(length, point.offset_mm + half)
        )

    def path_of(self, interval: Interval) -> list[LonLat]:
        """Map coordinates tracing the interval, including interior path vertices."""
        line = self._line(interval.segment_id)
        inner = [
            v
            for v, c in zip(line.vertices, line.cum_mm, strict=True)
            if interval.start_mm < c < interval.end_mm
        ]
        return [line.position(interval.start_mm), *inner, line.position(interval.end_mm)]

    def clip_intervals(self, segment_id: str, area: Area) -> list[Interval]:
        """Sub-intervals of the line inside `area` (boundary crossings included exactly)."""
        line = self._line(segment_id)
        lat0 = math.radians(sum(p[1] for p in area.polygon) / len(area.polygon))
        kx = math.cos(lat0) * math.radians(1) * EARTH_RADIUS_KM
        ky = math.radians(1) * EARTH_RADIUS_KM
        poly = [(lon * kx, lat * ky) for lon, lat in area.polygon]
        edges = list(zip(poly, poly[1:] + poly[:1], strict=True))

        intervals: list[tuple[float, float]] = []
        for i, (a, b) in enumerate(itertools.pairwise(line.vertices)):
            ax, ay, bx, by = a[0] * kx, a[1] * ky, b[0] * kx, b[1] * ky
            ts = {0.0, 1.0}
            for (cx, cy), (dx_, dy_) in edges:
                t = _segment_cross(ax, ay, bx, by, cx, cy, dx_, dy_)
                if t is not None:
                    ts.add(t)
            span = line.cum_mm[i + 1] - line.cum_mm[i]
            for t0, t1 in itertools.pairwise(sorted(ts)):
                mid = (t0 + t1) / 2
                if _inside(poly, ax + (bx - ax) * mid, ay + (by - ay) * mid):
                    intervals.append((line.cum_mm[i] + t0 * span, line.cum_mm[i] + t1 * span))
        merged: list[list[float]] = []
        for s, e in sorted(intervals):
            if merged and s <= merged[-1][1] + 1:
                merged[-1][1] = max(merged[-1][1], e)
            else:
                merged.append([s, e])
        return [
            Interval(segment_id, max(0, round(s)), min(line.length_mm, round(e))) for s, e in merged
        ]

    def sample_points(
        self, segment_id: str, every_km: float, inside: Area | None = None
    ) -> list[LinePoint]:
        """Points no more than `every_km` apart, always including interval boundaries."""
        line = self._line(segment_id)
        intervals = (
            [Interval(segment_id, 0, line.length_mm)]
            if inside is None
            else self.clip_intervals(segment_id, inside)
        )
        step_mm = every_km * MM_PER_KM
        points: set[int] = set()
        for iv in intervals:
            span = iv.end_mm - iv.start_mm
            n = max(1, math.ceil(span / step_mm)) if span > 0 else 0
            for k in range(n + 1):
                points.add(iv.start_mm + round(span * k / n) if n else iv.start_mm)
        return [LinePoint(segment_id, mm) for mm in sorted(points)]

    def inspection_points(self, segment_id: str, inside: Area | None = None) -> list[LinePoint]:
        return self.sample_points(segment_id, INSPECTION_SPACING_KM, inside)

    def landing_points(self, segment_id: str) -> list[LinePoint]:
        return self.sample_points(segment_id, LANDING_SPACING_KM)


def _segment_cross(ax, ay, bx, by, cx, cy, dx, dy) -> float | None:
    """Parameter t along A->B where it crosses C->D, or None."""
    r = (bx - ax, by - ay)
    s = (dx - cx, dy - cy)
    denom = r[0] * s[1] - r[1] * s[0]
    if denom == 0:
        return None
    t = ((cx - ax) * s[1] - (cy - ay) * s[0]) / denom
    u = ((cx - ax) * r[1] - (cy - ay) * r[0]) / denom
    if 0 <= t <= 1 and 0 <= u <= 1:
        return t
    return None


def _inside(poly: list[tuple[float, float]], x: float, y: float) -> bool:
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside
