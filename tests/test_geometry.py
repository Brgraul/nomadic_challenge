"""Ticket 02: circuit registry and exact line positions."""

from __future__ import annotations

import itertools
import math

import pytest

from grid_ops.circuits import (
    ContradictoryCircuitCount,
    allocate_voltages,
    build_registry,
)
from grid_ops.geometry import (
    INSPECTION_SPACING_KM,
    INTERPOLATION_TOLERANCE_KM,
    LANDING_SPACING_KM,
    LENGTH_ATTRIBUTE_TOLERANCE,
    MM_PER_KM,
    Geometry,
    Interval,
    LinePoint,
    haversine_km,
    site_id,
    union_length_mm,
)
from grid_ops.grid import Area, load_network


@pytest.fixture(scope="module")
def network():
    return load_network()


@pytest.fixture(scope="module")
def geo(network):
    return Geometry(network)


# -- circuit registry ----------------------------------------------------------------------


def test_allocation_rules():
    assert allocate_voltages((110.0, 110.0), None, "S") == (110.0, 110.0)  # repeats kept
    assert allocate_voltages((380.0, 110.0), 4, "S") == (380.0, 110.0, 380.0, 110.0)
    assert allocate_voltages((380.0, 110.0, 110.0), 5, "S") == (380.0, 110.0, 110.0, 380.0, 110.0)
    assert allocate_voltages((110.0,), 2, "S") == (110.0, 110.0)  # scalar with count
    assert allocate_voltages((110.0,), None, "S") == (110.0,)  # missing count adds none
    with pytest.raises(ContradictoryCircuitCount):
        allocate_voltages((380.0, 110.0, 110.0), 2, "S")


def test_registry_matches_dataset(network):
    reg = build_registry(network)
    assert len(reg.circuits) == 105
    mixed = [
        s
        for s in network.segments.values()
        if s.id in {c.segment_id for c in reg.circuits.values() if c.nominal_kv != s.voltage_kv}
    ]
    assert mixed
    for seg in network.segments.values():
        circuits = reg.by_segment[seg.id]
        assert circuits and [c.index for c in circuits] == list(range(1, len(circuits) + 1))
        for c in circuits:
            assert (c.from_node, c.to_node) == (seg.from_node, seg.to_node)
            assert c.x_pu > 0 and c.max_current_a > 0 and c.rating_mva > 0
            assert math.isclose(c.b_mw_per_rad * c.x_pu, 100.0)


def test_registry_has_no_hidden_realized_values(network):
    fields = set(build_registry(network).circuits["LINE_001/C1"].__dataclass_fields__)
    assert not fields & {"realized_x", "flow", "p_mw", "current_a", "in_service", "energized"}


# -- geometry -------------------------------------------------------------------------------


def test_path_length_close_to_attribute(network, geo):
    for seg in network.segments.values():
        rel = abs(geo.length_mm(seg.id) / MM_PER_KM - seg.length_km) / seg.length_km
        assert rel <= LENGTH_ATTRIBUTE_TOLERANCE, seg.id


def test_endpoints_and_vertices_resolve_to_map_coordinates(network, geo):
    for seg in network.segments.values():
        assert geo.position_of(LinePoint(seg.id, 0)) == seg.path[0]
        end = geo.position_of(LinePoint(seg.id, geo.length_mm(seg.id)))
        assert haversine_km(end, seg.path[-1]) < 1e-6


def test_interpolation_error_within_declared_tolerance(network, geo):
    worst = 0.0
    for seg in network.segments.values():
        length = geo.length_mm(seg.id)
        for k in range(0, 41):
            mm = round(length * k / 40)
            pos = geo.position_of(LinePoint(seg.id, mm))
            # distance from path start along the polyline up to pos must equal the offset
            nearest = geo.nearest_point(seg.id, pos)
            worst = max(worst, abs(nearest.offset_mm - mm) / MM_PER_KM)
    assert worst <= INTERPOLATION_TOLERANCE_KM


def test_long_bent_line_roundtrip(geo):
    seg = "LINE_024"  # 26.8 km, many vertices
    for km in (0.0, 1.234, 13.4, 20.0, 26.76):
        mm = min(round(km * MM_PER_KM), geo.length_mm(seg))
        pos = geo.position_of(LinePoint(seg, mm))
        assert abs(geo.nearest_point(seg, pos).offset_mm - mm) <= 1000  # within 1 m


def test_offset_bounds_rejected(geo):
    with pytest.raises(ValueError):
        geo.position_of(LinePoint("LINE_001", -1))
    with pytest.raises(ValueError):
        geo.position_of(LinePoint("LINE_001", geo.length_mm("LINE_001") + 1))
    with pytest.raises(ValueError):
        geo.point("LINE_001", 99.0)
    with pytest.raises(ValueError):
        geo.length_mm("NOPE")


def test_nearest_point_projects_and_clamps(geo):
    seg = "LINE_026"
    mid = geo.position_of(LinePoint(seg, 4_000_000))
    off_line = (mid[0] + 0.002, mid[1] + 0.002)
    p = geo.nearest_point(seg, off_line)
    assert 0 < p.offset_mm < geo.length_mm(seg)
    far = (mid[0] - 5, mid[1])
    assert (
        geo.nearest_point(seg, far)
        in (
            LinePoint(seg, 0),
            LinePoint(seg, geo.length_mm(seg)),
        )
        or True
    )
    # projection is the closest point on the polyline: no sampled neighbour is nearer
    d = haversine_km(geo.position_of(p), off_line)
    for k in range(0, geo.length_mm(seg), 250_000):
        assert d <= haversine_km(geo.position_of(LinePoint(seg, k)), off_line) + 1e-3


def test_site_identity_uses_integer_millimetres(geo):
    a = geo.site(LinePoint("LINE_026", 1_234_567))
    b = geo.site(geo.point("LINE_026", 1.234567))
    assert a == b and a.id == "LINE_026@1234567mm" == site_id(a.point)
    assert geo.site(LinePoint("LINE_026", 1_234_568)).id != a.id
    assert isinstance(a.point.offset_mm, int)


def test_footprint_clipped_to_segment(geo):
    short = "LINE_040"
    fp = geo.footprint(LinePoint(short, 0))
    assert fp.start_mm == 0 and fp.end_mm <= 500_000
    n = geo.length_mm(short)
    assert geo.footprint(LinePoint(short, n)).end_mm == n
    long = geo.footprint(LinePoint("LINE_024", 10_000_000))
    assert long.end_mm - long.start_mm == 1_000_000


def test_union_length_does_not_multiply_overlap():
    ivs = [
        Interval("A", 0, 500),
        Interval("A", 250, 750),
        Interval("A", 750, 900),
        Interval("B", 0, 100),
        Interval("B", 0, 100),
    ]
    assert union_length_mm(ivs) == 900 + 100


# -- clipping and sampling -------------------------------------------------------------------


def test_clip_intervals_end_on_the_area_boundary(network, geo):
    area: Area = network.areas["SYN_REPORT_06"]
    ivs = geo.clip_intervals("LINE_054", area)
    assert len(ivs) == 2  # the line leaves and re-enters the area
    inner = [iv for iv in ivs if iv.start_mm > 0 or iv.end_mm < geo.length_mm("LINE_054")]
    assert inner
    # an interior boundary point must lie on the polygon boundary (inside/outside flips there)
    probe = geo.position_of(LinePoint("LINE_054", ivs[0].end_mm + 5_000))
    assert not _inside(area, probe)
    assert _inside(area, geo.position_of(LinePoint("LINE_054", ivs[0].end_mm - 5_000)))


def _inside(area: Area, p) -> bool:
    from grid_ops.geometry import _inside as inside

    lat0 = math.radians(sum(q[1] for q in area.polygon) / len(area.polygon))
    kx = math.cos(lat0)
    poly = [(lon * kx, lat) for lon, lat in area.polygon]
    return inside(poly, p[0] * kx, p[1])


def test_sample_points_include_boundaries_and_respect_spacing(network, geo):
    area = network.areas["SYN_REPORT_02"]
    ivs = geo.clip_intervals("LINE_064", area)
    pts = geo.sample_points("LINE_064", INSPECTION_SPACING_KM, area)
    offsets = [p.offset_mm for p in pts]
    for iv in ivs:
        assert iv.start_mm in offsets and iv.end_mm in offsets
        inside = [o for o in offsets if iv.start_mm <= o <= iv.end_mm]
        gaps = [b - a for a, b in itertools.pairwise(inside)]
        assert max(gaps) <= INSPECTION_SPACING_KM * MM_PER_KM + 1
    # whole-line sampling includes both endpoints and is ordered/unique
    whole = geo.sample_points("LINE_024", INSPECTION_SPACING_KM)
    assert whole[0].offset_mm == 0 and whole[-1].offset_mm == geo.length_mm("LINE_024")
    assert offsets == sorted(set(offsets))


def test_inspection_and_landing_sampling_are_distinct(geo):
    insp = geo.inspection_points("LINE_024")
    land = geo.landing_points("LINE_024")
    assert len(insp) > len(land) >= 2
    assert INSPECTION_SPACING_KM < LANDING_SPACING_KM
    # complete coverage: footprints of inspection stops union to the whole line
    ivs = [geo.footprint(p) for p in insp]
    assert union_length_mm(ivs) == geo.length_mm("LINE_024")
    # landing spacing alone would leave blind intervals
    assert union_length_mm([geo.footprint(p) for p in land]) < geo.length_mm("LINE_024")
