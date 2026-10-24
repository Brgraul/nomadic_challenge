"""Hidden local fields: clearance, temperature and conductor motion along lines.

A LocalField is indexed by segment and along-line interval. Values and event times are scenario
inputs, separate from the AC steady state. Sensing reads them through the declared likelihood
model in `state.py`; nothing here is ever copied into public state.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field

from grid_ops.geometry import MM_PER_KM, Geometry, Interval
from grid_ops.scenario import ScenarioConfig, ScenarioMode

BASE_CLEARANCE_M = 2.0
BASE_MOTION_GUST_M = 0.05  # comparison amplitude during a qualifying gust
BASE_MOTION_CALM_M = 0.01
HOT_JOINT_EXCESS_C = 15.0
VEGETATION_CLEARANCE_M = 0.4
ABNORMAL_MOTION_M = 0.30


@dataclass(frozen=True)
class Anomaly:
    kind: str  # "vegetation" | "hot_joint" | "motion"
    interval: Interval
    value: float  # clearance m, excess C or amplitude m


@dataclass(frozen=True)
class ConditionWindow:
    """A scheduled window: e.g. light local loading (factor) or a qualifying gust."""

    kind: str  # "load" | "gust"
    segment_id: str | None
    start_s: int
    end_s: int
    factor: float = 1.0


@dataclass
class LocalField:
    anomalies: list[Anomaly] = field(default_factory=list)
    occluded: list[Interval] = field(default_factory=list)
    windows: list[ConditionWindow] = field(default_factory=list)

    def gust_at(self, t_s: int) -> bool:
        return any(w.kind == "gust" and w.start_s <= t_s < w.end_s for w in self.windows)

    def load_factor(self, segment_id: str, t_s: int) -> float:
        factor = 1.0
        for w in self.windows:
            if w.kind == "load" and w.segment_id == segment_id and w.start_s <= t_s < w.end_s:
                factor *= w.factor
        return factor


def build_local_field(geometry: Geometry, network, config: ScenarioConfig) -> LocalField:
    field_ = LocalField()
    # Gust windows exist in both modes; they only matter to motion measurements.
    field_.windows += [
        ConditionWindow("gust", None, 20 * 60, 50 * 60),
        ConditionWindow("gust", None, 120 * 60, 150 * 60),
    ]
    if config.mode is not ScenarioMode.INCIDENT_DEMO:
        return field_
    rng = random.Random(f"{config.seed}|local-fixtures")

    # Vegetation on the whole of LINE_058 (a 57 m span inside SYN_REPORT_02): clearance 0.4 m.
    n = geometry.length_mm("LINE_058")
    field_.anomalies.append(
        Anomaly("vegetation", Interval("LINE_058", 0, n), VEGETATION_CLEARANCE_M)
    )

    # Hot joint on LINE_029 at a seeded position; only visible above the excitation current.
    n = geometry.length_mm("LINE_029")
    start = rng.randrange(int(0.2 * n), int(0.8 * n))
    field_.anomalies.append(
        Anomaly("hot_joint", Interval("LINE_029", start, start + 30_000), HOT_JOINT_EXCESS_C)
    )
    # Light local loading for the first 20 minutes: readings in that window are inconclusive.
    field_.windows.append(ConditionWindow("load", "LINE_029", 0, 20 * 60, 0.1))

    # Abnormal motion on the re-entering part of LINE_054 inside SYN_REPORT_06, seeded.
    inside = geometry.clip_intervals("LINE_054", network.areas["SYN_REPORT_06"])
    longest = max(inside, key=lambda iv: iv.end_mm - iv.start_mm)
    width = 1 * MM_PER_KM
    s0 = rng.randrange(longest.start_mm, longest.end_mm - width)
    field_.anomalies.append(
        Anomaly("motion", Interval("LINE_054", s0, s0 + width), ABNORMAL_MOTION_M)
    )

    # Dense canopy hides part of LINE_064 from the sensor: occluded, not inspected.
    field_.occluded.append(Interval("LINE_064", 7_800_000, 8_800_000))
    return field_
