from dataclasses import dataclass, field
from typing import Literal, Optional
from enum import Enum


class Decision(str, Enum):
    DISPATCH_CREW = "DISPATCH_CREW"
    MEASURE_MORE = "MEASURE_MORE"
    DEPRIORITIZE = "DEPRIORITIZE"


@dataclass
class Report:
    id: str
    type: str
    description: str
    confidence: Literal["low", "medium", "high"]


@dataclass
class SensorReadings:
    vibration_anomaly: Optional[bool] = None
    sag_angle_anomaly: Optional[bool] = None
    pd_detected: Optional[bool] = None
    line_live: Optional[bool] = None
    baseline_comparison: Optional[str] = None


@dataclass
class VegetationContext:
    forest_cover_nearby: bool = False
    estimated_tree_height_m: Optional[float] = None
    estimated_line_clearance_m: Optional[float] = None
    wind_alignment: Literal["upwind", "downwind", "perpendicular", "unknown"] = "unknown"
    tree_species_fragility: Literal["high", "medium", "low", "unknown"] = "unknown"


@dataclass
class SegmentHypothesis:
    segment_id: str
    voltage_kv: float
    length_km: float
    line_is_live: bool
    in_storm_footprint: bool
    sensor_readings: SensorReadings = field(default_factory=SensorReadings)
    vegetation: VegetationContext = field(default_factory=VegetationContext)
    reports: list[Report] = field(default_factory=list)
    notes: Optional[str] = None


@dataclass
class EvidenceScores:
    vegetation_risk: float      # Noul 0-1: tree contact plausible
    sensor_anomaly: float       # Noul 0-1: sensor data shows anomaly
    report_credibility: float   # Noul 0-1: reports specifically implicate this segment
    innocent_explanation: float # Noul 0-1: benign explanation covers observations


@dataclass
class EvaluationResult:
    segment_id: str
    combined_probability: float
    evidence_count: int
    decision: Decision
    evidence: EvidenceScores
    reasoning: str
