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


# --- Vegetation risk: computed from real GIS data ---
@dataclass
class VegetationRisk:
    # From OSM forest polygons intersected with line buffer
    forest_overlap_pct: float = 0.0          # % of 50m line buffer covered by forest
    nearest_forest_m: Optional[float] = None  # distance to nearest OSM forest polygon

    # From Meta canopy height map or Bavarian laser scans
    max_canopy_height_m: Optional[float] = None   # tallest tree within 50m of line
    mean_canopy_height_m: Optional[float] = None

    # Estimated electrical clearance (derived from voltage class, standard engineering values)
    estimated_clearance_m: Optional[float] = None  # how much gap between wire and ground/trees

    # From DLR tree species map
    dominant_species: Literal["coniferous", "deciduous", "mixed", "unknown"] = "unknown"
    # coniferous (spruce etc.) = high fragility; deciduous bare in autumn = low

    # Computed from line bearing (grid_segments.geojson) vs assumed SW wind (225°)
    wind_exposure_score: float = 0.0  # 0=sheltered, 1=fully exposed/upwind


# --- Impact: computed from census + plant register + OSM critical infra ---
@dataclass
class ImpactContext:
    # From Zensus 2022 (100m grid cells linked to nearest substation)
    downstream_households: int = 0
    downstream_population: int = 0

    # From Marktstammdatenregister (Landkreis Augsburg, filtered)
    downstream_power_plants_mw: float = 0.0
    downstream_renewable_plants: int = 0

    # From OSM overpass (hospitals, water works, care homes)
    downstream_hospitals: int = 0
    downstream_water_works: int = 0
    downstream_care_homes: int = 0


@dataclass
class SegmentHypothesis:
    segment_id: str
    voltage_kv: float
    length_km: float
    line_is_live: bool
    in_storm_footprint: bool
    vegetation: VegetationRisk = field(default_factory=VegetationRisk)
    impact: ImpactContext = field(default_factory=ImpactContext)
    sensor_readings: SensorReadings = field(default_factory=SensorReadings)
    reports: list[Report] = field(default_factory=list)
    notes: Optional[str] = None


# --- Output ---
@dataclass
class EvidenceScores:
    vegetation_contact_probability: float  # P(tree is touching / about to touch conductor)
    impact_probability: float              # P(segment failure causes significant customer impact)
    report_credibility: float              # P(reports specifically implicate this segment)
    innocent_explanation: float            # P(benign explanation covers all observations)


@dataclass
class EvaluationResult:
    segment_id: str
    vegetation_contact_probability: float
    impact_probability: float
    dispatch_priority: float        # = P(contact) × P(impact), used for drone routing
    decision: Decision
    evidence: EvidenceScores
    reasoning: str
