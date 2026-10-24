"""Public three-method contracts, independent of selected hidden episodes."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
from typing import Literal

from grid_ops.geometry import Interval, LinePoint
from grid_ops.grid import LonLat

SENSOR_MODEL_REVISION = "three-method-v1"


class MeasurementMethod(StrEnum):
    LIDAR = "lidar"
    TDR = "tdr"
    CONTACT_CURRENT = "contact_current"


@dataclass(frozen=True)
class MethodCapability:
    method: MeasurementMethod
    airborne: bool
    description: str


METHOD_CATALOG = (
    MethodCapability(MeasurementMethod.LIDAR, True, "Moving geometric corridor inspection"),
    MethodCapability(MeasurementMethod.TDR, False, "Coupled reflectometry, valid return required"),
    MethodCapability(MeasurementMethod.CONTACT_CURRENT, False, "Landed single-conductor RMS amps"),
)


@dataclass(frozen=True)
class ConductorPoint:
    point: LinePoint
    circuit_id: str
    phase: Literal["A", "B", "C"] = "A"

    def __post_init__(self) -> None:
        if self.phase not in ("A", "B", "C"):
            raise ValueError("unknown phase")
        if not self.circuit_id.startswith(self.point.segment_id + "/"):
            raise ValueError("conductor circuit does not belong to segment")


@dataclass(frozen=True)
class ScanPose:
    position: LonLat
    height_m: float
    look_azimuth_deg: float
    look_elevation_deg: float = -30.0


@dataclass(frozen=True)
class ScanTrajectory:
    poses: tuple[ScanPose, ...]
    duration_s: int

    def __post_init__(self) -> None:
        if len(self.poses) < 2 or self.duration_s <= 0:
            raise ValueError("a moving scan needs two poses and positive duration")
        if all(p.position == self.poses[0].position for p in self.poses):
            raise ValueError("LiDAR requires motion")


@dataclass(frozen=True)
class LidarPass:
    target: Interval
    trajectory: ScanTrajectory
    speed_mps: float = 10.0
    sensor_profile_id: str = "synthetic-lidar-v1"
    method: Literal[MeasurementMethod.LIDAR] = field(default=MeasurementMethod.LIDAR, init=False)


@dataclass(frozen=True)
class ContactCurrent:
    conductor: ConductorPoint
    integration_s: int = 10
    method: Literal[MeasurementMethod.CONTACT_CURRENT] = field(
        default=MeasurementMethod.CONTACT_CURRENT, init=False
    )

    def __post_init__(self) -> None:
        if self.integration_s <= 0:
            raise ValueError("current integration must be positive")


@dataclass(frozen=True)
class TdrTest:
    access_site_id: str
    conductor: ConductorPoint
    return_path_id: str
    coupling_profile_id: str = "synthetic-live-compatible-v1"
    duration_s: int = 30
    isolation_evidence_ids: tuple[str, ...] = ()
    method: Literal[MeasurementMethod.TDR] = field(default=MeasurementMethod.TDR, init=False)

    def __post_init__(self) -> None:
        if self.duration_s <= 0:
            raise ValueError("TDR duration must be positive")


Acquisition = LidarPass | ContactCurrent | TdrTest


@dataclass(frozen=True)
class AcquisitionQuality:
    valid: bool
    reason: str = ""
    saturated: bool = False
    contact_ok: bool = True
    correlation_group: str = ""


@dataclass(frozen=True)
class LidarPayload:
    inspected: tuple[Interval, ...]
    missing: tuple[Interval, ...]
    clearance_m: float | None
    uncertainty_m: float
    resolution_mm: int
    method: Literal[MeasurementMethod.LIDAR] = field(default=MeasurementMethod.LIDAR, init=False)


@dataclass(frozen=True)
class TdrEcho:
    delay_s: float
    amplitude: float
    polarity: Literal["positive", "negative"]
    delay_uncertainty_s: float


@dataclass(frozen=True)
class TdrPayload:
    echoes: tuple[TdrEcho, ...]
    propagation_mps: float
    propagation_uncertainty_mps: float
    usable_range_m: float
    coupling_quality: str
    path_truncated: bool = False
    method: Literal[MeasurementMethod.TDR] = field(default=MeasurementMethod.TDR, init=False)


@dataclass(frozen=True)
class CurrentPayload:
    conductor: ConductorPoint
    rms_a: float
    uncertainty_a: float
    integration_s: int
    phase_model: str = "balanced-phase pi-section approximation"
    method: Literal[MeasurementMethod.CONTACT_CURRENT] = field(
        default=MeasurementMethod.CONTACT_CURRENT, init=False
    )


Payload = LidarPayload | TdrPayload | CurrentPayload


@dataclass(frozen=True)
class ExecutedAcquisition:
    id: str
    acquisition: Acquisition
    started_s: int
    ended_s: int
    drone_id: str
    landed_conductor: ConductorPoint | None = None
    request_ids: tuple[str, ...] = ()
    valid_isolation_evidence_ids: tuple[str, ...] = ()
    completed_fraction: float = 1.0

    def __post_init__(self) -> None:
        if self.ended_s < self.started_s or not 0 <= self.completed_fraction <= 1:
            raise ValueError("invalid acquisition interval")


@dataclass(frozen=True)
class AcquisitionSample:
    acquisition_id: str
    payload: Payload
    quality: AcquisitionQuality
    sensor_model_revision: str = SENSOR_MODEL_REVISION


@dataclass(frozen=True)
class Reading:
    id: str
    task_id: str
    request_ids: tuple[str, ...]
    drone_id: str
    acquisition: Acquisition
    acquired_start_s: int
    acquired_end_s: int
    payload: Payload
    quality: AcquisitionQuality
    sensor_model_revision: str = SENSOR_MODEL_REVISION
    evidence_epoch: str = "initial-observed-epoch"


@dataclass(frozen=True)
class SensorCapabilities:
    revision: str = SENSOR_MODEL_REVISION
    lidar_range_m: float = 250.0
    lidar_fov_deg: float = 90.0
    lidar_resolution_mm: int = 10_000
    lidar_max_cells: int = 2048
    conductor_height_m: float = 20.0
    propagation_mps: float = 2.0e8
    propagation_uncertainty_mps: float = 1.0e7
    tdr_delay_sigma_s: float = 5e-8
    tdr_range_m: float = 50_000.0
    tdr_max_paths: int = 128
    current_sigma_a: float = 1.0
    current_saturation_a: float = 2_000.0
    systematic_fraction: float = 0.5

    def __post_init__(self) -> None:
        if not 0 < self.lidar_fov_deg <= 180:
            raise ValueError("LiDAR FOV must be in (0, 180] degrees")
        positive = (
            self.lidar_range_m,
            self.lidar_resolution_mm,
            self.lidar_max_cells,
            self.propagation_mps,
            self.tdr_range_m,
            self.tdr_max_paths,
            self.current_saturation_a,
        )
        if any(value <= 0 for value in positive):
            raise ValueError("sensor ranges, rates and caps must be positive")
        if min(self.propagation_uncertainty_mps, self.tdr_delay_sigma_s, self.current_sigma_a) < 0:
            raise ValueError("sensor uncertainty must be nonnegative")
        if not 0 <= self.systematic_fraction <= 1:
            raise ValueError("systematic fraction must be in [0, 1]")


@dataclass(frozen=True)
class PublicEpisodeView:
    run_id: str
    fixture_id: str
    fixture_version: str
    policy_seed: int
    capabilities: SensorCapabilities = SensorCapabilities()
    assumptions: tuple[str, ...] = (
        "Synthetic balanced-phase post-trip AC model, not transient fault simulation",
        "LiDAR heights and obstacles are synthetic",
        "Live-compatible TDR is hypothetical and requires a declared return path",
        "Current magnitude alone does not establish energization or safe isolation",
    )


def echo_distance_interval(echo: TdrEcho, payload: TdrPayload) -> tuple[float, float]:
    lo_v = max(0.0, payload.propagation_mps - 2 * payload.propagation_uncertainty_mps)
    hi_v = payload.propagation_mps + 2 * payload.propagation_uncertainty_mps
    return (
        max(0.0, echo.delay_s - 2 * echo.delay_uncertainty_s) * lo_v / 2,
        (echo.delay_s + 2 * echo.delay_uncertainty_s) * hi_v / 2,
    )


@dataclass(frozen=True)
class TdrAccessSite:
    id: str
    conductor: ConductorPoint
    return_path_ids: tuple[str, ...]
    coupling_profiles: tuple[str, ...] = (
        "synthetic-live-compatible-v1",
        "isolated-only-v1",
    )


def tdr_access(conductor: ConductorPoint) -> TdrAccessSite:
    key = f"{conductor.circuit_id}/{conductor.phase}@{conductor.point.offset_mm}mm"
    return TdrAccessSite(f"access:{key}", conductor, (f"return:{key}",))
