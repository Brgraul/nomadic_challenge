"""Public three-method contracts, independent of selected hidden episodes."""

from __future__ import annotations

from collections.abc import Hashable, Mapping, Sequence
from dataclasses import dataclass, field
from enum import StrEnum
from typing import Annotated, Literal, Self

from pydantic import BaseModel, Field, model_validator

from grid_ops.geometry import Geometry, Interval, LinePoint
from grid_ops.grid import LonLat, Network

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


@dataclass(frozen=True)
class EchoEdge:
    """One conductor span between two electrical nodes, as the reflectometer sees it."""

    key: Hashable
    a: Hashable
    b: Hashable
    length_mm: int


def trace_echoes(
    adjacency: Mapping[Hashable, Sequence[EchoEdge]],
    start: EchoEdge,
    a_distance_mm: int,
    *,
    range_m: float,
    max_paths: int,
) -> tuple[tuple[tuple[float, float], ...], bool]:
    """Round-trip reflection distances (metres) and amplitudes from a coupling point.

    The pulse leaves along both directions of `start`, `a_distance_mm` from node `a`. A dead end
    reflects +1; a junction with several onward spans reflects a negative partial echo. An open
    protective device is simply a node with one span, so it masks everything beyond it. The walk
    is capped at `max_paths` expansions; the boolean reports that the cap left work undone.
    """
    stack: list[tuple[Hashable, float, tuple[Hashable, ...]]] = [
        (start.a, a_distance_mm / 1000, (start.key,)),
        (start.b, (start.length_mm - a_distance_mm) / 1000, (start.key,)),
    ]
    found: list[tuple[float, float]] = []
    explored = 0
    while stack and explored < max_paths:
        node, distance, visited = stack.pop()
        explored += 1
        if distance > range_m:
            continue
        outgoing = [e for e in adjacency.get(node, ()) if e.key not in visited]
        if not outgoing:
            if distance > 1e-9:
                found.append((distance, 1.0))
            continue
        if len(outgoing) > 1:
            found.append((distance, -min(0.8, len(outgoing) / 4)))
        for e in outgoing:
            other = e.b if e.a == node else e.a
            stack.append((other, distance + e.length_mm / 1000, (*visited, e.key)))
    return tuple(sorted(set(found))), bool(stack)


CONTACT_GRID_MM = 2_000_000
LIDAR_SPEED_MPS = 10.0
TDR_DURATION_S = 30
CIRCUIT_PHASES = ("A", "B", "C")


class CorridorSpan(BaseModel):
    segment_id: str
    start_mm: int
    end_mm: int

    @model_validator(mode="after")
    def _ordered(self) -> Self:
        if self.end_mm <= self.start_mm:
            raise ValueError("end_mm must exceed start_mm")
        return self


class ConductorScope(BaseModel):
    circuit_id: str
    phase: Literal["A", "B", "C"]
    start_mm: int
    end_mm: int

    @model_validator(mode="after")
    def _ordered(self) -> Self:
        if self.end_mm <= self.start_mm:
            raise ValueError("end_mm must exceed start_mm")
        return self


class OutcomeSpec(BaseModel):
    description: str


class RequestCommon(BaseModel):
    id: str
    hypothesis_ids: tuple[str, ...]
    rationale: str
    alternatives_distinguished: tuple[str, ...]
    expected_outcomes: tuple[OutcomeSpec, ...]
    evidence_ids: tuple[str, ...]
    urgency: int = Field(ge=1, le=5)
    source_revision: str
    replaces_request_ids: tuple[str, ...]


class LidarRequest(RequestCommon):
    method: Literal["lidar"] = "lidar"
    target: CorridorSpan


class TdrRequest(RequestCommon):
    method: Literal["tdr"] = "tdr"
    target: ConductorScope
    coupling_profile_id: str = "synthetic-live-compatible-v1"
    isolation_evidence_ids: tuple[str, ...] = ()


class CurrentRequest(RequestCommon):
    method: Literal["contact_current"] = "contact_current"
    target: ConductorScope


MeasurementRequest = Annotated[
    LidarRequest | TdrRequest | CurrentRequest, Field(discriminator="method")
]


class TaskStatusV2(StrEnum):
    PENDING = "PENDING"
    RESERVED = "RESERVED"
    MEASURING = "MEASURING"
    COMPLETED = "COMPLETED"
    CANCELLED = "CANCELLED"
    FAILED = "FAILED"


class RequestStatus(StrEnum):
    ACTIVE = "ACTIVE"
    RETIRED = "RETIRED"


@dataclass(frozen=True)
class AcceptedRequest:
    id: str
    request: MeasurementRequest
    status: RequestStatus
    accepted_s: int
    source_revision: str
    retired_reason: str = ""


@dataclass(frozen=True)
class MeasurementTaskV2:
    id: str
    request_id: str
    acquisition: Acquisition
    alternative_group_id: str
    status: TaskStatusV2
    urgency: int
    repeat_of_task_id: str | None = None
    assigned_drone_id: str | None = None
    reading_id: str | None = None
    note: str = ""


@dataclass(frozen=True)
class RequestRejection:
    request_id: str
    reason: str


@dataclass(frozen=True)
class TasksAndRejections:
    tasks: tuple[MeasurementTaskV2, ...]
    rejections: tuple[RequestRejection, ...]


def compile_requests(
    network: Network,
    geometry: Geometry,
    capabilities: SensorCapabilities,
    requests: Sequence[MeasurementRequest],
    *,
    covered_intervals: Sequence[Interval] = (),
    clock_s: int = 0,
) -> TasksAndRejections:
    """Expand accepted requests into pending tasks, rejecting each infeasible request."""
    # covered_intervals and clock_s are carried for the scoring/refresh callers; compilation
    # itself is position-based and time-blind.
    tasks: list[MeasurementTaskV2] = []
    rejections: list[RequestRejection] = []
    seen: set[str] = set()
    for request in requests:
        if request.id in seen:
            rejections.append(
                RequestRejection(request.id, f"duplicate request id {request.id} in the same call")
            )
            continue
        seen.add(request.id)
        if isinstance(request, LidarRequest):
            outcome = _lidar_tasks(network, geometry, capabilities, request)
        elif isinstance(request, CurrentRequest):
            outcome = _current_tasks(network, geometry, request)
        else:
            outcome = _tdr_tasks(network, geometry, request)
        if isinstance(outcome, RequestRejection):
            rejections.append(outcome)
        else:
            tasks.extend(outcome)
    return TasksAndRejections(tuple(tasks), tuple(rejections))


def _clipped_target(
    request_id: str, segment_id: str, start_mm: int, end_mm: int, length_mm: int
) -> tuple[int, int] | RequestRejection:
    start, end = max(start_mm, 0), min(end_mm, length_mm)
    if end <= start:
        return RequestRejection(
            request_id, f"requested span clips to empty on segment {segment_id}"
        )
    return start, end


def _lidar_tasks(
    network: Network, geometry: Geometry, capabilities: SensorCapabilities, request: LidarRequest
) -> tuple[MeasurementTaskV2, ...] | RequestRejection:
    target = request.target
    if target.segment_id not in network.segments:
        return RequestRejection(request.id, f"unknown segment {target.segment_id}")
    bounds = _clipped_target(
        request.id,
        target.segment_id,
        target.start_mm,
        target.end_mm,
        geometry.length_mm(target.segment_id),
    )
    if isinstance(bounds, RequestRejection):
        return bounds
    start, end = bounds
    height_m = capabilities.conductor_height_m + 30.0
    poses = tuple(
        ScanPose(geometry.position_of(LinePoint(target.segment_id, mm)), height_m, 0.0, -60.0)
        for mm in (start, end)
    )
    duration_s = max(10, round((end - start) / 1000.0 / LIDAR_SPEED_MPS))
    interval = Interval(target.segment_id, start, end)
    return tuple(
        MeasurementTaskV2(
            id="",
            request_id=request.id,
            acquisition=LidarPass(interval, ScanTrajectory(trajectory, duration_s)),
            alternative_group_id=request.id,
            status=TaskStatusV2.PENDING,
            urgency=request.urgency,
        )
        for trajectory in (poses, poses[::-1])
    )


def _clipped_conductor_scope(
    network: Network, geometry: Geometry, request: CurrentRequest | TdrRequest
) -> Interval | RequestRejection:
    scope = request.target
    segment_id, _, suffix = scope.circuit_id.partition("/")
    if segment_id not in network.segments:
        return RequestRejection(request.id, f"unknown segment {segment_id}")
    if not (suffix.startswith("C") and suffix[1:].isdecimal() and int(suffix[1:]) >= 1):
        return RequestRejection(
            request.id, f"circuit {scope.circuit_id} not on segment {segment_id}"
        )
    # pydantic rejects other phases at parse; this guards direct construction.
    if scope.phase not in CIRCUIT_PHASES:
        raise ValueError("unknown phase")
    bounds = _clipped_target(
        request.id, segment_id, scope.start_mm, scope.end_mm, geometry.length_mm(segment_id)
    )
    if isinstance(bounds, RequestRejection):
        return bounds
    start, end = bounds
    return Interval(segment_id, start, end)


def _current_tasks(
    network: Network, geometry: Geometry, request: CurrentRequest
) -> tuple[MeasurementTaskV2, ...] | RequestRejection:
    scope = _clipped_conductor_scope(network, geometry, request)
    if isinstance(scope, RequestRejection):
        return scope
    offsets = {
        k * CONTACT_GRID_MM
        for k in range(
            scope.start_mm // CONTACT_GRID_MM + 1, (scope.end_mm - 1) // CONTACT_GRID_MM + 1
        )
    }
    offsets.add((scope.start_mm + scope.end_mm) // 2)
    return tuple(
        MeasurementTaskV2(
            id="",
            request_id=request.id,
            acquisition=ContactCurrent(
                ConductorPoint(
                    LinePoint(scope.segment_id, offset_mm),
                    request.target.circuit_id,
                    request.target.phase,
                ),
                integration_s=10,
            ),
            alternative_group_id=request.id,
            status=TaskStatusV2.PENDING,
            urgency=request.urgency,
        )
        for offset_mm in sorted(offsets)
    )


def _tdr_tasks(
    network: Network, geometry: Geometry, request: TdrRequest
) -> tuple[MeasurementTaskV2, ...] | RequestRejection:
    scope = _clipped_conductor_scope(network, geometry, request)
    if isinstance(scope, RequestRejection):
        return scope
    conductor = ConductorPoint(
        LinePoint(scope.segment_id, (scope.start_mm + scope.end_mm) // 2),
        request.target.circuit_id,
        request.target.phase,
    )
    access = tdr_access(conductor)
    if request.coupling_profile_id not in access.coupling_profiles:
        return RequestRejection(
            request.id, f"unknown coupling profile {request.coupling_profile_id}"
        )
    if request.coupling_profile_id == "isolated-only-v1" and not request.isolation_evidence_ids:
        return RequestRejection(
            request.id, "validated isolation evidence required; zero current is insufficient"
        )
    acquisition = TdrTest(
        access.id,
        conductor,
        access.return_path_ids[0],
        request.coupling_profile_id,
        TDR_DURATION_S,
        tuple(request.isolation_evidence_ids),
    )
    return (
        MeasurementTaskV2(
            id="",
            request_id=request.id,
            acquisition=acquisition,
            alternative_group_id=request.id,
            status=TaskStatusV2.PENDING,
            urgency=request.urgency,
        ),
    )
