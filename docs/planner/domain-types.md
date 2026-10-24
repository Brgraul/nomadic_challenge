# Domain type sketches

[Design index](../planner-design.md). Read when changing a shared record, command payload, public snapshot or physics interface.

Related contracts: [Command semantics](workflow.md), [State semantics](drone-state.md).

## Data shape

```python
class Measurement(StrEnum):
    VOLTAGE_PRESENCE = "voltage_presence"   # E-field: is the line live?
    CURRENT = "current"
    CONDUCTOR_TEMP = "conductor_temp"       # hot joints, only visible on a loaded line
    CLEARANCE = "clearance"                 # sag / vegetation distance
    MOTION = "motion"                       # galloping, clashing

DWELL_MINUTES = {VOLTAGE_PRESENCE: 1, CURRENT: 2, CONDUCTOR_TEMP: 3, CLEARANCE: 3, MOTION: 4}

@dataclass(frozen=True)
class Hypothesis:               # manager → sim → planner
    id: str
    claim: str
    scope: tuple[SpanRef, ...]
    assessment: HypothesisAssessment
    lifecycle: HypothesisLifecycle
    priority: int
    prediction_ids: tuple[str, ...]
    supporting_evidence_ids: tuple[str, ...]
    contradicting_evidence_ids: tuple[str, ...]
    model_change: ModelChange | None

@dataclass(frozen=True)
class HypothesisRevision:
    id: str
    decision_id: str
    source_revision: SnapshotRevision
    semantic_revision: SemanticRevision
    hypothesis_id: str
    before: Hypothesis | None
    after: Hypothesis
    trigger_event_ids: tuple[str, ...]
    contributing_evidence_ids: tuple[str, ...]
    latest_measurement_ids: tuple[str, ...]
    diagnostic_ids: tuple[str, ...]
    finding_summary: str
    update_reason: str
    remaining_uncertainty: str

class TaskStatus(StrEnum):
    PENDING = "pending"
    RESERVED = "reserved"
    MEASURING = "measuring"
    COMPLETED = "completed"
    CANCELLED = "cancelled"
    FAILED = "failed"

@dataclass(frozen=True)
class MeasurementTask:
    id: str
    site_id: str
    hypothesis_ids: tuple[str, ...]
    point: LinePoint
    kind: Measurement
    urgency: int
    benefit_group_ids: tuple[str, ...]
    purpose: str
    source_revision: SnapshotRevision
    status: TaskStatus
    assigned_drone_id: str | None
    repeat_of_reading_id: str | None

@dataclass(frozen=True)
class CircuitFlow:
    circuit_id: str
    current_a: float
    p_mw: float
    q_mvar: float

@dataclass(frozen=True)
class Reading:
    id: str
    task_id: str
    site_id: str
    drone_id: str
    point: LinePoint
    kind: Measurement
    clock: float
    operating_epoch: str
    payload: MeasurementPayload
    sensor_model_revision: str
    covariance_ref: str
    detection_conditions: DetectionConditions
    summary: str

@dataclass(frozen=True)
class Sortie:
    drone_id: str
    stops: tuple[str, ...]      # task ids, in order
    available_from: float
    locked_dwell_minutes: float
    landing: LandingSite
    landing_clock: float
    airtime_minutes: float

@dataclass(frozen=True)
class Plan:
    id: str
    source_revision: SnapshotRevision
    sorties: tuple[Sortie, ...]
    waiting_drone_ids: tuple[str, ...]
    unassigned: tuple[UnassignedTask, ...]
    landing_policy_revision: str

def plan_sorties(inputs: PlanningInput, current: Plan | None) -> PlanningResult: ...
```

## World and ground truth

`World` holds only public observations, declared assumptions and derived results. Agent tools read a pinned immutable `DecisionSnapshot`, while the UI can read the latest public snapshot. The current implementation's `World.energized` moves into hidden `Simulation._truth` and is never copied into either public view.

```python
@dataclass
class World:                    # what the operator knows; snapshots of this reach the agents
    clock: float
    drones: dict[str, Drone]
    circuit_knowledge: dict[str, CircuitKnowledge]
    chargeability: dict[str, Chargeability]
    fault_assessments: dict[str, FaultAssessment]
    coverage: tuple[InspectionCoverage, ...]
    hypotheses: dict[str, Hypothesis]
    predictions: dict[str, Prediction]
    measurement_needs: dict[str, MeasurementNeed]
    benefit_groups: dict[str, BenefitGroup]
    revision: SnapshotRevision
    operating_epoch: str
    model: ElectricalModel
    hypotheses_version: int
    hypothesis_revisions: tuple[HypothesisRevision, ...]
    manager_turns: tuple[ManagerTurnRecord, ...]
    tasks: dict[str, MeasurementTask]
    readings: dict[str, Reading]
    reading_dispositions: tuple[ReadingDisposition, ...]
    estimate: GridEstimate | EstimateUnavailable
    diagnostics: tuple[CheckResult, ...]
    recommendations: tuple[Recommendation, ...]
    plan: Plan | None
    log: list[LogEntry]

class Physics(Protocol):        # implemented by grid_ops.truth; passed into advance
    def is_live(self, circuit_id: str) -> bool: ...
    def is_chargeable(self, landing: LandingSite) -> bool: ...
    def sense(self, point: LinePoint, kind: Measurement, clock: float,
              observation_key: str) -> SensorSample: ...
```

The physics callback produces a timestamped `SensorSample` from hidden state and the declared sensor model. The sim attaches task, site and drone identity when constructing the immutable `Reading`; sensing does not need access to planner metadata. An unpowered or lightly loaded line supplies insufficient thermal excitation to exclude a hot joint. Retained heat and environmental effects follow the local sensor model rather than an assumption of instant cooling.

Observations update circuit knowledge, chargeability and local coverage through the [operational knowledge rules](drone-state.md#energization-chargeability-and-inspection-state). The manager proposes evidence-linked fault assessments and model changes, rather than replacing operational observations with a combined line-status label.
