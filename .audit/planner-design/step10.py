from edit import swap,insert_before,section,record
swap('''- **Tasks have stable IDs.** A task's ID is `segment_id:kind:offset_km`. A task that already has a reading is never planned again, and the warm start can match old routes to new tasks.''','''- **Sites, task attempts and readings have different IDs.** A stable site identifies a canonical physical measurement location. A task ID identifies one authorized attempt, and each completed observation has its own reading ID. Task history survives replanning. A repeat attempt requires a recorded purpose such as confirmation, bad-data discrimination, expired evidence or a new operating epoch.''')
section('@dataclass(frozen=True)\nclass MeasurementTask:', '@dataclass(frozen=True)\nclass Sortie:', '''class TaskStatus(StrEnum):
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

''')
insert_before('## Data shape', '''### Task and observation lifecycle

A site has a stable registry ID and exact unrounded coordinates. Serialized local offsets use integer millimetres so identity does not depend on formatting a floating-point value. For electrical ranking, map all offsets observing the same circuit aggregate to one canonical observation group; distinct spatial site IDs do not create distinct state equations.

`PENDING -> RESERVED` occurs only when an accepted plan assigns the attempt to one drone. `RESERVED -> MEASURING` occurs on arrival. Completion stores one immutable reading and marks the attempt `COMPLETED`. Replanning may release an unstarted reservation back to `PENDING`, cancel it with a reason, or reassign it atomically. A failed attempt records `FAILED`; confirmation creates a new attempt rather than rewriting the failed or completed one. Histories and completed-task metadata are never removed by a replacement open-task list.

A `MEASURING` attempt is locked to its drone until completion or a recorded physical interruption. It is excluded from the optional solver set and cannot be assigned again. For that drone's continuation, start at its current stop at `world.clock + remaining_dwell`, with `battery_min - remaining_dwell`. Route acceptance includes the locked dwell and suffix exactly once. Even if its hypothesis is retired, the in-progress observation finishes; its subsequent interpretation may be inconclusive. Outstanding reservations and their dependencies are retained in each decision snapshot.

A repeat task requires a purpose and links its earlier reading or diagnostic. Its noise independence is assessed by the sensor model; repeated measurements do not automatically provide new structural rank or remove shared calibration bias. Full reading metadata includes position, kind, circuit payload, epoch, covariance and detection conditions, so a later explanation remains meaningful after the task leaves the open set.
''')
swap('''The physics callback produces every reading from ground truth with seeded noise. A dead line shows no current and no heating, so `CONDUCTOR_TEMP` can't find a hot joint on it. The power flow section below defines the ground truth.''','''The physics callback produces a timestamped `SensorSample` from hidden state and the declared sensor model. The sim attaches task, site and drone identity when constructing the immutable `Reading`; sensing does not need access to planner metadata. An unpowered or lightly loaded line supplies insufficient thermal excitation to exclude a hot joint. Retained heat and environmental effects follow the local sensor model rather than an assumption of instant cooling.''')
swap('''    def sense(self, point: LinePoint, kind: Measurement, clock: float) -> Reading: ...''','''    def sense(self, point: LinePoint, kind: Measurement, clock: float,
              observation_key: str) -> SensorSample: ...''')
record(10,12,'Define reserved attempts, locked dwell, repeat purposes and durable observation metadata','docs/planner-design.md#task-and-observation-lifecycle')
