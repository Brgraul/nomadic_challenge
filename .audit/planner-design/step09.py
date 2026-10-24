from edit import section,swap,record
section('## Architecture','## Hypotheses and physical predictions', '''## Architecture

The sim thread is the only writer of `World`. The coordinator runs the two LLM agents against one immutable `DecisionSnapshot` for each held cycle. Tools are pinned to that snapshot and return its revision. A single writer prevents simultaneous writes; revision checks and atomic acceptance prevent semantically stale decisions.

```mermaid
flowchart LR
    ui["Operator console"] --> queue["Versioned command queue"]
    subgraph sim["Sim thread"]
        ingest["advance and observable-evidence ingestion"]
        checks["Estimation, consistency and condition checks"]
        public["Public World and immutable decision snapshot"]
        accept["Validate draft, solve candidate, validate whole plan, atomic apply"]
        ingest --> checks --> public
        accept --> public
    end
    hidden["Hidden scenario physics"] -->|"Sensor and charging observations only"| ingest
    public -->|"Publish snapshot before event"| co["Coordinator"]
    co --> mgr["Manager LLM with read-only test tools"]
    mgr -->|"Explained hypothesis and model proposals"| co
    co --> pln["Planner LLM and previews"]
    pln -->|"Measurement needs and sites"| co
    co --> refresh["Deterministic task refresh"]
    refresh -->|"CommitDecision with source revision"| queue
    queue --> accept
    solver["Pure plan_sorties"] <--> accept
    public --> ui
    hidden --> judge["Evaluation isolated from agents"]
    public --> judge
```

| Module | Owns |
|---|---|
| `world.py` | Domain records, drone and task state machines, exact transition processing and observable events |
| `truth/` | Explicit synthetic circuit topology, AC state, local hazards, sensor likelihoods and charging physics; never copied into public state |
| `estimation.py` | Reduced public electrical model, accepted-data WLS, uncertainty, observability, detectability and hypothesis-model tests |
| `assessment.py` | Public consistency and operating-condition results, local coverage and evidence-linked recommendation checks |
| `planning.py` | Candidate refresh, information accounting, route optimization and independent route validation; pure, no I/O |
| `sim.py` | Single writer, evidence ingestion, versioned snapshots, commands, holds, atomic decision acceptance and deterministic fallbacks |
| `agents.py` | Coordinator, manager and planner prompts, snapshot-pinned tools, structured drafts and explanations |
| `evaluation.py` | Truth-based scoring and headless policy comparisons; no feedback into agents |
| `app.py`, `map_view.py` | Public hypotheses with findings and reasons, measured coverage, planned routes and unresolved actions; separate evaluation panel |

The manager can call `test_hypothesis` during its LLM turn. Candidate tests operate on the pinned snapshot, and their diagnostics are returned before the manager finalizes its explanation. A changed electrical candidate must be tested before its final revision is proposed. The sim verifies those test inputs or reruns the pure test during acceptance; an explanation referring to obsolete diagnostics is rejected for regeneration.
''')
swap('''    hypotheses_version: int
    tasks: dict[str, MeasurementTask]
    readings: dict[str, Reading]
    estimate: GridEstimate | None
    plan: Plan | None
    log: list[LogEntry]''','''    revision: SnapshotRevision
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
    log: list[LogEntry]''')
section('### Time','## Power flow ground truth', '''### Time and simultaneous transitions

`advance` updates position and battery exactly to the earliest arrival, dwell completion, charge transition, evidence expiry, observable scenario event or exercise deadline. Before publishing an event batch, process all transitions due at that timestamp to a fixed point, including zero-distance arrivals and co-located stops. Stable drone and event IDs determine processing order, but observations at the same time are assessed together. Reject a transition cycle that cannot make progress instead of looping forever.

Zero-distance travel completes without interpolation or battery drain. Otherwise interpolate the flight position only for a strictly positive leg duration. Use one distance model in both optimization and acceptance. For this synthetic exercise, great-circle flight length with a documented local interpolation approximation is acceptable; verification compares its maximum geometry error against the configured distance tolerance.

```text
advance(world, physics, until)
  repeatedly select the next transition time, no later than until
  move all drones and drain flight or measurement battery to that time
  process all due transitions to a fixed point
  emit one batch containing every observable event at that time
  return after an observable batch, or when until is reached
```

### Simulation and decision transaction

`SnapshotRevision` contains a monotonic public world sequence and the evidence-selection, operating-model, sensor-model, hypothesis-semantic, task-reservation, policy and drone-state revision IDs. `DecisionSnapshot` contains a deep-frozen public world, its revision, `cycle_id`, trigger events and clock. It contains no truth or evaluation fields. The UI may read the latest public snapshot; all tools in one agent turn read its pinned decision snapshot.

Commands are `Start`, `SubmitObservation`, `RequestReadingDisposition`, `RequestModelChange`, `RequestManualDispatch`, `CommitDecision`, `Release`, `SetPaused` and `Log`. Scenario control has a separate `SetInService` command inaccessible to agent tools. It changes and validates hidden physics and produces only declared observations. The old independently mutating `SetBelief`, `SetHypotheses`, `SetTasks` and bare `Replan` commands are removed. Operator hypothesis edits are proposals that require a manager-generated explained revision before acceptance.

`DecisionDraft` contains a unique decision ID, cycle and source revision, explained hypothesis revisions, candidate model changes, recommendations and a final measurement-needs/task proposal. Every command has a unique command ID and stored outcome for retry deduplication. No-op hypothesis proposals retain the semantic version while the manager's evidence assessment is still appended to turn history.

```text
on an observable batch or state-changing operator command
  hold or supersede the current cycle
  persist observations, dispositions and direct operational consequences
  invalidate affected assumptions and refresh diagnostics
  create the new immutable DecisionSnapshot
  publish it before enqueueing its DecisionEvent

on CommitDecision(command)
  verify command identity, active cycle and exact source revision
  build a private candidate world from the current held world
  validate explained revisions, before values, evidence and recommendations
  test any proposed model and hypothesis changes on candidate inputs
  refresh affected candidate estimates, diagnostics and tasks
  preserve ongoing measurement reservations and trim completed work
  solve candidate routes using current exact drone states
  validate the complete decision and all routes without mutating live state
  if any check rejects the draft, discard the candidate and report the cause
  otherwise atomically replace affected public state and executable routes
  publish the accepted decision snapshot and return its ID and plan

on Release(cycle_id, accepted_decision_id)
  require the active cycle and its still-current accepted decision
  reset the real-time baseline, then clear hold unless operator-paused
```

An operator command during an LLM turn either is read-only or creates a newer cycle and revision; old drafts cannot commit. A successful decision includes its explanations, hypothesis state, model revision, tasks, reservations, recommendations and plan together. Observations already ingested remain durable if that proposal fails. A valid deterministic fallback can accept a route-only decision with unchanged hypotheses and no invented revision. If a failure has no validated fallback, remain held as specified in the failure policy.
''')
swap('''    manager --> test["Test candidate hypotheses and model changes"]
    test --> semantic{"Semantic planning input changed?"}''','''    manager --> test["Read-only candidate tests during manager turn; finalize explanations"]
    test --> semantic{"Semantic planning input changed?"}''')
record(9,11,'Replace loose commands with pinned snapshots and two-phase whole-decision acceptance','docs/planner-design.md#simulation-and-decision-transaction')
