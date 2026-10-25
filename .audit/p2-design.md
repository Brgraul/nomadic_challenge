# P2 implementation design (in-place migration to the three-method runtime)

Base: 793596d (P1). Sources of truth: docs/three-method-outage-plan.md (Appendix B/C/E/F),
/tmp/three-method-design-verdict.md, /tmp/three-method-p2-brief.md (both lost in the
2026-10-25 /tmp wipe; their verdicts are absorbed here). One coherent change; no
legacy sensor enum, no parallel runtime, no silent aliases.

## 0. Method names and removals

`MeasurementMethod = lidar | tdr | contact_current` (sensing.py already owns this).
The legacy `Measurement` enum (voltage_presence/current/conductor_temp/clearance/motion),
`DWELL_S`, prediction-driven task creation, stationary-dwell sensing, `battery_s`,
`battery_s = FLIGHT_S` reset, terminal-only landing route shape, and every active call to
estimation/consistency/envelope MW inference are removed from the runtime in this unit.
estimation.py/consistency.py/envelope.py modules stay on disk (P4 deletes them) but nothing
in src/grid_ops may call them at runtime except truth/build internals and evaluation history.

## 1. sensing.py additions (pure contracts, no world dependency)

Pydantic (LLM boundary), discriminated on `method`:

- `CorridorSpan`: segment_id, start_mm, end_mm (end > start)
- `ConductorScope`: circuit_id, phase ("A"|"B"|"C"), start_mm, end_mm
- `OutcomeSpec`: description: str
- `RequestCommon`: id, hypothesis_ids, rationale, alternatives_distinguished,
  expected_outcomes, evidence_ids, urgency (1..5), source_revision, replaces_request_ids
- `LidarRequest(method="lidar", target: CorridorSpan)`
- `TdrRequest(method="tdr", target: ConductorScope, coupling_profile_id="synthetic-live-compatible-v1",
  isolation_evidence_ids=())`
- `CurrentRequest(method="contact_current", target: ConductorScope)`
- `MeasurementRequest = LidarRequest | TdrRequest | CurrentRequest`

Accepted (domain) records: `AcceptedRequest(id, request: MeasurementRequest,
status: RequestStatus (ACTIVE|RETIRED), accepted_s, source_revision, retired_reason)`.
Stable IDs: the request id the LLM chose, validated unique among active+retired.

`compile_requests(network, geometry, capabilities, requests, *, covered_intervals, clock_s)
-> TasksAndRejections` (new, in sensing.py; pure):
- lidar: one task per direction (forward/reverse trajectory) over the span; trajectory =
  poses at position_of(start) and position_of(end), height `conductor_height_m + 30`,
  look azimuth 0.0, elevation -60 deg, duration_s = span_m / 10.0 m/s, minimum 10 s. Both
  directions share `alternative_group_id = req.id`.
- contact_current: candidate contact points at every `CONTACT_GRID_MM = 2_000_000` (2 km)
  inside the scope, plus scope midpoint, deduplicated, sorted; one task per point, all in
  the request's alternative group; acquisition = ContactCurrent(ConductorPoint(point,
  circuit_id, phase), integration_s=10).
- tdr: access site from `tdr_access(conductor)` at the scope midpoint only (one task);
  coupling_profile must be in the site's catalog; `isolated-only-v1` requires non-empty
  isolation_evidence_ids (else rejection: "validated isolation evidence required; zero current
  is insufficient"); acquisition = TdrTest(site.id, conductor, site.return_path_ids[0],
  profile, 30, isolation_evidence_ids).
- Rejections (RequestRejection(request_id, reason)): unknown segment, circuit not on
  segment, phase mismatch, empty/clipped-away span, unknown coupling profile, missing
  isolation evidence, duplicate request id in the same call.
- Task record `MeasurementTaskV2(id, request_id, acquisition, alternative_group_id,
  repeat_of_task_id=None, status: TaskStatusV2 (PENDING|RESERVED|MEASURING|COMPLETED|CANCELLED
  |FAILED), assigned_drone_id=None, reading_id=None, urgency, note="")`. Tasks emit with
  id="" for the caller to renumber.

## 2. Request lifecycle

- `ManagerOutput` (hypotheses.py) gains `measurement_requests: list[MeasurementRequest] = []`
  and `retire_request_ids: list[str] = []`. Predictions keep their descriptive role.
- `proposals.validate(output, tb)` extended: every request id unique in the proposal; ids of
  retired/replaced requests exist and are ACTIVE; `replaces_request_ids` exist and ACTIVE;
  hypothesis_ids exist (accepted); evidence_ids exist in known_ids; urgency 1..5;
  source_revision == pinned snapshot revision; retire_request_ids exist and ACTIVE and are
  not also replaced. Request *feasibility* is NOT validated here (compile does that, with
  explicit rejections).
- `decisions.apply_candidate` extends the candidate with:
  `requests` (new AcceptedRequest list), `retired` (ids + reasons), `replaced_by`.
  `commit_apply` writes world.requests, retires replaced/retired, bumps `requests_version`.
- `world.requests: dict[str, AcceptedRequest]`; `world.tasks` holds MeasurementTaskV2
  from sensing (compile), refreshed by `sim._refresh_tasks` on every evidence/request change:
  compile all ACTIVE requests; keep RESERVED/MEASURING attempts (locked); retire PENDING tasks
  whose request is retired or whose group already has a COMPLETED valid attempt serving it
  (dupes earn nothing); cancellation notes mirror the current needs.py logic.
  `needs.py` shrinks to this thin lifecycle (rename internals; keep `Unavailable` reporting).

## 3. World execution (world.py rewrite of the drone layer)

Constants (declared, synthetic): LAND_S = 30, TAKEOFF_S = 30.

Operations (frozen dataclasses, in world.py):

- `Transit(origin, destination)` — duration `leg_seconds(origin, destination, speed)`
- `Scan(acquisition: LidarPass, task_id)` — duration `acquisition.trajectory.duration_s`
- `Land(site)` — LAND_S; `Takeoff(site)` — TAKEOFF_S
- `GroundSample(acquisitions: tuple[ContactCurrent|TdrTest, ...], site, task_ids)`
  — duration = max(integration_s/duration_s) of members
- `Charge(site, charge_s)` — charge_s > 0 only when a validated handshake made it observed
- `Wait(airborne: bool)` — stationary hold at current location/state

`ScheduledStep(index, start_s, end_s, operation)`. `Itinerary = (steps, final_perch)`;
steps are contiguous, monotonic; the drone's authoritative schedule.

Drone state: `Perched(perch) | Charging(perch, until_s, observed) | Stranded(perch) |
Executing(itinerary: Itinerary, step_index)`. `Executing` covers every airborne/ground
operation; `next_transition` = current step end. Position at t: interpolate Transit/Scan
(fraction of step), else fixed at op anchor. Old Ready/Flying/Measuring names disappear.

Battery: `Drone.battery: BatteryState` (energy.py). Energy commits only at step
boundaries: on completing step i, `battery, entry = integrate(battery, kind, dur, ...)`
(CHARGE uses `charging_observed`); `battery.observed_s` = boundary. Invariants: never
negative (RuntimeError like today), never above capacity. `WAIT_PERCHED`/`SAMPLE` cost 6/s.

Execution (`settle`): when clock reaches current step end: commit energy; on Scan end →
ExecutedAcquisition(completed_fraction=1.0) → physics.acquire → Reading + events;
on GroundSample end → one ExecutedAcquisition per member with landed_conductor set;
on Charge end → if handshake observed, integrate with charging_observed=True, event "charged";
else cost 6/s (failed handshake → Perched, event "charge_failed"); on Land end →
ingest_handshake(charging=charge step follows); on Takeoff end → nothing; on last
step end → Perched(final_perch). Reading publication and coordinator notification flow
exactly as today (`_publish`).

Scan interruption: replan may replace the remaining steps of an Executing drone only after
the current step, EXCEPT an in-progress Scan may be cut at the current clock (a deliberate
cancellation): produce the partial Reading with completed_fraction = elapsed/duration
(episode honours partial coverage), then continue with the new itinerary. GroundSample
steps are always locked.

Charging model: charging is only "observed" if the landing handshake succeeded on a
chargeable circuit at that moment. A failed handshake never strands: state = Perched.
`Stranded` only when remaining energy < cost of any takeoff+transit to a mechanically
valid perch and no charging is available (checked on route failure paths).

Physics protocol (world.py) becomes:

```python
class Physics(Protocol):
    def is_chargeable(self, site: LandingSite) -> bool: ...
    def acquire(self, executed: ExecutedAcquisition) -> AcquisitionSample: ...
```

sim.py adapts P1's Episode to this. `sense()` is removed from the runtime.

Reading records stored in world.readings are sensing.Reading instances (P1 record).

## 4. Routing (routes.py)

`SortieRequest(drone_id, steps: tuple[ScheduledStep,...], final_perch, based_on_revision,
energy_model_revision, sensor_model_revision, evidence_dependency_ids=())`.

`validate_sorties(world, geometry, circuits, requests, *, capabilities)`:
- per drone: steps contiguous, monotonic, positive durations; first operation consistent
  with the live state (Perched → Takeoff; Executing → continuation after current step, or
  cut-scan rule); last step ends with the drone Perched at final_perch (or a Charge whose
  end is the itinerary end); all steps end <= DEADLINE_S.
- state machine: Takeoff only from a Perched site == op.site; Land only airborne; Scan
  only airborne; GroundSample/Charge/Wait(ground) only at the perched site; Charge only at
  final position of a Land at the same site.
- recompute the whole energy ledger with energy.integrate from the drone's current
  battery (never trust caller times): nonneg at every boundary, capacity cap, takeoff/land
  positive costs. A Charge step requires chargeability evidence: CONFIRMED through
  charge_end (knowledge), or policy.allow_uncertain with a "risk" note.
- method fidelity: each Scan/GroundSample step's acquisition.method must equal its task's
  method; task ids must be PENDING/RESERVED and unowned or owned by this drone (never
  silently substituted).
- occupancy: per exact site, the perched interval [Land.start, Takeoff.end) of any two
  drones must not overlap; a site already occupied by a Perched/Charging/Stranded drone is
  only valid for that drone.
- endpoint evidence freshness: charge steps re-checked against expiry at their start.
`apply_sorties` commits: new tasks RESERVED to this drone, released reservations trimmed,
drone state := Executing/Perched per plan, `based_on_revision` recorded.

`current_requests(world)`: the remaining steps of every Executing drone (for incumbent
revalidation). `route_only_continuation`: if the remaining itinerary validates, keep it;
else land-only: nearest mechanically valid perch (hazards only — dead lines are valid
perches now) reachable with observed energy, no charge step; else HELD.

## 5. Planning (planning.py)

Keep: staged lexicographic solves, attempt cap, response budget, deterministic budget,
reranking rounds, incumbent candidate, `_kept_targets` pruning, hint paths, exhaustive
`exact` mode for tiny instances (needed by the enumeration test).

Model: vehicles = Perched (start_s=now, at perch) or Executing (start_s = current step end,
position = position at that boundary; the locked prefix is not modelled). Nodes:
- scan tasks (airborne): visit duration = trajectory.duration_s
- perch visits: for each contact/tdr task and each candidate perch site near it: visit
  duration = land+sample+takeoff (+ chosen charge_s option)
Charge options per visit: charge_s ∈ {0, 900, 1800, 3600, 5400, 7200, 9000, 10800} capped
by deadline; a charge option may only be used at sites with CONFIRMED chargeability
(uncertain-allowed policy may include UNKNOWN with risk flag). CP-SAT optional intervals per
(drone, site) with AddNoOverlap per site for occupancy.

Timing: arrival propagation on arcs: travel = leg_seconds(prev_exit_pos, next_entry_pos),
perch visit internal = LAND_S + sample + charge_s + TAKEOFF_S; scan visits need travel to
the scan start position. Energy per arc+visit from energy.py cost table as static upper
bounds; the authoritative ledger is recomputed by routes.validate_sorties on extraction.
T+04:00 deadline as today.

Objective stages (scoring.py rewritten, see §6): tier-5..1 (request urgency, unique
alternative group value), risk (uncertain charge endpoints), switching (owner changes),
airtime_s, then charge_s total. Candidate = SortieRequest set; extraction builds steps
explicitly. tiny-instance enumeration test uses `exact=True`.

## 6. Scoring (scoring.py rewrite, three-method value model)

- Benefit groups: ("req", request_id) for contact/tdr requests — value = 500 points, tier =
  request urgency, earned once per alternative group (a completed valid attempt of any task
  in the group retires the group's value); ("cell", segment, cell) for lidar — value = cell
  length in metres (10 m cells over the requested span, un-inspected only), tier = request
  urgency. `covered_intervals` = union of inspected Intervals of all completed valid lidar
  readings.
- PlanScore keeps (tiers, risk, switching, airtime_s, charge_s, notes). MW variance code,
  `_hypothetical`, estimation imports: deleted from the runtime scorer.
- `unresolved`: every accepted ACTIVE request with no completed valid attempt and no
  feasible task appears in `unassigned` with a reason; the UI shows requests as
  EXPLICITLY UNRESOLVED (P3 adds calibrated belief).

## 7. Sim, coordinator, agents, scripted manager

- sim.Simulation.__init__: builds `Episode` via `truth.build_episode(...)` — episode and
  noise seeds are private (NOT on ScenarioConfig; a ScenarioConfig.policy_seed is public),
  run_id opaque uuid. `EpisodePhysics` adapter in sim.py: `is_chargeable(site) =
  episode.current_at(ConductorPoint(point, circuit_id, "A")) >= I_CHARGE_MIN_A and span
  supplied` (synthetic declared rule; balanced-phase); `acquire = episode.acquire`.
  SetInService removed from runtime commands (evaluation-only).
- Startup knowledge: starting perches get observed handshakes as today; the episode's
  observed starting evidence (reports) stays public; NO fault/seed/label anywhere public.
- sim._refresh_tasks: needs lifecycle (§2) — compile ACTIVE requests, cancel/retire as
  today's needs.refresh does, but never invents a method.
- sim._refresh_estimate/_score_preview: estimation/consistency calls REMOVED; score_preview
  uses the new scorer.
- coordinator: `manager_fn(context: InvestigationContext) -> TurnResult` where
  InvestigationContext = Toolbox (pinned snapshot, network, registry, capabilities, public
  episode view). TurnResult = (output: ManagerOutput, errors, transcript, author). The
  coordinator (not the agent) submits RecordProposal and CommitDecision as today; agents.py
  loses `_record` and any Simulation import. LLM tools: overview, list_reports,
  segments_in_report, list_readings, get_reading, list_events, list_hypotheses,
  list_requests (accepted requests + task/rejection status), scan_coverage (per-segment
  inspected/missing union), charge_sites (chargeability + expiry), capabilities
  (METHOD_CATALOG + assumptions). estimate/consistency/electrical-candidate tools removed.
  Missing OPENROUTER_API_KEY ⇒ scripted mode labelled in the turn record and the app.
- scripted_manager: startup creates three hypotheses (vegetation in SYN_REPORT_02 →
  LidarRequest over the corridor; de-energized/SYN_REPORT_03 switching on LINE_024 →
  CurrentRequest on its circuits; conductor damage in SYN_REPORT_05/06 area → TdrRequest)
  plus matching measurement_requests citing report ids; measurement turns revise from
  readings: lidar clearance below CLEARANCE_THRESHOLD_M on inspected intervals → hazard +
  SUPPORTED vegetation; current > 0 (above 2σ) on a "switched off" line → contradicts the
  de-energized claim; current == 0 keeps BOTH live-unloaded and disconnected alternatives
  explicitly (never certified dead); tdr echoes constrain intervals via
  echo_distance_interval (no fault claim beyond the evidence); add_conclusion follows the
  same hazard+switching linkage rules as today.

## 8. Knowledge / assessment (three-method evidence)

- knowledge.py: energization/chargeability evidence from handshakes, telemetry and switching
  reports only. Voltage/current reading claims are REMOVED (zero-current ambiguity):
  current readings never set LIVE/DEAD. `assess_perch(world, site)` = mechanical landing
  safety (hazards only) — used for all perches; `assess_landing` (charge endpoints) keeps
  chargeability + hazard logic for Charge steps. `landing_candidates` unchanged in shape.
- assessment.py: local condition machinery rewritten to the lidar payload:
  `interpret_lidar(reading)` → hazard when payload.clearance_m < CLEARANCE_THRESHOLD_M over
  inspected intervals (an observed geometric fact with evidence provenance);
  `coverage_report(world, scope, "lidar")` unions inspected/missing from all lidar readings.
  conductor_temp/motion interpretations: deleted from the runtime (conditions the three
  methods cannot test become NOT_SUPPORTED).

## 9. UI consumers (app.py, views.py, map_view.py, e2e/drive.py)

Replace legacy sensor enums/dwell tables with METHOD_CATALOG-driven method options; show
per drone: phase, position, battery units (and % capacity), current operation and remaining
itinerary; per request: method, target, status, served-by task/drone, rejections; per
reading: payload-type-specific rendering; unresolved requests shown explicitly. Sortie
builder emits the new SortieRequest (steps auto-composed from chosen tasks + perch +
optional charge). No MW estimate/consistency panels; no RGB/IR/temperature claims anywhere.
e2e/drive.py and AppTest migrate to drive: request → accept → plan → execute → reading →
next turn.

## 10. Tests, driver, evidence

- Migrate: test_manager, test_findings, test_needs, test_decisions, test_sorties,
  test_landing, test_planning, test_coordinator, test_resilience (+ conftest helpers).
  New assertions: resource continuity, optional charging, timed occupancy, stale proposals,
  invalid method substitution, cut-scan partial coverage, exhaustive tiny-instance
  enumeration vs CP-SAT (exact=True), independent ledger replay in validate_sorties.
- scripts/verify_outage.py gains `--case itinerary` end-to-end lanes wiring the real
  Simulation through request → compile → plan → execute → reading, and the P2 live lanes
  1-10 write traces/asserts to /tmp/three-method-P2/lane-N/ (receipts also mirrored under
  the worktree's .audit/ because /tmp is volatile).
- Perf probe: ingestion→validated-route latency p50/p95/max; keep 1s p95 / 2s budget.

## 11. Migration order inside this unit

1. sensing request contracts + compile_requests (+ tests/test_sensing additions)
2. world.py itinerary/energy execution + EpisodePhysics-facing settle (+ world tests)
3. routes.py step validation/apply (+ test_sorties/test_landing migration)
4. needs.py request-lifecycle refresh (+ test_needs)
5. scoring.py value model + planning.py visit model (+ test_planning, enumeration)
6. hypotheses/proposals/decisions request lifecycle (+ test_decisions/test_findings)
7. agents/scripted_manager/coordinator (+ test_manager/test_coordinator)
8. sim.py wiring, knowledge/assessment three-method evidence (+ test_resilience/fleet)
9. UI + e2e + driver lanes; full suite + ruff; commit locally with the verdict trail.
