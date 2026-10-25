"""Background simulation loop.

The sim thread is the only writer of the World. Agents and the UI send commands through a queue
and read `snapshot`, a deep copy republished after every tick, so readers never need a lock.
Hidden physics lives in `_truth` and is only ever asked questions through `world.Physics`.
"""

from __future__ import annotations

import copy
import hashlib
import queue
import threading
import time
import uuid
from concurrent.futures import Future
from dataclasses import dataclass

from grid_ops import assessment, decisions, fleet, knowledge, needs, outage, planning, scoring
from grid_ops.circuits import CircuitRegistry, build_registry
from grid_ops.conclusion import Investigation, InvestigationStatus, reopen_relevance
from grid_ops.energy import DEFAULT_ENERGY_MODEL
from grid_ops.geometry import Geometry, LandingSite, LinePoint
from grid_ops.grid import Network
from grid_ops.knowledge import LandingPolicy
from grid_ops.proposals import ProposedFinding, ProposedStatus
from grid_ops.routes import SortieRequest, apply_sorties, validate_sorties
from grid_ops.scenario import (
    I_CHARGE_MIN_A,
    ScenarioConfig,
    ScenarioMode,
    SensorModel,
    public_brief,
)
from grid_ops.sensing import ConductorPoint, SensorCapabilities, TaskStatusV2
from grid_ops.truth import Episode, EpisodeSpec, build_episode
from grid_ops.world import (
    LAND_S,
    TAKEOFF_S,
    CommandRejected,
    Event,
    Land,
    LineStatus,
    Perched,
    ReadingDisposition,
    RejectionRecord,
    ScheduledReport,
    ScheduledStep,
    Stranded,
    Takeoff,
    Transit,
    World,
    advance_to,
    format_clock,
    initial_world,
    leg_seconds,
    seed_starting_handshakes,
    settle,
)

TICK_SECONDS = 0.5
# Fixture: a public switching report that hidden physics contradicts (the line stays live).
CONTRADICTED_SEGMENT = "LINE_009"


class EpisodePhysics:
    """Adapts the hidden episode to the `world.Physics` protocol the runtime is allowed to ask."""

    def __init__(self, episode: Episode) -> None:
        self._episode = episode

    def is_chargeable(self, site: LandingSite) -> bool:
        # Balanced-phase declared rule: phase A must carry at least the minimum charging current;
        # unsupplied spans read 0.0 A, so the threshold alone decides.
        return (
            self._episode.current_at(ConductorPoint(site.point, site.circuit_id, "A"))
            >= I_CHARGE_MIN_A
        )

    def acquire(self, executed):
        return self._episode.acquire(executed)


def _private_seeds(policy_seed: int) -> tuple[int, int]:
    """Deterministic but unpublished: the public seed selects the episode without revealing it."""
    digest = hashlib.sha256(f"episode|{policy_seed}".encode()).digest()
    return (
        int.from_bytes(digest[:4], "big"),
        int.from_bytes(digest[4:8], "big"),
    )


@dataclass(frozen=True)
class Dispatch:
    """Fly to a segment's midpoint and land on its first circuit (a landing-only sortie)."""

    drone_id: str
    segment_id: str
    author: str


@dataclass(frozen=True)
class CommitSorties:
    """Validate every sortie, then change every drone or none."""

    sorties: tuple[SortieRequest, ...]
    author: str


@dataclass(frozen=True)
class SetBelief:
    segment_id: str
    status: LineStatus
    note: str
    author: str


@dataclass(frozen=True)
class SetInService:
    """Ground-truth switching or a fault trip, not something the agents can see directly."""

    segment_id: str
    in_service: bool
    author: str


@dataclass(frozen=True)
class RegisterSite:
    """Register an exact inspection or landing location (idempotent: same offset, same site)."""

    segment_id: str
    offset_mm: int
    author: str


@dataclass(frozen=True)
class SetLandingPolicy:
    """Explicitly opt in to (or out of) uncertain recharge endpoints. Hazards stay forbidden."""

    allow_uncertain: bool
    author: str


@dataclass(frozen=True)
class ExcludeReading:
    reading_id: str
    reason: str
    author: str


@dataclass(frozen=True)
class ReinstateReading:
    reading_id: str
    reason: str
    author: str


@dataclass(frozen=True)
class ReassignEpoch:
    reading_id: str
    epoch: str
    reason: str
    author: str


@dataclass(frozen=True)
class RecordProposal:
    """Store a validated (or rejected) manager proposal. It changes no accepted state."""

    author: str
    snapshot_revision: int
    output: object  # ManagerOutput
    status: object  # ProposedStatus
    note: str
    semantic_key: str = ""


@dataclass(frozen=True)
class CommitDecision:
    """Accept an explained proposal and its operational consequences as one atomic decision.
    Retrying a decision id returns the stored outcome without repeating any effect."""

    decision_id: str
    finding_id: str
    semantic_key: str
    author: str
    sorties: tuple[SortieRequest, ...] = ()


@dataclass(frozen=True)
class Release:
    """Release an acceptance hold. Only its current cycle and its accepted decision can."""

    cycle_id: int
    decision_id: str
    author: str


@dataclass(frozen=True)
class RequestModelChange:
    """Stage a candidate topology. Enqueueing it never changes the accepted model."""

    segments: tuple[str, ...]
    author: str


@dataclass(frozen=True)
class RejectFinding:
    finding_id: str
    reason: str
    author: str


@dataclass(frozen=True)
class RecordCoordinatorTurn:
    """Create or update (by id) the public record of an asynchronous manager turn."""

    turn: object  # coordinator.CoordinatorTurn


@dataclass(frozen=True)
class RecordPlannerPreview:
    preview: object  # coordinator.PlannerPreview


@dataclass(frozen=True)
class RoutePassCommand:
    """Ask for an immediate route pass (also used to retry while held)."""

    reason: str
    author: str


@dataclass(frozen=True)
class AcceptLimits:
    """The operator explicitly accepts a stated unresolved limit and closes the case with it."""

    statement: str
    author: str


@dataclass(frozen=True)
class ReopenInvestigation:
    reason: str
    author: str


@dataclass(frozen=True)
class Log:
    author: str
    text: str


Command = (
    Dispatch
    | CommitSorties
    | SetBelief
    | SetInService
    | RegisterSite
    | SetLandingPolicy
    | ExcludeReading
    | ReinstateReading
    | ReassignEpoch
    | RecordProposal
    | CommitDecision
    | Release
    | RequestModelChange
    | RejectFinding
    | RecordCoordinatorTurn
    | RecordPlannerPreview
    | RoutePassCommand
    | AcceptLimits
    | ReopenInvestigation
    | Log
)


class Simulation:
    def __init__(
        self,
        network: Network,
        minutes_per_second: float = 1.0,
        scenario: ScenarioConfig | None = None,
        auto_replan: bool = False,
    ) -> None:
        self.network = network
        self.geometry = Geometry(network)
        self.circuits: CircuitRegistry = build_registry(network)
        self.minutes_per_second = minutes_per_second
        self.paused = False
        self.scenario = scenario or ScenarioConfig()
        self._world = initial_world(network, self.geometry)
        policy_seed = getattr(self.scenario, "policy_seed", self.scenario.seed)
        episode_seed, noise_seed = _private_seeds(policy_seed)
        self._truth = build_episode(
            network,
            self.circuits,
            self.geometry,
            EpisodeSpec(episode_seed, noise_seed, run_id=uuid.uuid4().hex),
            fixture_id="bavaria-public",
            fixture_version="1",
            policy_seed=policy_seed,
            capabilities=SensorCapabilities(),
        )
        self._physics = EpisodePhysics(self._truth)
        self._commands: queue.Queue[tuple[Command, Future[str]]] = queue.Queue()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._hidden_log: list[str] = []
        self._remainder_s = 0.0
        self.auto_replan = auto_replan
        self.coordinator = fleet.Coordinator()
        self._ingest_wall = time.perf_counter()

        w = self._world
        w.scenario_mode, w.scenario_seed = self.scenario.mode, self.scenario.seed
        w.brief = public_brief(network, self.scenario)
        w.capabilities = SensorCapabilities()
        w.energy_model = DEFAULT_ENERGY_MODEL
        w.sensor_model = SensorModel(shared_rho=self.scenario.shared_rho)
        w.circuits_by_segment = dict(self.circuits.by_segment)
        w.segment_lengths = {sid: self.geometry.length_mm(sid) for sid in network.segments}
        knowledge.recompute(w)
        seed_starting_handshakes(w)
        if self.scenario.mode is ScenarioMode.INCIDENT_DEMO and "LINE_024" in w.circuits_by_segment:
            w.known_switching["LINE_024"] = 0  # SYN_REPORT_03's status check
            knowledge.ingest_report(
                w, "LINE_024", 0, "SYN_REPORT_03: LINE_024 precautionarily switched off"
            )
        if self.scenario.contradictory_switching:
            w.pending_reports.append(
                ScheduledReport(
                    20 * 60,
                    CONTRADICTED_SEGMENT,
                    f"Dispatch reports {CONTRADICTED_SEGMENT} switched to de-energized.",
                )
            )
        self.outage_model = outage.build_model(
            network, self.circuits, self.geometry, w.capabilities
        )
        self.belief_valued_scheduling = True
        self._outage_key: tuple | None = None
        self._value_key: tuple | None = None
        self._proposal_key: tuple | None = None
        self._best_eig = 0.0
        self._update_outage()
        self._update_values()
        self.snapshot: World = copy.deepcopy(self._world)

    def submit(self, command: Command) -> Future[str]:
        """Queue a command for the sim thread. Before `start()` (tests, headless runs) there is
        no thread, so the command is applied inline with the same semantics."""
        future: Future[str] = Future()
        if self._thread is None:
            self._resolve(command, future)
            self.snapshot = copy.deepcopy(self._world)
        else:
            self._commands.put((command, future))
        return future

    def _resolve(self, command: Command, future: Future[str]) -> None:
        try:
            future.set_result(self._apply(command))
        except CommandRejected as e:
            self._world.record(getattr(command, "author", "sim"), f"Rejected: {e}")
            future.set_exception(e)
        except Exception as e:  # a bad command must never kill the sim thread
            self._world.record("sim", f"Command failed: {e!r}")
            future.set_exception(e)

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="grid-sim", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        last = time.monotonic()
        while not self._stop.is_set():
            while not self._commands.empty():
                command, future = self._commands.get_nowait()
                self._resolve(command, future)
            now = time.monotonic()
            if not self.paused:
                self.step((now - last) * self.minutes_per_second)
            else:
                self.snapshot = copy.deepcopy(self._world)
            last = now
            time.sleep(TICK_SECONDS)

    def step(self, minutes: float) -> list[Event]:
        """Advance simulated time and republish the snapshot. Called by the sim thread."""
        if self._world.hold is not None:  # an open acceptance hold freezes simulated time
            self._publish([])
            return []
        self._remainder_s += minutes * 60
        whole = int(self._remainder_s)
        self._remainder_s -= whole
        events = advance_to(self._world, self._physics, self._world.clock_s + whole)
        self._publish(events)
        return events

    def _publish(self, events: list[Event]) -> None:
        """Refresh derived public diagnostics before the snapshot is published."""
        w = self._world
        ingested = time.perf_counter()
        first_new = len(w.events)
        for e in events:
            w.events.append((f"EVT{len(w.events) + 1:04d}", e))
        self._ingest_readings(first_new)
        if events:
            w.revision += 1
        self._update_outage()
        self._refresh_tasks(events)
        inv = w.investigation
        if inv.status is not InvestigationStatus.OPEN and events and inv.concluded_s is not None:
            flags = tuple(reopen_relevance(w, inv.concluded_s))
            if flags != inv.reopen_flags:
                w.investigation = Investigation(
                    inv.status,
                    inv.conclusion,
                    inv.decision_id,
                    inv.concluded_s,
                    inv.history,
                    flags,
                    inv.unresolved,
                )
        self.snapshot = copy.deepcopy(self._world)  # evidence is published first ...
        published = time.perf_counter()
        ids = tuple(i for i, _ in w.events[first_new:])
        relevant = [e for e in events if e.kind in fleet.RELEVANT_EVENTS]
        if events:
            self.coordinator.notify(w.revision, ids)  # ... and the coordinator hears of it ...
        if self.auto_replan and relevant:  # ... before the bounded route pass runs
            self._route_pass(",".join(sorted({e.kind for e in relevant})), ids, ingested, published)
        elif self.auto_replan and w.hold is not None:
            self._route_pass("retry while held", (), time.perf_counter(), time.perf_counter())

    def _ingest_readings(self, first_new: int) -> None:
        """Feed new readings to the lidar coverage and hazard interpretation."""
        w = self._world
        for _, e in w.events[first_new:]:
            if e.kind != "reading":
                continue
            reading = w.readings.get(e.detail)
            if reading is not None:
                assessment.ingest_lidar(w, reading)

    def _route_pass(self, trigger, event_ids, ingested, published, **plan_kw) -> fleet.RoutePass:
        """Immediate, bounded route pass under an acceptance hold; releases only on a validated
        continuation. Hypotheses are never touched."""
        w = self._world
        cycle = w.next_cycle
        w.next_cycle += 1
        w.hold = decisions.Hold(cycle, None, f"route pass ({trigger})", w.clock_s)
        rp = fleet.route_pass(
            w,
            self.geometry,
            self.circuits,
            self._physics,
            self.coordinator,
            trigger=trigger,
            event_ids=tuple(event_ids),
            ingested_at=ingested,
            published_at=published,
            **plan_kw,
        )
        w.route_passes.append(rp)
        if rp.outcome == "HELD":
            w.hold = decisions.Hold(cycle, None, f"no safe continuation: {rp.detail}", w.clock_s)
        else:
            w.hold = None
        settle_first = len(w.events)
        settle_events = settle(w, self._physics)
        for e in settle_events:
            w.events.append((f"EVT{len(w.events) + 1:04d}", e))
        self._ingest_readings(settle_first)
        w.record("route", f"{rp.id} {rp.outcome}: {rp.detail}")
        self._refresh_tasks()
        self.snapshot = copy.deepcopy(w)
        return rp

    def _dispatch(self, drone_id: str, segment_id: str, author: str) -> list[str]:
        point = LinePoint(segment_id, self.geometry.length_mm(segment_id) // 2)
        landing = LandingSite(point, f"{segment_id}/C1")
        drone = self._world.drones[drone_id]
        steps = []
        t = self._world.clock_s
        if isinstance(drone.state, Perched):
            perch = drone.state.perch
            perch_pos = self.geometry.position_of(perch.point)
            site_pos = self.geometry.position_of(landing.point)
            if perch == landing:
                raise CommandRejected(f"{drone_id} is already perched on {landing.circuit_id}.")
            steps.append(ScheduledStep(0, t, t + TAKEOFF_S, Takeoff(perch, perch_pos)))
            t += TAKEOFF_S
            leg = leg_seconds(perch_pos, site_pos, drone.speed_km_per_min)
            steps.append(ScheduledStep(1, t, t + leg, Transit(perch_pos, site_pos)))
            t += leg
            steps.append(ScheduledStep(2, t, t + LAND_S, Land(landing, site_pos)))
        else:
            raise CommandRejected(f"{drone_id} is {drone.phase} and cannot be dispatched.")
        request = SortieRequest(
            drone_id, tuple(steps), landing, self._world.revision,
            self._world.energy_model.revision, self._world.capabilities.revision,
        )
        return self._commit((request,), author)

    def submit_route_pass(self, reason: str, author: str = "coordinator"):
        return self.submit(RoutePassCommand(reason, author)).result(timeout=10)

    def preview_plan(self, **kw) -> planning.PlanningResult:
        """A revision-labelled preview from the latest published snapshot. It runs off the sim
        thread on an immutable copy and authorises nothing: only a validated CommitSorties moves
        a drone."""
        return planning.plan_sorties(
            self.snapshot,
            self.geometry,
            self.circuits,
            revision=self.snapshot.revision,
            **kw,
        )

    def _refresh_tasks(self, events=()) -> None:
        """Deterministic task refresh: runs for any evidence, coverage, expiry or availability
        change, with or without new hypotheses."""
        w = self._world
        notes, _ = needs.refresh(w, self.network, self.geometry)
        for n in notes:
            w.task_log.append((w.clock_s, n))
        self._update_values()
        w.score_preview = self._score_preview()

    def _operational_evidence(self) -> list[outage.OperationalEvidence]:
        """Public operational events only. The t=0 switching report is already a prior."""
        w = self._world
        out = [
            outage.OperationalEvidence(f"SW:{seg}", "switching_report", seg, clock_s=t)
            for seg, t in sorted(w.known_switching.items())
            if t > 0
        ]
        for e in sorted(w.knowledge.evidence.values(), key=lambda e: e.id):
            if e.source == "handshake":
                ok = e.claim is knowledge.Claim.CHARGE_OK
                kind = "handshake_ok" if ok else "handshake_failed"
                out.append(
                    outage.OperationalEvidence(
                        e.id, kind, e.circuit_id.split("/")[0], e.circuit_id, e.clock_s
                    )
                )
        return out

    def _update_outage(self) -> None:
        """Refresh the public posterior when (and only when) the evidence set changed."""
        w = self._world
        operational = self._operational_evidence()
        key = (tuple(sorted(w.readings)), tuple(e.id for e in operational))
        if key == self._outage_key:
            return
        t0 = time.perf_counter()
        w.outage_belief = outage.reconstruct(self.outage_model, w.readings, operational)
        w.outage_timing["belief_update_s"] = time.perf_counter() - t0
        self._outage_key = key
        self._value_key = None

    def _cost_of(self, acquisition) -> float | None:
        """Seconds for the cheapest drone that can reach and fly this acquisition on its present
        energy (one-way, takeoff and landing included); None if none can."""
        from grid_ops.sensing import LidarPass
        from grid_ops.world import ground_duration_s, scan_duration_s

        w = self._world
        if isinstance(acquisition, LidarPass):
            point = LinePoint(acquisition.target.segment_id, acquisition.target.start_mm)
            dwell = scan_duration_s(acquisition)
        else:
            point = acquisition.conductor.point
            dwell = ground_duration_s((acquisition,))
        target = self.geometry.position_of(point)
        model = w.energy_model
        best = None
        for d in w.drones.values():
            if not isinstance(d.state, Perched):
                continue
            travel = leg_seconds(d.position, target, d.speed_km_per_min)
            need = (
                model.units_per_airborne_s * (travel + TAKEOFF_S + LAND_S + dwell)
                + model.takeoff_units
                + model.landing_units
            )
            if need <= d.battery.remaining_units:
                best = min(best, travel + dwell) if best is not None else travel + dwell
        return best

    def _fleet_blocked(self, open_tasks) -> bool:
        """Open requests exist, the whole fleet is perched, and the latest route pass after the
        tasks were compiled found nothing it could fly (energy, reach or eligibility)."""
        w = self._world
        pending = [t for t in open_tasks if t.status is TaskStatusV2.PENDING]
        if not pending or not w.route_passes:
            return False
        if not all(isinstance(d.state, Perched | Stranded) for d in w.drones.values()):
            return False
        last_pass = w.route_passes[-1]
        compiled = max((c for c, n in w.task_log if "compiled" in n), default=0)
        return last_pass.outcome != "accepted-plan" and last_pass.sim_clock_s >= compiled

    def _update_values(self) -> None:
        """Condition expected-discrimination values on the current belief for the open tasks."""
        w = self._world
        belief = w.outage_belief
        open_tasks = sorted(
            (t for t in w.tasks.values() if t.status in scoring.OPEN), key=lambda t: t.id
        )
        blocked = self._fleet_blocked(open_tasks)
        key = (belief.digest, tuple((t.id, t.status) for t in open_tasks), blocked)
        if key == self._value_key:
            return
        t0 = time.perf_counter()
        values = outage.conditioned_values(
            self.outage_model, belief, [(t.id, t.acquisition) for t in open_tasks], w.readings
        )
        # routing-only scheduling is an ablation switch: tasks then earn the legacy flat award
        w.experiment_values = (
            {v.key: v.eig_bits for v in values.values} if self.belief_valued_scheduling else {}
        )
        w.experiment_omitted = values.omitted
        held = frozenset(
            (d.state.perch.circuit_id, d.state.perch.point.offset_mm)
            for d in w.drones.values()
            if isinstance(d.state, Perched | Stranded)
        )
        fleet = tuple(
            sorted((d.id, d.battery.remaining_units // 50_000, d.phase) for d in w.drones.values())
        )
        if self._proposal_key != (belief.digest, held, fleet):
            proposals, scored = outage.propose_experiments(
                self.outage_model, belief, w.readings, occupied=held, cost_of=self._cost_of
            )
            w.experiment_proposals = proposals
            self._best_eig = max((v.eig_bits for v in scored.values), default=0.0)
            self._proposal_key = (belief.digest, held, fleet)
        w.outage_stop = outage.assess_stop(
            self.outage_model,
            belief,
            best_eig_bits=self._best_eig,
            experiments_available=bool(w.experiment_proposals) and not blocked,
            budget_exhausted=w.finished,
        )
        w.outage_timing["request_value_s"] = time.perf_counter() - t0
        self._value_key = key

    def _score_preview(self) -> list[dict]:
        """Standalone synthetic score of each open task: a preview, never an authorisation."""
        w = self._world
        open_tasks = [t for t in w.tasks.values() if t.status in scoring.OPEN]
        if not open_tasks:
            return []
        ctx = scoring.build_context(w)
        rows = []
        for t in sorted(open_tasks, key=lambda t: t.id)[:80]:
            s = scoring.score_plan(w, ctx, [t.id])
            rows.append(
                {
                    "task": t.id,
                    "tier_points_5_to_1": s.tiers,
                    "owner_urgency": next(
                        (i for i, v in zip((5, 4, 3, 2, 1), s.tiers, strict=True) if v), None
                    ),
                    "notes": "; ".join(s.notes),
                }
            )
        return rows

    def evaluator_truth(self):
        """Handle for `grid_ops.evaluation` only; agents and the UI must never call this."""
        return self._truth

    def _commit(self, requests: tuple[SortieRequest, ...], author: str) -> list[str]:
        world = self._world
        if world.finished:
            raise CommandRejected("The exercise is over (T+04:00).")
        result = validate_sorties(world, self.geometry, self.circuits, requests)
        if not result.ok:
            reasons = tuple(str(r) for r in result.rejections)
            world.rejections.append(RejectionRecord(world.clock_s, author, reasons))
            raise CommandRejected("; ".join(reasons))
        apply_sorties(world, self._physics, result.plans)
        events = settle(world, self._physics)  # zero-distance arrivals complete immediately
        first_new = len(world.events)
        for e in events:
            world.events.append((f"EVT{len(world.events) + 1:04d}", e))
        self._ingest_readings(first_new)
        if events:
            world.revision += 1
        self._refresh_tasks()
        return [
            f"{p.drone_id}: itinerary until {format_clock(p.itinerary.end_s / 60)}, "
            f"final perch {p.itinerary.final_perch.circuit_id}"
            for p in result.plans
        ]

    def _disposition(
        self, reading_id: str, action: str, reason: str, author: str, epoch: str | None = None
    ) -> None:
        w = self._world
        reading = w.readings.get(reading_id)
        if reading is None:
            raise CommandRejected(f"Unknown reading {reading_id}.")
        if not reason.strip():
            raise CommandRejected("A disposition must state its reason.")
        current = w.epoch_overrides.get(reading_id, reading.evidence_epoch)
        if action == "EXCLUDED":
            if reading_id in w.excluded_readings:
                raise CommandRejected(f"{reading_id} is already excluded.")
            w.excluded_readings.add(reading_id)
        elif action == "REINSTATED":
            if reading_id not in w.excluded_readings:
                raise CommandRejected(f"{reading_id} is not excluded.")
            w.excluded_readings.discard(reading_id)
        else:
            if epoch is None or epoch == current:
                raise CommandRejected("Epoch reassignment needs a different epoch.")
            w.epoch_overrides[reading_id] = epoch
        w.dispositions.append(
            ReadingDisposition(
                reading_id,
                action,
                reason,
                w.clock_s,
                author,
                current if action == "EPOCH_REASSIGNED" else None,
                epoch,
            )
        )


    def _commit_decision(self, cmd: CommitDecision) -> str:
        w = self._world
        stored = w.decision_outcomes.get(cmd.decision_id)
        if stored is not None:  # a retry returns the stored outcome, with no repeated effects
            if stored.outcome is decisions.Outcome.ACCEPTED:
                return f"{cmd.decision_id} already {stored.outcome}."
            raise CommandRejected(
                f"{cmd.decision_id} already {stored.outcome}: " + "; ".join(stored.reasons)
            )
        ctx_args = ((self.network, self.circuits), self.geometry, self.circuits)

        def reject(reasons, cycle, fallback, superseded=False):
            status = ProposedStatus.PROPOSED_NOT_ACCEPTED
            if superseded:
                status = ProposedStatus.SUPERSEDED
            decisions.note_proposal(
                w,
                cmd.finding_id,
                status,
                f"{cmd.decision_id} not accepted: {'; '.join(reasons)}"
                + (f" (fleet: {fallback})" if fallback != "none needed" else ""),
            )
            outcome = decisions.DecisionOutcome(
                cmd.decision_id,
                decisions.Outcome.SUPERSEDED if superseded else decisions.Outcome.REJECTED,
                tuple(reasons),
                cycle,
                fallback,
                w.clock_s,
            )
            w.decision_outcomes[cmd.decision_id] = outcome
            w.rejections.append(RejectionRecord(w.clock_s, cmd.author, tuple(reasons)))
            raise CommandRejected("; ".join(reasons))

        # phase A: no hold, so a rejection leaves the running continuation untouched
        try:
            decisions.evaluate_commit(w, ctx_args[0], self.geometry, self.circuits, cmd)
        except decisions.DecisionRejected as e:
            reject(e.reasons, None, "none needed", e.superseded)
        # phase B: a new acceptance cycle; nothing is applied until every check has passed
        cycle = w.next_cycle
        w.next_cycle += 1
        w.hold = decisions.Hold(cycle, cmd.decision_id, "decision acceptance", w.clock_s)
        try:
            settle_first = len(w.events)
            settle(w, self._physics)  # process observations due right now
            self._ingest_readings(settle_first)
            ctx = decisions.evaluate_commit(w, ctx_args[0], self.geometry, self.circuits, cmd)
            cand = decisions.apply_candidate(w, ctx, cmd, cycle, w.clock_s)
            for segs in cand["model_changes"]:
                if not any(set(m["segments"]) == set(segs) for m in w.staged_models.values()):
                    raise decisions.DecisionRejected(
                        [f"model change {sorted(segs)} was never staged"]
                    )
                problems = decisions._test_candidate_model(ctx_args[0], w, frozenset(segs))
                if problems:
                    raise decisions.DecisionRejected(problems)
            closing = ctx["pf"].output.conclusion is not None and (
                ctx["pf"].output.conclusion.kind == "cause_identified"
            )
            if closing:
                if cmd.sorties:
                    raise decisions.DecisionRejected(
                        ["a concluding decision carries no new sorties"]
                    )
                _, problems = decisions.recovery_requests(
                    w, self.geometry, self.circuits, self._physics
                )
                if problems:
                    raise decisions.DecisionRejected(problems)
                batch, plans_wanted = (), set()
            else:
                others = {
                    d.id for d in w.drones.values() if d.id not in {s.drone_id for s in cmd.sorties}
                }
                batch = (
                    *cmd.sorties,
                    *(r for r in decisions.current_requests(w) if r.drone_id in others),
                )
                plans_wanted = {s.drone_id for s in cmd.sorties}
            validation = validate_sorties(w, self.geometry, self.circuits, batch)
            if not validation.ok:
                raise decisions.DecisionRejected([str(r) for r in validation.rejections])
            plans = tuple(p for p in validation.plans if p.drone_id in plans_wanted)
        except decisions.DecisionRejected as e:
            fallback, problems = decisions.route_only_continuation(
                w, self.geometry, self.circuits, self._physics
            )
            if fallback == "HELD":
                w.hold = decisions.Hold(
                    cycle,
                    cmd.decision_id,
                    "no complete safe continuation: " + "; ".join(problems),
                    w.clock_s,
                )
            else:
                w.hold = None
            reject(e.reasons, cycle, fallback, e.superseded)
        decisions.commit_apply(w, cand, ctx, cmd, plans, self._physics)
        apply_first = len(w.events)
        events = settle(w, self._physics)
        for e_ in events:
            w.events.append((f"EVT{len(w.events) + 1:04d}", e_))
        self._ingest_readings(apply_first)
        self._refresh_tasks()
        w.decision_outcomes[cmd.decision_id] = decisions.DecisionOutcome(
            cmd.decision_id, decisions.Outcome.ACCEPTED, (), cycle, "none needed", w.clock_s
        )
        w.hold = None  # this acceptance cycle releases itself: the accepted decision is current
        w.record(cmd.author, f"Decision {cmd.decision_id} accepted ({cmd.finding_id}).")
        w.operator_events.append(
            (f"OP{len(w.operator_events) + 1:04d}", f"accepted {cmd.decision_id}")
        )
        if self.auto_replan:  # an accepted explained update gets its own immediate route pass
            now = time.perf_counter()
            self._route_pass("accepted decision", (), now, now)
        return f"{cmd.decision_id} accepted."

    def _apply(self, command: Command) -> str:
        world = self._world
        match command:
            case Dispatch(drone_id, segment_id, author):
                if segment_id not in self.network.segments:
                    raise CommandRejected(f"Unknown segment {segment_id}.")
                lines = self._dispatch(drone_id, segment_id, author)
                result = f"{drone_id} departed for {segment_id}. " + lines[0]
            case CommitSorties(sorties, author):
                result = "Accepted: " + " | ".join(self._commit(sorties, author))
            case SetBelief(segment_id, status, note, author):
                if segment_id not in world.belief:
                    raise CommandRejected(f"Unknown segment {segment_id}.")
                world.belief[segment_id] = status
                result = f"{segment_id} marked {status}: {note}"
            case SetInService():
                raise CommandRejected(
                    "SetInService is not part of the three-method runtime; scenario mutation "
                    "belongs to the privileged evaluator"
                )
            case RegisterSite(segment_id, offset_mm, author):
                try:
                    site = self.geometry.site(LinePoint(segment_id, offset_mm))
                except ValueError as e:
                    raise CommandRejected(str(e)) from e
                world.sites[site.id] = site
                result = (
                    f"Site {site.id} registered at {site.position[0]:.6f}, {site.position[1]:.6f}."
                )
            case ExcludeReading(reading_id, reason, author):
                self._disposition(reading_id, "EXCLUDED", reason, author)
                result = f"{reading_id} excluded from the fit: {reason}"
            case ReinstateReading(reading_id, reason, author):
                self._disposition(reading_id, "REINSTATED", reason, author)
                result = f"{reading_id} reinstated: {reason}"
            case ReassignEpoch(reading_id, epoch, reason, author):
                self._disposition(reading_id, "EPOCH_REASSIGNED", reason, author, epoch)
                result = f"{reading_id} reassigned to epoch {epoch}: {reason}"
            case CommitDecision():
                return self._commit_decision(command)
            case RecordCoordinatorTurn(turn):
                world.coordinator_turns = [t for t in world.coordinator_turns if t.id != turn.id]
                world.coordinator_turns.append(turn)
                return turn.id
            case RecordPlannerPreview(preview):
                world.planner_previews.append(preview)
                return preview.id
            case AcceptLimits(statement, author):
                inv = world.investigation
                if inv.status is not InvestigationStatus.OPEN or not inv.unresolved:
                    raise CommandRejected("There is no stated unresolved limit to accept.")
                if not statement.strip():
                    raise CommandRejected("State the limits being accepted.")
                world.investigation = Investigation(
                    InvestigationStatus.CONCLUDED_WITH_LIMITS,
                    {"accepted_limits": statement},
                    None,
                    world.clock_s,
                    (*inv.history, (world.clock_s, f"{author} accepted limits: {statement}")),
                    (),
                    inv.unresolved,
                )
                self._refresh_tasks()
                return "Case closed with the stated limits accepted."
            case ReopenInvestigation(reason, author):
                inv = world.investigation
                if inv.status is InvestigationStatus.OPEN:
                    raise CommandRejected("The investigation is not concluded.")
                if not reason.strip():
                    raise CommandRejected("Reopening needs a reason.")
                world.investigation = Investigation(
                    InvestigationStatus.OPEN,
                    inv.conclusion,
                    inv.decision_id,
                    inv.concluded_s,
                    (*inv.history, (world.clock_s, f"reopened by {author}: {reason}")),
                )
                self._refresh_tasks()
                return "Investigation reopened; diagnostic needs regenerate from active hypotheses."
            case RoutePassCommand(reason, author):
                now = time.perf_counter()
                rp = self._route_pass(reason, (), now, now)
                return f"{rp.id} {rp.outcome}"
            case Release(cycle_id, decision_id, author):
                hold = world.hold
                accepted = world.decision_outcomes.get(decision_id)
                if (
                    hold is None
                    or hold.cycle_id != cycle_id
                    or hold.decision_id != decision_id
                    or accepted is None
                    or accepted.outcome is not decisions.Outcome.ACCEPTED
                ):
                    raise CommandRejected(
                        f"Late or unauthorised release (cycle {cycle_id}, decision {decision_id})."
                    )
                world.hold = None
                result = f"Hold cycle {cycle_id} released by accepted decision {decision_id}."
            case RequestModelChange(segments, author):
                unknown = [s for s in segments if s not in self.network.segments]
                if unknown:
                    raise CommandRejected(f"Unknown segment(s) {unknown}.")
                sid = decisions.stage_model_change(world, segments, author)
                return f"{sid} staged; accepted model unchanged."
            case RejectFinding(finding_id, reason, author):
                if not reason.strip():
                    raise CommandRejected("A rejection must state its reason.")
                found = next((p for p in world.proposals if p.id == finding_id), None)
                if found is None:
                    raise CommandRejected(f"Unknown finding {finding_id}.")
                if found.status is not ProposedStatus.PROPOSED_NOT_ACCEPTED:
                    raise CommandRejected(f"{finding_id} is {found.status}, not pending.")
                decisions.note_proposal(
                    world, finding_id, ProposedStatus.REJECTED, f"rejected by {author}: {reason}"
                )
                result = f"{finding_id} rejected: {reason}"
            case RecordProposal(author, revision, output, status, note, key):
                fid = f"PF{len(world.proposals) + 1:04d}"
                world.proposals.append(
                    ProposedFinding(
                        fid,
                        revision,
                        author,
                        status,
                        output,
                        world.clock_s,
                        ((world.clock_s, note),),
                        key,
                    )
                )
                world.record(author, f"{fid} {status}: {output.summary}")
                return fid
            case SetLandingPolicy(allow_uncertain, author):
                world.landing_policy = LandingPolicy(allow_uncertain)
                result = f"Landing policy: {world.landing_policy.name}."
            case Log(author, text):
                result = text
            case _:
                raise CommandRejected(f"Unknown command {type(command).__name__}.")
        world.record(author, result)
        if author == "operator" and isinstance(
            command,
            CommitSorties | SetLandingPolicy | ExcludeReading | ReinstateReading | ReassignEpoch,
        ):
            world.operator_events.append((f"OP{len(world.operator_events) + 1:04d}", result))
        if self.auto_replan and isinstance(
            command, SetLandingPolicy | ExcludeReading | ReinstateReading | ReassignEpoch
        ):
            now = time.perf_counter()
            self._route_pass(type(command).__name__, (), now, now)
        return result
