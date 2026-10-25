"""Asynchronous manager and planner roles around a running simulation.

The simulation keeps flying and replanning (see `fleet`) whatever the manager is doing: the
coordinator only reads published snapshots, coalesces arriving evidence into the next manager
turn without losing any trigger id, records the returned proposal, and submits it back as an
ordinary explained decision. Accepted state never depends on an unfinished model call: a slow,
failed or dead manager leaves the validated fleet continuation running. There are no per-drone
agents.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

from grid_ops import agents, manager_tools
from grid_ops.proposals import ProposedStatus
from grid_ops.sim import (
    CommitDecision,
    RecordCoordinatorTurn,
    RecordPlannerPreview,
    RecordProposal,
    Simulation,
)
from grid_ops.world import CommandRejected

MAX_REGENERATIONS = 3
# Evidence the manager must assess (every reading batch, even if routes are blocked).
ASSESSED_EVENT_KINDS = frozenset({"reading", "switching_report", "evidence_expired"})


@dataclass(frozen=True)
class CoordinatorTurn:
    id: str
    trigger_ids: tuple[str, ...]  # every evidence event this turn was asked to assess
    snapshot_revision: int
    status: str  # running | proposed | accepted | superseded | failed | timed-out
    finding_id: str | None
    note: str
    started_s: int
    wall_s: float = 0.0


@dataclass(frozen=True)
class PlannerPreview:
    id: str
    semantic_key: str
    snapshot_revision: int
    summary: str
    blocked: str | None
    unassigned: tuple[tuple[str, str], ...]
    authorises_dispatch: bool = False  # previews never dispatch anything


@dataclass
class Coordinator:
    sim: Simulation
    manager_fn: object = agents.run_scripted_turn  # (toolbox, trigger_ids=...) -> TurnResult
    auto_accept: bool = True
    debounce_s: float = 0.15
    _pending: list[str] = field(default_factory=list)
    _lock: threading.Lock = field(default_factory=threading.Lock)
    _wake: threading.Event = field(default_factory=threading.Event)
    _stop: threading.Event = field(default_factory=threading.Event)
    _thread: threading.Thread | None = None
    _planner_thread: threading.Thread | None = None
    _last_plan_key: str = ""
    turns_run: int = 0
    dead: bool = False
    failure: str = ""

    # -- lifecycle ------------------------------------------------------------------------------

    def start(self) -> None:
        self.sim.coordinator.listeners.append(self._on_batch)
        self._thread = threading.Thread(target=self._guarded_loop, name="manager", daemon=True)
        self._planner_thread = threading.Thread(
            target=self._planner_loop, name="planner", daemon=True
        )
        self._thread.start()
        self._planner_thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._wake.set()

    def join(self, timeout: float = 5.0) -> None:
        for t in (self._thread, self._planner_thread):
            if t is not None:
                t.join(timeout)

    def _on_batch(self, revision: int, event_ids: tuple[str, ...]) -> None:
        """Called by the sim right after it publishes evidence, before any route pass."""
        kinds = {i: e.kind for i, e in self.sim.snapshot.events}
        relevant = [i for i in event_ids if kinds.get(i) in ASSESSED_EVENT_KINDS]
        if not relevant:
            return
        with self._lock:
            self._pending.extend(relevant)
        self._wake.set()

    def pending_trigger_ids(self) -> tuple[str, ...]:
        with self._lock:
            return tuple(self._pending)

    # -- the manager role -------------------------------------------------------------------------

    def _guarded_loop(self) -> None:
        try:
            self._loop()
        except BaseException as e:  # the coordinator dying must never endanger the fleet
            self.dead = True
            self.failure = f"{type(e).__name__}: {e}"
            self._fail_safe()

    def _fail_safe(self) -> None:
        """Leave a validated continuation (or an explicit safety exception) behind."""
        try:
            self.sim.submit(
                RecordCoordinatorTurn(
                    CoordinatorTurn(
                        f"CT{self.turns_run + 1:03d}",
                        (),
                        self.sim.snapshot.revision,
                        "failed",
                        None,
                        f"coordinator died: {self.failure}",
                        self.sim.snapshot.clock_s,
                    )
                )
            ).result(timeout=5)
            self.sim.submit_route_pass("coordinator death")
        except Exception:
            pass

    def _loop(self) -> None:
        while not self._stop.is_set():
            self._wake.wait(timeout=0.5)
            if self._stop.is_set():
                return
            if not self._pending:
                self._wake.clear()
                continue
            time.sleep(self.debounce_s)  # coalesce a burst of readings into one turn
            with self._lock:
                triggers = tuple(dict.fromkeys(self._pending))
                self._pending.clear()
                self._wake.clear()
            self._run_turn(triggers)

    def run_turn_now(self, trigger_ids: tuple[str, ...] = ()) -> CoordinatorTurn:
        """Synchronous single turn (the loop uses the same path)."""
        return self._run_turn(trigger_ids or self.pending_trigger_ids())

    def _run_turn(self, triggers: tuple[str, ...]) -> CoordinatorTurn:
        snap = self.sim.snapshot
        n = len(snap.coordinator_turns) + 1
        turn_id = f"CT{n:03d}"
        t0 = time.time()
        self.sim.submit(
            RecordCoordinatorTurn(
                CoordinatorTurn(turn_id, triggers, snap.revision, "running", None, "", snap.clock_s)
            )
        ).result(timeout=5)
        status, finding, note = "failed", None, ""
        for _ in range(MAX_REGENERATIONS):
            try:
                result = self.manager_fn(self._toolbox(), trigger_ids=triggers)
            except Exception as e:
                note = f"manager error: {type(e).__name__}: {e}"
                break
            author = "manager-llm" if self.manager_fn is agents.run_llm_turn else "manager-scripted"
            if not agents.llm_available() and author == "manager-llm":
                author = "manager-scripted (no model credentials)"
            finding = self._record(result, author, snap.revision)
            if result.errors:
                note = "proposal rejected by validation: " + "; ".join(result.errors[:2])
                status = "failed"
                break
            status, note = "proposed", "valid proposal recorded; not accepted"
            if not self.auto_accept:
                break
            pf = self._proposal(finding)
            try:
                self.sim.submit(
                    CommitDecision(f"AUTO-{finding}", finding, pf.semantic_key, "coordinator")
                ).result(timeout=30)
                status, note = "accepted", f"accepted as AUTO-{finding}"
                break
            except CommandRejected as e:
                again = self._proposal(finding)
                if again.status is ProposedStatus.SUPERSEDED:
                    status = "superseded"
                    note = f"stale, archived and regenerated from current evidence ({e})"
                    triggers = tuple(dict.fromkeys((*triggers, *self.pending_trigger_ids())))
                    continue  # regenerate on the new evidence, keeping every trigger id
                note = f"not accepted: {e}"
                break
        wall = time.time() - t0
        final = CoordinatorTurn(
            turn_id, triggers, snap.revision, status, finding, note, snap.clock_s, round(wall, 2)
        )
        self.sim.submit(RecordCoordinatorTurn(final)).result(timeout=5)
        self.turns_run += 1
        return final

    def _toolbox(self) -> manager_tools.Toolbox:
        return manager_tools.build_toolbox(self.sim.snapshot, self.sim.network, self.sim.circuits)

    def _record(self, result, author: str, revision: int) -> str | None:
        status = (
            ProposedStatus.REJECTED
            if result.errors
            else ProposedStatus.PROPOSED_NOT_ACCEPTED
        )
        note = (
            "; ".join(result.errors) if result.errors else "validated against the pinned snapshot"
        )
        key = decisions_semantic_key(self.sim.snapshot, result.output)
        return self.sim.submit(
            RecordProposal(author, revision, result.output, status, note, key)
        ).result(timeout=10)

    def _proposal(self, finding_id: str):
        """The recorded proposal, once the next snapshot publishes it."""
        end = time.time() + 5
        while time.time() < end:
            for p in self.sim.snapshot.proposals:
                if p.id == finding_id:
                    return p
            time.sleep(0.05)
        raise RuntimeError(f"{finding_id} never appeared in a snapshot")

    # -- the planner role

    def _planner_loop(self) -> None:
        try:
            while not self._stop.is_set():
                time.sleep(0.4)
                self.planner_step()
        except BaseException as e:
            self.failure = self.failure or f"planner: {type(e).__name__}: {e}"

    def planner_step(self) -> PlannerPreview | None:
        """Preview a plan, but only when the semantic planning inputs changed."""
        w = self.sim.snapshot
        key = self.planning_key(w)
        if key == self._last_plan_key or not w.tasks:
            return None
        self._last_plan_key = key
        res = self.sim.preview_plan()
        best = res.best
        summary = (
            f"best {best.source}: tiers {best.score.tiers}, {len(best.sorties)} sortie(s)"
            if best
            else "no feasible plan"
        )
        prev = PlannerPreview(
            f"PV{len(w.planner_previews) + 1:03d}",
            key,
            w.revision,
            summary,
            res.blocked.reason if res.blocked else None,
            tuple((u.task_id, u.reason) for u in res.unassigned[:20]),
        )
        self.sim.submit(RecordPlannerPreview(prev)).result(timeout=5)
        return prev

    @staticmethod
    def planning_key(w) -> str:
        from grid_ops.decisions import _digest

        return _digest(
            {
                "hyp": w.hypotheses_version,
                "tasks": sorted(t.id for t in w.tasks.values() if t.status == "PENDING"),
                "requests": sorted(r for r, a in w.requests.items() if a.status == "ACTIVE"),
                "policy": w.landing_policy.name,
                "readings": len(w.readings),
            }
        )


def decisions_semantic_key(world, output) -> str:
    from grid_ops.decisions import semantic_key

    return semantic_key(world, output)
