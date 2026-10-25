"""Response-time and failure-recovery machinery around the planner.

`PlannerWorker` runs the CP-SAT planner in a separate warm process so that a hung or killed
solver cannot stall the simulation: a watchdog terminates it and the route pass falls back to the
validated incumbent. `latency_report` and `operator_actions` turn recorded route passes into the
numbers and concrete instructions an operator needs; budget misses are reported, never hidden by
simulation holds.
"""

from __future__ import annotations

import multiprocessing as mp
import os
import statistics
import time
from dataclasses import dataclass

from grid_ops import planning

P95_TARGET_S = 1.0
RESPONSE_BUDGET_S = planning.RESPONSE_BUDGET_S
WATCHDOG_S = planning.RESPONSE_BUDGET_S - 0.1


def _serve(conn) -> None:
    """Worker loop. `worker_fault` simulates a crash ('exit') or a hang ('hang')."""
    conn.send(("ready", None))
    while True:
        msg = conn.recv()
        if msg is None:
            return
        args, kw = msg
        fault = kw.pop("worker_fault", None)
        if fault == "exit":
            os._exit(3)
        if fault == "hang":
            time.sleep(3600)
        try:
            conn.send(("ok", planning.plan_sorties(*args, **kw)))
        except Exception as e:
            conn.send(("error", f"{type(e).__name__}: {e}"))


class PlannerFailure(RuntimeError):
    """Raised to the route pass when the worker could not deliver a result."""

    def __init__(self, kind: str, detail: str):
        super().__init__(f"{kind}: {detail}")
        self.kind = kind


class PlannerWorker:
    def __init__(self, watchdog_s: float = WATCHDOG_S):
        self.watchdog_s = watchdog_s
        self.restarts = 0
        self.failures: list[str] = []
        self._proc = None
        self._conn = None
        self._spawn()

    def _spawn(self) -> None:
        ctx = mp.get_context("spawn")
        parent, child = ctx.Pipe()
        self._proc = ctx.Process(target=_serve, args=(child,), daemon=True)
        self._proc.start()
        child.close()
        self._conn = parent
        if not self._conn.poll(60):
            raise RuntimeError("planner worker failed to start")
        self._conn.recv()

    @property
    def alive(self) -> bool:
        return self._proc is not None and self._proc.is_alive()

    def _restart(self) -> None:
        self.close()
        self.restarts += 1
        self._spawn()

    def plan(self, world, geometry, circuits, **kw) -> planning.PlanningResult:
        """The planner contract (`Coordinator.planner`): returns a result or raises."""
        if not self.alive:
            self._restart()
        try:
            self._conn.send(((world, geometry, circuits), kw))
        except (BrokenPipeError, OSError):
            self._restart()
            self._conn.send(((world, geometry, circuits), kw))
        try:
            if not self._conn.poll(self.watchdog_s):
                self._note("WATCHDOG", f"no answer in {self.watchdog_s:.1f}s; worker terminated")
                self._restart()
                raise PlannerFailure("watchdog", f"worker killed after {self.watchdog_s:.1f}s")
            status, payload = self._conn.recv()
        except (EOFError, ConnectionResetError, OSError):
            self._note("TERMINATED", "worker process died during the solve")
            self._restart()
            raise PlannerFailure("worker-terminated", "worker process died") from None
        if status == "error":
            self._note("ERROR", payload)
            raise PlannerFailure("worker-error", payload)
        return payload

    def _note(self, kind: str, detail: str) -> None:
        self.failures.append(f"{kind}: {detail}")

    def close(self) -> None:
        try:
            if self._conn is not None:
                self._conn.close()
        except OSError:
            pass
        if self._proc is not None and self._proc.is_alive():
            self._proc.terminate()
            self._proc.join(2)
        self._proc = None


# -- measurement and operator view ----------------------------------------------------------


@dataclass(frozen=True)
class LatencyReport:
    n: int
    p50_s: float
    p95_s: float
    max_s: float
    budget_misses: tuple[str, ...]  # route-pass ids over the response budget
    p95_ok: bool
    fallbacks: tuple[tuple[str, str], ...]  # (pass id, why the planner result was not used)
    held: tuple[str, ...]
    stage_means_s: dict

    def summary(self) -> str:
        if not self.n:
            return "no route passes measured"
        verdict = "within" if self.p95_ok and not self.budget_misses else "OUTSIDE"
        return (
            f"{self.n} passes: p50 {self.p50_s:.3f}s, p95 {self.p95_s:.3f}s, max {self.max_s:.3f}s"
            f" ({verdict} the p95 {P95_TARGET_S:.0f}s / {RESPONSE_BUDGET_S:.0f}s budget); "
            f"{len(self.budget_misses)} budget miss(es), {len(self.fallbacks)} fallback(s), "
            f"{len(self.held)} held"
        )


def _percentile(xs: list[float], q: float) -> float:
    xs = sorted(xs)
    return xs[min(len(xs) - 1, max(0, int(-(-q * len(xs) // 1)) - 1))]


def is_fallback(rp) -> bool:
    """A pass whose accepted route did not come from a completed solver run."""
    text = rp.solver or ""
    statuses = set(rp.stage_statuses)
    return (
        "solver failure" in text
        or "no feasible" in text
        or "wall cap" in text
        or bool(statuses & {"UNKNOWN", "MODEL_INVALID", "INFEASIBLE"})
        or rp.outcome in ("landing-only", "HELD")
    )


def latency_report(passes) -> LatencyReport:
    passes = [p for p in passes if p.outcome != "no-op" or p.planner_s > 0]
    if not passes:
        return LatencyReport(0, 0, 0, 0, (), True, (), (), {})
    lat = [p.latency_s for p in passes]
    p95 = _percentile(lat, 0.95)
    misses = tuple(p.id for p in passes if p.latency_s > RESPONSE_BUDGET_S)
    return LatencyReport(
        len(passes),
        statistics.median(lat),
        p95,
        max(lat),
        misses,
        p95 <= P95_TARGET_S,
        tuple((p.id, p.solver or p.outcome) for p in passes if is_fallback(p)),
        tuple(p.id for p in passes if p.outcome == "HELD"),
        {
            "solver_and_scoring": statistics.fmean(p.planner_s for p in passes),
            "ingest_to_publish": statistics.fmean(p.published_at - p.ingested_at for p in passes),
            "publish_to_route": statistics.fmean(
                p.route_published_at - p.published_at - p.planner_s for p in passes
            ),
        },
    )


def operator_actions(world) -> list[str]:
    """Concrete instructions when safe continuation is impossible or the planner is degraded."""
    out: list[str] = []
    if world.hold is not None:
        out.append(
            f"HELD ({world.hold.reason}): simulation time is frozen. Land the airborne drones "
            "with landing-only sorties in the sortie builder, or open the landing policy to "
            "uncertain sites; the hold releases on a validated continuation."
        )
    last = world.route_passes[-1] if world.route_passes else None
    if last is not None and last.outcome == "landing-only":
        out.append(
            f"{last.id}: recovery used landing-only routes; review the Routes tab before more "
            "measurements are scheduled."
        )
    if last is not None and is_fallback(last) and last.outcome != "HELD":
        out.append(
            f"{last.id}: planner fell back ({last.solver or last.outcome}); the validated "
            "incumbent is flying, nothing needs doing unless this repeats."
        )
    return out
