"""Manager agent: reads a pinned public snapshot with read-only tools and proposes findings.

There are no per-drone agents. The manager can inspect evidence and test electrical candidates
in isolation; it cannot inspect hidden truth, dispatch drones, overwrite observations, change
the model or release a hold. Whatever it returns is validated deterministically and recorded as
a PROPOSED_NOT_ACCEPTED finding; accepting it is a separate, atomic decision.
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass
from pathlib import Path

from grid_ops import decisions, proposals, scripted_manager
from grid_ops.env import load_env
from grid_ops.hypotheses import ManagerOutput
from grid_ops.manager_tools import Toolbox, build_toolbox
from grid_ops.sim import RecordProposal, Simulation

load_env()
MODEL_NAME = os.environ.get("GRID_OPS_MODEL", "z-ai/glm-5.3-flash")
MODEL = f"openrouter:{MODEL_NAME}"


def llm_available() -> bool:
    return bool(os.environ.get("OPENROUTER_API_KEY"))


def make_model():
    """The manager's model, served through OpenRouter."""
    from langchain_openai import ChatOpenAI

    return ChatOpenAI(
        model=MODEL_NAME,
        base_url="https://openrouter.ai/api/v1",
        api_key=os.environ["OPENROUTER_API_KEY"],
        max_tokens=8000,
        timeout=180,
    )


RECORDINGS = Path(__file__).resolve().parents[2] / "recordings"

SYSTEM = """You are the investigation manager for a power-grid outage after a windstorm.
Your job is an evidence-supported cause of the outage and a scoped recommended action. You own
hypotheses, urgency (1-5; urgency is not probability) and recommendations. You work only from a
pinned, read-only snapshot through the tools. You cannot see ground truth and you cannot
command drones.

Rules:
- State falsifiable claims with spatial scope, predictions and required conditions.
- A qualified negative contradicts only the interval actually inspected. Thermal negatives
  without excitation and motion negatives outside a gust are inconclusive.
- A detected local hazard is an urgent hazard action, but it does not by itself explain the outage.
- Test any electrical candidate (a changed topology) with test_electrical_candidate before you
  propose it, and cite the returned diagnostic id.
- Every revision needs: before/after values, a finding_summary, a causal update_reason, and the
  remaining_uncertainty. Cite real ids (reports SYN_REPORT_xx, readings Rxxxx, events EVTxxxx,
  hazards HZxxx, diagnostics DX-...). Quote only numbers present in cited readings.
- At startup, cite the reports and say plainly that no drone measurements exist yet.
- For a measurement-triggered revision, cite the trigger events and EVERY latest tied measurement
  with value, unit, location and conditions, plus any contrary evidence.
- Hypotheses unchanged this turn get a turn assessment, not a revision.
- Say what you could not distinguish in untested_alternatives.

Checklist for every revision (validation rejects anything missing):
1. `scope`: non-empty list of spans taken from segments_in_report (never invented).
2. `predictions`: at least one with a statement, `expected`, a `measurement` name and conditions.
3. `finding_summary`, `update_reason`, `remaining_uncertainty`: non-empty plain sentences.
4. `change_kinds` includes "creation" and `before` is null for a new hypothesis.
5. Startup: trigger_kind "startup", `contributing_evidence_ids` = the SYN_REPORT ids you used, and
   the finding_summary or update_reason contains the words "no drone measurements".
6. Measurement turn: trigger_kind "measurement", trigger_event_ids from list_events (kind
   reading), latest_measurement_ids = every latest reading inside the scope, cited_values with
   the exact payload numbers, contrary_evidence_ids for readings that disagree with the claim.
7. `priority` is an integer 1-5 (urgency); `assessment` is one of SUPPORTED, PLAUSIBLE,
   UNRESOLVED, UNLIKELY, REFUTED.
Keep the number of hypotheses small (at most 3) and scopes to the 6 most relevant spans."""


def _tools(tb: Toolbox):
    from langchain.tools import tool

    @tool
    def overview() -> str:
        """Snapshot overview: clock, drones, switching, hazards, whether measurements exist."""
        return tb.overview()

    @tool
    def list_reports() -> str:
        """The six fictional storm reports (approximate, possibly wrong)."""
        return tb.list_reports()

    @tool
    def segments_in_report(report_id: str) -> str:
        """Exact spans {segment_id,start_km,end_km} of mapped lines inside a report area, e.g.
        SYN_REPORT_02. Use these as hypothesis scope."""
        return tb.segments_in_report(report_id)

    @tool
    def list_readings(kind: str | None = None) -> str:
        """Immutable readings with numeric values, conditions and interpretation."""
        return tb.list_readings(kind)

    @tool
    def get_reading(reading_id: str) -> str:
        """One reading in full."""
        return tb.get_reading(reading_id)

    @tool
    def scope_assessment(segment_id: str, start_km: float, end_km: float, condition: str) -> str:
        """PASS/FAIL/INDETERMINATE/NOT_SUPPORTED for vegetation_clearance, hot_joint or
        conductor_motion over a span."""
        return tb.scope_assessment(segment_id, start_km, end_km, condition)

    @tool
    def estimate_summary() -> str:
        """Reduced DC estimate: prior-only or data-conditioned, observable flows."""
        return tb.estimate_summary()

    @tool
    def consistency_summary() -> str:
        """Residual diagnostics: testability, critical sets, candidate explanations."""
        return tb.consistency_summary()

    @tool
    def list_events(limit: int = 30) -> str:
        """Recent observable events with ids you can cite as triggers."""
        return tb.list_events(limit)

    @tool
    def list_hypotheses() -> str:
        """Currently accepted hypotheses (their values are the 'before' of any revision)."""
        return tb.list_hypotheses()

    @tool
    def test_electrical_candidate(out_of_service_segments: list[str]) -> str:
        """Test a candidate topology (segments assumed out of service) on the pinned readings."""
        return tb.test_electrical_candidate(out_of_service_segments)

    return [
        overview,
        list_reports,
        list_readings,
        get_reading,
        scope_assessment,
        estimate_summary,
        consistency_summary,
        list_events,
        list_hypotheses,
        test_electrical_candidate,
    ]


@dataclass
class TurnResult:
    finding_id: str | None
    output: ManagerOutput | None
    errors: list[str]
    transcript: list[dict]


def trigger_events(tb: Toolbox, since_revision: int = -1) -> list[str]:
    return [i for i, e in tb.world.events if e.kind == "reading"]


def run_scripted_turn(
    sim: Simulation, trigger_ids: tuple[str, ...] = (), conclude: bool = False
) -> TurnResult:
    tb = build_toolbox(sim.snapshot, sim.network, sim.circuits)
    if not tb.world.readings:
        output = scripted_manager.startup_proposal(tb)
    else:
        output = scripted_manager.measurement_proposal(
            tb, list(trigger_ids) if trigger_ids else trigger_events(tb)
        )
        if conclude:
            output = scripted_manager.add_conclusion(tb, output)
    return _record(sim, tb, output, "manager-scripted", [])


def run_llm_turn(
    sim: Simulation,
    instruction: str | None = None,
    trigger_ids: tuple[str, ...] = (),
    record: bool = False,
    max_repairs: int = 2,
) -> TurnResult:
    """One manager turn with a real model. If validation rejects the output, the errors are fed
    back for up to `max_repairs` corrections; the final output is what gets recorded."""
    from langchain.agents import create_agent
    from langchain.agents.structured_output import ToolStrategy

    tb = build_toolbox(sim.snapshot, sim.network, sim.circuits)
    agent = create_agent(
        make_model(),
        tools=_tools(tb),
        system_prompt=SYSTEM,
        response_format=ToolStrategy(ManagerOutput),
    )
    prompt = instruction or (
        "Startup turn: no drone measurements exist. Propose initial hypotheses from the reports."
        if not tb.world.readings
        else (
            f"New measurements arrived (events {list(trigger_ids) or trigger_events(tb)}). "
            + (
                "Revise the accepted hypotheses."
                if tb.world.hypotheses
                else "No hypotheses are accepted yet: create them from the reports and the "
                "readings (change kind 'creation', before null) and cite the readings."
            )
        )
    )
    t0 = time.time()
    messages = [{"role": "user", "content": prompt}]
    attempts: list[list[str]] = []
    output = None
    result: dict = {"messages": []}
    for _ in range(max_repairs + 1):
        result = agent.invoke({"messages": messages})
        output = result.get("structured_response")
        if output is None:
            errors = ["the response was not a structured ManagerOutput"]
        else:
            errors = proposals.validate(output, tb)
        attempts.append(errors)
        if not errors:
            break
        # a fresh turn that carries the rejected draft and its errors (a replayed tool-call
        # history confuses structured output on some models)
        draft = output.model_dump_json(indent=1) if output is not None else "(none)"
        messages = [
            {
                "role": "user",
                "content": prompt
                + "\n\nYour previous proposal was REJECTED by validation.\nPrevious proposal:\n"
                + draft
                + "\n\nValidation errors to fix:\n- "
                + "\n- ".join(errors[:14])
                + "\n\nCall the tools again if you need to, then return a corrected, complete "
                "ManagerOutput.",
            }
        ]
    transcript = [
        {
            "role": getattr(m, "type", "?"),
            "content": getattr(m, "content", ""),
            "tool_calls": getattr(m, "tool_calls", None),
        }
        for m in result["messages"]
    ]
    if output is None:
        output = ManagerOutput(summary="The model returned no structured proposal.")
    turn = _record(sim, tb, output, "manager-llm", transcript)
    if record:
        RECORDINGS.mkdir(exist_ok=True)
        path = RECORDINGS / f"manager_turn_r{tb.snapshot.revision}_{int(t0)}.json"
        path.write_text(
            json.dumps(
                {
                    "model": MODEL,
                    "snapshot_revision": tb.snapshot.revision,
                    "prompt": prompt,
                    "system": SYSTEM,
                    "transcript": transcript,
                    "output": output.model_dump(mode="json"),
                    "validation_errors": turn.errors,
                    "attempt_errors": attempts,
                    "seconds": round(time.time() - t0, 1),
                },
                indent=1,
                default=str,
            )
        )
    return turn


def _record(sim, tb: Toolbox, output: ManagerOutput, author: str, transcript) -> TurnResult:
    errors = proposals.validate(output, tb)
    status = (
        proposals.ProposedStatus.REJECTED
        if errors
        else proposals.ProposedStatus.PROPOSED_NOT_ACCEPTED
    )
    note = "; ".join(errors) if errors else "validated against the pinned snapshot"
    key = decisions.semantic_key(tb.world, output)
    fid = sim.submit(
        RecordProposal(author, tb.snapshot.revision, output, status, note, key)
    ).result(timeout=10)
    return TurnResult(fid, output, errors, transcript)
