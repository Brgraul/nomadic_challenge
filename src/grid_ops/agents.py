"""Manager agent: reads a pinned public snapshot with read-only tools and proposes findings.

There are no per-drone agents. The manager can inspect evidence and request measurements; it
cannot inspect hidden truth, dispatch drones, overwrite observations, change the model or release
a hold. Whatever it returns is validated deterministically by the coordinator; accepting it is a
separate, atomic decision. The agent never sees a Simulation object: the coordinator hands it a
pinned Toolbox and records whatever comes back.
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass
from pathlib import Path

from grid_ops import proposals, scripted_manager
from grid_ops.env import load_env
from grid_ops.hypotheses import ManagerOutput
from grid_ops.manager_tools import Toolbox

load_env()
MODEL_NAME = os.environ.get("GRID_OPS_MODEL", "z-ai/glm-5.3-flash")
MODEL = f"openrouter:{MODEL_NAME}"

RECORDINGS = Path(__file__).resolve().parents[2] / "recordings"

SYSTEM = """You are the investigation manager for a power-grid outage after a windstorm.
Your job is an evidence-supported cause of the outage and a scoped recommended action. You own
hypotheses, urgency (1-5; urgency is not probability) and explicit measurement requests. You work
only from a pinned, read-only snapshot through the tools. You cannot see ground truth and you
cannot command drones.

Rules:
- State falsifiable claims with spatial scope, predictions and required conditions.
- You may request exactly three measurement methods: lidar (a moving geometric corridor scan),
  tdr (coupled reflectometry from a landed access site; a valid return path is required and zero
  current is never valid isolation evidence), and contact_current (a landed single-conductor RMS
  sample). The optimizer chooses drones, routes and charging; you choose only method and scope.
- A zero-current contact reading never proves a circuit is de-energized: it is compatible with
  both a live-unloaded and a disconnected circuit. Name both alternatives instead of concluding.
- A detected low-clearance LiDAR observation is an urgent hazard finding, but it does not by
  itself explain the outage; cite switching or electrical evidence for any causal claim.
- Every revision needs: before/after values, a finding_summary, a causal update_reason, and the
  remaining_uncertainty. Cite real ids (reports SYN_REPORT_xx, readings Rxxxx, events EVTxxxx,
  hazards HZxxx). Quote only numbers present in cited readings.
- At startup, cite the reports and say plainly that no drone measurements exist yet.
- For a measurement-triggered revision, cite the trigger events and every latest tied reading.
- Hypotheses unchanged this turn get a turn assessment, not a revision.
- Say what you could not distinguish in untested_alternatives.
- Measurement requests must cite the hypothesis they serve and the evidence they rest on, with
  rationale, alternatives_distinguished and expected_outcomes filled in.
- Never propose a conclusion on a startup turn or while reconstruction.stop_assessment.state is
  "continue": request the measurement instead. Propose a cause only when the state is
  "supported", and an unresolved stop only for the other stop states.
- A startup revision (no readings yet) must list at least one report id (SYN_REPORT_xx) in
  contributing_evidence_ids and say "no drone measurements" in its text. Every recommendation of
  kind measure or inspect needs a scope span (segment_id, start_km, end_km). Always finish by
  calling the ManagerOutput tool; never end with prose.
- A new hypothesis has before = null and change_kinds ["creation"]; a revision of an accepted one
  copies its current values into before. Set each request's source_revision to the snapshot
  revision number from the tool labels.
- Write ranges with "to" (0.2 to 0.7 km) and quote only numbers copied from tool output.
- Call reconstruction before each turn's requests. It returns the system's supported facts, the
  alternatives it cannot yet separate and a stop assessment; never state a posterior probability
  or an information score yourself. Request the measurement that separates the leading
  alternatives (suggest_experiments ranks candidates), not one that repeats an equivalent
  sample. A hazard, a zero-current reading or a truncated candidate pool never completes a
  diagnosis; stop only as the stop assessment allows and report the unresolved remainder.
Keep the number of hypotheses small (at most 3) and scopes to the 6 most relevant spans."""


SYSTEM_MESSAGE = {"role": "system", "content": SYSTEM}


def llm_available() -> bool:
    return bool(os.environ.get("OPENROUTER_API_KEY"))


def make_model():
    """The manager's model, served through OpenRouter."""
    from langchain_openai import ChatOpenAI

    return ChatOpenAI(
        model=MODEL_NAME,
        base_url="https://openrouter.ai/api/v1",
        api_key=os.environ["OPENROUTER_API_KEY"],
        max_tokens=16000,
        timeout=180,
        # a reasoning model can spend the whole budget thinking and never write the answer
        extra_body={"reasoning": {"effort": "low"}},
    )


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
        SYN_REPORT_02. Use these as hypothesis scope and request targets."""
        return tb.segments_in_report(report_id)

    @tool
    def list_readings(method: str | None = None) -> str:
        """Recorded acquisitions with their typed payloads, quality and provenance."""
        return tb.list_readings(method)

    @tool
    def get_reading(reading_id: str) -> str:
        """One reading in full."""
        return tb.get_reading(reading_id)

    @tool
    def list_requests() -> str:
        """Accepted measurement requests with their lifecycle status and served-by tasks."""
        return tb.list_requests()

    @tool
    def scan_coverage(segment_id: str) -> str:
        """Inspected versus missing intervals of a segment from the LiDAR readings so far."""
        return tb.scan_coverage(segment_id)

    @tool
    def charge_sites() -> str:
        """Charging capability per circuit: state, reason and evidence expiry."""
        return tb.charge_sites()

    @tool
    def capabilities() -> str:
        """The three-method catalog, sensor capabilities and declared model assumptions."""
        return tb.capabilities()

    @tool
    def reconstruction() -> str:
        """Supported outage facts, alternatives not yet separated, unknown regions, model
        checks and the stop assessment. Computed by the system from executed readings only."""
        return tb.reconstruction()

    @tool
    def suggest_experiments() -> str:
        """Ranked candidate measurements by expected discrimination between the remaining
        alternatives. Request one (or another you can justify); you do not invent the scores."""
        return tb.suggest_experiments()

    @tool
    def list_events(limit: int = 30) -> str:
        """Recent observable events with ids you can cite as triggers."""
        return tb.list_events(limit)

    @tool
    def list_hypotheses() -> str:
        """Currently accepted hypotheses (their values are the 'before' of any revision)."""
        return tb.list_hypotheses()

    return [
        overview,
        list_reports,
        segments_in_report,
        list_readings,
        get_reading,
        list_requests,
        scan_coverage,
        charge_sites,
        capabilities,
        reconstruction,
        suggest_experiments,
        list_events,
        list_hypotheses,
    ]


@dataclass
class TurnResult:
    output: ManagerOutput
    errors: list[str]
    transcript: list[dict]


ASSESSED = ("reading", "switching_report", "evidence_expired")


def assessment_trigger_ids(tb: Toolbox) -> list[str]:
    return [i for i, e in tb.world.events if e.kind in ASSESSED]


def run_scripted_turn(
    tb: Toolbox, trigger_ids: tuple[str, ...] = (), conclude: bool = False
) -> TurnResult:
    if not tb.world.readings:
        output = scripted_manager.startup_proposal(tb)
    else:
        output = scripted_manager.measurement_proposal(
            tb, list(trigger_ids) or assessment_trigger_ids(tb)
        )
        if conclude:
            output = scripted_manager.add_conclusion(tb, output)
    errors = proposals.validate(output, tb)
    return TurnResult(output, errors, [])


def run_adaptive_scripted_turn(
    tb: Toolbox, trigger_ids: tuple[str, ...] = (), conclude: bool = False
) -> TurnResult:
    """The scripted policy plus requests chosen from the public reconstruction's ranking."""
    triggers = list(trigger_ids) or assessment_trigger_ids(tb)
    output = ManagerOutput(summary="the report-driven hypotheses are not tracked by this policy.")
    output = scripted_manager.add_adaptive(tb, output, triggers, conclude)
    return TurnResult(output, proposals.validate(output, tb), [])


def _finalize(history) -> ManagerOutput | None:
    """The tool phase ended without a structured answer (the model stopped on an empty reply):
    ask once, with no tools, for the complete proposal from what it already gathered."""
    try:
        structured = make_model().with_structured_output(ManagerOutput)
        ask = {
            "role": "user",
            "content": "You have gathered the evidence. Return the complete ManagerOutput now.",
        }
        return structured.invoke([SYSTEM_MESSAGE, *history, ask])
    except Exception:
        return None


def run_llm_turn(
    tb: Toolbox,
    trigger_ids: tuple[str, ...] = (),
    instruction: str | None = None,
    record: bool = False,
    max_repairs: int = 3,
) -> TurnResult:
    """One manager turn with a real model. If validation rejects the output, the errors are fed
    back for up to `max_repairs` corrections; the final output is what the coordinator records."""
    from langchain.agents import create_agent
    from langchain.agents.structured_output import ToolStrategy

    agent = create_agent(
        make_model(),
        tools=_tools(tb),
        system_prompt=SYSTEM,
        response_format=ToolStrategy(ManagerOutput),
    )
    prompt = instruction or (
        "Startup turn: no drone measurements exist. Propose initial hypotheses from the reports "
        "and request the measurements that would test them."
        if not tb.world.readings
        else (
            f"New measurements arrived (events "
            f"{list(trigger_ids) or assessment_trigger_ids(tb)}). "
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
        if output is None and result.get("messages"):
            output = _finalize(result["messages"])
        if output is None:
            errors = [
                "the response was not a structured ManagerOutput: call the ManagerOutput tool "
                "with the complete proposal instead of replying in prose"
            ]
        else:
            errors = proposals.validate(output, tb)
        attempts.append(errors)
        if not errors:
            break
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
        errors = ["no structured proposal"]
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
                    "validation_errors": errors,
                    "attempt_errors": attempts,
                    "seconds": round(time.time() - t0, 1),
                },
                indent=1,
                default=str,
            )
        )
    return TurnResult(output, errors, transcript)
