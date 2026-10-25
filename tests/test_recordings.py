"""Recorded live manager turns (real model, real simulation): the LLM-authored findings."""

from __future__ import annotations

import pytest

pytest.skip(
    "DEFERRED to the recordings slice: recordings/ still hold legacy-runtime turns (old sensor "
    "enums, estimate/consistency/electrical-candidate tools) and re-recording needs an "
    "OPENROUTER_API_KEY, so the UI slice adapted scripts/record_* to the three-method toolbox "
    "but could not regenerate offline. Run scripts/record_startup_turn.py and "
    "scripts/record_manager_turn.py with credentials, then re-enable this file unchanged: the "
    "expectations below already match the three-method tool set.",
    allow_module_level=True,
)

import json  # noqa: E402
from pathlib import Path  # noqa: E402

RECORDINGS = sorted(
    (Path(__file__).resolve().parents[1] / "recordings").glob("manager_turn_*.json")
)

# The three-method LLM tool set (grid_ops.agents._tools): no estimate/consistency/electrical
# tools exist any more. "ManagerOutput" is the structured-response tool call.
ALLOWED_TOOLS = {
    "overview",
    "list_reports",
    "segments_in_report",
    "list_readings",
    "get_reading",
    "list_requests",
    "scan_coverage",
    "charge_sites",
    "capabilities",
    "list_events",
    "list_hypotheses",
    "ManagerOutput",
}


def load():
    return [json.loads(p.read_text()) for p in RECORDINGS]


def test_there_is_a_valid_recorded_startup_turn_and_a_valid_measurement_turn():
    turns = load()
    assert turns, "run scripts/record_startup_turn.py and scripts/record_manager_turn.py"
    startup = [t for t in turns if "startup" in t["prompt"].lower() and not t["validation_errors"]]
    measured = [
        t for t in turns if "measurements arrived" in t["prompt"] and not t["validation_errors"]
    ]
    assert startup and measured


@pytest.mark.parametrize("turn", load(), ids=lambda t: f"r{t['snapshot_revision']}")
def test_recorded_turns_keep_inputs_outputs_and_tool_use(turn):
    assert "glm" in turn["model"].lower()
    assert turn["system"] and turn["prompt"] and turn["seconds"] > 0
    calls = [c for m in turn["transcript"] for c in (m.get("tool_calls") or [])]
    assert calls, "the manager must have used its read-only tools"
    assert {c["name"] for c in calls} <= ALLOWED_TOOLS
    assert turn["output"]["revisions"]


def test_llm_authored_revisions_carry_findings_and_reasons():
    for turn in load():
        if turn["validation_errors"]:
            continue
        for rev in turn["output"]["revisions"]:
            assert rev["finding_summary"].strip() and rev["update_reason"].strip()
            assert rev["remaining_uncertainty"].strip()
            assert 1 <= rev["after"]["priority"] <= 5  # urgency, not a probability