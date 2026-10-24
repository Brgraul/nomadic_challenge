"""Ticket-level e2e through the real app script (see drive.py)."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from drive import AppDriver


def test_app_boots_and_shows_prior_only_estimate():
    app = AppDriver()
    assert "PRIOR-ONLY" in app.text()
    assert "Scenario incident_demo" in app.text()


def fly_through_ui(app, drone, segment, kind):
    app.pick("inspect_segment", segment)
    app.pick("draft_drone", drone)
    app.pick("draft_kind", kind)
    app.pick("draft_circuit", f"{segment}/C1")
    app.click("Add stop here")
    app.click("Land here")
    app.click("Submit sorties")


def test_t07_current_reading_updates_the_public_estimate_in_the_console():
    app = AppDriver()
    fly_through_ui(app, "DRONE_3", "LINE_026", "current")
    deadline = __import__("time").time() + 40
    while (
        "1 accepted aggregate-P reading" not in app.text() and __import__("time").time() < deadline
    ):
        app.refresh(1.0)
    text = app.text()
    assert "1 accepted aggregate-P reading(s)." in text
    assert "PRIOR-ONLY" not in text
    assert "Electrical solves so far: 2" in text
    assert "Reading R0001 (current)" in text


def wait_for(app, needle, seconds=45):
    import time

    end = time.time() + seconds
    while needle not in app.text() and time.time() < end:
        app.refresh(1.0)
    assert needle in app.text(), app.text()[-1500:]


def test_t08_consistency_and_envelope_in_the_console_healthy_mode():
    app = AppDriver(mode="healthy_benchmark")
    fly_through_ui(app, "DRONE_3", "LINE_026", "current")
    wait_for(app, "Reading R0001 (current)")
    # a second drone reads the same line: redundancy appears, the data are model-limited
    app.pick("inspect_segment", "LINE_026")
    app.pick("draft_drone", "DRONE_2")
    app.pick("draft_kind", "current")
    app.click("Add stop here")
    app.pick("inspect_segment", "LINE_025")
    app.pick("draft_circuit", "LINE_025/C1")
    app.click("Land here")
    app.click("Submit sorties")
    wait_for(app, "Reading R0002 (current)")
    text = app.text()
    assert "Global test:" in text
    assert "degrees of freedom 1" in text
    assert "Discrepancy envelope discrepancy-v1" in text
    assert "held-out validation seeds" in text
    # exclusion through the sidebar is recorded and recomputes the estimate
    app.at.text_input(key="disp_reason").set_value("calibration doubt").run()
    app.click("Exclude")
    app.refresh(1.0)
    assert (
        "Electrical solves so far: 4" in app.text() or "Electrical solves so far: 3" in app.text()
    )
    assert any("EXCLUDED" in str(f) for f in app.frames())


def test_t09_scripted_manager_turn_shows_proposed_findings_in_the_console():
    app = AppDriver(mode="incident_demo", seed=11)
    app.click("Manager turn (scripted)")
    wait_for(app, "PF0001 PROPOSED_NOT_ACCEPTED")
    frames = " ".join(str(f) for f in app.frames())
    assert "manager-scripted" in frames and "H_VEG" in frames
    assert "no drone measurements" in frames
    # real readings arrive (DRONE_2 clearance on LINE_058), and the manager revises citing them
    app.pick("inspect_segment", "LINE_058")
    app.pick("draft_drone", "DRONE_2")
    app.pick("draft_kind", "clearance")
    app.click("Add stop here")
    app.pick("inspect_segment", "LINE_025")
    app.pick("draft_circuit", "LINE_025/C1")
    app.click("Land here")
    app.click("Submit sorties")
    wait_for(app, "Reading R0001 (clearance)")
    app.click("Manager turn (scripted)")
    wait_for(app, "PF0002 PROPOSED_NOT_ACCEPTED")
    frames = " ".join(str(f) for f in app.frames())
    assert "measurement" in frames and "R0001" in frames and "SUPPORTED" in frames
    assert "Field inspection of LINE_058" in frames


def test_t10_accept_a_proposed_finding_through_the_console():
    app = AppDriver(mode="incident_demo", seed=21)
    app.click("Manager turn (scripted)")
    wait_for(app, "PF0001 PROPOSED_NOT_ACCEPTED")
    assert any("PROPOSED_NOT_ACCEPTED" in f for f in app.frames())  # pending notice visible
    app.click("Accept")
    wait_for(app, "accepted (PF0001)")
    frames = " ".join(app.frames())
    assert "ACCEPTED" in frames and "H_VEG" in frames and "REV0001" in frames
    assert "No acceptance hold." in app.text()
    # a second accept of the same finding is refused: it is no longer pending
    assert not [b for b in app.at.selectbox if b.key == "decide_finding"]


def test_t11_accepting_hypotheses_creates_visible_needs_and_tasks():
    app = AppDriver(mode="incident_demo", seed=31)
    assert not any("N-H_VEG" in f for f in app.frames())  # nothing before acceptance
    app.click("Manager turn (scripted)")
    wait_for(app, "PF0001 PROPOSED_NOT_ACCEPTED")
    app.refresh(1.0)
    app.click("Accept")
    wait_for(app, "accepted (PF0001)")
    app.refresh(1.0)
    frames = " ".join(app.frames())
    assert "N-H_VEG-H_VEG-P1-LINE_058" in frames and "diagnostic" in frames
    assert "could_change" in frames and "minimum clearance below 0.5 m" in frames.replace(
        "shows: ", ""
    )
    assert "local:LINE_058@" in frames  # exact spatial identity of the local task


def test_t12_console_previews_scores_with_their_assumptions():
    app = AppDriver(mode="incident_demo", seed=41)
    app.click("Manager turn (scripted)")
    wait_for(app, "PF0001 PROPOSED_NOT_ACCEPTED")
    app.refresh(1.0)
    app.click("Accept")
    wait_for(app, "accepted (PF0001)")
    app.refresh(1.5)
    text = app.text()
    assert "synthetic units" in text.lower() and "not a calibrated causal entropy" in text.lower()
    frames = " ".join(app.frames())
    assert "points_by_urgency_5_to_1" in frames and "owner_urgency" in frames


def test_t13_plan_preview_is_labelled_and_commit_moves_drones():
    app = AppDriver(mode="incident_demo", seed=51)
    app.click("Manager turn (scripted)")
    wait_for(app, "PF0001 PROPOSED_NOT_ACCEPTED")
    app.refresh(1.0)
    app.click("Accept")
    wait_for(app, "accepted (PF0001)")
    app.refresh(1.0)
    app.toggle("allow_uncertain", True)  # a plan may need endpoints that are not yet confirmed
    app.refresh(1.0)
    app.click("Preview plan")
    text = app.text()
    assert "Snapshot r" in text and "Replay settings: ortools" in text and "seed 0" in text
    frames = " ".join(app.frames())
    assert "tiers_5_to_1" in frames and "tier-3" in frames and "round" in frames
    app.click("Commit best plan")
    wait_for(app, "operator" if False else "planner:")
    assert any(d in app.text() for d in ("flying", "measuring")) or any(
        "flying" in f or "measuring" in f for f in app.frames()
    )


def test_t14_accepted_findings_trigger_immediate_autonomous_route_passes():
    app = AppDriver(mode="incident_demo", seed=61, auto_replan=True)
    app.click("Manager turn (scripted)")
    wait_for(app, "PF0001 PROPOSED_NOT_ACCEPTED")
    app.refresh(1.0)
    app.toggle("allow_uncertain", True)  # permit unconfirmed endpoints for the exercise
    app.refresh(1.0)
    app.click("Accept")
    wait_for(app, "route: RP")
    app.refresh(1.5)
    frames = " ".join(app.frames())
    assert "accepted decision" in frames and "accepted-plan" in frames and "latency_ms" in frames
    assert "flying" in frames  # the whole fleet was re-planned from live positions
    # readings arrive on their own and trigger further passes, without the manager
    wait_for(app, "Reading R0001", seconds=60)
    app.refresh(1.5)
    assert "reading" in " ".join(app.frames())


def test_t15_autonomous_manager_runs_beside_flying_routes():
    app = AppDriver(mode="incident_demo", seed=71, auto_replan=True)
    app.click("Manager turn (scripted)")
    wait_for(app, "PF0001 PROPOSED_NOT_ACCEPTED")
    app.refresh(1.0)
    app.toggle("allow_uncertain", True)
    app.pick("auto_manager", "scripted")  # the coordinator starts: manager + planner roles
    app.click("Accept")
    wait_for(app, "route: RP")
    wait_for(app, "Reading R0001", seconds=60)
    end = __import__("time").time() + 90
    while "accepted as AUTO-PF" not in " ".join(app.frames()) and __import__("time").time() < end:
        app.refresh(1.0)
    frames = " ".join(app.frames())
    assert "pending_reasoning" in frames and "authorises_dispatch" in frames
    assert "accepted as AUTO-PF" in frames  # the coordinator's turn was accepted by decision
    assert "PREVIEW" not in frames.upper() or "False" in frames  # previews dispatch nothing
    assert any(d in frames for d in ("flying", "charging", "measuring"))


def test_t16_conclusion_is_accepted_and_the_console_reports_the_closed_case():
    app = AppDriver(mode="incident_demo", seed=81)
    app.click("Manager turn (scripted)")
    wait_for(app, "PF0001 PROPOSED_NOT_ACCEPTED")
    app.refresh(1.0)
    app.click("Accept")
    wait_for(app, "accepted (PF0001)")
    app.toggle("allow_uncertain", True)
    app.pick("inspect_segment", "LINE_058")
    app.pick("draft_drone", "DRONE_2")
    app.pick("draft_kind", "clearance")
    app.click("Add stop here")
    app.pick("inspect_segment", "LINE_025")
    app.pick("draft_circuit", "LINE_025/C1")
    app.click("Land here")
    app.click("Submit sorties")
    wait_for(app, "Reading R0001 (clearance)")
    app.click("Manager turn (scripted, may conclude)")
    wait_for(app, "PF0002 PROPOSED_NOT_ACCEPTED")
    app.refresh(1.0)
    app.click("Accept")
    wait_for(app, "accepted (PF0002)")
    app.refresh(1.0)
    text = app.text()
    assert "Incident investigation: CONCLUDED" in text
    assert "not a proven mechanism" in text and "Remaining uncertainty" in text
    frames = " ".join(app.frames())
    assert "RETIRED" in frames  # diagnostic needs are retired
    # reopening is an explicit operator action, and relevant evidence is flagged for review
    app.click("Reopen")
    app.refresh(1.0)
    assert "Incident investigation: OPEN" in app.text()


def test_t17_resilience_tab_reports_latency_fallbacks_and_actions():
    app = AppDriver(mode="incident_demo", seed=82, auto_replan=True)
    app.click("Manager turn (scripted)")
    wait_for(app, "PF0001 PROPOSED_NOT_ACCEPTED")
    app.refresh(1.0)
    assert "Latency: no route passes measured" in app.text()
    app.toggle("allow_uncertain", True)
    app.refresh(1.0)
    app.click("Accept")
    wait_for(app, "route: RP")
    wait_for(app, "Reading R0001", seconds=60)
    app.refresh(1.5)
    text = app.text()
    assert "Latency:" in text and "p95" in text and "budget" in text
    assert "BUDGET MISS" not in text
    assert "No operator action required." in text or "ACTION REQUIRED" in text
    assert "Mean stage time" in text


def test_t18_four_hour_headless_comparison_runs_to_the_deadline(tmp_path):
    import json
    import subprocess
    import sys

    outs = {}
    for policy in ("agent", "greedy"):
        done = subprocess.run(
            [
                sys.executable,
                "-m",
                "grid_ops.evaluation",
                "--policy",
                policy,
                "--seed",
                "7",
                "--out",
                str(tmp_path),
            ],
            capture_output=True,
            text=True,
            timeout=600,
        )
        assert done.returncode == 0, done.stderr[-400:]
        outs[policy] = json.loads((tmp_path / "incident_demo" / f"{policy}-seed7.json").read_text())
    for data in outs.values():
        assert data["final"]["sim_s"] >= 240 * 60  # ran to T+04:00
        assert data["batches"] and data["replay"]["solver"].startswith("ortools")
    assert outs["agent"]["operations"]["route_passes"] > 0
    assert outs["greedy"]["operations"]["route_passes"] == 0  # baselines have no reactive replanner
