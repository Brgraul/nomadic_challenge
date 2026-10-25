"""Ticket-level e2e through the real app script (see drive.py).

Flow under test: the app boots, a scripted startup turn is accepted, its measurement requests
are visible, a route pass accepts a plan, the fleet executes until a reading publishes, the
reading and request panels reflect it, and battery drains without ever resetting.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from drive import AppDriver

CAPACITY_UNITS = 1_080_000  # a full battery; a landing must never restore it
METHODS = ("lidar", "contact_current", "tdr")


def wait_for(app, needle, seconds=60):
    end = time.time() + seconds
    while needle not in app.text() and time.time() < end:
        app.refresh(1.0)
    assert needle in app.text(), app.text()[-1500:]


def wait_row(app, pred, seconds=60, what="row"):
    end = time.time() + seconds
    while time.time() < end:
        for row in app.rows():
            if pred(row):
                return row
        app.refresh(1.0)
    raise AssertionError(f"no {what} appeared; frames:\n" + "\n".join(app.frames())[-2000:])


def battery_samples(app):
    return {r["drone"]: r["battery_units"] for r in app.rows() if "battery_units" in r}


def assert_battery_never_resets(app, dropped):
    """Once a drone has burned energy its battery stays below capacity for the rest of the run."""
    for drone, units in battery_samples(app).items():
        if units < CAPACITY_UNITS:
            dropped.add(drone)
        assert drone not in dropped or units < CAPACITY_UNITS, f"{drone} battery was reset"


def test_app_boots_with_three_method_panels():
    app = AppDriver()
    assert "Scenario incident_demo" in app.text()
    frames = " ".join(app.frames())
    assert "battery_pct" in frames and "phase" in frames
    # no legacy panels; drone phases are exactly the three-method itinerary states
    phases = {r["phase"] for r in app.rows() if "phase" in r}
    assert phases <= {"perched", "executing", "stranded"}
    for banned in ("PRIOR-ONLY", "degrees of freedom", "Discrepancy envelope"):
        assert banned not in frames, banned


def test_startup_turn_requests_route_pass_reading_and_battery_drain():
    app = AppDriver(seed=107, auto_replan=True)
    dropped: set[str] = set()
    assert not [r for r in app.rows() if "EXPLICITLY UNRESOLVED" in str(r.get("unresolved", ""))]
    app.pause(True)  # freeze the clock: the request panel must show the pre-execution state

    # the scripted startup turn proposes findings and measurement requests
    app.click("Manager turn (scripted)")
    wait_for(app, "PROPOSED_NOT_ACCEPTED")

    # accepting the proposal is one explained decision; it compiles the requests and the fleet
    # replans immediately
    app.refresh(1.0)
    app.click("Accept")
    wait_for(app, "accepted (PF0001)")
    request = wait_row(
        app, lambda r: str(r.get("request", "")).startswith("REQ-"), what="accepted request row"
    )
    assert request["method"] in METHODS
    assert request["unresolved"] == "EXPLICITLY UNRESOLVED"
    assert request["tasks"], "an accepted request must have compiled tasks"

    wait_for(app, "accepted-plan", seconds=30)
    executing = wait_row(app, lambda r: r.get("phase") == "executing", what="executing drone")
    assert executing["itinerary_left"] > 0 and executing["operation"]

    # step the sim until a reading publishes and the sortie lands; battery only ever drains
    app.pause(False)
    end = time.time() + 120
    while time.time() < end:
        app.refresh(1.5)
        assert_battery_never_resets(app, dropped)
        if "Reading R0001" in app.text() and not any(
            r.get("phase") == "executing" for r in app.rows() if "phase" in r
        ):
            break
    assert "Reading R0001" in app.text(), app.text()[-1500:]
    assert dropped, "an executed itinerary must have committed energy"

    # the reading and the request panels reflect the executed measurement
    wait_row(app, lambda r: str(r.get("reading", "")).startswith("R0"), what="reading row")
    done = wait_row(
        app, lambda r: r.get("status") == "COMPLETED" and r.get("reading"), what="completed task"
    )
    assert done["method"] in METHODS
    served = wait_row(
        app,
        lambda r: r.get("request") == done["request"]
        and "EXPLICITLY UNRESOLVED" not in str(r.get("unresolved", "")),
        what="served request",
    )
    assert served["method"] == done["method"]
    # every rendered row is three-method only: no electrical or optical-imaging claims
    for row in app.rows():
        text = " ".join(str(v) for v in row.values()).lower()
        for banned in ("voltage", " mw", "mw ", "rgb", "infrared", "temperature", " p_mw"):
            assert banned not in text, (banned, row)