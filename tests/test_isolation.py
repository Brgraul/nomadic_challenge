"""Ticket 01: public state never contains hidden physics, and only allowed modules import it."""

from __future__ import annotations

import ast
import dataclasses
from enum import Enum
from pathlib import Path

import pytest
from conftest import permissive

from grid_ops.evaluation import energized_segments
from grid_ops.grid import load_network
from grid_ops.sim import Dispatch, SetInService, Simulation
from grid_ops.world import Charging, Ready, Stranded

SRC = Path(__file__).resolve().parents[1] / "src" / "grid_ops"
HIDDEN_MODULES = {"grid_ops.truth", "grid_ops.evaluation", "pandapower"}
ALLOWED_IMPORTERS = {"sim.py", "evaluation.py", "app.py"}  # app.py only via evaluation
FORBIDDEN_KEYS = {"energized", "truth", "in_service", "_truth", "net", "ext_grid"}


@pytest.fixture(scope="module")
def network():
    return load_network()


def drain(sim: Simulation) -> None:
    """Apply queued commands synchronously, as the sim thread would."""
    while not sim._commands.empty():
        command, future = sim._commands.get_nowait()
        future.set_result(sim._apply(command))


def walk(obj, path="root"):
    """Yield (path, name_or_value) for every nested field name and leaf of a public payload."""
    if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        yield path, type(obj).__module__
        for f in dataclasses.fields(obj):
            yield f"{path}.{f.name}", f.name
            yield from walk(getattr(obj, f.name), f"{path}.{f.name}")
    elif isinstance(obj, dict):
        for k, v in obj.items():
            yield f"{path}[{k!r}]", k
            yield from walk(v, f"{path}[{k!r}]")
    elif isinstance(obj, list | tuple | set | frozenset):
        for i, v in enumerate(obj):
            yield from walk(v, f"{path}[{i}]")
    elif isinstance(obj, Enum):
        yield path, obj.value
    else:
        yield path, obj


def assert_no_hidden(payload):
    for path, item in walk(payload):
        if isinstance(item, str):
            assert item not in FORBIDDEN_KEYS, f"hidden field name at {path}"
            assert not item.startswith("grid_ops.truth"), f"truth type at {path}"
            assert "segments energized" not in item, path
            assert "not energized" not in item, path


def test_public_snapshot_has_no_hidden_fields(network):
    sim = permissive(Simulation(network))
    sim.step(1)
    assert_no_hidden(sim.snapshot)
    assert not hasattr(sim.snapshot, "energized")


def test_command_results_and_logs_do_not_leak(network):
    sim = permissive(Simulation(network))
    switch = sim.submit(SetInService("LINE_001", False, "scenario"))
    sim.submit(Dispatch("DRONE_1", "LINE_041", "operator"))
    drain(sim)
    assert_no_hidden(switch.result(timeout=1))
    sim.step(60)
    assert_no_hidden([e.text for e in sim.snapshot.log])


def test_public_view_is_independent_of_hidden_state(network):
    """Differential probe: hidden changes away from every drone alter nothing public."""
    a, b = permissive(Simulation(network)), permissive(Simulation(network))
    used = {"LINE_040", "LINE_025", "LINE_026", "LINE_002", "LINE_056", "LINE_024"}
    far = next(s for s in network.segments if s not in used)
    b._apply(SetInService(far, False, "scenario"))
    a.step(5)
    b.step(5)
    assert a.snapshot == b.snapshot


def test_hidden_state_reaches_public_only_as_observable_consequence(network):
    sim = permissive(Simulation(network))
    assert isinstance(sim.snapshot.drones["DRONE_1"].state, Ready)
    # Hidden switching the operator knows nothing about: a drone landing there cannot charge.
    sim._apply(SetInService("LINE_041", False, "scenario"))
    sim._apply(Dispatch("DRONE_1", "LINE_041", "operator"))
    sim.step(40)
    assert isinstance(sim.snapshot.drones["DRONE_1"].state, Stranded)
    assert "LINE_041" not in energized_segments(sim)
    assert_no_hidden(sim.snapshot)


def test_existing_dispatch_charge_cycle_still_works(network):
    sim = permissive(Simulation(network))
    sim._apply(Dispatch("DRONE_1", "LINE_041", "operator"))
    sim.step(31)
    assert isinstance(sim.snapshot.drones["DRONE_1"].state, Charging)
    sim.step(181)
    assert isinstance(sim.snapshot.drones["DRONE_1"].state, Ready)


def test_no_public_handle_to_physics(network):
    sim = permissive(Simulation(network))
    for name in vars(sim):
        if not name.startswith("_"):
            assert "truth" not in name
            assert "power" not in name


def _imports(path: Path) -> set[str]:
    found = set()
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.Import):
            found |= {a.name for a in node.names}
        elif isinstance(node, ast.ImportFrom) and node.module:
            found.add(node.module)
            found |= {f"{node.module}.{a.name}" for a in node.names}
    return found


def test_import_boundary():
    for path in SRC.glob("*.py"):
        if path.name in ALLOWED_IMPORTERS:
            continue
        for module in _imports(path):
            hidden = any(module == h or module.startswith(h + ".") for h in HIDDEN_MODULES)
            assert not hidden, f"{path.name} imports {module}"
    assert not any(m.startswith("grid_ops.truth") for m in _imports(SRC / "app.py"))
    assert not any(m.startswith("grid_ops.evaluation") for m in _imports(SRC / "sim.py"))
