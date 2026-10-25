"""Ticket 01: public state never contains hidden physics, and only allowed modules import it.

The three-method runtime replaced the hidden switching differential with a paired-episode one:
two simulations that differ only in their private episode construction must publish equal public
views, and no private seed or fault label may reach the snapshot.
"""

from __future__ import annotations

import ast
import dataclasses
from enum import Enum
from pathlib import Path

import pytest
from conftest import permissive

import grid_ops.sim as sim_module
from grid_ops.sim import Dispatch, SetInService, Simulation
from grid_ops.world import CommandRejected, Perched

SRC = Path(__file__).resolve().parents[1] / "src" / "grid_ops"
HIDDEN_MODULES = {"grid_ops.truth", "grid_ops.evaluation", "pandapower"}
ALLOWED_IMPORTERS = {"sim.py", "evaluation.py", "app.py"}  # app.py only via evaluation
FORBIDDEN_KEYS = {"energized", "truth", "in_service", "_truth", "net", "ext_grid"}


@pytest.fixture(scope="module")
def network():
    from grid_ops.grid import load_network

    return load_network()


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


def test_paired_episode_public_views_are_equal_before_any_measurement(network, monkeypatch):
    """P1 isolation, restated for episodes: only the private episode differs, so nothing public
    may differ either — before any measurement is taken."""
    a = permissive(Simulation(network))
    seeds = iter(((111, 222), (333, 444)))
    monkeypatch.setattr(sim_module, "_private_seeds", lambda policy_seed: next(seeds))
    b = permissive(Simulation(network))
    a.step(5)
    b.step(5)
    assert a.snapshot == b.snapshot
    assert_no_hidden(a.snapshot)
    assert_no_hidden(b.snapshot)


def test_public_json_contains_no_private_episode_seeds(network):
    sim = permissive(Simulation(network))
    sim.step(1)
    blob = repr(sim.snapshot)
    for secret in ("episode_seed", "noise_seed", "FaultEvent", "ProtectionAction"):
        assert secret not in blob


def test_set_in_service_is_not_a_runtime_command(network):
    sim = permissive(Simulation(network))
    with pytest.raises(CommandRejected, match="not part of the three-method runtime"):
        sim._apply(SetInService("LINE_041", False, "evaluator"))


def test_dispatch_lands_perched_without_recharging(network):
    """A landing is never an energy reset: the drone perches with strictly less battery."""
    sim = permissive(Simulation(network))
    before = sim.snapshot.drones["DRONE_1"].battery_units
    sim._apply(Dispatch("DRONE_1", "LINE_041", "operator"))
    sim.step(90)
    drone = sim.snapshot.drones["DRONE_1"]
    assert isinstance(drone.state, Perched)
    assert drone.state.perch.circuit_id == "LINE_041/C1"
    assert drone.battery_units < before


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


def test_three_method_public_context_has_no_private_episode_handle():
    from grid_ops.sensing import PublicEpisodeView

    context = PublicEpisodeView("opaque-run-id", "analytic", "1", 0)
    fields = dataclasses.asdict(context)
    assert set(fields) == {
        "run_id",
        "fixture_id",
        "fixture_version",
        "policy_seed",
        "capabilities",
        "assumptions",
    }
    assert not any("seed" in name for name in fields if name != "policy_seed")