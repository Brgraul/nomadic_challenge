from dataclasses import replace

import pytest

from grid_ops.energy import BatteryState, OperationKind, integrate


def test_no_charge_hops_preserve_energy_and_landing_costs():
    battery = BatteryState()
    entries = []
    for kind, duration in [
        (OperationKind.TAKEOFF, 3),
        (OperationKind.TRANSIT, 60),
        (OperationKind.LAND, 3),
        (OperationKind.SAMPLE, 10),
        (OperationKind.TAKEOFF, 3),
        (OperationKind.TRANSIT, 60),
        (OperationKind.LAND, 3),
    ]:
        before = battery.remaining_units
        battery, entry = integrate(battery, kind, duration)
        entries.append(entry)
        assert battery.remaining_units < before
    assert entries[4].before_units == entries[3].after_units
    assert all(e.observed_charge_units == 0 for e in entries)


def test_partial_charging_is_net_gain_and_capacity_bounded():
    battery = replace(BatteryState(), remaining_units=100_000)
    charged, entry = integrate(battery, OperationKind.CHARGE, 600, charging_observed=True)
    assert charged.remaining_units == 160_000
    assert entry.observed_charge_units == 60_000
    full, _ = integrate(charged, OperationKind.CHARGE, 20_000, charging_observed=True)
    assert full.remaining_units == full.capacity_units


def test_tick_subdivision_cannot_mint_or_destroy_energy():
    start = replace(BatteryState(), remaining_units=100_000)
    direct, _ = integrate(start, OperationKind.CHARGE, 100, charging_observed=True)
    split = start
    for _ in range(100):
        split, _ = integrate(split, OperationKind.CHARGE, 1, charging_observed=True)
    assert direct == split


def test_failed_handshake_never_gains_energy():
    start = BatteryState()
    end, entry = integrate(start, OperationKind.CHARGE, 100)
    assert end.remaining_units < start.remaining_units
    assert entry.observed_charge_units == 0


def test_energy_exhaustion_and_zero_duration_hops_rejected():
    empty = replace(BatteryState(), remaining_units=0)
    with pytest.raises(ValueError, match="insufficient"):
        integrate(empty, OperationKind.TRANSIT, 1)
    with pytest.raises(ValueError, match="positive"):
        integrate(BatteryState(), OperationKind.LAND, 0)
