"""Normalized persistent energy. One unit is 1/600 of an airborne-equivalent second."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum


class OperationKind(StrEnum):
    TRANSIT = "transit"
    SCAN = "scan"
    LAND = "land"
    SAMPLE = "ground_sample"
    CHARGE = "charge"
    TAKEOFF = "takeoff"
    WAIT_AIRBORNE = "wait_airborne"
    WAIT_PERCHED = "wait_perched"


@dataclass(frozen=True)
class EnergyModel:
    revision: str = "normalized-energy-v1"
    units_per_airborne_s: int = 600
    capacity_units: int = 1_080_000
    ground_units_per_s: int = 6
    takeoff_units: int = 3_000
    landing_units: int = 3_000
    net_charge_units_per_s: int = 100

    def cost(self, kind: OperationKind, duration_s: int) -> int:
        if duration_s < 0:
            raise ValueError("operation duration must be nonnegative")
        match kind:
            case OperationKind.TRANSIT | OperationKind.SCAN | OperationKind.WAIT_AIRBORNE:
                return self.units_per_airborne_s * duration_s
            case OperationKind.TAKEOFF:
                return self.units_per_airborne_s * duration_s + self.takeoff_units
            case OperationKind.LAND:
                return self.units_per_airborne_s * duration_s + self.landing_units
            case OperationKind.SAMPLE | OperationKind.WAIT_PERCHED:
                return self.ground_units_per_s * duration_s
            case OperationKind.CHARGE:
                return 0
        raise ValueError(f"unknown operation {kind}")


DEFAULT_ENERGY_MODEL = EnergyModel()


@dataclass(frozen=True)
class BatteryState:
    remaining_units: int = DEFAULT_ENERGY_MODEL.capacity_units
    capacity_units: int = DEFAULT_ENERGY_MODEL.capacity_units
    energy_model_revision: str = DEFAULT_ENERGY_MODEL.revision
    observed_s: int = 0

    def __post_init__(self) -> None:
        if not 0 <= self.remaining_units <= self.capacity_units:
            raise ValueError("battery outside capacity")


@dataclass(frozen=True)
class EnergyEntry:
    start_s: int
    end_s: int
    operation: OperationKind
    before_units: int
    cost_units: int
    observed_charge_units: int
    after_units: int


def integrate(
    battery: BatteryState,
    kind: OperationKind,
    duration_s: int,
    *,
    charging_observed: bool = False,
    model: EnergyModel = DEFAULT_ENERGY_MODEL,
) -> tuple[BatteryState, EnergyEntry]:
    if battery.energy_model_revision != model.revision:
        raise ValueError("energy model revision mismatch")
    if battery.capacity_units != model.capacity_units:
        raise ValueError("battery capacity differs from energy model")
    if kind in (OperationKind.LAND, OperationKind.TAKEOFF) and duration_s <= 0:
        raise ValueError("landing and takeoff need positive duration")
    cost = model.cost(kind, duration_s)
    if cost > battery.remaining_units:
        raise ValueError("insufficient energy")
    gain = (
        model.net_charge_units_per_s * duration_s
        if (kind is OperationKind.CHARGE and charging_observed)
        else 0
    )
    if kind is OperationKind.CHARGE and not charging_observed:
        cost = model.ground_units_per_s * duration_s
        if cost > battery.remaining_units:
            raise ValueError("insufficient ground energy")
    after = min(battery.capacity_units, battery.remaining_units - cost + gain)
    gain = after - battery.remaining_units + cost
    end_s = battery.observed_s + duration_s
    updated = BatteryState(after, battery.capacity_units, model.revision, end_s)
    return updated, EnergyEntry(
        battery.observed_s, end_s, kind, battery.remaining_units, cost, gain, after
    )
