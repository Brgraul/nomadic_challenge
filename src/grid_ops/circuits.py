"""Public synthetic circuit registry.

Each ordered entry of a segment's `voltages_kv` list is one synthetic circuit; extra circuits
(source count above the entry count) are allocated round-robin in listed order. This is a
reproducible scenario assignment, not recovered asset connectivity. Hidden realized values
(impedance variation, flows) never appear here; only declared catalogue parameters do.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass

from grid_ops.grid import DATA_DIR, Network

BASE_MVA = 100.0

# Declared catalogue parameters (pandapower std types "243-AL1/39-ST1A 110.0" and
# "490-AL1/64-ST1A 380.0"), copied here so the public side never imports hidden physics.
CATALOGUE = {
    110.0: {"type": "243-AL1/39-ST1A 110.0", "x_ohm_per_km": 0.39, "max_i_a": 645.0},
    380.0: {"type": "490-AL1/64-ST1A 380.0", "x_ohm_per_km": 0.253, "max_i_a": 960.0},
}


class ContradictoryCircuitCount(ValueError):
    """The source circuit count is smaller than the number of listed voltage entries."""


@dataclass(frozen=True)
class Circuit:
    id: str
    segment_id: str
    index: int
    nominal_kv: float
    from_node: str
    to_node: str
    line_type: str
    length_km: float
    x_ohm: float
    max_current_a: float

    @property
    def x_pu(self) -> float:
        """Reactance in per unit on the 100 MVA base and this circuit's voltage base."""
        return self.x_ohm / (self.nominal_kv**2 / BASE_MVA)

    @property
    def b_mw_per_rad(self) -> float:
        return BASE_MVA / self.x_pu

    @property
    def rating_mva(self) -> float:
        return 3**0.5 * self.nominal_kv * self.max_current_a / 1000


def allocate_voltages(
    voltages_kv: tuple[float, ...], count: int | None, segment_id: str
) -> tuple[float, ...]:
    """Circuit voltages for one segment, preserving repeated entries and listed order."""
    if count is not None and count < len(voltages_kv):
        raise ContradictoryCircuitCount(
            f"{segment_id}: {count} circuits but {len(voltages_kv)} listed voltages"
        )
    result = list(voltages_kv)
    for k in range(len(voltages_kv), count or 0):
        result.append(voltages_kv[(k - len(voltages_kv)) % len(voltages_kv)])
    return tuple(result)


class CircuitRegistry:
    def __init__(self, network: Network, voltage_lists: dict[str, tuple[float, ...]]) -> None:
        self.circuits: dict[str, Circuit] = {}
        self.by_segment: dict[str, tuple[Circuit, ...]] = {}
        for seg in network.segments.values():
            voltages = allocate_voltages(
                voltage_lists.get(seg.id, (seg.voltage_kv,)), seg.circuits, seg.id
            )
            made = []
            for k, kv in enumerate(voltages, start=1):
                cat = CATALOGUE[kv]
                circuit = Circuit(
                    id=f"{seg.id}/C{k}",
                    segment_id=seg.id,
                    index=k,
                    nominal_kv=kv,
                    from_node=seg.from_node,
                    to_node=seg.to_node,
                    line_type=cat["type"],
                    length_km=seg.length_km,
                    x_ohm=cat["x_ohm_per_km"] * seg.length_km,
                    max_current_a=cat["max_i_a"],
                )
                self.circuits[circuit.id] = circuit
                made.append(circuit)
            self.by_segment[seg.id] = tuple(made)


def load_voltage_lists() -> dict[str, tuple[float, ...]]:
    with (DATA_DIR / "grid_segments.csv").open() as f:
        return {
            row["segment_id"]: tuple(float(v) for v in row["voltages_kv"].split(";"))
            for row in csv.DictReader(f)
        }


def build_registry(network: Network) -> CircuitRegistry:
    return CircuitRegistry(network, load_voltage_lists())
