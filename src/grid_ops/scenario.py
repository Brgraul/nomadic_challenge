"""Public scenario definitions: modes, briefs, the declared sensor model and its covariance.

Everything here is public. The seed is public (it makes runs reproducible) but nothing derived
from it, such as hidden fault coordinates or noise realisations, is stored here.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

import numpy as np

from grid_ops.circuits import Circuit
from grid_ops.grid import Network

# Exercise assumptions, not claims about real charging hardware.
I_CHARGE_MIN_A = 5.0
VOLTAGE_LIVE_FRACTION = 0.10  # 95% lower bound above 10% of nominal supports LIVE
VOLTAGE_DEAD_FRACTION = 0.02  # 95% upper bound below 2% of nominal supports DE_ENERGIZED
SENSOR_REVISION = "sensor-v1"

# Local sensor Gaussian errors and thresholds (exercise policies, not operational standards).
CLEARANCE_SIGMA_M = 0.05
TEMP_SIGMA_C = 1.0
MOTION_SIGMA_M = 0.02
CLEARANCE_THRESHOLD_M = 0.5
THERMAL_EXCESS_THRESHOLD_C = 10.0
THERMAL_EXCITATION_A = 20.0
MOTION_THRESHOLD_M = 0.20
DETECTION_REQUIREMENT = 0.95

# Public observation model for local sensors: environmental baseline and self-heating are part of
# the declared model, so a reading's excess over baseline can be computed without hidden state.
ENV_TEMP_C = 12.0
HEATING_C_PER_A2 = 0.002
# Defect levels the detection requirement is tested against (worst credible defects).
DEFECT_CLEARANCE_M = 0.4
DEFECT_EXCESS_C = 15.0
DEFECT_MOTION_M = 0.30

MAX_OPERATING_POINTS = 100


class ScenarioMode(StrEnum):
    INCIDENT_DEMO = "incident_demo"
    HEALTHY_BENCHMARK = "healthy_benchmark"


class ScenarioInfeasible(Exception):
    """No seeded operating point met the fixture constraints within the construction budget."""

    def __init__(self, message: str, diagnostics: dict) -> None:
        super().__init__(message)
        self.diagnostics = diagnostics


@dataclass(frozen=True)
class ScenarioConfig:
    mode: ScenarioMode = ScenarioMode.INCIDENT_DEMO
    seed: int = 7
    # Fixture switches. A shared calibration component with correlation `shared_rho` across the
    # circuits of one segment (and across repeats) tests that repeating cannot remove it.
    shared_rho: float = 0.0
    contradictory_switching: bool = False


@dataclass(frozen=True)
class Covariance:
    """Public calibration covariance over named quantities of one observation."""

    names: tuple[str, ...]
    matrix: tuple[tuple[float, ...], ...]

    def as_array(self) -> np.ndarray:
        return np.array(self.matrix)

    def std(self, name: str) -> float:
        i = self.names.index(name)
        return float(self.matrix[i][i] ** 0.5)


@dataclass(frozen=True)
class CircuitSensorConfig:
    """Fixed calibration references; the standard deviations are not a share of the hidden flow."""

    i_ref_a: float = 100.0
    p_ref_mw: float = 100.0
    q_ref_mvar: float = 100.0

    def sigma(self, quantity: str, nominal_kv: float) -> float:
        match quantity:
            case "current_a":
                return max(0.01 * self.i_ref_a, 1.0)
            case "p_mw":
                return max(0.02 * self.p_ref_mw, 0.5)
            case "q_mvar":
                return max(0.02 * self.q_ref_mvar, 0.5)
            case "voltage_kv":
                return max(0.01 * nominal_kv, 0.1)
        raise KeyError(quantity)


ELECTRICAL_QUANTITIES = {
    "voltage_presence": ("voltage_kv",),
    "current": ("current_a", "p_mw", "q_mvar", "voltage_kv"),
}


@dataclass(frozen=True)
class SensorModel:
    """The declared, versioned sensor model. Changing references or correlations is a revision."""

    revision: str = SENSOR_REVISION
    shared_rho: float = 0.0
    circuit_config: CircuitSensorConfig = field(default_factory=CircuitSensorConfig)

    def quantities(self, kind: str) -> tuple[str, ...]:
        return ELECTRICAL_QUANTITIES[kind]

    def covariance(self, kind: str, circuits: tuple[Circuit, ...]) -> Covariance:
        """Covariance of one electrical observation over its circuits and quantities.

        Circuits and quantities are independent except for the shared calibration component:
        circuits of one segment share a fraction `shared_rho` of each quantity's variance.
        """
        names, sigmas, groups = [], [], []
        for c in circuits:
            for q in self.quantities(kind):
                names.append(f"{c.id}.{q}")
                sigmas.append(self.circuit_config.sigma(q, c.nominal_kv))
                groups.append((c.segment_id, q))
        n = len(names)
        m = np.zeros((n, n))
        for i in range(n):
            m[i, i] = sigmas[i] ** 2
            for j in range(i):
                if groups[i] == groups[j]:
                    m[i, j] = m[j, i] = self.shared_rho * sigmas[i] * sigmas[j]
        return Covariance(tuple(names), tuple(tuple(float(x) for x in row) for row in m))


def public_brief(network: Network, config: ScenarioConfig) -> tuple[str, ...]:
    """The operator brief. It matches the mode's objective and hidden physics."""
    lines = [f"Scenario {config.mode} (seed {config.seed}). All times are relative to T+00:00."]
    if config.mode is ScenarioMode.INCIDENT_DEMO:
        lines.append(
            "Objective: find an evidence-supported cause of the outage and recommend an action. "
            "Full flow reconstruction is not required."
        )
        for r in network.reports.values():
            lines.append(f"{r.id} ({r.timestamp}, {r.confidence}): {r.description}")
        lines.append(
            "LINE_024 was precautionarily switched to de-energized (SYN_REPORT_03). A protection "
            "event was recorded separately; its relationship to that switching is unknown and "
            "it is not a confirmed trip on LINE_024."
        )
    else:
        lines.append(
            "Counterfactual benchmark with no incident: every circuit is initially in service. "
            "The storm reports are unused synthetic background and SYN_REPORT_03's switching "
            "assertion is withdrawn."
        )
        lines.append(
            "Objective: reconstruct circuit flows with quantified uncertainty from drone "
            "measurements (flow reconstruction is explicitly selected for this mode)."
        )
    lines.append(
        "Drones fly straight legs at 60 km/h, 30 minutes of airtime including measurement dwell, "
        "3 h charge on a live circuit carrying at least 5 A, scenario deadline T+04:00."
    )
    return tuple(lines)


@dataclass(frozen=True)
class PublicFixture:
    """Reproducible public inputs, independent of privileged episode and sensor noise seeds."""

    fixture_id: str = "bavaria-public"
    version: str = "1"
    policy_seed: int = 0
