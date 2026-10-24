"""Residual diagnostics for the reduced DC model: what the real data can and cannot test.

Only real aggregate measurements and exact zero-injection constraints count as validation
evidence; pseudo-measurements never do. Normalised residuals use the residual covariance, not the
input sigma. A measurement is TESTABLE only when it has redundancy and its residual distribution
is trustworthy; a critical measurement (residual variance ~ 0) is NOT_TESTABLE; and where model
discrepancy dominates measurement noise the statistics are MODEL_LIMITED. Inconsistency here is
evidence for the manager, never an automatic fault verdict or an automatic exclusion.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import TYPE_CHECKING

import numpy as np
import scipy.linalg as sla
from scipy import stats

from grid_ops.envelope import DiscrepancyEnvelope
from grid_ops.estimation import (
    RANK_TOL,
    ElectricalModel,
    _measurement_rows,
    _p_covariance,
)

if TYPE_CHECKING:
    from grid_ops.world import Reading

NORMALISED_THRESHOLD = 3.0
GLOBAL_ALPHA = 0.001
CRITICAL_VARIANCE_RATIO = 1e-9
DOMINANCE = 2.0  # discrepancy bound above this many sigma makes noise-based tests model-limited


class Testability(StrEnum):
    __test__ = False
    TESTABLE = "TESTABLE"
    NOT_TESTABLE = "NOT_TESTABLE"
    MODEL_LIMITED = "MODEL_LIMITED"


class GlobalStatus(StrEnum):
    NOT_REFUTED = "NOT_REFUTED"  # no inconsistency detected; not proof of correctness
    INCONSISTENT = "INCONSISTENT"
    NOT_TESTABLE = "NOT_TESTABLE"
    MODEL_LIMITED = "MODEL_LIMITED"


@dataclass(frozen=True)
class MeasurementCheck:
    reading_id: str
    segment_id: str
    testability: Testability
    normalised_residual: float | None
    residual_variance_ratio: float  # (residual variance) / (input variance); ~0 means critical
    candidate: bool  # |normalised residual| above threshold on a TESTABLE row
    note: str


@dataclass(frozen=True)
class CandidateExplanation:
    label: str  # wrong_topology | sensor_bias | parameter_mismatch | changed_epoch | prior_conflict
    for_readings: tuple[str, ...]
    support: str


@dataclass(frozen=True)
class ConsistencyReport:
    degrees_of_freedom: int
    component_dof: dict[str, int]
    global_chi2: float | None
    global_p_value: float | None
    global_status: GlobalStatus
    checks: tuple[MeasurementCheck, ...]
    critical_sets: tuple[tuple[str, ...], ...]
    suspect_reading_ids: tuple[str, ...]
    suspect_meaning: str
    explanations: tuple[CandidateExplanation, ...]
    prior_conflicts: tuple[str, ...]
    notes: tuple[str, ...]


def _whitened_system(model: ElectricalModel, rows, readings_by_id, shared_rho):
    if model.Z.size:
        _, sz, vt = np.linalg.svd(model.Z, full_matrices=True)
        rz = int((sz > RANK_TOL * max(sz[0], 1.0)).sum()) if sz.size else 0
        nullspace = vt[rz:].T
    else:
        nullspace = np.eye(len(model.columns))
    h = np.array([model.flow_rows[r.segment_id] for r in rows])
    cov = _p_covariance(rows, readings_by_id, shared_rho)
    chol = np.linalg.cholesky(cov)
    a = sla.solve_triangular(chol, h @ nullspace, lower=True)
    b = sla.solve_triangular(chol, np.array([r.z for r in rows]), lower=True)
    return a, b, chol, cov, nullspace


def check_consistency(
    model: ElectricalModel,
    readings: list[Reading] | tuple[Reading, ...],
    *,
    shared_rho: float = 0.0,
    envelope: DiscrepancyEnvelope | None = None,
    known_switching_clock: dict[str, int] | None = None,
    active_seed: int | None = None,
) -> ConsistencyReport:
    """Residual tests on the real measurement rows (priors excluded)."""
    readings = [r for r in readings if r.operating_epoch == model.epoch]
    by_id = {r.id: r for r in readings}
    rows, problems = _measurement_rows(model, readings)
    notes = list(problems)
    if not rows:
        return ConsistencyReport(
            0,
            {},
            None,
            None,
            GlobalStatus.NOT_TESTABLE,
            (),
            (),
            (),
            "NOT TESTED: there are no real measurements (prior-only).",
            (),
            (),
            tuple(notes),
        )
    try:
        a, b, chol, cov, _ = _whitened_system(model, rows, by_id, shared_rho)
    except np.linalg.LinAlgError:
        return ConsistencyReport(
            0,
            {},
            None,
            None,
            GlobalStatus.NOT_TESTABLE,
            (),
            (),
            (),
            "NOT TESTED: measurement covariance is not positive definite.",
            (),
            (),
            (*notes, "covariance invalid"),
        )
    m = len(rows)
    u, s, _ = np.linalg.svd(a, full_matrices=True)
    rank = int((s > RANK_TOL * s[0]).sum()) if s.size else 0
    ur = u[:, :rank]
    omega = np.eye(m) - ur @ ur.T  # whitened residual projector
    res_w = omega @ b
    res = chol @ res_w  # residual in original MW units
    res_cov = chol @ omega @ chol.T
    dof = m - rank

    # degrees of freedom per model component: redundancy elsewhere never counts here
    comp_dof: dict[str, int] = {}
    by_comp: dict[int, list[int]] = {}
    for i, r in enumerate(rows):
        seg_node = model._ends(r.segment_id)[0]
        by_comp.setdefault(model.comp_of[seg_node], []).append(i)
    for ci, idx in by_comp.items():
        sub = a[idx]
        sub_rank = int(np.linalg.matrix_rank(sub, tol=RANK_TOL * max(np.abs(sub).max(), 1.0)))
        comp_dof[model.component_names[ci]] = len(idx) - sub_rank

    # model-limited where the discrepancy envelope dominates noise or is unsupported
    known_out = frozenset(model.out_of_service)
    checks: list[MeasurementCheck] = []
    for i, r in enumerate(rows):
        var_in = float(cov[i, i])
        ratio = float(res_cov[i, i] / var_in) if var_in > 0 else 0.0
        sigma = var_in**0.5
        limited, why = False, ""
        if envelope is not None:
            ok, reason = envelope.supports(r.segment_id, known_out, model.epoch, active_seed)
            bound = (
                envelope.bounds[r.segment_id].bound_mw if r.segment_id in envelope.bounds else None
            )
            if not ok:
                limited, why = True, f"discrepancy unsupported: {reason}"
            elif bound is not None and bound > DOMINANCE * sigma:
                limited, why = True, f"discrepancy bound {bound:.1f} MW exceeds {DOMINANCE:g} sigma"
        norm = (
            res[i] / np.sqrt(res_cov[i, i])
            if res_cov[i, i] > CRITICAL_VARIANCE_RATIO * var_in
            else None
        )
        if ratio < CRITICAL_VARIANCE_RATIO:
            testability, note = Testability.NOT_TESTABLE, "critical: residual variance is ~0"
            norm = None
        elif limited:
            testability, note = Testability.MODEL_LIMITED, why
        else:
            testability, note = Testability.TESTABLE, "redundant, model assumptions supported"
        candidate = (
            testability is Testability.TESTABLE
            and norm is not None
            and abs(norm) > NORMALISED_THRESHOLD
        )
        checks.append(
            MeasurementCheck(
                r.reading_id,
                r.segment_id,
                testability,
                None if norm is None else float(norm),
                ratio,
                bool(candidate),
                note,
            )
        )

    # critical sets: readings whose residuals are perfectly correlated share an undetectable error
    sets: list[tuple[str, ...]] = []
    live = [i for i, c in enumerate(checks) if c.testability is not Testability.NOT_TESTABLE]
    seen: set[int] = set()
    for i in live:
        if i in seen:
            continue
        group = [i]
        for j in live:
            if j <= i or j in seen:
                continue
            den = (res_cov[i, i] * res_cov[j, j]) ** 0.5
            if den > 0 and abs(res_cov[i, j]) / den > 1 - 1e-9:
                group.append(j)
        if len(group) > 1:
            seen.update(group)
            sets.append(tuple(rows[k].reading_id for k in group))

    testable = [c for c in checks if c.testability is Testability.TESTABLE]
    limited_rows = [c for c in checks if c.testability is Testability.MODEL_LIMITED]
    chi2 = float(res_w @ res_w) if dof > 0 else None
    p = float(stats.chi2.sf(chi2, dof)) if (chi2 is not None and dof > 0) else None
    if dof == 0 or not (testable or limited_rows):
        status = GlobalStatus.NOT_TESTABLE
    elif limited_rows and not testable:
        status = GlobalStatus.MODEL_LIMITED
    elif p is not None and p < GLOBAL_ALPHA:
        status = GlobalStatus.INCONSISTENT
    else:
        status = GlobalStatus.NOT_REFUTED
    suspects = tuple(c.reading_id for c in checks if c.candidate)
    if status is GlobalStatus.NOT_TESTABLE:
        meaning = (
            "An empty suspect list here means NOT TESTED: the measurements are minimally "
            "observable (or redundancy lies in other components), so bad data can fit exactly."
        )
    elif status is GlobalStatus.MODEL_LIMITED:
        meaning = "Model discrepancy dominates; residuals are retained but no data is flagged."
    else:
        meaning = "No testable residual exceeds the threshold; this is not proof of consistency."

    explanations = _explanations(
        model, rows, checks, by_id, known_switching_clock or {}, shared_rho
    )
    return ConsistencyReport(
        dof,
        comp_dof,
        chi2,
        p,
        status,
        tuple(checks),
        tuple(sets),
        suspects,
        meaning,
        tuple(explanations),
        (),
        tuple(notes),
    )


def _explanations(
    model, rows, checks, by_id, switching_clock, shared_rho
) -> list[CandidateExplanation]:
    """Plausible causes of a flagged or untestable residual, kept apart: none is chosen."""
    out: list[CandidateExplanation] = []
    flagged = [c for c in checks if c.candidate or c.testability is Testability.MODEL_LIMITED]
    if not flagged:
        return out
    ids = tuple(c.reading_id for c in flagged)
    late = tuple(
        c.reading_id
        for c in flagged
        if any(by_id[c.reading_id].clock_s < t for t in switching_clock.values())
    )
    out.append(
        CandidateExplanation(
            "wrong_topology",
            ids,
            "a different out-of-service set would change the zero-injection constraints and "
            "references; de-energization alone does not locate an open switch",
        )
    )
    out.append(
        CandidateExplanation(
            "sensor_bias",
            ids,
            "shared calibration bias"
            + (" (declared correlation)" if shared_rho > 0 else "")
            + " moves repeated readings together and cannot be averaged away",
        )
    )
    out.append(
        CandidateExplanation(
            "parameter_mismatch",
            ids,
            "catalogue reactances, transformer reduction and AC/DC differences are outside "
            "measurement noise",
        )
    )
    if late:
        out.append(
            CandidateExplanation(
                "changed_epoch", late, "known switching was reported after these readings"
            )
        )
    return out


# -- direct AC operating checks ------------------------------------------------------

SUPPORTED_AC_CHECKS = {"terminal_current_within_rating"}
UNSUPPORTED_AC_CHECKS = {
    "far_end_voltage_limit": "only from-terminal quantities are represented",
    "far_end_current_within_rating": "far-end current is not measured",
    "bus_voltage_band": "bus voltage is not measured",
    "transformer_loading": "transformer quantities are not measured",
    "line_sag_thermal_limit": "needs a thermal/mechanical model, not terminal electrical data",
}


def ac_check(world, check: str, circuit_id: str):
    """A direct AC operating check, scoped to measured from-terminal quantities.

    Returns (verdict, explanation). Anything needing an unmeasured quantity is NOT_SUPPORTED, so
    a clean DC consistency result is never read as an AC clearance."""
    from grid_ops.assessment import Verdict

    if check in UNSUPPORTED_AC_CHECKS:
        return Verdict.NOT_SUPPORTED, f"{check}: {UNSUPPORTED_AC_CHECKS[check]}"
    if check not in SUPPORTED_AC_CHECKS:
        return Verdict.NOT_SUPPORTED, f"{check}: unknown check"
    circuit = next(
        (c for cs in world.circuits_by_segment.values() for c in cs if c.id == circuit_id), None
    )
    if circuit is None:
        return Verdict.NOT_SUPPORTED, f"unknown circuit {circuit_id}"
    name = f"{circuit_id}.current_a"
    readings = [
        r
        for r in world.readings.values()
        if r.kind == "current"
        and r.covariance is not None
        and name in dict(r.payload)
        and r.id not in world.excluded_readings
    ]
    if not readings:
        return Verdict.NOT_SUPPORTED, "no current reading of this circuit"
    r = max(readings, key=lambda r: r.clock_s)
    value, sigma = dict(r.payload)[name], r.covariance.std(name)
    lower, upper = value - 1.96 * sigma, value + 1.96 * sigma
    if lower > circuit.max_current_a:
        return (
            Verdict.FAIL,
            f"from-terminal current >= {lower:.0f} A exceeds "
            f"{circuit.max_current_a:.0f} A ({r.id})",
        )
    if upper < circuit.max_current_a:
        return (
            Verdict.PASS,
            f"from-terminal current <= {upper:.0f} A within {circuit.max_current_a:.0f} A "
            f"({r.id}); far end not checked",
        )
    return (
        Verdict.INDETERMINATE,
        f"current interval {lower:.0f}-{upper:.0f} A straddles the rating",
    )
