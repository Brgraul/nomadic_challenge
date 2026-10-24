"""Versioned model-discrepancy envelope for the reduced DC model.

The envelope is built offline from held-out scenario simulations (see
`grid_ops.truth.calibrate`) and stored as data. It records the domain it was calibrated on, the
sample counts and the empirical coverage on a validation split. It is never recomputed from, or
corrected with, the active scenario's hidden truth.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

ENVELOPE_PATH = Path(__file__).with_name("calibration") / "discrepancy_envelope_v1.json"
MIN_SAMPLES = 10
MIN_COVERAGE = 0.8


@dataclass(frozen=True)
class SegmentBound:
    bound_mw: float
    sample_count: int
    empirical_coverage: float  # fraction of validation samples within the bound


@dataclass(frozen=True)
class DiscrepancyEnvelope:
    version: str
    domain: str
    calibration_seeds: tuple[int, ...]
    validation_seeds: tuple[int, ...]
    quantile: float
    bounds: dict[str, SegmentBound]
    matching_model: bool = False  # test fixtures whose truth is the DC model itself

    def supports(
        self,
        segment_id: str,
        known_out_of_service: frozenset[str],
        epoch: str,
        active_seed: int | None = None,
    ) -> tuple[bool, str]:
        """Is the bound valid for this segment under the current topology and epoch?"""
        if self.matching_model:
            return True, "declared matching model: zero discrepancy"
        if known_out_of_service:
            return False, "topology differs from the calibration domain (known switching)"
        if epoch != "E0":
            return False, f"epoch {epoch} is outside the calibration domain"
        if (
            active_seed is not None
            and active_seed in self.calibration_seeds + self.validation_seeds
        ):
            return False, "active seed overlaps calibration data (not held out)"
        b = self.bounds.get(segment_id)
        if b is None:
            return False, "segment not calibrated"
        if b.sample_count < MIN_SAMPLES or b.empirical_coverage < MIN_COVERAGE:
            return False, (
                f"calibration too weak (n={b.sample_count}, coverage {b.empirical_coverage:.2f})"
            )
        return True, f"calibrated on {b.sample_count} held-out samples"


ZERO_ENVELOPE = DiscrepancyEnvelope(
    "matching-model", "declared matching model", (), (), 1.0, {}, matching_model=True
)


def load_envelope(path: Path = ENVELOPE_PATH) -> DiscrepancyEnvelope | None:
    if not path.exists():
        return None
    raw = json.loads(path.read_text())
    return DiscrepancyEnvelope(
        raw["version"],
        raw["domain"],
        tuple(raw["calibration_seeds"]),
        tuple(raw["validation_seeds"]),
        raw["quantile"],
        {k: SegmentBound(**v) for k, v in raw["bounds"].items()},
    )
