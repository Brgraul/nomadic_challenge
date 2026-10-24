# State estimation and uncertainty

[Design index](../planner-design.md). Read for WLS, observability, critical sets, residual tests, priors, calibration or model discrepancy.

Related contracts: [Model assumptions](electrical-model.md), [Completion criteria](validation.md).

## State estimation

`estimate(model, accepted_readings, clock=...) -> GridEstimate` is pure. Runtime is measured during implementation rather than guaranteed here. The sim recomputes it when electrical inputs change, before publishing the manager's decision snapshot. Startup supplies an empty accepted-reading set and produces a labelled prior-only result.

- **The model is the explicitly reduced DC network below.** Unknowns are one angle per mapped node and one reference per currently connected reduced component. Observability is conditional on that chosen operating topology. AC/DC, transformer and parameter mismatch are modeled limitations, not measurement noise alone.
- **Three kinds of rows feed a weighted least squares solve.**

| Source | Row | σ |
|---|---|---|
| `CURRENT` reading | From-terminal P summed over the segment's accepted circuit observations | Derived aggregate covariance, including correlations |
| Junction or intermediate node | Net injection = 0 in the declared reduced model | Exact constraint eliminated before solving; physical-model discrepancy reported separately |
| Pseudo-measurement at substations, boundaries and the terminal | Net injection prior is -30 MW for substations and the terminal, 0 MW for boundaries | 30 MW, deliberately wide |

- **A solvable prior model provides conditional estimates.** Eliminate reference angles and declared exact zero-injection constraints, then solve the remaining real-measurement and Gaussian-prior system using a rank-revealing factorization. Compute conditional flow covariance as `H_f C_theta H_fᵀ`, including all declared sensor and prior correlations. Do not explicitly invert an ill-conditioned normal matrix. If the remaining information matrix is singular or observations are invalid, return `EstimateUnavailable` with a diagnostic rather than inventing finite error bars.
- **Observable means pinned by real readings.** A flow counts as observable only when the readings and zero-injection rows alone determine it, with pseudo-measurements left out. This is the same rank test as in the minimum sampling section.
- **Bad-data detection is a scoped diagnostic.** Determine residual degrees of freedom, critical measurements and critical sets from the accepted real measurement matrix and declared exact constraints, excluding pseudo-measurements as independent validation evidence. Each measurement receives `TESTABLE`, `NOT_TESTABLE` or `MODEL_LIMITED`. At a minimally observable set, bad readings can fit exactly; an empty suspect list then means `NOT_TESTABLE`, not passed. Redundancy in another component does not remove this limitation.
- **Inconsistency is not fault localization.** After checking model applicability and normalized residual covariance, a calibrated global test detects inconsistency and the largest normalized residual above the configured threshold, initially 3, identifies a candidate observation for review. Distinguish measurement error, wrong topology, parameter error, model discrepancy and changed operating epoch. No residual alone confirms a faulty asset, and model-limited statistics do not justify automatic data removal. The manager requests a discriminating or repeat reading. Rejected data remains in the immutable history with an exclusion reason; reinstatement is also recorded and triggers recomputation.

```python
@dataclass(frozen=True)
class FlowEstimate:
    p_mw: float
    conditional_std_mw: float
    discrepancy_bound_mw: float | None
    uncertainty_radius_95_mw: float | None
    observable: bool

@dataclass(frozen=True)
class GridEstimate:
    clock: float
    model_revision: str
    operating_epoch: str
    flows: dict[str, FlowEstimate]           # per segment
    injections_mw: dict[str, FlowEstimate]   # per node
    diagnostics: tuple[CheckResult, ...]
    detectability: dict[str, Detectability]
    suspect_reading_ids: tuple[str, ...]
    accepted_reading_ids: tuple[str, ...]

def estimate(model: ElectricalModel, accepted_readings: Sequence[Reading],
             *, clock: float) -> GridEstimate | EstimateUnavailable: ...
def observability_gaps(model: ElectricalModel, est: GridEstimate,
                       k: int) -> list[MeasurementNeed]: ...
```

## Conditional uncertainty and model discrepancy

`conditional_std_mw` describes uncertainty under the chosen topology, line parameters, sensor covariance and prior distributions. Pseudo-measurements regularize the estimate but do not establish real observability. Finite-weight priors can bias even a noiseless real-data fit; an exact-truth test must either use exact real constraints or compatible priors, and must state which case it tests.

Parameter uncertainty, transformer reduction, AC/DC discrepancy and unresolved topology are additional uncertainty sources. Build a versioned discrepancy envelope from declared parameter ranges and offline, held-out scenario calibration; never read the active hidden truth to correct a live estimate. Retain separate results for competing topology models rather than averaging incompatible predictions into a falsely precise state. Record calibration domain, sample count and empirical coverage with the envelope.

For a calibrated Gaussian conditional model and a validated discrepancy bound `b`, publish the conservative radius `1.96 * conditional_std_mw + b`. This is a stated conditional interval with an added model-error allowance, not automatically an unconditional 95% guarantee. Label its measured coverage and assumptions. If the discrepancy bound or coverage is unsupported outside the calibration domain, `uncertainty_radius_95_mw` is `None` and the reconstruction-completion check remains unmet.

Additional readings can reduce conditional noise uncertainty without reducing systematic discrepancy. Refresh task values using the conditional covariance and separately report the discrepancy floor, so routing does not promise to eliminate model error by repeated sampling. Sensor standard deviations, absolute floors and correlations come from the versioned sensing model below.

## Bad-data interpretation

The implementation must compute normalized residuals with the residual covariance, not merely divide a residual by its input sensor standard deviation. Zero or nearly zero residual variance identifies a critical or numerically untestable case. A pseudo-measurement disagreement is reported as prior conflict rather than treated as proof that a real sensor is faulty. If candidate model changes or discrepancy invalidate the assumed residual distribution, report `MODEL_LIMITED` and retain the observation until a reviewed exclusion is accepted.

These checks follow the distinction between global inconsistency and candidate-data identification in the [pandapower state-estimation documentation](https://pandapower.readthedocs.io/en/v3.5.6/estimation.html). Their result is evidence for the manager, not an automatic root-cause verdict.
