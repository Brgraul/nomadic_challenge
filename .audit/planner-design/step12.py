from edit import swap,insert_before,record
swap('''`estimate(network, readings) -> GridEstimate` is pure and runs in milliseconds. The sim runs it after every reading, before the manager's turn, so the manager always sees the current estimate.''','''`estimate(model, accepted_readings, clock=...) -> GridEstimate` is pure. Runtime is measured during implementation rather than guaranteed here. The sim recomputes it when electrical inputs change, before publishing the manager's decision snapshot. Startup supplies an empty accepted-reading set and produces a labelled prior-only result.''')
swap('''- **Every branch always gets an estimate and an error bar.** The pseudo-measurements keep the system solvable from the first reading. Flow covariance is `H_f (HᵀWH)⁻¹ H_fᵀ`, so unmeasured areas show wide error bars instead of no answer.''','''- **A solvable prior model provides conditional estimates.** Eliminate reference angles and declared exact zero-injection constraints, then solve the remaining real-measurement and Gaussian-prior system using a rank-revealing factorization. Compute conditional flow covariance as `H_f C_theta H_fᵀ`, including all declared sensor and prior correlations. Do not explicitly invert an ill-conditioned normal matrix. If the remaining information matrix is singular or observations are invalid, return `EstimateUnavailable` with a diagnostic rather than inventing finite error bars.''')
swap('''    std_mw: float
    observable: bool            # pinned by readings, not by pseudo-measurements''','''    conditional_std_mw: float
    discrepancy_bound_mw: float | None
    uncertainty_radius_95_mw: float | None
    observable: bool''')
swap('''class GridEstimate:
    clock: float''','''class GridEstimate:
    clock: float
    model_revision: str
    operating_epoch: str''')
swap('''def estimate(network: Network, readings: Mapping[str, Reading]) -> GridEstimate: ...''','''def estimate(model: ElectricalModel, accepted_readings: Sequence[Reading],
             *, clock: float) -> GridEstimate | EstimateUnavailable: ...''')
insert_before('### Bad-data interpretation', '''### Conditional uncertainty and model discrepancy

`conditional_std_mw` describes uncertainty under the chosen topology, line parameters, sensor covariance and prior distributions. Pseudo-measurements regularize the estimate but do not establish real observability. Finite-weight priors can bias even a noiseless real-data fit; an exact-truth test must either use exact real constraints or compatible priors, and must state which case it tests.

Parameter uncertainty, transformer reduction, AC/DC discrepancy and unresolved topology are additional uncertainty sources. Build a versioned discrepancy envelope from declared parameter ranges and offline, held-out scenario calibration; never read the active hidden truth to correct a live estimate. Retain separate results for competing topology models rather than averaging incompatible predictions into a falsely precise state. Record calibration domain, sample count and empirical coverage with the envelope.

For a calibrated Gaussian conditional model and a validated discrepancy bound `b`, publish the conservative radius `1.96 * conditional_std_mw + b`. This is a stated conditional interval with an added model-error allowance, not automatically an unconditional 95% guarantee. Label its measured coverage and assumptions. If the discrepancy bound or coverage is unsupported outside the calibration domain, `uncertainty_radius_95_mw` is `None` and the reconstruction-completion check remains unmet.

Additional readings can reduce conditional noise uncertainty without reducing systematic discrepancy. Refresh task values using the conditional covariance and separately report the discrepancy floor, so routing does not promise to eliminate model error by repeated sampling. Sensor standard deviations, absolute floors and correlations come from the versioned sensing model below.
''')
record(12,8,'Separate conditional covariance, prior bias and calibrated model-discrepancy allowance','docs/planner-design.md#conditional-uncertainty-and-model-discrepancy')
