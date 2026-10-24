from edit import swap,insert_before,record
swap('''- **Bad readings are found only with redundancy.** With more readings than the minimum, a normalized residual above 3 flags a suspect reading. At the minimum set every reading is critical, so a bad one can't be detected.''','''- **Bad-data detection is a scoped diagnostic.** Determine residual degrees of freedom, critical measurements and critical sets from the accepted real measurement matrix and declared exact constraints, excluding pseudo-measurements as independent validation evidence. Each measurement receives `TESTABLE`, `NOT_TESTABLE` or `MODEL_LIMITED`. At a minimally observable set, bad readings can fit exactly; an empty suspect list then means `NOT_TESTABLE`, not passed. Redundancy in another component does not remove this limitation.
- **Inconsistency is not fault localization.** After checking model applicability and normalized residual covariance, a calibrated global test detects inconsistency and the largest normalized residual above the configured threshold, initially 3, identifies a candidate observation for review. Distinguish measurement error, wrong topology, parameter error, model discrepancy and changed operating epoch. No residual alone confirms a faulty asset, and model-limited statistics do not justify automatic data removal. The manager requests a discriminating or repeat reading. Rejected data remains in the immutable history with an exclusion reason; reinstatement is also recorded and triggers recomputation.''')
swap('''    suspect_readings: tuple[str, ...]        # normalized residual above 3
    readings_used: int''','''    diagnostics: tuple[CheckResult, ...]
    detectability: dict[str, Detectability]
    suspect_reading_ids: tuple[str, ...]
    accepted_reading_ids: tuple[str, ...]''')
insert_before('### Completion and operator actions', '''### Bad-data interpretation

The implementation must compute normalized residuals with the residual covariance, not merely divide a residual by its input sensor standard deviation. Zero or nearly zero residual variance identifies a critical or numerically untestable case. A pseudo-measurement disagreement is reported as prior conflict rather than treated as proof that a real sensor is faulty. If candidate model changes or discrepancy invalidate the assumed residual distribution, report `MODEL_LIMITED` and retain the observation until a reviewed exclusion is accepted.

These checks follow the distinction between global inconsistency and candidate-data identification in the [pandapower state-estimation documentation](https://pandapower.readthedocs.io/en/v3.5.6/estimation.html). Their result is evidence for the manager, not an automatic root-cause verdict.
''')
record(6,7,'Distinguish testable bad-data checks, critical sets and model inconsistency from asset diagnosis','docs/planner-design.md#bad-data-interpretation')
