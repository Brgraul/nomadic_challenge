# 08: Expose trustworthy consistency and uncertainty limits

**What to build:** Show what electrical evidence can validate, which readings are untestable, and how model error limits published uncertainty, without turning inconsistency into an automatic fault verdict.

**Blocked by:** 07: Publish public electrical estimates and observability.

**Status:** ready-for-agent

- [ ] Residual diagnostics use residual covariance and real-measurement redundancy, report degrees of freedom and critical sets, and distinguish TESTABLE, NOT_TESTABLE and MODEL_LIMITED.
- [ ] Minimally observable data and unrelated-component redundancy cannot produce a false passed bad-data check; an empty suspect list is not proof of consistency.
- [ ] Wrong topology, sensor bias, parameter mismatch, changed epoch and prior conflict remain distinguishable candidate explanations rather than automatic data exclusion.
- [ ] Exclusion, reinstatement and epoch reassignment preserve immutable readings with explicit dispositions and trigger dependent recomputation.
- [ ] Conditional standard deviation and a versioned discrepancy envelope are reported separately. Held-out calibration records its domain, sample count and empirical coverage without consulting active truth.
- [ ] Publish the specified conditional-plus-discrepancy radius only where supported; unsupported intervals remain unavailable and cannot satisfy reconstruction completion.
- [ ] Repeated correlated observations do not remove shared bias or systematic discrepancy, and unmeasured AC checks remain NOT_SUPPORTED.
- [ ] Operator views and fixtures expose these limits for prior-only, critical, biased, correlated and AC/DC-mismatch cases.
