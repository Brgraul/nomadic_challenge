from edit import section,record
section('**Grid health has a concrete stopping rule.**','**The planner gets baseline tasks from the estimator.**', '''### Completion and operator actions

There is no global `known_healthy` state in this design. Maintain three independently scoped results.

| Result | Completion rule | What it does not establish |
|---|---|---|
| Reduced-model flow reconstruction | Every target aggregate flow is observable from accepted real measurements plus declared exact constraints; calibrated 95% uncertainty radius is at most 5 MW; consistency checks pass and required bad-data checks are testable and pass | AC voltage adequacy, every circuit's loading, mechanical integrity or correct real-world topology |
| Electrical operating assessment | Each requested voltage, circuit-current, loading or island check has adequate evidence, stated uncertainty and a result | Unmeasured quantities remain `INDETERMINATE` or `NOT_SUPPORTED`; completion of supported checks cannot silently complete the others |
| Local investigation | Each target hypothesis is supported with an action, refuted within its inspected scope and detection conditions, or explicitly unresolved with a reason | Clean flow estimates and partial local coverage cannot clear uninspected hazards |

The manager proposes an evidence-linked `Recommendation` with action, scope, urgency, supporting readings, applicable synthetic threshold and remaining uncertainty. A local clearance violation below the exercise's 0.5 m escalation threshold is enough to recommend inspection of that interval. A credible immediate local hazard receives an urgent inspection recommendation even while H0 remains unresolved. Repair recommendations require a supported defect and an explained operator review request; the simulation never treats a hypothesis as an executed repair or switching action.

When evidence is ambiguous, recommend a discriminating reading or field inspection and identify what cannot yet be concluded. End a regional investigation only after its scoped criteria are met or the operator explicitly accepts the unresolved limits. At T+04:00, report remaining work and unmet checks instead of declaring success. An empty suspect list and the exhaustion of reachable tasks are not completion criteria.

The initial DC implementation reports global unmeasured AC checks as `NOT_SUPPORTED`. This is a truthful completion report, not a requirement to implement an AC estimator before routes can run. Action recommendations and route feasibility are checked independently of reconstruction completion.
''')
record(5,6,'Replace unsupported grid-health stop with scoped reconstruction, condition and investigation results','docs/planner-design.md#completion-and-operator-actions')
