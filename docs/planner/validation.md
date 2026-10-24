# Validation and completion

[Design index](../planner-design.md). Read for grid plausibility checks, supported conclusions, investigation completion or operator actions.

Related contracts: [Supported electrical diagnostics](estimation.md), [Check triggers](workflow.md).

## Validation contract

Four checks have separate owners and results. None may substitute for another.

| Check | Owner and inputs | Trigger | Consequence |
|---|---|---|---|
| Synthetic truth validity | `truth.build`, using hidden AC buses, circuits, injections and scenario switching | Scenario construction and every physical switching or incident that changes the electrical network | Require a finite, converged scenario state under that scenario's specified limits, or reject the scenario transition. Results never enter agent snapshots. |
| Observation and model consistency | `check_grid(model, accepted_readings, estimate)`, using only public evidence and declared model assumptions | Startup; accepted, rejected or reinstated electrical readings; known topology, parameter, prior or sensor-model changes | Publish residual, observability and model-consistency diagnostics before the manager reasons. Mark inconsistent or unsupported conclusions explicitly. |
| Operating condition and local hazards | `assess_condition`, using public readings, uncertainty, inspection coverage and consistency diagnostics | Each relevant electrical or local observation and each relevant model or hypothesis revision | Report measured violations, supported inferences, unresolved hazards and unsupported checks separately. Drive investigation and operator recommendations. |
| Executable drone route | `validate_plan`, using the proposed complete plan, current public decision revision, exact drone state and accepted landing policy | Every autonomous plan or manual dispatch, including retained continuations and landing-only fallbacks | Validate all drones before any mutation. Resume only with a valid continuation for every airborne drone. |

`CheckResult` contains `check_name`, scope, evidence IDs, model revision, method and assumptions, plus `PASS`, `FAIL`, `INDETERMINATE` or `NOT_SUPPORTED` and a concise diagnostic. An empty violation list is never a substitute for a completed check. Observability and bad-data detectability are separate diagnostic fields.

The runtime grid check validates units, circuit references and finite observations, checks active-power consistency and topology assumptions, and assesses available direct voltage and current evidence against the synthetic ratings. Its DC model cannot certify unmeasured voltage magnitudes, reactive flows or all circuit loadings. Those checks return `NOT_SUPPORTED` unless an explicitly defined model and adequate evidence support them. An optional AC candidate screen uses public assumptions and evidence only; convergence shows that a modeled state exists, not that it is the actual state or the true root cause.

Recompute estimation and these diagnostics before the manager sees a changed evidence set. Local-only readings refresh hazard and coverage diagnostics without needlessly solving the unchanged electrical model. At startup, compute a prior-only estimate and label real-measurement observability absent. Route optimization may investigate an uncertain or inconsistent grid; it must not treat that uncertainty as proof that a landing site is chargeable. Grid inconsistency therefore generates diagnostic tasks and constrains claims, while route infeasibility prevents release.

Hidden physical switching first rebuilds and validates the hidden electrical state. It affects public state only through declared telemetry, sensor readings or charging observations. A known switching report separately revises the public topology assumptions and triggers the public checks. There is no direct hidden-truth-to-estimator update.

## Completion and operator actions

There is no global `known_healthy` state in this design. Maintain three independently scoped results.

**Incident completion is a diagnostic decision.** The manager may conclude the outage investigation when public evidence supports a cause and its connection to the outage, material competing explanations have been refuted or shown not to change the next action, and the recommendation states its scope, supporting evidence and remaining uncertainty. Finding a local defect alone does not establish that it caused the outage. Full flow reconstruction, global observability and unrelated operating checks are not prerequisites. Unmeasured quantities remain explicitly unknown.

When these criteria are met, accept an explained conclusion and retire the remaining incident diagnostic needs; cancel unstarted attempts, finish locked measurements and recover airborne drones through validated routes. Preserve urgent hazard recommendations and explicit operator or field-inspection handoffs. If evidence cannot distinguish material causes, report the investigation as unresolved and recommend a discriminating reading or field inspection; do not declare a root cause merely to stop. An operator may explicitly accept those unresolved limits. Relevant new evidence or an operator request may reopen a concluded investigation through an explained revision.

| Result | Completion rule | What it does not establish |
|---|---|---|
| Reduced-model flow reconstruction (`healthy_benchmark` objective; optional incident diagnostic) | Every target aggregate flow is observable from accepted real measurements plus declared exact constraints; calibrated 95% uncertainty radius is at most 5 MW; consistency checks pass and required bad-data checks are testable and pass | Outage root cause, AC voltage adequacy, every circuit's loading, mechanical integrity or correct real-world topology; an unmet reconstruction result does not prevent incident completion |
| Electrical operating assessment | Each requested voltage, circuit-current, loading or island check has adequate evidence, stated uncertainty and a result | Unmeasured quantities remain `INDETERMINATE` or `NOT_SUPPORTED`; completion of supported checks cannot silently complete the others |
| Local investigation | Each target hypothesis is supported with an action, refuted within its inspected scope and detection conditions, or explicitly unresolved with a reason | Clean flow estimates and partial local coverage cannot clear uninspected hazards |

The manager proposes an evidence-linked `Recommendation` with action, scope, urgency, supporting readings, applicable synthetic threshold and remaining uncertainty. A local clearance violation below the exercise's 0.5 m escalation threshold is enough to recommend inspection of that interval. A credible immediate local hazard receives an urgent inspection recommendation even while the outage cause remains unresolved. Repair recommendations require a supported defect and an explained operator review request; the simulation never treats a hypothesis as an executed repair or switching action.

When evidence is ambiguous, recommend a discriminating reading or field inspection and identify what cannot yet be concluded. End a regional investigation only after its scoped criteria are met or the operator explicitly accepts the unresolved limits. At T+04:00, report remaining work and unmet checks instead of declaring success. An empty suspect list and the exhaustion of reachable tasks are not completion criteria.

The initial DC implementation reports global unmeasured AC checks as `NOT_SUPPORTED`. This is a truthful completion report, not a requirement to implement an AC estimator before routes can run. Action recommendations and route feasibility are checked independently of reconstruction completion.

**Task refresh stays within the selected objective.** In incident mode, `observability_gaps` may propose canonical `CURRENT` or confirmation sites only to serve an active, evidence-linked diagnostic need. A covariance reduction or missing rank alone is insufficient; each task's purpose must explain its diagnostic relevance. Refresh candidates after relevant estimate changes even when hypotheses are unchanged, but never regenerate retired needs. In `healthy_benchmark` only, H0 tracks reduced-model flow gaps and generates reconstruction tasks independently of hypothesis changes. All H0 tasks inherit its manager-owned urgency, while deterministic benefit values remain separate.

For eligible electrical candidates, use current conditional covariance, observability and bad-data detectability. Use a rank-one covariance update for an independent scalar candidate, or a block update for correlated observations. Recompute marginal gains after each assumed selection when ranking a set; do not sum stale single-candidate gains. Mark gains as conditional on the model and noise assumptions. These calculations rank eligible work; they cannot authorize broader collection.
