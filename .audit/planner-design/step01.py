from edit import insert_before,record
insert_before('## Agentic workflow', '''## Validation contract

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
''')
record(1,1,'Separate four validation operations and specify runtime triggers and outcomes','docs/planner-design.md#validation-contract')
