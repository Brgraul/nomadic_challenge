# Planner design validation

This records the importance ranking and closure audit for the 16 findings in the preceding review in this chat. The design starts at [planner-design.md](planner-design.md), which routes readers to the focused chapters in `docs/planner/`. This is document-level validation. The proposed simulator, CP-SAT solver, estimation functions, CLI and acceptance tests remain unimplemented.

Subsequent scope clarification: incident investigations now collect only evidence needed to identify the outage cause and recommend an action. General H0 reconstruction belongs only to `healthy_benchmark`; see the [root design contract](planner-design.md) and [diagnostic completion rules](planner/validation.md#completion-and-operator-actions). The historical review and frozen section-preservation checks below predate this clarification and do not certify these later edits.

## Importance and change order

Higher rank means an earlier dependency or a greater risk of an unsupported diagnosis or invalid operational decision. The original finding IDs preserve the review's numbering.

| Rank | Original finding | Importance and dependency rationale |
|---|---|---|
| 1 | 1 | Define runtime validation before any downstream state or routing claim can rely on it. |
| 2 | 4 | Make hypotheses physically testable before accepting or explaining their changes. |
| 3 | 3 | Enforce the user's required measurement finding and causal explanation at the revision boundary. |
| 4 | 10 | Prevent contradicted energization and inspection outcomes from authorizing a landing. |
| 5 | 6 | Remove an unsupported global health conclusion and define useful scoped operator actions. |
| 6 | 7 | Prevent undetectable bad data from being counted as successful validation. |
| 7 | 2 | Refresh measurement choices from current evidence before routing them. |
| 8 | 5 | Close missing triggers so changed assumptions invalidate their dependent outputs. |
| 9 | 11 | Make the resulting evidence, explanations, task state and accepted routes mutually consistent. |
| 10 | 12 | Preserve in-progress work, repeat measurements and immutable observation history. |
| 11 | 13 | Make complete drone routes reproducible and independently feasible. |
| 12 | 8 | Prevent conditional statistical precision from hiding model error. |
| 13 | 9 | Define the electrical reduction and observation abstraction on which numerical claims depend. |
| 14 | 14 | Make spatial negative evidence respect actual coverage and detection conditions. |
| 15 | 15 | Choose an explicit reward policy and prevent shared or subdivided evidence from multiplying value. |
| 16 | 16 | Correct backend guarantees, scenario construction and evaluation claims after the contracts are settled. |

## Requirement closure

All 16 original findings are resolved at document-specification level. Final source inspection and independent re-review verified the contracts below across prose, diagrams, type sketches, examples and acceptance scenarios. Structural checks supplement that review; they do not replace it.

| Original finding | Required resolution | Final-document evidence |
|---|---|---|
| 1 | Separate truth, runtime consistency, condition and route checks, with owners and triggers. | [Validation contract](planner/validation.md#validation-contract), [events](planner/workflow.md#events-and-invalidation). |
| 2 | Refresh candidates, eligibility and information value independently of hypothesis changes. | [Agentic workflow](planner/workflow.md#agentic-workflow), [baseline tasks](planner/validation.md#completion-and-operator-actions). |
| 3 | Require LLM findings and causal reasons with evidence, before/after values and atomic acceptance. | [Explained revisions](planner/hypotheses.md#explained-hypothesis-revisions), [revision schema](planner/domain-types.md#data-shape), [example](planner/hypotheses.md#example-request). |
| 4 | Define electrical and local predictions, candidate tests, competing explanations and unsupported outcomes. | [Hypotheses and physical predictions](planner/hypotheses.md#hypotheses-and-physical-predictions), [architecture](planner/overview.md#architecture). |
| 5 | Cover known and hidden switching, readings, exclusions, model changes, expiry, availability and manual requests. | [Events and invalidation](planner/workflow.md#events-and-invalidation), [failure policy](planner/workflow.md#failure-policy). |
| 6 | Replace global health with scoped completion and evidence-linked operator actions. | [Completion and operator actions](planner/validation.md#completion-and-operator-actions). |
| 7 | Distinguish testable residual checks from critical or model-limited cases and from root-cause localization. | [State estimation](planner/estimation.md#state-estimation), [bad-data interpretation](planner/estimation.md#bad-data-interpretation). |
| 8 | Separate conditional covariance, prior bias, model discrepancy and calibrated interval claims; define noise. | [Uncertainty](planner/estimation.md#conditional-uncertainty-and-model-discrepancy), [noise](planner/geometry-and-sensing.md#noise-and-detection-conditions), [evaluation](planner/evaluation.md#evaluation). |
| 9 | Define circuit allocation, reduced admittance, transformer assumptions, terminal-equivalent sensing and rank scope. | [Electrical reduction](planner/electrical-model.md#explicit-electrical-reduction), [sensing](planner/geometry-and-sensing.md#electrical-observation-abstraction), [minimum sampling](planner/electrical-model.md#minimum-sampling). |
| 10 | Separate energization, charging capability, fault assessment and coverage; invalidate contradictory beliefs deterministically. | [Independent operational states](planner/drone-state.md#energization-chargeability-and-inspection-state). |
| 11 | Pin snapshots before notification, reject stale commands, validate before mutation and make releases idempotent. | [Simulation transaction](planner/workflow.md#simulation-and-decision-transaction), [simultaneous transitions](planner/drone-state.md#time-and-simultaneous-transitions). |
| 12 | Define site/attempt/reading identities, reservations, locked dwell, retries and semantic revision rules. | [Task lifecycle](planner/tasks-and-routes.md#task-and-observation-lifecycle), [replanning](planner/tasks-and-routes.md#replanning-rules). |
| 13 | Preserve exact circuit landing sites; validate complete routes, occupancy, expiry, waiting and deadline. | [Route invariants](planner/tasks-and-routes.md#route-acceptance-invariants), [data shape](planner/domain-types.md#data-shape), [routing constraints](planner/optimization.md#routing-constraints). |
| 14 | Cover intervals without blind gaps and distinguish qualified negatives from inconclusive evidence. | [Positions](planner/geometry-and-sensing.md#positions-on-the-line), [local coverage](planner/geometry-and-sensing.md#local-coverage-and-negative-evidence). |
| 15 | Keep urgency manager-owned; value shared evidence once; conserve subdivision utility and distinguish surrogate from joint information. | [Benefit ownership](planner/optimization.md#benefit-ownership), [reward order](planner/optimization.md#reward-order). |
| 16 | Bound search/construction, repair incumbents, state horizon limits, separate scenario briefs, fix COMPONENT_07 and compare policies honestly. | [Solver guarantees](planner/optimization.md#solver-guarantees-and-search-budget), [scenarios](planner/electrical-model.md#power-flow-ground-truth-and-scenarios), [evaluation](planner/evaluation.md#evaluation), [acceptance scenarios](planner/implementation.md#implementation-scope-and-acceptance-scenarios). |

## Regressions found during cross-validation

Independent reviews found these issues in the revised draft. Each was corrected and checked again before closure.

- Route infeasibility could hide an urgent recommendation. Validated proposed findings and recommendations now remain visible as pending diagnostic notices while accepted operational state stays atomic; a hold persists only when a complete operational continuation is unavailable.
- Coverage subdivision could inflate reward. A candidate-independent 10 m grid, conserved length-weighted utility and a fixed joint scorer now prevent that multiplication.
- Same-time events had stable IDs but no physical precedence. Physical transitions now precede sensing, with hidden provenance kept private and public epoch compatibility explicit. A final reader also caught unobserved physical transitions missing from the internal next-time list; those are now scheduled exactly without creating unobservable public events.
- Old premise and warm-start language contradicted the new objective and fallback rules. Those statements were reconciled with unique benefits, valid hints and one-stop or landing-only routes.
- Pseudo-measurement injection signs were ambiguous. Generation is now positive, load negative, with an explicit -30 MW load prior.
- Projections only from current drone positions could omit a feasible landing after a final task. Candidate-final-stop projections are now included.

## Evidence and limits

The baseline, original-finding checklist, sequential edit records, reconciliation scripts and append-only decision trail are preserved in [the audit directory](../.audit/planner-design/README.md). The baseline was captured before editing. The separate checklist was materialized after the serial changes from the earlier review and was used for the final audit; it is not evidence that a separate checklist file existed before editing. The ranking was presented in chat before the first edit.

Run the document checks and the fresh nominal rank calculation from the repository root:

```sh
.venv/bin/python .audit/planner-design/check_document.py
```

The current checker verifies the 30-line index, all 13 bounded chapters, ownership of all 42 original sections, declared timing/navigation amendments, five Python type sketches, local links and anchors, all 16 closure rows and absence of obsolete claims. It reruns the numerical rank calculation. It excludes code fences when identifying Markdown headings. These checks are supplementary evidence only. Independent readers and the final requirement-by-requirement source inspection assess conceptual consistency across prose, diagrams, types, examples and failure cases.

The nominal reduced-model calculation gives 59 nodes, 64 segments, seven components, zero-injection rank 24, total angle rank 52, and an independent 28-reading set. It uses the stated circuit allocation and installed catalogue reactances with row-normalized tolerance `1e-9`. This verifies the algebraic claim for that nominal model, not incident-mode observability, estimator accuracy or route feasibility.

## Final review result

The final state-machine reader verified original findings 1 through 6 and 10 through 13 apart from the hidden-transition scheduling clarification. A separate final confirmation verified that correction and the model-request staging rule against the event table, validation contract and advance/commit sketches. The final estimation/optimization reader verified findings 7 through 9 and 14 through 16. The [review record](../.audit/planner-design/review-results.md) preserves scopes, corrections and evidence limits. No unresolved finding from the original review remains.

The Model the Domain principle shaped the independent energization, charging, fault and coverage states. Prove It Works shaped the direct final-artifact inspection, independent reviews and fresh numerical rank reproduction. The audit trail's initial weak ranking rationale and temporary-only evidence were corrected by this durable ranking/report, the preserved audit directory and later append-only audit records. One audit child could not read the chat transcript; the primary agent checked the 16-item mapping against its earlier message directly.

The final independent audit-trail confirmation found no unresolved flags within its available evidence and reproduced the mechanical/numerical checks in an isolated replica. Its transcript-access limit is retained in the review record. The reviewed monolithic snapshot is preserved as `pre-split.md`; the split manifest and preservation check cover the current chapter layout.

## Modular layout and in-flight response

The index now routes to 13 focused chapters. The largest chapter is 158 lines. Before changing the timing contract, a section-by-section comparison verified that all 42 original sections survived the split with only heading/navigation/link changes; that checkpoint is preserved in `split-preservation-result.json`. The final checker verifies 32 unchanged section bodies and 10 explicitly recorded timing or navigation amendments rather than claiming the entire final revision is lossless.

The user also required near-real-time mid-flight response. [Workflow and latency](planner/workflow.md#in-flight-rerouting-and-latency) now specifies an immediate deterministic fleet-wide route pass, asynchronous LLM reasoning, and a second bounded pass for an explained manager update. The targets are p95 within one wall-clock second and a two-second end-to-end budget. These are acceptance requirements, not measured performance or implemented functionality. Continuous motion can rebase against unchanged meaning; new evidence or relevant operator edits invalidate stale explanations.

Independent review caught and corrected manager notification being gated by route success, a rejected draft leaving a hold open, and an ordering regression that notified the manager before refreshing diagnostics. A final targeted reader verified the corrected ingest/diagnostics/publish/notify/route order across the workflow, architecture diagram and worked trace. The reactive path does not wait for an LLM, and rejection releases a new acceptance hold when a valid route-only continuation exists. The [modular review record](../.audit/planner-design/review-results.md#modular-and-near-real-time-review) records this scope.
