from edit import insert_before,record,swap
insert_before('## Example request', '''## Hypotheses and physical predictions

A hypothesis is a falsifiable claim with a spatial scope, supporting and contradicting evidence, a qualitative assessment (`UNTESTED`, `SUPPORTED`, `REFUTED` or `INCONCLUSIVE`), investigation priority, and predicted observations. Priority 1 to 5 expresses investigation urgency, not probability. No numeric posterior is claimed until a likelihood model is defined and calibrated. Several hypotheses may coexist, and one reading may inform several of them.

Each `Prediction` names a measurement kind, circuit or covered interval, expected result or bounded range, required operating conditions, and what would support, contradict or fail to distinguish the claim. The manager must record when existing sensors cannot distinguish alternatives. The system does not manufacture confidence from narrative fluency.

| Hypothesis class | Required test | Limits |
|---|---|---|
| Electrical topology or parameter explanation | A `ModelChange` identifying the affected circuits, switches or declared parameters; recompute the candidate estimate, observability and residual diagnostics against the same accepted observations and priors | An open line, bad sensor and wrong impedance are competing explanations. A lower fitted residual alone is not proof; report degrees of freedom, assumptions and indistinguishable candidates. |
| Local clearance, motion or thermal hazard | Predicted local observations and sensor detection conditions over a `SpanRef`, the segment ID plus start and end offsets | A clean reading outside the suspect interval is irrelevant. Low load makes a clean thermal reading inconclusive. Normal power flow cannot refute a mechanical hazard. |
| Baseline flow state unknown, H0 | Remaining observable-flow and uncertainty gaps in the current reduced model | H0 is an investigation objective, not a physical fault or an assertion of grid health. |

`test_hypothesis(candidate, decision_snapshot)` returns evidence-linked `CheckResult`s and predicted discriminating measurements. It never reads hidden truth. Electrical candidate testing runs when such a candidate is created or its physical effect, supporting evidence or operating model changes. Local tests run on relevant local or operating-condition evidence. Unsupported physical predictions remain `NOT_SUPPORTED`; the hypothesis can remain an explicitly untested investigation target, but cannot be promoted to an established physical explanation.

Candidate electrical models are evaluated in isolation. The manager proposes a model revision with its hypothesis revision; the sim accepts both together only after checks and evidence references are valid. A diagnostic hypothesis does not silently change the network used by the estimator. If several topologies remain possible, keep the accepted operating assumption explicit, label observability conditional on it, and test route endpoints against all retained plausible energization states.
''')
swap('''    suspect_segments: tuple[str, ...]
    priority: int               # 1-5, how much the manager wants it tested''','''    scope: tuple[SpanRef, ...]
    assessment: HypothesisAssessment
    priority: int
    prediction_ids: tuple[str, ...]
    supporting_evidence_ids: tuple[str, ...]
    contradicting_evidence_ids: tuple[str, ...]
    model_change: ModelChange | None''')
record(2,4,'Define falsifiable predictions, hypothesis-conditioned tests and model-change acceptance','docs/planner-design.md#hypotheses-and-physical-predictions')
