# Hypotheses and explanations

[Design index](../planner-design.md). Read for physical predictions, evidence interpretation, LLM-written revision reasons or the worked feedback example.

Related contracts: [Evidence and detection limits](geometry-and-sensing.md), [Acceptance and latency](workflow.md).

## Hypotheses and physical predictions

A hypothesis is a falsifiable claim with a spatial scope, supporting and contradicting evidence, a qualitative assessment (`UNTESTED`, `SUPPORTED`, `REFUTED` or `INCONCLUSIVE`), investigation priority, and predicted observations. Priority 1 to 5 expresses investigation urgency, not probability. No numeric posterior is claimed until a likelihood model is defined and calibrated. Several hypotheses may coexist, and one reading may inform several of them.

Each `Prediction` names a measurement kind, circuit or covered interval, expected result or bounded range, required operating conditions, and what would support, contradict or fail to distinguish the claim. The manager must record when existing sensors cannot distinguish alternatives. The system does not manufacture confidence from narrative fluency.

| Hypothesis class | Required test | Limits |
|---|---|---|
| Electrical topology or parameter explanation | A `ModelChange` identifying the affected circuits, switches or declared parameters; recompute the candidate estimate, observability and residual diagnostics against the same accepted observations and priors | An open line, bad sensor and wrong impedance are competing explanations. A lower fitted residual alone is not proof; report degrees of freedom, assumptions and indistinguishable candidates. |
| Local clearance, motion or thermal hazard | Predicted local observations and sensor detection conditions over a `SpanRef`, the segment ID plus start and end offsets | A clean reading outside the suspect interval is irrelevant. Low load makes a clean thermal reading inconclusive. Normal power flow cannot refute a mechanical hazard. |
| Baseline flow state unknown, H0 (`healthy_benchmark` only) | Remaining observable-flow and uncertainty gaps in the current reduced model | H0 is a benchmark objective, not a physical fault or an incident completion requirement. Incident electrical measurements must instead serve a specific causal question. |

`test_hypothesis(candidate, decision_snapshot)` returns evidence-linked `CheckResult`s and predicted discriminating measurements. It never reads hidden truth. Electrical candidate testing runs when such a candidate is created or its physical effect, supporting evidence or operating model changes. Local tests run on relevant local or operating-condition evidence. Unsupported physical predictions remain `NOT_SUPPORTED`; the hypothesis can remain an explicitly untested investigation target, but cannot be promoted to an established physical explanation.

Candidate electrical models are evaluated in isolation. The manager proposes a model revision with its hypothesis revision; the sim accepts both together only after checks and evidence references are valid. A diagnostic hypothesis does not silently change the network used by the estimator. If several topologies remain possible, keep the accepted operating assumption explicit, label observability conditional on it, and test route endpoints against all retained plausible energization states.

## Explained hypothesis revisions

Every creation, assessment change, scope change, prediction change, priority change or retirement of a hypothesis requires an immutable `HypothesisRevision`. The manager's existing LLM call must produce both `finding_summary` and `update_reason` before acceptance. A deterministic reading summary or a later free-text chat message does not satisfy this requirement.

The revision records the decision and source snapshot, hypothesis ID, full before and after values, actual trigger event IDs, all contributing evidence IDs, and the latest measurement IDs and timestamps in that evidence. Every measurement-triggered revision must identify the current triggering batch and explicitly state what its latest observation added; older supporting evidence cannot be substituted for the new finding. `finding_summary` states in natural language what those measurements revealed, including values, units, location and detection conditions. `update_reason` explains why that finding changed this hypothesis in this particular way, or why an earlier measurement remains relevant to a non-measurement-triggered change. It distinguishes assessment from urgency, names contrary evidence and alternatives, and states remaining uncertainty. Reference the estimator and physical-test results used in the explanation.

For a simultaneous batch, retain all latest measurements tied at the latest timestamp and distinguish their joint contribution from older evidence. For startup, cite reports and explicitly say no drone measurement exists yet. For operator-driven updates, name the operator event as the trigger and link the latest relevant measurement if one exists; never invent a new measurement. An unchanged hypothesis receives an evidence assessment in the manager turn record but no new semantic revision.

`CommitDecision` validates the revision schema, evidence references, source revision, before values and numerical claims against stored readings. It cannot prove the LLM's causal interpretation correct; physical tests, evidence links and reviewer-visible uncertainty make that interpretation inspectable. The sim accepts the hypothesis state, its explanation and any related model changes in one transaction. Missing or invalid explanations reject the proposed update. Retrying the same decision ID is idempotent.

## Example request

This is a hypothetical incident-mode trace. IDs are from the dataset; readings, priorities, times and routes are illustrative rather than solver output. All six reports are available at startup, even though their reported timestamps differ.

Initially H1 investigates vegetation in the SYN_REPORT_02 area, priority 3. H2 investigates a hot joint and H3 investigates abnormal motion, both priority 3. H4 investigates the reported protection event and its uncertain relationship to LINE_024's confirmed precautionary switching, priority 5. These are candidate explanations whose relationship to the outage must be tested; H0 is inactive in this incident trace. PLAN_1 sends DRONE_3 to a clearance task on LINE_058.

```mermaid
sequenceDiagram
    participant D3 as DRONE_3
    participant Sim as Sim thread
    participant Co as Coordinator
    participant Mgr as Manager LLM
    participant Pln as Planner LLM
    participant Sol as plan_sorties
    D3->>Sim: Dwell completes at T+00:07
    Sim->>Sim: Store reading and coverage, hold cycle C2
    Sim->>Sim: Refresh diagnostics, publish evidence snapshot R2
    Sim-->>Co: Async evidence event with R2, even if routes will be blocked
    Sim->>Sim: Refresh deterministic tasks
    Sim->>Sol: Immediate fleet route solve without LLM
    Sol-->>Sim: Valid reactive continuation
    Sim->>Sim: Accept reactive plan and release C2
    Co->>Mgr: Reading, diagnostics, hypotheses and latest accepted plan
    Mgr-->>Co: H1 revision with finding and update reason
    Co->>Co: Verify manager test references and refresh deterministic tasks
    Co->>Pln: Changed hypothesis scope and measurement needs
    Pln->>Sol: Preview candidate tasks on R2
    Sol-->>Pln: Assignments and reasons for unassigned tasks
    Pln-->>Co: Task proposal with unchanged manager-owned urgency
    Co->>Sim: CommitDecision with R2 semantic dependencies
    Sim->>Sim: Recheck meaning, acquire new short acceptance cycle
    Sim->>Sim: Validate revisions and candidate model state
    Sim->>Sol: Solve on candidate state with live drone positions
    Sol-->>Sim: Candidate PLAN_2
    Sim->>Sim: Validate whole plan, atomically apply decision
    Sim-->>Co: Accepted decision, explanations and PLAN_2
    Sim->>Sim: Release new cycle; flight continues
```

H1's stored LLM explanation, with H1-alt creation explained separately, reads for example:

> At T+00:07, DRONE_3 measured vegetation 0.4 m from the lowest conductor at the sampled location on LINE_058. This supports a local vegetation hazard and raises inspection priority from 3 to 5 under the scenario's 0.5 m clearance escalation threshold. It does not establish conductor contact, a cause of the protection event, or safe clearance outside the inspected interval. The finding does not independently implicate LINE_064. That segment remains an untested alternative only because it crosses the original report area.

The 0.5 m threshold is a synthetic exercise policy, not an operational safety standard. A numerical violation can justify an inspection recommendation before the rest of the grid is observable. H1 narrows to the measured local vegetation claim and becomes `SUPPORTED`. A separately created H1-alt retains the original report-area alternatives as `UNTESTED`, including LINE_064. Both changes have explained revisions; the local finding is not generalized to the broad alternatives.

The local hazard above does not by itself complete the outage diagnosis: the manager still needs evidence linking a cause to the outage and addressing material alternatives. It must not respond by collecting unrelated flow data. Once the incident completion criteria are met, retire diagnostic needs even if much of the grid remains unobserved; preserve the hazard recommendation and any explicit field-inspection handoff.

If hypotheses remain unchanged, skip unnecessary planner LLM interpretation, but still refresh eligibility, reservations, covariance-based candidates and values within active accepted needs before solving. An unassigned urgent task is reported to the manager and operator; the planner cannot raise its urgency merely to make the preview assign it.
