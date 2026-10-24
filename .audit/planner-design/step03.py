from edit import insert_before,record,swap,section
insert_before('## Example request', '''### Explained hypothesis revisions

Every creation, assessment change, scope change, prediction change, priority change or retirement of a hypothesis requires an immutable `HypothesisRevision`. The manager's existing LLM call must produce both `finding_summary` and `update_reason` before acceptance. A deterministic reading summary or a later free-text chat message does not satisfy this requirement.

The revision records the decision and source snapshot, hypothesis ID, full before and after values, actual trigger event IDs, all contributing evidence IDs, and the latest measurement IDs and timestamps in that evidence. `finding_summary` states in natural language what those measurements revealed, including values, units, location and detection conditions. `update_reason` explains why that finding changed this hypothesis in this particular way, or why an earlier measurement remains relevant to a non-measurement-triggered change. It distinguishes assessment from urgency, names contrary evidence and alternatives, and states remaining uncertainty. Reference the estimator and physical-test results used in the explanation.

For a simultaneous batch, retain all latest measurements tied at the latest timestamp and distinguish their joint contribution from older evidence. For startup, cite reports and explicitly say no drone measurement exists yet. For operator-driven updates, name the operator event as the trigger and link the latest relevant measurement if one exists; never invent a new measurement. An unchanged hypothesis receives an evidence assessment in the manager turn record but no new semantic revision.

`CommitDecision` validates the revision schema, evidence references, source revision, before values and numerical claims against stored readings. It cannot prove the LLM's causal interpretation correct; physical tests, evidence links and reviewer-visible uncertainty make that interpretation inspectable. The sim accepts the hypothesis state, its explanation and any related model changes in one transaction. Missing or invalid explanations reject the proposed update. Retrying the same decision ID is idempotent.
''')
insert_before('@dataclass(frozen=True)\nclass MeasurementTask:', '''@dataclass(frozen=True)
class HypothesisRevision:
    id: str
    decision_id: str
    source_revision: SnapshotRevision
    hypothesis_id: str
    before: Hypothesis | None
    after: Hypothesis
    trigger_event_ids: tuple[str, ...]
    contributing_evidence_ids: tuple[str, ...]
    latest_measurement_ids: tuple[str, ...]
    diagnostic_ids: tuple[str, ...]
    finding_summary: str
    update_reason: str
    remaining_uncertainty: str
''')
section('## Example request','## Events', '''## Example request

This is a hypothetical incident-mode trace. IDs are from the dataset; readings, priorities, times and routes are illustrative rather than solver output. All six reports are available at startup, even though their reported timestamps differ.

Initially H1 investigates vegetation in the SYN_REPORT_02 area, priority 3. H2 investigates a hot joint and H3 investigates abnormal motion, both priority 3. H4 investigates the reported protection event and its uncertain relationship to LINE_024's confirmed precautionary switching, priority 5. H0 tracks reduced-model flow-estimation gaps. PLAN_1 sends DRONE_3 to a clearance task on LINE_058.

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
    Sim->>Sim: Refresh affected diagnostics, publish snapshot R2
    Sim-->>Co: Decision event with R2
    Co->>Mgr: Reading, diagnostics, hypotheses and latest accepted plan
    Mgr-->>Co: H1 revision with finding and update reason
    Co->>Co: Test H1 and refresh deterministic tasks
    Co->>Pln: Changed hypothesis scope and measurement needs
    Pln->>Sol: Preview candidate tasks on R2
    Sol-->>Pln: Assignments and reasons for unassigned tasks
    Pln-->>Co: Task proposal with unchanged manager-owned urgency
    Co->>Sim: CommitDecision based on R2 and cycle C2
    Sim->>Sim: Validate revisions and candidate model state
    Sim->>Sol: Solve on candidate state with live drone positions
    Sol-->>Sim: Candidate PLAN_2
    Sim->>Sim: Validate whole plan, atomically apply decision
    Sim-->>Co: Accepted decision, explanations and PLAN_2
    Co->>Sim: Release C2 for accepted decision
```

H1's stored LLM explanation reads, for example:

> At T+00:07, DRONE_3 measured vegetation 0.4 m from the lowest conductor at the sampled location on LINE_058. This supports a local vegetation hazard and raises inspection priority from 3 to 5 under the scenario's 0.5 m clearance escalation threshold. It does not establish conductor contact, a cause of the protection event, or safe clearance outside the inspected interval. The finding does not independently implicate LINE_064. That segment remains an untested alternative only because it crosses the original report area.

The 0.5 m threshold is a synthetic exercise policy, not an operational safety standard. A numerical violation can justify an inspection recommendation before the rest of the grid is observable. The revision retains the broad untested alternatives but marks only the measured local scope as supported. If the hypothesis schema uses one assessment per claim, split the local claim from the broad untested alternatives with explained revisions for both.

If hypotheses remain unchanged, skip unnecessary planner LLM interpretation, but still refresh eligibility, reservations, covariance-based candidates and values before solving. An unassigned urgent task is reported to the manager and operator; the planner cannot raise its urgency merely to make the preview assign it.
''')
record(3,3,'Require atomic LLM-generated evidence summaries and causal explanations for every hypothesis revision','docs/planner-design.md#explained-hypothesis-revisions')
