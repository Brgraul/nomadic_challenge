from edit import section,swap,record
section('## Agentic workflow','## Architecture', '''## Agentic workflow

There are two LLM agents and one route solver. The manager owns diagnostic hypotheses, investigation urgency and operator recommendations. The planner proposes measurement needs and spatial task placement. Deterministic code maintains task eligibility, observation history, reservations and current information values. `plan_sorties` assigns eligible tasks and orders them. Drones execute accepted routes without drone agents.

Every accepted reading batch reaches a manager LLM turn after observation ingestion and affected diagnostics are complete. Hypothesis revisions include the LLM's finding summary and update reason. The coordinator then refreshes deterministic tasks even if the manager changed no hypothesis. It invokes the planner LLM only when the semantic planning input changes, such as hypothesis scope, prediction, sensor availability or an unresolved measurement need. A covariance update or completion of a task normally needs no planner LLM call.

```mermaid
flowchart TD
    start([Start scenario]) --> ingest["Ingest reports or observable events; hold cycle"]
    ingest --> checks["Refresh affected estimates, grid checks and coverage"]
    checks --> snapshot["Publish versioned decision snapshot before notification"]
    snapshot --> needmgr{"Diagnostic or evidence event?"}
    needmgr -->|yes| manager["Manager LLM proposes explained revisions and recommendations"]
    needmgr -->|no| refresh
    manager --> test["Test candidate hypotheses and model changes"]
    test --> semantic{"Semantic planning input changed?"}
    semantic -->|yes| planner["Planner LLM proposes measurement needs and sites"]
    semantic -->|no| refresh
    planner --> refresh["Refresh candidates, reservations, eligibility and conditional value"]
    refresh --> solve["Sim validates source revision and solves candidate decision"]
    solve --> valid{"Whole decision and every route valid?"}
    valid -->|yes| commit["Atomically accept revisions, explanations, tasks and routes"]
    commit --> release["Release matching hold cycle"]
    release --> fly["Fly, measure, land and charge"]
    fly --> ingest
    valid -->|no| fallback["Try validated continuation or landing-only plan; otherwise stay held"]
```

`refresh_tasks(snapshot, hypothesis_proposal, measurement_needs)` runs on startup and after accepted evidence, measurement exclusions, relevant model or hypothesis changes, reservations, expiry and drone availability changes. It removes redundant or invalid candidates, preserves in-progress work, generates baseline and confirmation candidates, and recomputes conditional information values. Each generated task carries its input revisions. Only semantically changed hypotheses increment `hypotheses_version`; evidence-assessment history has its own IDs.

The planner's last preview does not dispatch or implicitly define the committed task set. Its final structured proposal names measurement needs and sites. The coordinator validates it, merges deterministic candidates, and submits a single decision draft. Preview and final-solve inputs show their revisions and can differ if fresh evidence invalidates the draft.
''')
swap('''**The planner gets baseline tasks from the estimator.** The manager keeps one standing hypothesis, H0 "baseline grid state unknown", and sets its priority against the fault hypotheses. `observability_gaps` ranks `CURRENT` tasks by how much each would shrink the summed flow variance, using a rank-one update of the current covariance. It is deterministic. The planner maps the ranking to priorities 1 to 5 and merges these tasks with the fault tasks, so one solver run trades baseline coverage against fault hunting.''','''**Baseline tasks refresh independently of hypothesis changes.** H0 tracks reduced-model flow gaps. `observability_gaps` proposes canonical `CURRENT` sites and confirmation sites using current conditional covariance, observability and bad-data detectability. Use a rank-one covariance update for an independent scalar candidate, or a block update for correlated observations. Recompute marginal gains after each assumed selection when ranking a set; do not sum stale single-candidate gains. Mark gains as conditional on the model and noise assumptions. All H0 tasks inherit its manager-owned urgency, while deterministic benefit values remain separate. Merge these tasks after every relevant estimate change, including when H0 itself is unchanged.''')
record(7,2,'Separate deterministic task refresh, semantic planner calls and route optimization','docs/planner-design.md#agentic-workflow')
