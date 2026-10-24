from edit import section,record
section('## Events','## Replanning rules', '''## Events and invalidation

The sim processes all observable events at one simulation timestamp as one decision batch. It holds the clock, ingests observations, refreshes affected diagnostics and publishes the immutable snapshot before notifying the coordinator. `hold` is separate from the operator's pause toggle. Decision time is free in the exercise; reset the wall-time baseline on release so LLM latency is not charged as flight time.

| Event | Public estimation and checks | Manager LLM | Deterministic task refresh | Planner LLM | Route solve and acceptance |
|---|---|---|---|---|---|
| Startup with reports | Prior-only estimate, model validation, report and initial charging observations | Initial explained hypotheses | Yes | Initial measurement needs | Yes |
| Electrical reading batch | Update accepted observations, estimate, observability, detectability and condition checks | Yes, including a no-change evidence assessment | Yes | Only changed semantic planning input | Yes |
| Local reading batch | Update coverage, detection conditions and hazard checks; electrical solve only if inputs changed | Yes | Yes | Only changed semantic planning input | Yes |
| Reading rejected or reinstated | Rebuild accepted-data estimate and all dependent diagnostics | Yes | Yes, including confirmation candidates | If measurement needs change | Yes |
| Known switching, model or prior change | Rebuild affected electrical model, references and diagnostics; invalidate incompatible evidence | Yes; explain any hypothesis or model proposal | Yes | If measurement needs change | Yes |
| Proposed hypothesis or urgency change | Run affected hypothesis tests; candidate model changes refresh candidate diagnostics | Required explained revision | Yes | If semantic input changes | Yes |
| Drone ready | No unchanged grid solve; refresh availability | No, unless an unresolved diagnostic is also present | Yes | Normally no | Yes |
| Expected successful landing or charge telemetry | Update the observed circuit's charging evidence and occupancy | No unless evidence conflicts with expectations | Refresh affected eligibility and reservations | Normally no | Only if another route or endpoint eligibility changes |
| Stranding, lost charging power or contradictory voltage | Invalidate affected charging evidence; refresh electrical checks only from observable information | Yes | Yes | If measurement needs change | Yes |
| Evidence expiry | Invalidate affected knowledge and endpoint eligibility; retain historical readings | If diagnosis or a claim changes; not for an availability-only expiry | Yes | If new measurements are needed | Yes |
| Manual dispatch or operator evidence | Ingest evidence if provided; validate against the same public model and policies | If diagnostic facts or hypotheses change | Yes | If new measurement needs arise | Yes; manual requests cannot bypass acceptance |
| T+04:00 | Freeze scored observations, record unresolved checks and finish any permitted zero-time transitions | Final report, with explained revisions only if hypotheses change | Cancel unstarted work | No | Require landing by the deadline; a pre-existing infeasible state stays held for explicit intervention |

A physical incident inside `Simulation._truth` is not itself public evidence. Validate the hidden transition, then emit only declared observable consequences such as telemetry, a sensor reading or charging interruption. Hidden event IDs and evaluator scores never enter manager messages.

The dependency key includes evidence selection, operating epoch, electrical model and priors, sensor model, hypothesis semantics, coverage, task reservations, landing policy and drone state. Each output records the subset it depends on. Changing an explanation alone does not invalidate task geometry; changing covariance does invalidate information values. Compare full semantic hypothesis values before bumping `hypotheses_version`.

A solve reporting unassigned urgent work produces one deduplicated `PlanningBlocked` notification per unchanged cause, not an infinite manager/replan loop. The manager can explain the limitation, propose a new measurement need or recommend field inspection. It cannot make an infeasible route valid by renaming its urgency.

### Failure policy

Agent exceptions, malformed explanations, stale drafts, failed estimation, solver timeout, infeasibility and commit rejection have distinct recorded outcomes. No path releases the clock unconditionally. A failed LLM turn accepts no proposed hypothesis changes. Existing tasks may be reused only after refresh against current evidence and reservations.

Try the trimmed current continuation only if it still passes all route checks. Otherwise attempt a landing-only plan for every airborne drone under the accepted landing policy. An estimator failure marks the estimate unavailable and its conclusions unsupported; it does not invalidate independently supported landing evidence. If no valid continuation exists, keep the cycle held and expose the affected drone, constraint and required operator action. A coordinator watchdog can invoke the same deterministic fallback; it cannot invent explanations or silently accept a risk-policy change.

Each hold has a unique `cycle_id`. Only `Release(cycle_id, accepted_decision_id)` for that cycle's accepted decision can resume it. Late commands and releases from older cycles are rejected. Retrying a previously accepted command ID returns its stored result without replaying its effects.
''')
record(8,5,'Specify event dependency matrix, observable switching boundary and conditional failure recovery','docs/planner-design.md#events-and-invalidation')
