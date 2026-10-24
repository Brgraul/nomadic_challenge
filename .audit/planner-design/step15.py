from edit import section,record
section('## Where speed enters the solver','## How good the optimization will be', '''## Optimization model and objective

Use OR-Tools CP-SAT for the route model. The original additive RoutingModel disjunction sketch is replaced because a shared observation may satisfy several hypotheses and several overlapping stops may serve the same coverage group. Explicit visit and served-group variables make those rewards and ownership constraints inspectable. `plan_sorties` remains pure over immutable inputs; the sim owns acceptance.

### Benefit ownership

The manager owns urgency 1 to 5 on each investigation need. The planner may change locations, propose alternatives or report infeasibility, but cannot raise urgency to force assignment. A task supporting several hypotheses has one physical attempt and a set of benefit groups. The task itself has no independently inflated prize.

A `BenefitGroup` represents a unique evidence need, such as electrical observability of a canonical circuit aggregate, a specified confirmation test, or a disjoint local coverage interval. It records its hypothesis associations, one manager-approved urgency, an integer utility in a declared scale, and the task alternatives that can satisfy it. Multiple hypotheses sharing the same physical need share that group; they do not multiply its value. A group is rewarded at most once per decision. The group registry and discretization are deterministic, so splitting a task or inventing another hypothesis cannot create new reward.

Partition local inspection targets at coverage boundaries into disjoint cells of at most 1 km. A task serves a cell only if its qualified inspected footprint would cover the complete cell; partial cells are split before modeling. The Boolean group-served variable is the OR of qualifying selected visits. Overlapping stops cannot earn the same cell twice. A repeat electrical observation receives a distinct confirmation group only when its documented diagnostic purpose and sensor covariance justify it. Independent noise reduction is not new structural observability.

Compute conditional information values sequentially from current covariance and already assumed selections. They are a surrogate for information gained by a set, not a Bayes root-cause probability. Within a bounded outer loop, update conditional benefits from the selected set, solve again, and keep the best independently scored feasible plan. Use at most three reranking rounds and report both the surrogate score and joint recomputed information/coverage metrics. Calibration uncertainty and set interactions prevent any claim that this surrogate is an exact entropy objective.

### Reward order

Urgency is lexicographic. Compare the total unique-group utilities at priority 5, then 4, 3, 2 and 1, followed by uncertain-landing risk if enabled, route switching cost, and total airtime. Six priority-1 tasks cannot displace a feasible priority-5 benefit solely because their additive count is larger. Hard feasibility constraints always dominate rewards. A confirmed urgent hazard may instead require operator inspection if no drone task is feasible.

Encode this order with mixed-radix integer weights derived from explicit upper bounds for every lower tier and cost. Assert that the full expression fits signed 64-bit arithmetic. If it does not, use staged solves, fixing a tier only after proven optimality; if the budget ends without that proof, return the best feasible incumbent and report which objective tiers remain unproved. Never silently approximate strict urgency by `priority * 100_000`.

Switching cost discourages changing a drone's next stop when benefit and risk are unchanged; it is a lower objective tier than diagnostic benefit. It does not forbid an endpoint change needed for feasibility. The score of a retained route and a new route is recomputed with identical current inputs and benefit groups.

### Routing constraints

```text
vehicles = Ready, Flying and post-dwell continuations of Measuring drones
candidate nodes = unique pending attempts, eligible exact landing sites and vehicle starts
arc[v,i,j] = selected travel arc for drone v
visit[v,t] = task attempt t assigned to drone v
served[g] = OR of selected visits qualified to satisfy benefit group g
require at most one vehicle per task and preserve locked measurement ownership
require a connected ordered path per active vehicle; eliminate subtours
require one eligible terminal landing for every airborne or dispatched drone
allow a Ready vehicle to wait without an artificial flight or recharge
propagate arrival times with conservative rounded travel and dwell seconds
limit remaining flight plus dwell by the drone's remaining battery
require observation completion and final landing no later than T+04:00
enforce endpoint evidence validity at arrival and landing occupancy constraints
maximize lexicographic unique-group utility, then minimize risk, switching and airtime
```

Each vehicle starts at its exact current position or its locked stop after the remaining dwell. For each speed, travel seconds are `ceil(60 * distance_km / speed_km_per_min)`. Dwell is attached once to the visited task, and the post-dwell vehicle budget subtracts locked work before routing. Endpoint validity is computed from public knowledge and declared assumptions, never hidden truth. Warm starts are hints only; correctness rests on the complete constraint set and independent acceptance.

`PlanningResult` records status, optional plan, independently recomputed score, model and input revisions, reasoned unassigned tasks, objective bounds where available, search budget and replay settings. A result without a feasible plan cannot be committed. Every preview uses the same modeling rules as acceptance and labels its source revision.
''')
record(15,15,'Replace additive task prizes with unique evidence groups and explicit lexicographic CP-SAT route objective','docs/planner-design.md#optimization-model-and-objective')
