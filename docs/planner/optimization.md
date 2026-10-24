# Optimization and solver guarantees

[Design index](../planner-design.md). Read for shared-evidence rewards, urgency, CP-SAT constraints, search budgets, fallbacks or the sortie arithmetic example.

Related contracts: [Hard invariants](tasks-and-routes.md), [Response deadline](workflow.md).

## Optimization model and objective

Use OR-Tools CP-SAT for the route model. The original additive RoutingModel disjunction sketch is replaced because a shared observation may satisfy several hypotheses and several overlapping stops may serve the same coverage group. Explicit visit and served-group variables make those rewards and ownership constraints inspectable. `plan_sorties` remains pure over immutable inputs; the sim owns acceptance.

## Benefit ownership

Eligibility precedes scoring. In incident mode, every rewarded group must serve an active accepted diagnostic need with a stated causal question and possible effect on the diagnosis or next action. Restrict electrical variance reduction to quantities relevant to that need, and local coverage to its diagnostic scope. Unrelated flow gaps and coverage earn no benefit and cannot generate tasks. Retiring a need removes its otherwise unsupported groups and unstarted stops before incumbent repair or scoring. H0 may reward general reconstruction only in the explicitly selected `healthy_benchmark`. Safe recovery remains a hard operational requirement after diagnostic collection stops.

The manager owns urgency 1 to 5 on each investigation need. `Hypothesis.priority` stores that urgency; `Hypothesis.lifecycle` is `ACTIVE` or `RETIRED`, independent of evidential assessment. The planner may change locations, propose alternatives or report infeasibility, but cannot raise urgency to force assignment. A task supporting several hypotheses has one physical attempt and a set of benefit groups. The task itself has no independently inflated prize.

A `BenefitGroup` represents a unique evidence need, such as electrical observability of a canonical circuit aggregate, a specified confirmation test, or a local coverage cell. It records its hypothesis associations, one manager-approved urgency, deterministic utility and qualifying task alternatives. Shared physical needs share a group and are rewarded once. The highest urgency requested by its accepted hypothesis associations owns that shared group; raising it requires an explained manager revision.

Use a fixed 10 m along-line coverage grid anchored at segment offset zero, clipped at segment ends. This grid is independent of candidate tasks and hypothesis wording. Union all overlapping target intervals for the same sensor kind, segment and epoch before assigning benefit; new hypothesis IDs cannot multiply that union. A newly inspected full 10 m cell contributes 10 utility points and a shorter terminal cell contributes its length in metres, represented as integer millimetre-points divided by 1000. A task qualifies only if its predicted valid footprint covers the full cell. Coverage uncertainty is shown as predicted, never ingested as actual evidence before sensing. If geometry requires splitting a cell, distribute its fixed integer millimetre-points by exact length so children sum to the parent; never round each child up. Overlapping visits cannot earn the same spatial points twice.

A distinct confirmation group contributes 10 utility points only when a current, evidence-linked diagnostic needs that test. It has at most one rewarded completion in the decision, regardless of the number of referring hypotheses or repeat sites. A repeat electrical observation is otherwise rewarded only for its joint conditional noise reduction, not new structural rank.

For electrical information, one unit of total conditional aggregate-flow variance reduction in MW squared contributes `1000 / max(V0, 1 MW squared)` utility points, where `V0` is the summed flow variance over the eligible target quantities at the start of this decision. In incident mode these quantities must serve the accepted diagnostic needs; in `healthy_benchmark` they are the declared reconstruction targets. Thus eliminating all initially eligible variance is worth at most 1000 points, independent of task splitting. Use the full selected observation covariance including correlations. When no valid covariance exists, electrical information utility is unknown and contributes no fabricated points; declared diagnostic confirmation groups may still be valued.

`score_plan` is fixed for all rounds of one decision. In urgency order 5 through 1, condition on measurements selected at higher tiers, then compute that tier's joint additional variance reduction plus its unique spatial and confirmation points. Shared observations are assigned once to their owning tier. Express solver coefficients in integer milli-points by flooring nonnegative surrogate gains; independent comparison uses the same fixed-scale joint score, with a recorded quantization tolerance. This gives explicit comparability for the initial exercise policy without claiming these units are calibrated root-cause entropy or economic value.

CP-SAT uses a linearized conditional-information surrogate. Within at most three outer reranking rounds, update marginal coefficients from the proposed selected set, solve again, then compare every feasible result and the retained incumbent with the fixed `score_plan` function above. Do not compare raw objective values from different surrogate rounds. Report surrogate bounds only for their specific round, plus the independently recomputed joint score. The surrogate and discretization do not guarantee a globally optimal information policy.

## Reward order

The intended joint diagnostic score and each round's declared surrogate use the same urgency ordering. Urgency is lexicographic. Compare the total unique-group utilities at priority 5, then 4, 3, 2 and 1, followed by uncertain-landing risk if enabled, route switching cost, and total airtime. Six priority-1 tasks cannot displace a feasible priority-5 benefit solely because their additive count is larger. Hard feasibility constraints always dominate rewards. A confirmed urgent hazard may instead require operator inspection if no drone task is feasible.

Encode this order with mixed-radix integer weights derived from explicit upper bounds for every lower tier and cost. Assert that the full expression fits signed 64-bit arithmetic. If it does not, use staged solves, fixing a tier only after proven optimality; if the budget ends without that proof, return the best feasible incumbent and report which objective tiers remain unproved. Never silently approximate strict urgency by `priority * 100_000`.

Switching cost discourages changing a drone's next stop when benefit and risk are unchanged; it is a lower objective tier than diagnostic benefit. It does not forbid an endpoint change needed for feasibility. The score of a retained route and a new route is recomputed with identical current inputs and benefit groups.

## Routing constraints

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

## Solver guarantees and search budget

Trim completed work and invalid endpoints from the current plan, preserve locked measurements, and validate the resulting continuation. If feasible, retain it as an incumbent and pass its assignments as solver hints. Independently score every candidate with current groups and the same objective. Accept a new feasible candidate only if it is at least as good as that still-valid incumbent. A stale or infeasible old plan creates no such guarantee.

CP-SAT may return `OPTIMAL`, `FEASIBLE`, `INFEASIBLE`, `MODEL_INVALID` or `UNKNOWN`. Only the first two provide a candidate for independent validation; `OPTIMAL` proves optimality only for that finite, discretized, surrogate model. If a retained incumbent is used after `UNKNOWN` or timeout, report that it is a validated fallback, not a newly solved optimum. See the [official CP-SAT status documentation](https://developers.google.com/optimization/cp/cp_solver).

The initial total search budget per decision is 0.1 deterministic-time units across at most three reranking rounds, one worker, a fixed random seed, sorted inputs and a pinned solver version. The [workflow's two-second end-to-end response budget](workflow.md#in-flight-rerouting-and-latency) includes checks and acceptance, not just optimization. Initially reserve at least 0.5 seconds for those stages and cap the pure solver worker at the lesser of 1.5 seconds or the remaining deadline. Its watchdog includes model construction, all rounds and candidate scoring and can terminate the worker without mutating `World`. If the deterministic budget or watchdog expires, use only an independently validated incumbent or landing-only fallback. The sim remains held if neither exists. Search settings are based on the [CP-SAT parameter definitions](https://github.com/google/or-tools/blob/stable/ortools/sat/sat_parameters.proto), and performance must be measured before changing these initial budgets.

Deterministic budget completion is replayable under recorded inputs and versions; wall-watchdog termination can vary across machines and is labelled accordingly. Record elapsed time, status and stop reason. A limit on solution count alone does not bound time to the first solution or construction cost. LLM calls have a separate coordinator timeout and run asynchronously while simulation time advances. They do not consume the fast route-response budget or keep the fleet held.

Cap optimization candidates at 20 task attempts for the initial implementation, preserving locked work, required confirmation needs and a representative from each highest-urgency unresolved group before deterministic marginal-value selection. Report every omitted candidate and reason; omission is not evidence of low importance. Landing candidates include the mandatory projections and occupied-site accounting described above. If required work alone exceeds the cap, expose `PlanningBlocked` and use a validated continuation while the operator sees the limitation.

Tiny exact enumeration checks both multi-drone assignment and route ordering, task sharing, landing choices and objective tiers. It tests feasibility and quantifies optimization gaps; a nonzero gap does not automatically mandate replacing the backend. Set acceptance targets from measured latency, feasibility and objective quality. Do not claim that independent per-drone searches solve shared-task allocation or that a complete exact replacement takes a fixed small number of lines.

The baseline optimizes one sortie per available drone. It does not value the next flight's start location or charging opportunity, so its four-hour performance is explicitly myopic. The evaluation baseline tests that limitation. A later two-sortie implementation must model both landing time and position rather than adding an unexplained perch-distance reward.

## Example sortie

This arithmetic example is illustrative rather than solver output. Each stop has an exact registered offset and the landing has a circuit ID. Assume fresh confirmed charging evidence for that endpoint through arrival, no known landing hazard, and no conflicting occupancy.

```text
DRONE_2 starts on LINE_025 at T+00:00, 60 km/h, 30 min battery
  6 km, 6 min -> LINE_031 clearance task, dwell 3 min
  4 km, 4 min -> LINE_024 voltage task, dwell 1 min
  5 km, 5 min -> LINE_029 temperature task, dwell 3 min
  7 km, 7 min -> exact eligible landing on LINE_027
  total flight and dwell = 29 min; landing T+00:29; charged T+03:29
```

The LINE_024 task confirms a reported state or discriminates a contradiction; it does not presume that the protection event occurred there. The thermal task is useful only with its recorded excitation conditions. At 40 km/h, the same distances and dwells require 40 minutes. Re-optimize task subsets and landing choices under the full urgency and feasibility policy; dropping a single nominally lowest-priority stop is not guaranteed to suffice.
