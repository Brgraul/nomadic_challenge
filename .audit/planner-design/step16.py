from edit import section,swap,record
section('## Power flow ground truth','## Sensing', '''## Power flow ground truth and scenarios

The synthetic electrical network is an answer key for the exercise, not a reconstruction of Bavaria's actual electrical connectivity. A shared mapped substation or a mixed-voltage tag does not prove a busbar connection or transformer. Independent sources per mapped component, station interconnections, ratings and load assignments below are explicit scenario assumptions. They are public modeling assumptions; the realized injections, randomized impedances and fault locations are hidden.

Two named modes prevent a benchmark fixture from contradicting its own operator brief.

| Mode | Public inputs and hidden state | Purpose |
|---|---|---|
| `healthy_benchmark` | Counterfactual no-incident fixture. Its generated brief explicitly states every circuit is initially in service and removes SYN_REPORT_03's confirmed switching assertion; storm reports are marked unused synthetic background | Compare reduced-model reconstruction policies under a controlled operating point |
| `incident_demo` | Original six reports, all available at startup; LINE_024 initially switched off as reported; at least one local vegetation hazard, one qualified clean observation and one inconclusive thermal observation | Demonstrate evidence-driven diagnosis, explained revisions, operator actions and route adaptation |

The incident brief distinguishes the recorded protection event from LINE_024's precautionary switching. Their causal relationship is unknown. No scenario labels the protection event as a confirmed trip on LINE_024. The manager sees neither fault placement nor truth-based scores.

`truth.build(network, scenario_config, seed)` constructs the explicit circuit registry described in the electrical reduction, then builds the AC network.

| Element | Rule |
|---|---|
| Buses and transformers | One bus per mapped node and voltage level. Synthetic mixed-voltage connections use explicit 380/110 kV transformers with recorded rating, tap, impedance and phase shift. The nominal reduction assumes zero phase shift; deviations require a model-discrepancy case. |
| Circuits | One physical AC branch per registry circuit, preserving its voltage and ID. Do not multiply the source circuit count independently at every voltage. Catalogue impedances times length vary by seeded factors within plus or minus 10%. |
| Supply | Choose the highest-voltage boundary endpoint per initial component, breaking ties by node ID, as its declared external source. If no boundary exists, use a declared substation source; COMPONENT_07 uses SUBSTATION_03. Reject source-less initial components. Other boundaries have bounded seeded P and Q exchanges. |
| Loads and generation | Seeded 110 kV substation and terminal loads of 10 to 60 MW at power factor 0.95; selected substations receive 5 to 40 MW of renewable generation. Junctions and intermediate nodes have no external injection. |
| Operating islands | Recompute after switching. An island without a declared grid-forming source is de-energized and recorded explicitly; do not create a hidden replacement slack to conceal an outage. Balance and solve each energized island. |
| Validation | Require finite results and convergence for energized buses, valid power balance, explicit unsupplied islands, and line and transformer ratings. The healthy fixture requires 0.95 to 1.05 pu voltage and line and transformer loading below 60%. Incident fixtures declare their expected violations; they do not silently rescale an intended fault away. |
| Bounded construction | Try at most 100 seeded operating points, adjusting loads, generation and boundary exchanges within their declared bounds together. Validate initial charging capability at the chosen starting circuits. If none satisfies the fixture constraints, return `ScenarioInfeasible` with diagnostics. Reducing loads alone is not a guaranteed repair. |

`TruthState` stores scenario and epoch IDs; per-circuit P and Q at both terminals, both terminal currents, losses and rating; per-voltage-bus voltage; explicit transformer results; source and island status; local fields; and scheduled incidents. It implements the `Physics` protocol. Only the sim, evaluator and truth package may import it. Keep the banned-import lint rule and also test that public snapshots, tools, logs, diagnostics and command results contain no hidden scenario fields. Import restrictions alone do not prove data-flow isolation.

### Local incident fixtures

A `LocalField` is indexed by circuit and along-line interval and contains clearance, environmental temperature, thermal defect excess, and motion amplitude during qualifying gust windows. Its values and event times are scenario inputs, separate from the AC steady-state calculation. A hot-joint excess may use a declared `k * I_squared` relation; absence of excess at low current cannot exclude the defect. Environmental temperature and retained heat remain present without load.

The required incident fixture places vegetation on a clipped interval of LINE_058 with minimum clearance 0.4 m. Normal comparison intervals have minimum clearance 2 m. It includes a hot-joint candidate on LINE_029 with 15 degrees Celsius excess when circuit current exceeds the synthetic 20 A excitation threshold, plus a low-load reading whose excitation flag makes it inconclusive. Motion fixtures use 0.30 m abnormal and 0.05 m comparison amplitudes during recorded gust windows. Local threshold examples are clearance below 0.5 m, thermal excess above 10 degrees Celsius under adequate load, and motion above 0.20 m under a qualifying gust. All are exercise policies, not operational standards.

Apply the local Gaussian errors and visibility flags from the sensing section. Derive threshold-event likelihoods from those distributions and field conditions, rather than assuming perfect detection inside the radius. A separate correlated-error fixture tests repeat observations. A scripted public switching report followed by incompatible qualified voltage evidence exercises deterministic invalidation and the manager's explained response. The incident integration test must include positive, qualified negative, inconclusive and contradictory evidence, even if the full fault catalogue remains deferred.

Within one operating epoch, injections and local fields are steady except for explicitly scheduled condition windows. Physical switching rebuilds and validates a new hidden state. Its public consequences trigger a new accepted operating epoch when observed; archived readings remain available as history but cannot be pooled into a single steady-state fit without an explicit compatibility decision. Every excluded or reassigned observation has a recorded disposition.
''')
section('## Minimum sampling','## Evaluation', '''## Minimum sampling

For the nominal all-in-service reduced model defined above, the dataset has 59 mapped nodes, 64 segments and seven components. There are 24 declared zero-injection nodes, comprising 20 junctions and four intermediate nodes. The remaining 35 nodes comprise 15 substations, 19 boundary endpoints and one terminal. The mapped graph has 12 independent cycles.

Reference-angle elimination leaves 52 angle unknowns. A row-normalized rank calculation with tolerance `1e-9` gives rank 24 for the zero-injection constraints and rank 52 after adding all aggregate flow rows. Selecting independent real rows reaches rank 52 with 28 readings. Thus 28 is the achievable algebraic minimum for this nominal reduced model, not a guarantee of usable uncertainty, bad-data detection or physical-network observability. The minimum by component is 20 for COMPONENT_01, two each for COMPONENT_02 and COMPONENT_03, and one each for COMPONENT_04 through COMPONENT_07. No aggregate flow is pinned by the zero-injection rows alone.

The implementation must make this calculation reproducible with `estimation.observability()` and a fixture test, including catalogue types, circuit allocation, topology, row scaling and rank tolerance. Do not retain the old greedy coverage curve without recomputing it for this exact model and selection rule. Incident-mode switching changes the matrix, references and minimum; it must be recalculated rather than reusing 28.

A rough budget of four current stops per 30-minute sortie would allow about 40 stops across five drones with two sorties. It omits location-specific travel, landing evidence and confirmation work. Second departures occur only after first landing plus 180 minutes, so each minute spent on the first flight can reduce the second flight's remaining window. Only the headless benchmark can establish achieved coverage for a particular policy. Its failure does not prove that all policies must fail.
''')
section('## Evaluation','## Optimization model and objective', '''## Evaluation

The evaluator alone compares public estimates and recommendations with hidden scenario state. It runs after each scored reading batch and records final unresolved work at T+04:00. Scores are stored outside `World` and never appear in agent tools, public event payloads or diagnostic decisions.

| Metric | Meaning |
|---|---|
| Flow error | Aggregate from-terminal P MAE, worst error and share within `max(5% of abs(true flow), 1 MW)` over the stated current-epoch target segments |
| Observability and detectability | Observable aggregate share, critical sets and share of real readings whose bad-data checks are actually testable |
| Conditional calibration | Empirical coverage of conditional intervals on matching-model fixtures; do not assume exactly 95% on a biased or correlated single realization |
| Discrepancy-aware calibration | Coverage and width of published intervals on held-out AC and parameter-mismatch fixtures; count unsupported intervals explicitly |
| Diagnosis and actions | Supported defect localization within sensor scope, false clearance, inconclusive-result handling, recommendation timing and explanation evidence correctness |
| Operational cost | Readings and repeats, flight and dwell time, charge timing, landing failures, unassigned urgent work and deadline compliance |
| Optimization quality | Feasible incumbent score, objective bounds or enumerated optimum on tiny cases, search budget, fallback frequency and surrogate versus joint information score |

The planned command `uv run python -m grid_ops.evaluation --scenario healthy_benchmark --policy greedy --seed 0` runs a policy to T+04:00 and writes results by evidence batch and sim time. It is a future implementation interface, not a command available today. `random` uses seeded canonical candidates; `greedy` uses deterministic task refresh and the same route constraints without an LLM; `agents` uses the complete manager/planner workflow. Incident scenarios also score hazard decisions and explanations.

Use paired scenario seeds across policies and report distributions over a declared seed set, not only one favorable run. Key sensor noise by scenario seed, circuit or canonical local site, measurement kind, operating epoch and repeat index. Drawing one global random stream in visit order would give different sites different noise across policies. Equal observation keys share noise; different sites or times are not falsely described as identical measurements. Retain LLM inputs and structured outputs for replay; the seed alone does not make live LLM calls deterministic.

Compare the one-sortie policy with at least a small two-sortie lookahead baseline before making four-hour coverage claims. All policies share initialization, landing policy, task-cap rules, sensor model and deadline; report any mode-specific differences. The benchmark evaluates a policy, not the impossibility of solving the general problem.

## Agent tools

All tools are read-only and pinned to the current `DecisionSnapshot`; results include its revision. Agents return structured proposals instead of mutating the world through tools.

| Agent | Tools |
|---|---|
| Manager | `describe_world`, `describe_segment`, `segments_in_report`, `read_evidence`, `grid_diagnostics`, `test_hypothesis`, `latest_plan`, `planning_limits` |
| Planner | `describe_segment`, `segments_in_report`, `segments_near`, `sample_points`, `drone_status`, `coverage_gaps`, `observability_gaps`, `preview_plan` |

The manager returns explained revisions, candidate model changes and recommendations. The planner returns measurement needs and exact sites. Previews show feasibility, score assumptions and unassigned reasons but authorize no dispatch. The coordinator validates the final proposal and submits `CommitDecision`; neither agent can call `SetInService`, inspect truth, overwrite direct observations or release the hold.
''')
section('## How good the optimization will be','## Example sortie', '''## Solver guarantees and search budget

Trim completed work and invalid endpoints from the current plan, preserve locked measurements, and validate the resulting continuation. If feasible, retain it as an incumbent and pass its assignments as solver hints. Independently score every candidate with current groups and the same objective. Accept a new feasible candidate only if it is at least as good as that still-valid incumbent. A stale or infeasible old plan creates no such guarantee.

CP-SAT may return `OPTIMAL`, `FEASIBLE`, `INFEASIBLE`, `MODEL_INVALID` or `UNKNOWN`. Only the first two provide a candidate for independent validation; `OPTIMAL` proves optimality only for that finite, discretized, surrogate model. If a retained incumbent is used after `UNKNOWN` or timeout, report that it is a validated fallback, not a newly solved optimum. See the [official CP-SAT status documentation](https://developers.google.com/optimization/cp/cp_solver).

The initial total search budget per decision is 0.1 deterministic-time units across at most three reranking rounds, one worker, a fixed random seed, sorted inputs and a pinned solver version. A two-second wall watchdog includes model construction, all rounds and candidate scoring; it runs the pure solver in an owned worker process that can be terminated without mutating `World`. If the deterministic budget or watchdog expires, use only an independently validated incumbent or landing-only fallback. The sim remains held if neither exists. Search settings are based on the [CP-SAT parameter definitions](https://github.com/google/or-tools/blob/stable/ortools/sat/sat_parameters.proto), and performance must be measured before changing these initial budgets.

Deterministic budget completion is replayable under recorded inputs and versions; wall-watchdog termination can vary across machines and is labelled accordingly. Record elapsed time, status and stop reason. A limit on solution count alone does not bound time to the first solution or construction cost. LLM latency is separately bounded by the coordinator timeout and does not advance simulation time.

Cap optimization candidates at 20 task attempts for the initial implementation, preserving locked work, required confirmation needs and a representative from each highest-urgency unresolved group before deterministic marginal-value selection. Report every omitted candidate and reason; omission is not evidence of low importance. Landing candidates include the mandatory projections and occupied-site accounting described above. If required work alone exceeds the cap, expose `PlanningBlocked` and use a validated continuation while the operator sees the limitation.

Tiny exact enumeration checks both multi-drone assignment and route ordering, task sharing, landing choices and objective tiers. It tests feasibility and quantifies optimization gaps; a nonzero gap does not automatically mandate replacing the backend. Set acceptance targets from measured latency, feasibility and objective quality. Do not claim that independent per-drone searches solve shared-task allocation or that a complete exact replacement takes a fixed small number of lines.

The baseline optimizes one sortie per available drone. It does not value the next flight's start location or charging opportunity, so its four-hour performance is explicitly myopic. The evaluation baseline tests that limitation. A later two-sortie implementation must model both landing time and position rather than adding an unexplained perch-distance reward.
''')
section('## Example sortie','## Code changes', '''## Example sortie

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
''')
section('## Code changes',None, '''## Implementation scope and acceptance scenarios

This remains a design, not implemented functionality. The implementation introduces the circuit registry and geometry helpers in `grid.py`; scenario physics and local fields in `truth/`; public estimation and hypothesis tests in `estimation.py`; scoped condition assessment in `assessment.py`; candidate refresh, benefit groups, CP-SAT planning and independent validation in `planning.py`; domain histories and exact state transitions in `world.py`; versioned decisions and fallback handling in `sim.py`; and snapshot-pinned agents in `agents.py`. The UI shows each hypothesis revision beside its latest finding, update reason, evidence and unresolved limits. Evaluation stays separate. Add OR-Tools and the test tooling as dependencies when implementing these modules.

The implementation acceptance suite must cover these behaviors. These are required future tests, not claims that the current application passes them.

| Scenario | Required outcome |
|---|---|
| New current reading, unchanged hypotheses | Covariance, marginal values and deterministic candidates refresh; routes are reconsidered without an unnecessary planner LLM call |
| Local positive, qualified negative and low-excitation negative | Supported scope, actual coverage and inconclusive evidence remain distinct; each changed hypothesis has an LLM finding and causal explanation |
| Explained revision missing, duplicated or stale | Reject missing or stale evidence and explanation records; identical retries do not duplicate revisions or side effects |
| Known switching or qualified dead voltage contradicts a live belief | Invalidate affected chargeability before routing; update public models only through observable evidence and explained accepted assumptions |
| Prior-only and minimally observable estimates | Report real observability and bad-data detectability honestly; empty suspect lists do not imply successful validation |
| Extra reading in an unrelated component | Critical measurements elsewhere remain untestable |
| Wrong topology, shared sensor bias or AC/DC mismatch | Separate model inconsistency from sensor suspicion; retain discrepancy and correlation limits in uncertainty |
| Co-located tasks and simultaneous reading/charge events | Process all same-time transitions before publishing the snapshot, with no division by zero or duplicated assignments |
| Operator edit during a manager turn | Old revision cannot commit or release the new cycle |
| Measuring drone rerouted or its hypothesis retired | Preserve the locked attempt, remaining dwell and unique ownership; change only the executable suffix |
| Later sortie fails acceptance | No earlier drone, hypothesis or task is partially mutated |
| Near-line drone with sparse landing samples | Include an exact eligible projection and avoid false discretization infeasibility |
| Occupied or expired landing site and Ready drone with no useful work | Enforce occupancy and evidence validity; permit waiting without an artificial recharge |
| Solver timeout, invalid model or coordinator death | Bounded failure, validated continuation or landing-only fallback, otherwise remain held with a specific cause |
| Start at T+03:58 with a 30-minute battery | Reject any route whose measurements or landing exceed T+04:00 |
| Overlapping tasks or several hypotheses sharing one observation | Unique benefits cannot be multiplied by task or hypothesis splitting; strict urgency order remains intact |
| COMPONENT_07 and an infeasible operating seed | Use the declared SUBSTATION_03 source; bound scenario construction and return an explicit failure when necessary |
| Healthy versus incident modes | Distinct consistent briefs, original precautionary switching in incident mode, local hazards and explanation adaptation actually exercised |
| Truth leakage probe | No hidden injections, local fault coordinates, latent noise scales or evaluator scores in public snapshots or tools |
| Nominal reduced-model rank fixture | Reproduce ranks 24 and 52 and the 28-reading minimum; recalculate after topology changes |
| Tiny multi-drone exact instances | Check full allocation, ordering and landings against enumeration and report objective gaps honestly |
| Matching-model noiseless estimation | State exact-constraint or compatible-prior assumptions and recover the specified DC state; mismatched priors are tested as a bias case |
| Paired policy seeds and replay | Observation-keyed noise, recorded LLM outputs and solver budgets reproduce the applicable run; watchdog variability is labelled |

## Deferred extensions

Bayesian root-cause probabilities and exact entropy rewards require calibrated hypothesis likelihoods beyond the scoped synthetic local tests. Full AC state estimation requires a richer observable model. Continuous load profiles require time-aware estimation beyond explicit epochs. Two-sortie optimization requires joint flight and charge scheduling. A larger fault catalogue can follow the required incident fixture. None of these extensions may be represented as already available or used to justify a global grid-health claim.
''')
swap('''- **The problem is small.** Charging takes 3 hours, and the scenario window is 4 hours. Each of the 5 drones gets at most 2 flights. The solver finishes in milliseconds.''','''- **The planning horizon is bounded.** Charging takes 3 hours, and the scenario window is 4 hours. Each of the 5 drones gets at most 2 flights. Solver work has explicit budgets and fallbacks; no millisecond or optimality guarantee is assumed.''')
swap('''- **The planner must use what the operator believes.** It chooses perches from `world.belief`, never from ground truth.''','''- **The planner uses public evidence and assumptions.** Landing eligibility uses circuit-specific charging knowledge, hazards, expiry and a declared risk policy, never ground truth.''')
swap('''and it must end perched on a live line.''','''and a dispatched drone must reach an exact eligible recharge landing before its battery and the scenario deadline expire.''')
swap('''`World` holds only what the operator could know. Agent tools read `sim.snapshot`, which is a deep copy of `World`. So ground truth must not be in `World` at all. Today `World.energized` sits next to `belief` and is copied into every snapshot. In the design it moves to `Simulation._truth`, which is never copied.''','''`World` holds only public observations, declared assumptions and derived results. Agent tools read a pinned immutable `DecisionSnapshot`, while the UI can read the latest public snapshot. The current implementation's `World.energized` moves into hidden `Simulation._truth` and is never copied into either public view.''')
swap('''Flying --> Charging: reached perch on a live line''','''Flying --> Charging: reached chargeable landing circuit''')
swap('''Flying --> Stranded: reached perch on a dead line''','''Flying --> Stranded: landing cannot supply charging power''')
swap('''Charging --> Stranded: line lost power''','''Charging --> Stranded: charging capability lost''')
swap('''Stranded --> Charging: line re-energized''','''Stranded --> Charging: charging capability restored''')
swap('''arrives: float              # departed + distance(origin, route.target) / speed''','''arrives: float              # departed + conservative rounded leg seconds / 60''')
swap('''    Co->>Co: Test H1 and refresh deterministic tasks''','''    Co->>Co: Verify manager test references and refresh deterministic tasks''')
record(16,16,'Correct scenario construction, bounded CP-SAT guarantees, evaluation claims and implementation acceptance scope','docs/planner-design.md#solver-guarantees-and-search-budget')
