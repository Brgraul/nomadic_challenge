# Electrical model and scenarios

[Design index](../planner-design.md). Read for synthetic truth, incident fixtures, circuit allocation, DC reduction, topology assumptions or the minimum sampling calculation.

Related contracts: [Observation model](geometry-and-sensing.md), [Reduced-model inference](estimation.md).

## Power flow ground truth and scenarios

The synthetic electrical network is an answer key for the exercise, not a reconstruction of Bavaria's actual electrical connectivity. A shared mapped substation or a mixed-voltage tag does not prove a busbar connection or transformer. Independent sources per mapped component, station interconnections, ratings and load assignments below are explicit scenario assumptions. They are public modeling assumptions; the realized injections, randomized impedances and fault locations are hidden.

Two named modes prevent a benchmark fixture from contradicting its own operator brief.

| Mode | Public inputs and hidden state | Purpose |
|---|---|---|
| `healthy_benchmark` | Counterfactual no-incident fixture. Its generated brief explicitly states every circuit is initially in service and removes SYN_REPORT_03's confirmed switching assertion; storm reports are marked unused synthetic background | Compare reduced-model reconstruction policies under a controlled operating point |
| `incident_demo` | Original six reports, all available at startup; LINE_024 initially switched off as reported; at least one local vegetation hazard, one qualified clean observation and one inconclusive thermal observation | Demonstrate evidence-driven diagnosis, explained revisions, operator actions and route adaptation |

The incident brief distinguishes the recorded protection event from LINE_024's precautionary switching. Their causal relationship is unknown. No scenario labels the protection event as a confirmed trip on LINE_024. The manager sees neither fault placement nor truth-based scores.

Selecting `incident_demo` enables root-cause investigation and its diagnostic stopping rule; it does not enable H0 or require global flow reconstruction. `healthy_benchmark` explicitly selects reconstruction as a separate objective. Its sampling targets and completion thresholds must not become incident measurement quotas.

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

## Local incident fixtures

A `LocalField` is indexed by circuit and along-line interval and contains clearance, environmental temperature, thermal defect excess, and motion amplitude during qualifying gust windows. Its values and event times are scenario inputs, separate from the AC steady-state calculation. A hot-joint excess may use a declared `k * I_squared` relation; absence of excess at low current cannot exclude the defect. Environmental temperature and retained heat remain present without load.

The required incident fixture places vegetation on a clipped interval of LINE_058 with minimum clearance 0.4 m. Normal comparison intervals have minimum clearance 2 m. It includes a hot-joint candidate on LINE_029 with 15 degrees Celsius excess when circuit current exceeds the synthetic 20 A excitation threshold, plus a low-load reading whose excitation flag makes it inconclusive. Motion fixtures use 0.30 m abnormal and 0.05 m comparison amplitudes during recorded gust windows. Local threshold examples are clearance below 0.5 m, thermal excess above 10 degrees Celsius under adequate load, and motion above 0.20 m under a qualifying gust. All are exercise policies, not operational standards.

Apply the local Gaussian errors and visibility flags from the sensing section. Derive threshold-event likelihoods from those distributions and field conditions, rather than assuming perfect detection inside the radius. A separate correlated-error fixture tests repeat observations. A scripted public switching report followed by incompatible qualified voltage evidence exercises deterministic invalidation and the manager's explained response. The incident integration test must include positive, qualified negative, inconclusive and contradictory evidence, even if the full fault catalogue remains deferred.

Within one operating epoch, injections and local fields are steady except for explicitly scheduled condition windows. Physical switching rebuilds and validates a new hidden state. Its public consequences trigger a new accepted operating epoch when observed; archived readings remain available as history but cannot be pooled into a single steady-state fit without an explicit compatibility decision. Every excluded or reassigned observation has a recorded disposition.

## Explicit electrical reduction

Build a public synthetic circuit registry first. Use every ordered `voltages_kv` list entry as one synthetic circuit, including repeated voltages. If the source circuit count exceeds the number of entries, distribute additional circuits round-robin in listed order; a scalar voltage with a known count gets that many circuits. A missing count adds none beyond the listed entries. Reject contradictory counts smaller than the entry count for fixture review. This is a reproducible scenario assignment, not recovered asset connectivity. Store every circuit's nominal voltage, endpoints, type, reactance and current rating.

Use a 100 MVA base. For circuit `c`, convert catalogue reactance to per-unit with its voltage base, then set `b_c = 100 / x_pu_c` MW per radian. Its lossless prediction is `P_c = b_c * (theta_from - theta_to)`. For a segment aggregate, sum active circuit coefficients. Build `H_f` from those aggregate rows and build the nodal injection matrix from their signed incidence. Nodal net injection is generation minus load, so generation is positive and consumption negative. The reduction collapses voltage-specific buses at the same mapped node, assumes ideal lossless transformers with unit per-unit taps and zero phase shift, and ignores their impedance in the reduced graph. The separate AC model retains explicit transformers and their limits. Quantify the resulting discrepancy rather than treating the collapse as exact electrical equivalence.

Known out-of-service circuits have zero through-flow in the candidate model. Recompute connectivity and reference angles after topology changes. A de-energization observation alone does not uniquely locate an open switch, so it triggers candidate-model review rather than silently deleting a particular branch. Keep incompatible historical readings out of the current steady-state fit by explicit epoch disposition. Unknown internal station connections and external ties remain scenario assumptions as explained in the [dataset limitations](../../data/README.md).

`observability(model, accepted_readings)` uses real aggregate measurement rows plus exact reduced zero-injection constraints and a declared numerical rank tolerance. It excludes pseudo-measurements. Repeated rows can reduce independent noise but do not add structural rank. Physical circuit loading and voltage observability require their own models and measurements.

## Minimum sampling

For the nominal all-in-service reduced model defined above, the dataset has 59 mapped nodes, 64 segments and seven components. There are 24 declared zero-injection nodes, comprising 20 junctions and four intermediate nodes. The remaining 35 nodes comprise 15 substations, 19 boundary endpoints and one terminal. The mapped graph has 12 independent cycles.

Reference-angle elimination leaves 52 angle unknowns. A row-normalized rank calculation with tolerance `1e-9` gives rank 24 for the zero-injection constraints and rank 52 after adding all aggregate flow rows. Selecting independent real rows reaches rank 52 with 28 readings. Thus 28 is the achievable algebraic minimum for this nominal reduced model, not a guarantee of usable uncertainty, bad-data detection or physical-network observability. The minimum by component is 20 for COMPONENT_01, two each for COMPONENT_02 and COMPONENT_03, and one each for COMPONENT_04 through COMPONENT_07. No aggregate flow is pinned by the zero-injection rows alone.

The implementation must make this calculation reproducible with `estimation.observability()` and a fixture test, including catalogue types, circuit allocation, topology, row scaling and rank tolerance. Do not retain the old greedy coverage curve without recomputing it for this exact model and selection rule. Incident-mode switching changes the matrix, references and minimum; it must be recalculated rather than reusing 28. This is a reconstruction rank result, not a minimum number of measurements required to diagnose an outage.

A rough budget of four current stops per 30-minute sortie would allow about 40 stops across five drones with two sorties. It omits location-specific travel, landing evidence and confirmation work. Second departures occur only after first landing plus 180 minutes, so each minute spent on the first flight can reduce the second flight's remaining window. Only the headless benchmark can establish achieved coverage for a particular policy. Its failure does not prove that all policies must fail.
