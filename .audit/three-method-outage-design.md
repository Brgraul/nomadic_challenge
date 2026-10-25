# Candidate A. One authoritative operation itinerary

## Existing ownership and flow

Simulation is the single World writer. World transitions execute exact timestamps, agents see pinned deep copies, proposals validate before acceptance, and routes independently reconstruct every submitted route. Preserve those boundaries.

Startup currently calls truth.build with ScenarioConfig.seed, publishes that seed and LINE_024 switching knowledge, seeds handshakes unconditionally, and refreshes aggregate-MW estimation. Measurement needs are generated from accepted prediction.measurement fields. Planning CP-SAT chooses task stop sequences and one final charging perch. routes.validate_sorties computes straight-flight travel plus airborne dwell. World._arrive turns a task stop into stationary Measuring, and the final perch into automatic Charging or Stranded. Charge completion restores full battery. Those are production assumptions requiring replacement together.

Agents currently receive Simulation, build_toolbox(snapshot), then _record submits RecordProposal. Coordinator invokes manager_fn(sim) and commits the recorded proposal. Move recording and all Simulator imports out of agents, preserving proposal acceptance in coordinator.

## Caller usage and data shape

context = InvestigationContext(snapshot, nominal_topology, sensor_capabilities)
result = run_llm_turn(context)
coordinator records result.output and accepts through existing CommitDecision.
compiled = compile_requests(context, accepted_requests)
plan = plan_sorties(PlanningInput(public_world, tasks, energy_model, belief))
validated = validate_sorties(context, plan.sorties)
executor advances validated ScheduledStep records.

The organizing structure is a frozen discriminated union of scheduled operations. It is shared by optimizer output, validator result, worker serialization and executor state. No layer reconstructs an unrecorded wait or contact cost.

sensing.py owns MeasurementMethod = lidar | tdr | contact_current, capability catalog, immutable target/acquisition types, quality and payload records, and request compiler. Separate public method definitions from private acquisition implementation.

Request boundary models use a Pydantic discriminated union. Every request has stable ID, source_revision, hypothesis/evidence references, rationale, replacements and expected outcomes. LidarRequest targets CorridorSpan; CurrentRequest targets circuit, phase and allowed interval; TdrRequest targets named access/coupling/return path. Accepted request records live in World, with active/retired lifecycle and history. Predictions describe expectations only. Atomic acceptance applies hypotheses and requests together and rejects stale source_revision, ID reuse with changed content, unknown provenance and invalid replacement.

MeasurementTask carries request_ids, acquisition, alternative_group_id, status and repeat link. Lidar task alternatives select entry/exit direction using public geometry. Current and TDR alternatives select valid public contact locations. No compiler reads hidden obstacles or true fault coordinates. Acquisitions are counted by stable attempt ID, irrespective of progressive LiDAR chunks.

ScheduledStep(start_s, end_s, operation) carries one of Transit(path), Scan(trajectory, acquisition), Land(site), GroundSample(acquisition), Charge(site), Takeoff(site), Wait(location/state). SortieRequest carries steps, final_perch, source revision, model revisions and evidence dependencies. Every action has contiguous time and explicit state transition. Planned Scan and Transit positions vary with time. GroundSample requires landed contact identity. Charge requires successful observed capability. Charge duration zero is normal.

energy.py owns scaled normalized integer EnergyModel, BatteryState and operation integration. Validator replays from actual remaining energy and applies the same pure cost law as executor. Costs round up, charge gains down, capacity clamps. Positive takeoff/landing and ground costs prevent unlimited free hops. Use exact accumulator/remainder or integrate from operation start so tick subdivision cannot change rounding. Observed charging is the only positive source.

World executor state can be compactly Perched(site) or Executing(step_index, itinerary, step_start_energy). Current Flying/Measuring/Charging UI phases become views of the active operation rather than alternate route schemas. Interrupted scans preserve immutable acquired intervals and remaining unexecuted span. Recovery produces another ordinary itinerary and passes independent validation. Dead charge capability does not alter mechanical perch suitability.

## Module migration by stage

P1 adds standalone contracts, private causal episode and sample APIs without turning old runtime onto half-migrated sensors. truth/episode.py privately owns EpisodeSpec and replay manifest. Episode builder chooses fault, applies protective opening, solves post-trip AC and records affected supply. Public fixture identifier and policy seed remain independent of private episode/noise seeds. Public premeasurement views are equal for paired hidden worlds with the same observed starting evidence. Current uses connected conductor subspan; balanced-phase model is declared. TDR follows post-protection paths, not hidden fault beyond an opening. Shared calibration error keys follow physical scope, not attempt ID.

P2 replaces production request creation, routes, planner worker protocol, validator, executor, recovery and consumers in one unit. Remove active aggregate-P estimator calls; do not alias legacy sensors under new names. Adapt existing CP-SAT stop assignment internally to visit alternatives but emit canonical steps and validate them. Contact visits may contain multiple samples, optional charge then takeoff. Recovery candidates use mechanical suitability separately from charge evidence. Current zero never establishes de-energization or TDR isolation.

P3 outage.py takes only public nominal model, candidate family and immutable evidence. It computes versioned likelihoods over candidate/nuisance combinations, retains equivalence classes, tracks omitted candidates/model checks and produces supported facts with evidence IDs. Correlated repeats cannot be multiplied as independent likelihoods. Candidate caps produce model-limited uncertainty rather than certainty. Information value uses public predictions, deduplicates attempts and combines acquisition/energy/elapsed costs. Stop states are supported, ambiguous, model-mismatched, no-safe-experiment and budget-exhausted.

P4 evaluator owns privileged comparison and private manifests; policies receive identical public context. Explicitly label actual LLM versus scripted mode. Quality and operational cost metrics count acquisition attempts and integrated energy rather than JSON chunks. UI derives method options from catalog, shows real trajectory/coverage/request/evidence IDs, energy and optional charging, and keeps current/service/charge claims separate. Remove obsolete estimator/consistency paths only after final caller search.

## Rejected shape

Keep old StopRequest and add a method discriminator plus a charge boolean. This is initially smaller but cannot represent moving scans, intermediate landed contacts, no-charge departure, partial charging, explicit waits or timed occupancy. A second executor beside old route execution would allow planner/validator/executor to disagree and preserve production legacy measurement authority. Reject that compatibility shape.

## Load-bearing verification

Independent validator tests must include forged method substitution, wrong conductor, missing return path, illegal airborne wait, omitted landing/takeoff energy, occupancy overlap, tick-split charging, stale revision and no safe recovery. Sensor tests need hand-computable LiDAR visibility and echo delay, branched TDR ambiguity, current position on opposite sides of a protection split and paired private-world isolation. End-to-end trace must link proposal request, accepted lifecycle, optimizer alternative, executed acquisition, evidence ID and energy ledger. No completion claim substitutes sensor metadata for actual evidence.
# Independent design verdict

Use candidate A as the implementation base. Graft candidate B's explicit residual outcome distribution, public candidate truncation checks, correlated-error nuisance treatment, and narrow policy context. Both candidates independently recommend one in-place Simulation/World migration. The competing new-runtime design is rejected because a parallel production path violates the plan and complete replacement discards mature command, hold, worker and recovery behavior.

| Criterion | Candidate A, 1–5 | Candidate B, 1–5 | Evidence |
| --- | --- | --- | --- |
| Private truth isolation | 5 | 5 | Both remove simulator access from policy interfaces and separate episode/noise seeds from public fixture and policy seeds. B explicitly includes evaluator score and labels in the capability boundary. |
| One runtime and one writer | 5 | 5 | Both retain Simulation as the sole live World writer and reject keeping a second active runtime. |
| Sensor honesty | 5 | 5 | Both retain TDR protection masking and invalid acquisitions, single-conductor current, actual LiDAR coverage and zero-current ambiguity. |
| Complete energy and route propagation | 5 | 4 | A provides the concrete discriminated ScheduledStep union and tick-invariant integration needed across optimizer, validator, worker, executor and recovery. B states the same requirement with less implementation detail. |
| Practical migration size | 4 | 4 | Both expose a large coupled P2 migration rather than disguising it as additive sensor aliases. A gives clearer record ownership; B adds useful inference boundaries. |

Candidate A wins on the route contract. Candidate B supplies the stronger inference contract. These are complementary and do not require another production runtime.

## Accepted concrete shape

`sensing.py` owns the method catalog, typed request targets, acquisitions, quality and payload records. Pydantic discriminated requests terminate at the coordinator acceptance boundary. Accepted immutable request records create tasks; predictions never create tasks.

`energy.py` owns normalized integer resources and pure operation integration. Route validation independently derives each operation's geometry and resource cost from authoritative public inputs. The executor applies the same energy law. Both account for contiguous waits, positive landing/takeoff costs, landed sampling drain, actual observed charge and capacity. Integrate from operation-start energy or carry exact remainders so tick subdivision cannot create or destroy energy.

`SortieRequest.steps` is the sole executable schedule. Transit, Scan, Land, GroundSample, Charge, Takeoff and Wait records carry actual durations and positions. A dead but mechanically suitable perch permits sampling and departure when energy allows. A charge-dependent continuation must tolerate failed charging by safely remaining perched. Ground contact and LiDAR motion cannot be encoded as the old airborne dwell stop.

`outage.py` reads public model candidates and immutable observed evidence only. Candidate generation is deterministic from nominal topology and declared public priors. Its broad residual has an outcome likelihood, and predictive-mismatch checks have a tested meaning. Repeats share nuisance errors. Invalid acquisitions and unobserved intervals do not supply negative evidence. Conditional agreement in a truncated pool cannot certify full reconstruction.

## Implementation and verification constraints

P1 may expose standalone contracts through the real driver while the production workflow remains old. P2 must replace every active route/request/schema consumer together and disable MW inference before publishing the new runtime. Do not ship permanent aliases that preserve old sensor payloads or prediction task authority.

Require independent scheduled-step replay tests for missing landing/takeoff, illegal wait state, method substitution, wrong conductor, unavailable return path, occupancy overlap, stale dependency and split-tick charge accounting. Sensor fixtures must analytically establish visibility and echo delay. Paired private episodes must yield equal public premeasurement views when observed starting evidence is equal.

For P3, test identical-evidence beliefs, duplicate ingestion and correlated repeats, branch equivalence, zero-current ambiguity, unrelated visible hazards and candidate omission. Noisy supported-fact confidence remains conditional until held-out calibration exists. A successful tiny noiseless case does not establish noisy calibration.

This is an architecture verdict only. No implementation, live application, real LLM transcript, performance budget or quality gate has been verified by this review.
