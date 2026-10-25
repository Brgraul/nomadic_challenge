# Implement drone-based outage reconstruction

Build an investigation that starts with a private, causally consistent outage and reveals evidence through LiDAR, TDR, and landed current sensing.
Let the LLM choose measurement methods and investigative targets. Let the optimizer assign drones, routes, contact visits, and optional charging.
Preserve battery across measurement landings. A landing never forces a recharge or resets the battery.
Deliver four dependent changes, P1 through P4. This document is a plan. The implementation does not exist yet.

## How to read this

One box is one unit of work. Every box names the evidence that checks it. Check a box only when its evidence exists.
The implementation contract, schema sketches, call paths, objectives, and rationale are in Appendices B through F.
The program uses `skills/poteto-mode/playbooks/autopilot-stack.md` from the installed pstack plugin if execution is requested. P1, P2, P3, and P4 form one dependency chain. The operator owns merging.
Tests alone are not sufficient verification. A PR is verified only when its unit, live, and perf boxes are all checked.
The user's later charging correction overrides the original exercise rule. Measurement landings permit departure on remaining energy. Three hours describes the proposed synthetic empty-to-full charging time, not a mandatory wait after a landing.

## Program checklist

### Arm the program

- [ ] Record the approved scope and this plan in standing orders when implementation starts. This planning request does not authorize starting an autonomous program, publishing PRs, or merging.
- [ ] Read the execution playbook and the architecture, sensor, energy, and verification contracts from the installed plugin and this document before implementation.
- [ ] Keep a local decision trail. Skip an hourly scheduled automation for this bounded implementation unless the user asks for continued unattended work. Send a status message when a decision, blocker, or verified milestone changes.

### Spawn owners

- [ ] Execute P1, then P2, then P3, then P4. Base each dependent change on its predecessor. Avoid concurrent edits to shared schemas.
- [ ] Give each implementation unit one writer. Delegate independent read-only verification when the applicable execution skill calls for it.
- [ ] Keep interaction review for P2, P3, and P4. The screenshots and video must show the real running application and its evidence, not the HTML prototype's fixed story.

### PR mechanics

- [ ] Record base and head revisions for every verification receipt. Keep the user's existing changes outside these diffs.
- [ ] Run `uv run ruff check src tests e2e` and `uv run pytest -q` at each coherent migration boundary. Update recordings and callers in the same change as their schema migration.
- [ ] If PR creation is requested during execution, follow the installed opening-a-pr and stack playbooks. Do not treat this document as permission to publish or merge.

### Verdict and merge

- [ ] Require the unit, live, and performance evidence for each unit at its current head. A test of the previous head is not a verdict for changed behavior.
- [ ] Run the ten live scenarios for each unit, with independent verification under the execution playbook. Keep every failed scenario visible until fixed.
- [ ] Deliver reviewable changes in P1 to P4 order. The operator merges. No deployment or automatic landing is part of this plan.

### Boot recipe

- [ ] Use an isolated checkout at the unit's recorded head and install dependencies with `uv sync --locked`.
- [ ] For headless live scenarios, run the proposed `uv run python scripts/verify_outage.py --case CASE --out OUTPUT_DIR`. This driver must invoke real simulation and policy interfaces, not mocks of sensing, energy, or optimization. Add the driver in P1 and extend it in later units.
- [ ] For the real UI, use `uv run streamlit run src/grid_ops/app.py --server.port 8501`. Drive the browser through the available computer-use tools after reading `/Users/pc/.agents/skills/computer-use/SKILL.md`. Use `e2e/drive.py` for repeatable AppTest assertions as well.
- [ ] Save JSON traces and assertion logs beside terminal screenshots for headless lanes and browser screenshots for UI lanes. Use `/tmp/three-method-PN/lane-N/`. Screenshots supplement the trace and assertions; they do not replace them.
- [ ] For latency, warm the existing planner worker, record cold startup separately, and time ingestion through validated publication. Keep LLM latency separate from fast route-response latency. Do not freeze simulation time to improve a latency result.

## Establish private causal episodes and contracts (P1)

**Depends on.** None.

**Files.**

- [ ] Add `src/grid_ops/sensing.py`, `src/grid_ops/energy.py`, `src/grid_ops/truth/episode.py`, and `scripts/verify_outage.py`.
- [ ] Change `src/grid_ops/scenario.py`, `src/grid_ops/truth/build.py`, `src/grid_ops/truth/state.py`, and `src/grid_ops/truth/local.py`.
- [ ] Extend `tests/test_isolation.py` and `tests/test_scenarios.py`. Add `tests/test_sensing.py` and `tests/test_energy.py`.

**Build.**

- [ ] Define the public three-method contracts and energy law in Appendix C. Keep this first unit callable and testable through the driver before switching the production runtime to them in P2.
- [ ] Construct a hidden event chain with a physical fault, protective switching, and loss of supply. Solve the resulting post-trip AC operating state. A steady-state power-flow solve is not a transient short-circuit model.
- [ ] Separate private episode and noise seeds from public fixture and policy seeds. Keep only an opaque run identifier in agent-facing data. Privileged replay manifests stay outside public snapshots and agent tools.
- [ ] Implement path-conditioned LiDAR samples, coupled TDR responses, and single-conductor RMS current samples. Preserve actual inspected intervals, uncertainty, invalid acquisitions, and correlated repeat errors.
- [ ] Define position-dependent current using the connected conductor subspan in the private post-trip network. Split modeled spans at protective devices and discontinuities as needed. Do not retain the old rule that every point reads the segment's from-terminal current. Declare a balanced-phase approximation if individual phase models are unavailable.
- [ ] Declare the TDR coupling profile. A specialized live-compatible profile is a hypothetical simulator capability requiring a return path. An isolated-only profile rejects a test without valid isolation evidence. Zero current is never that evidence.
- [ ] Implement persistent energy, landed measurement consumption, positive takeoff/landing costs, and optional partial charging. Do not connect landing to automatic full recharge.

**You see.**

- [ ] The headless driver produces exactly the three allowed sample shapes, a private evaluator answer key, and a separate public trace without the answer key or its seed.
- [ ] Two hidden episodes with the same public inputs and the same observed starting-perch evidence have equal pre-measurement public views. After sampling, differences have an identifiable observation or operational telemetry source.

**Verify, unit.** Tests alone are not sufficient verification. A PR is verified only when its unit, live, and perf boxes are all checked.

- [ ] Run `uv run pytest -q tests/test_isolation.py tests/test_scenarios.py tests/test_sensing.py tests/test_energy.py`. Assert no voltage/P/Q payloads, no other-conductor leakage, no clean negative from missing visibility, and no energy gain on landing.
- [ ] Check a known-distance echo against the propagation-delay equation, including uncertainty and branch ambiguity. Check LiDAR visibility on analytic geometry with hand-computable distances.

**Verify, live.** Tests alone are not sufficient verification. A PR is verified only when its unit, live, and perf boxes are all checked. Ten lanes on the configured `swarm workers` model at the PR head.

- [ ] Lane 1. Regression lane against trunk exercises hidden-state isolation. Save `/tmp/three-method-P1/lane-1/regression.png`. Pass when existing isolation behavior remains valid and the new causal episode stays private; record that trunk lacks the new sensors.
- [ ] Lane 2. Sample an energized unloaded conductor and a disconnected conductor. Save `/tmp/three-method-P1/lane-2/zero-current.png`. Pass when zero current does not become proof of de-energization.
- [ ] Lane 3. Scan a bent corridor with a narrow FOV. Save `/tmp/three-method-P1/lane-3/fov.png`. Pass when only the geometrically visible portion is observed.
- [ ] Lane 4. Scan behind an occluder. Save `/tmp/three-method-P1/lane-4/occlusion.png`. Pass when hidden intervals remain unknown.
- [ ] Lane 5. Test a known TDR discontinuity. Save `/tmp/three-method-P1/lane-5/echo.png`. Pass when distance uncertainty covers the fixture position under the declared model.
- [ ] Lane 6. Test a branched TDR fixture. Save `/tmp/three-method-P1/lane-6/branches.png`. Pass when multiple compatible locations survive.
- [ ] Lane 7. Attempt TDR without a return path. Save `/tmp/three-method-P1/lane-7/coupling.png`. Pass when the acquisition is rejected or invalid, never a clean result.
- [ ] Lane 8. Land and take off without charging. Save `/tmp/three-method-P1/lane-8/hop.png`. Pass when measured remaining energy decreases and is carried into the next flight.
- [ ] Lane 9. Charge for a partial interval on a capable perch. Save `/tmp/three-method-P1/lane-9/partial-charge.png`. Pass when only the modeled energy is added and capacity is respected.
- [ ] Lane 10. Replay the same private manifest and acquisition sequence. Save `/tmp/three-method-P1/lane-10/replay.png`. Pass when samples match while the private manifest remains absent from the policy interface.

**Verify, perf.** Tests alone are not sufficient verification. A PR is verified only when its unit, live, and perf boxes are all checked.

- [ ] Metric. Record episode construction time and per-acquisition time, separating AC construction from sample lookup and geometry work.
- [ ] Probe. Run cold and warm probes for the small analytic network and the full dataset, interleaving baseline and head where both implement the same operation.
- [ ] Baseline. Measure trunk first for existing initialization and current lookup. Record that trunk has no comparable TDR or trajectory LiDAR implementation.
- [ ] Rule. Proposed initial budgets are 30 seconds for full-network episode construction and 250 milliseconds for a bounded acquisition. These are acceptance targets to test, not measured performance claims. Record the configured geometry/sample caps.

**Review gate.** None. P1 is not review-gated because the new contracts are not yet the app's active interaction.

**Merge.**

- [ ] Record the exact-head verification verdict and operator disposition. Append P1 as the stack root if a stack is requested. The operator owns merging.

## Migrate requests, flight execution, and routing together (P2)

**Depends on.** P1.

**Files.**

- [ ] Change `src/grid_ops/hypotheses.py`, `src/grid_ops/agents.py`, `src/grid_ops/scripted_manager.py`, `src/grid_ops/manager_tools.py`, `src/grid_ops/proposals.py`, and `src/grid_ops/decisions.py`.
- [ ] Change `src/grid_ops/world.py`, `src/grid_ops/needs.py`, `src/grid_ops/routes.py`, `src/grid_ops/planning.py`, `src/grid_ops/fleet.py`, `src/grid_ops/resilience.py`, `src/grid_ops/sim.py`, and `src/grid_ops/coordinator.py`.
- [ ] Update `src/grid_ops/knowledge.py`, `src/grid_ops/assessment.py`, `src/grid_ops/app.py`, and their schema consumers in `src/grid_ops/views.py` and `src/grid_ops/map_view.py`.
- [ ] Migrate tests in `tests/test_manager.py`, `tests/test_findings.py`, `tests/test_needs.py`, `tests/test_decisions.py`, `tests/test_sorties.py`, `tests/test_landing.py`, `tests/test_planning.py`, `tests/test_coordinator.py`, and `tests/test_resilience.py`. Extend the verification driver and migrate recording producers under `scripts/`.

**Build.**

- [ ] Add explicit measurement requests to `ManagerOutput`. Remove measurement-driven task creation from predictions. Accept hypothesis revisions and requests atomically, with stable request IDs and retirement/replacement behavior.
- [ ] Give the LLM and scripted policies the same public investigation context. Move simulator submission out of `agents.run_llm_turn`; return a proposal to the coordinator. Do not hand policies a `Simulation` object.
- [ ] Compile accepted requests into method-specific task alternatives. The optimizer may choose a drone, pass direction, order, contact location within the authorized scope, and charge duration. It may not silently replace the requested method.
- [ ] Replace the terminal-landing-only route shape with the itinerary in Appendix C. Migrate execution, the optimizer worker protocol, independent validation, locked work, recovery, and UI consumers in the same change.
- [ ] Make a normal dead-line measurement perch usable for departure when energy and mechanical access permit. Separate contact suitability, energy state, and charge availability. A failed charging handshake does not itself strand a drone.
- [ ] Remove the automatic `battery_s = FLIGHT_S` reset on landing/charge completion. Only integrated observed charging may increase energy.
- [ ] Replace stationary LiDAR dwell with motion and progressive coverage. Preserve completed coverage after interruption. Define acquisition attempts independently of output chunks.
- [ ] Stop invoking aggregate-P estimation and MW-information scoring in the three-method runtime. Until P3 adds calibrated reconstruction, present evidence and explicitly unresolved states rather than invented estimates.

**You see.**

- [ ] A request for LiDAR becomes a moving scan. A request for current becomes an actual conductor landing and sample. A subsequent hop starts with the remaining battery and does not wait three hours.
- [ ] An actual LLM request, validated acceptance, optimizer assignment, energy ledger, executed reading, and returned evidence IDs appear in one trace. Missing model credentials visibly identify scripted mode or prevent a live-LLM run.

**Verify, unit.** Tests alone are not sufficient verification. A PR is verified only when its unit, live, and perf boxes are all checked.

- [ ] Run `uv run pytest -q tests/test_manager.py tests/test_needs.py tests/test_decisions.py tests/test_sorties.py tests/test_landing.py tests/test_planning.py tests/test_coordinator.py tests/test_resilience.py`. Extend assertions to resource continuity, optional charging, timed occupancy, stale proposals, and invalid method substitutions.
- [ ] Compare tiny route instances with exhaustive enumeration over the same discretized tasks and charge-duration options. Independently replay every chosen itinerary against geometry and the energy law.

**Verify, live.** Tests alone are not sufficient verification. A PR is verified only when its unit, live, and perf boxes are all checked. Ten lanes on the configured `swarm workers` model at the PR head.

- [ ] Lane 1. Regression lane against trunk follows the same public investigation inputs. Save `/tmp/three-method-P2/lane-1/regression.png`. Pass when evidence isolation and safe continuation remain; record that landed multi-hop sampling is new.
- [ ] Lane 2. Let an actual LLM choose a LiDAR interval and run it. Save `/tmp/three-method-P2/lane-2/llm-lidar.png`. Pass when the requested method and scope match the executed pass.
- [ ] Lane 3. Perform two current contacts with no recharge. Save `/tmp/three-method-P2/lane-3/two-contacts.png`. Pass when the second flight starts promptly with reduced energy and final recovery remains feasible.
- [ ] Lane 4. Land on a dead but mechanically valid conductor. Save `/tmp/three-method-P2/lane-4/dead-perch.png`. Pass when charging is unavailable and departure remains possible with sufficient energy.
- [ ] Lane 5. Select only enough partial charging to finish a route. Save `/tmp/three-method-P2/lane-5/selected-charge.png`. Pass when no full-charge wait is imposed and the ledger matches the chosen duration.
- [ ] Lane 6. Fail a planned handshake while retaining a no-charge escape. Save `/tmp/three-method-P2/lane-6/failed-charge.png`. Pass when replanning uses observed energy and returns to a reachable perch.
- [ ] Lane 7. Interrupt LiDAR mid-pass. Save `/tmp/three-method-P2/lane-7/interruption.png`. Pass when completed coverage survives and the unflown interval earns no evidence.
- [ ] Lane 8. Request an invalid TDR coupling or airborne current sample. Save `/tmp/three-method-P2/lane-8/rejection.png`. Pass when neither request produces a valid reading or a method substitution.
- [ ] Lane 9. Force planner timeout and delayed LLM output. Save `/tmp/three-method-P2/lane-9/fallback.png`. Pass when a validated continuation persists and stale output cannot overwrite it.
- [ ] Lane 10. Route two drones through a shared contact site at different times. Save `/tmp/three-method-P2/lane-10/occupancy.png`. Pass when nonoverlapping occupancy is permitted and conflicting occupancy is rejected.

**Verify, perf.** Tests alone are not sufficient verification. A PR is verified only when its unit, live, and perf boxes are all checked.

- [ ] Metric. Record p50/p95/max ingestion-to-validated-route latency, solver status, task/visit caps, omissions, and fallback rate.
- [ ] Probe. Interleave identical compatible cases on trunk and head, then measure new scan/contact/partial-charge cases separately. Include worker construction, serialization, validation, and publication.
- [ ] Baseline. Capture trunk's current point-task latency before mutation. Do not compare it as a speed ratio against new multi-hop workloads.
- [ ] Rule. Preserve the existing proposed p95 target of one second and two-second route-response budget at the declared cap. On timeout, require a validated incumbent or explicit blocked result. LLM calls remain outside this fast budget.

**Review gate.** Review the changed sensing and charging interactions.

- [ ] Show the operator screenshot evidence of a moving scan, a no-charge contact hop, and partial charging, plus a 30 to 60 second video of the real application. Record the review before merge.

**Merge.**

- [ ] Record a current exact-head verdict and append P2 after P1 if a stack is requested. The operator owns merging.

## Reconstruct supported outage facts and value experiments (P3)

**Depends on.** P2.

**Files.**

- [ ] Add `src/grid_ops/outage.py` for the finite public candidate model, evidence likelihoods, reconstruction, experiment value, and stopping assessment.
- [ ] Change `src/grid_ops/scenario.py`, `src/grid_ops/sim.py`, `src/grid_ops/manager_tools.py`, `src/grid_ops/agents.py`, `src/grid_ops/needs.py`, `src/grid_ops/scoring.py`, `src/grid_ops/planning.py`, `src/grid_ops/knowledge.py`, and `src/grid_ops/conclusion.py`.
- [ ] Add `tests/test_outage.py` and migrate `tests/test_scoring.py`, `tests/test_conclusion.py`, and the verification driver.

**Build.**

- [ ] Build the same candidate catalog from public topology, declared model family, and public priors for every episode with those inputs. Never insert the selected true candidate or use a private seed to construct the public candidate pool.
- [ ] Start on tiny networks with exhaustive candidates. Bound full-dataset candidates and outcome bins explicitly. Publish truncation and model limits. Keep posterior-predictive checks and an explicitly specified broad residual model; a placeholder residual weight alone does not detect unknown faults.
- [ ] Reconstruct from immutable readings and allowed operational telemetry using versioned likelihoods. Marginalize nuisance load regimes, propagation uncertainty, missed returns, and systematic repeat errors.
- [ ] Expose supported facts, alternative states, spatial uncertainty, and unknowns. Keep fault condition, electrical service, and recharge availability separate.
- [ ] Implement public-only expected discrimination value and joint or conditioned reranking for a bounded task set. Replace the old measured-MW variance reward. Do not let the LLM invent information-gain scores or directly write posterior probabilities.
- [ ] Optimize the declared objective in Appendix E. Penalize acquisition attempts, flight energy, and elapsed time while satisfying the evidence target. Retain route time/energy bounds and enforce positive visit overheads.
- [ ] Extend the planning horizon to optional charging and subsequent flights. Commit only the executable prefix; replan after observations. Do not schedule a fictitious future fault coordinate before TDR actually narrows it.
- [ ] Stop as supported, ambiguous, model-mismatched, no-safe-experiment, or budget-exhausted. Agreement inside a truncated candidate pool is insufficient for an unconditional claim that the whole grid is reconstructed.

**You see.**

- [ ] The map and tool output distinguish observed current, inferred service state, supported fault geometry, and unknown regions. A zero-current sample can leave both live-unloaded and disconnected alternatives.
- [ ] The LLM requests a measurement that distinguishes remaining alternatives, sees its actual outcome, and revises the next request. Repeating an equivalent sample provides little or no new value.

**Verify, unit.** Tests alone are not sufficient verification. A PR is verified only when its unit, live, and perf boxes are all checked.

- [ ] Run `uv run pytest -q tests/test_outage.py tests/test_scoring.py tests/test_conclusion.py tests/test_isolation.py`. Check identical-evidence beliefs, stable replay, idempotent ingestion, evidence provenance, duplicate utility, and model-mismatch behavior.
- [ ] Enumerate optimal adaptive experiment trees on tiny noiseless fixtures. Compare route feasibility and acquisition count to that oracle, separately from the fixed-task CP-SAT optimum.

**Verify, live.** Tests alone are not sufficient verification. A PR is verified only when its unit, live, and perf boxes are all checked. Ten lanes on the configured `swarm workers` model at the PR head.

- [ ] Lane 1. Regression lane against trunk checks supported evidence claims. Save `/tmp/three-method-P3/lane-1/regression.png`. Pass when legacy MW claims are absent in the three-method mode and existing isolation holds; record that trunk lacks this posterior.
- [ ] Lane 2. Solve a tiny identifiable open-conductor case. Save `/tmp/three-method-P3/lane-2/open-conductor.png`. Pass when the correct interval is supported within the declared resolution.
- [ ] Lane 3. Retain two zero-current explanations. Save `/tmp/three-method-P3/lane-3/current-ambiguity.png`. Pass when neither is falsely certified as dead.
- [ ] Lane 4. Retain symmetric TDR branches. Save `/tmp/three-method-P3/lane-4/tdr-ambiguity.png`. Pass when the next request targets a genuine distinguishing observation or the result stays ambiguous.
- [ ] Lane 5. Detect a geometric hazard unrelated to the outage. Save `/tmp/three-method-P3/lane-5/confounder.png`. Pass when a hazard finding alone does not conclude outage causation.
- [ ] Lane 6. Add redundant correlated repeats. Save `/tmp/three-method-P3/lane-6/repeats.png`. Pass when confidence and utility do not grow as though the repeats were independent.
- [ ] Lane 7. Use an out-of-family multi-fault fixture. Save `/tmp/three-method-P3/lane-7/mismatch.png`. Pass when unsupported certainty is refused; report cases the model cannot distinguish rather than promising universal mismatch detection.
- [ ] Lane 8. Compare no-charge hopping with optional partial charging. Save `/tmp/three-method-P3/lane-8/efficiency.png`. Pass when the selected plan respects energy and beats or matches the tiny-case declared cost optimum.
- [ ] Lane 9. Exhaust the candidate or route budget. Save `/tmp/three-method-P3/lane-9/budget.png`. Pass when omissions and unresolved facts remain visible without false completion.
- [ ] Lane 10. Run a real LLM measurement-result-replanning loop. Save `/tmp/three-method-P3/lane-10/closed-loop.png`. Pass when method choice, rationale, scheduled execution, evidence, and changed request are linked in the trace.

**Verify, perf.** Tests alone are not sufficient verification. A PR is verified only when its unit, live, and perf boxes are all checked.

- [ ] Metric. Record belief-update time, request-value time, end-to-end fast replanning, and resolved-fact quality at declared candidate/task/outcome caps.
- [ ] Probe. Interleave P2 and P3 on the same requests and compatible operations. Measure the new inference work separately. Run ablations for fixed requests versus explicit LLM requests and for routing-only versus belief-valued scheduling.
- [ ] Baseline. Capture P2 before adding the belief model. Trunk and P2 do not supply comparable calibrated outage reconstruction, so no inference speedup ratio is valid.
- [ ] Rule. Proposed initial pure belief/value budget is 250 milliseconds within the existing two-second route-response envelope. Start with at most 256 represented hypotheses, 20 acquisition alternatives, and 32 outcome bins per method. If the cap cannot represent the target faithfully, return model-limited or refine the public hierarchy; never silently claim full reconstruction.

**Review gate.** Review the meaning of confidence and completion.

- [ ] Show the operator screenshot evidence of an identifiable case, an unresolved case, and reduced acquisition count under the declared objective, plus a real-application video. Record the review before merge.

**Merge.**

- [ ] Record the exact-head verdict, all unproven model limits, and operator disposition. Append P3 after P2 if requested. The operator owns merging.

## Prove end-to-end behavior and align the presentation (P4)

**Depends on.** P3.

**Files.**

- [ ] Change `src/grid_ops/evaluation.py`, `src/grid_ops/app.py`, `src/grid_ops/views.py`, `src/grid_ops/map_view.py`, `e2e/drive.py`, `e2e/test_app_flow.py`, `tests/test_evaluation.py`, `tests/test_views.py`, and `tests/test_recordings.py`.
- [ ] Change `frontend-prototype.html`, `README.md`, and the affected documents under `docs/planner/`. Update all recording scripts and regenerate versioned `recordings/` from actual executions.
- [ ] Remove the obsolete three-method-runtime dependencies on `src/grid_ops/estimation.py`, `src/grid_ops/consistency.py`, `src/grid_ops/envelope.py`, `src/grid_ops/truth/calibrate.py`, and their MW-only tests after the final caller search. Delete unused modules if no separate supported caller remains.
- [ ] Keep old `results/` as historical, version-labelled artifacts. Never compare their old sensor scores directly with the new three-method benchmark.

**Build.**

- [ ] Give every policy the same public context and action capabilities. Keep truth, private manifests, labels, and scoring outside the policy runtime.
- [ ] Provide separately labelled policies for uniform feasible sampling, public-belief greedy sampling, scripted requests plus optimizer, and actual LLM requests plus optimizer. Add component ablations using the same accepted request pool.
- [ ] Record actual supported-fact accuracy, false certainty, localization coverage/width, unresolved fraction, time to decision, acquisition attempts, energy, visits, charge time, failed contacts, and LLM/optimizer latency separately.
- [ ] Replace all hardcoded RGB, IR, temperature, and unsupported outage-cause claims in the UI and prototype. Show requested method, assignment, viewing geometry, actual coverage, remaining energy, optional charge, and evidence provenance.
- [ ] Freeze a development set, a separate calibration set, and a held-out private evaluation manifest before tuning. Randomize fault locations and confounders; include weak or misleading reports and inaccessible tests.
- [ ] Add the acceptance gates in Appendix F. Publish unresolved and failed episodes, not only successful runs. Never treat a scripted run as evidence of actual LLM agency.

**You see.**

- [ ] A real episode starts with unknown outage details, performs drone measurements and optional charging, and ends with a supported scoped reconstruction or an honest unresolved result. The evaluator reveals the hidden answer only in its separate report.
- [ ] Results show whether fewer acquisitions and less charge waiting actually preserve accuracy, rather than counting fewer JSON reading chunks as fewer measurements.

**Verify, unit.** Tests alone are not sufficient verification. A PR is verified only when its unit, live, and perf boxes are all checked.

- [ ] Run `uv run ruff check src tests e2e`, `uv run pytest -q`, and `uv run pytest -q e2e`. Fix obsolete assertions rather than preserving voltage, IR, or measured-MW behaviors behind renamed methods.
- [ ] Search production prompts, schemas, readings, scoring, recordings, and UI for legacy methods and truth access. Review any remaining mentions as historical documentation, explicit unsupported examples, or privileged evaluation.

**Verify, live.** Tests alone are not sufficient verification. A PR is verified only when its unit, live, and perf boxes are all checked. Ten lanes on the configured `swarm workers` model at the PR head.

- [ ] Lane 1. Regression lane against trunk runs the same compatible app startup and evidence workflow. Save `/tmp/three-method-P4/lane-1/regression.png`. Pass when the app remains operable and the three-method features added by this stack satisfy their new end state.
- [ ] Lane 2. Run an identifiable held-out outage through the real UI and LLM. Save `/tmp/three-method-P4/lane-2/identified.png`. Pass when the scoped result agrees with private evaluation and cites executed evidence.
- [ ] Lane 3. Run an observationally ambiguous outage. Save `/tmp/three-method-P4/lane-3/ambiguous.png`. Pass when the UI keeps alternatives and gives no false all-clear.
- [ ] Lane 4. Run moving LiDAR with incomplete coverage. Save `/tmp/three-method-P4/lane-4/coverage.png`. Pass when path, covered intervals, and gaps match the recorded acquisition.
- [ ] Lane 5. Run contact sensing, departure, and partial charging. Save `/tmp/three-method-P4/lane-5/charging.png`. Pass when no mandatory full-charge wait appears and energy matches the trace.
- [ ] Lane 6. Disable the LLM or let it time out. Save `/tmp/three-method-P4/lane-6/llm-failure.png`. Pass when the run is honestly labelled and validated fleet continuation remains.
- [ ] Lane 7. Force low battery or unavailable safe recovery. Save `/tmp/three-method-P4/lane-7/energy-block.png`. Pass when the system reports the limitation rather than executing an infeasible route.
- [ ] Lane 8. Replay a held-out private manifest under two policies. Save `/tmp/three-method-P4/lane-8/policy-comparison.png`. Pass when hidden worlds, sensor models, public inputs, and budgets match and policies cannot read private labels.
- [ ] Lane 9. Inspect prompts, tool responses, UI, and result serialization. Save `/tmp/three-method-P4/lane-9/isolation.png`. Pass when private seeds, fault labels, and hidden AC values stay outside public records.
- [ ] Lane 10. Reproduce the displayed demonstration from an actual saved trace. Save `/tmp/three-method-P4/lane-10/replay-ui.png`. Pass when every displayed finding has an observation and no fixed RGB/IR story remains.

**Verify, perf.** Tests alone are not sufficient verification. A PR is verified only when its unit, live, and perf boxes are all checked.

- [ ] Metric. Report quality-cost curves, completion/ambiguity rates, acquisition count, elapsed decision time, energy, charge waiting, and fast route-response latency. Report cold startup and LLM wall time separately.
- [ ] Probe. Run at least 30 paired held-out episodes for policy comparisons, stratified by identifiable, ambiguous, and inaccessible cases. Use interval estimates for differences and publish the seeds only in the privileged replay manifest.
- [ ] Baseline. Measure the new feasible-sampling and greedy baselines before tuning the LLM policy. Old synthetic electrical-bundle results are not a baseline for this sensor set.
- [ ] Rule. Require zero tested hard-constraint violations, preserved quality gates, and a positive held-out efficiency result before claiming improvement. If the result is inconclusive, report it. Keep the existing one-second p95/two-second fast-response targets at the recorded cap.

**Review gate.** Review the complete user journey.

- [ ] Show the operator screenshots and a 30 to 60 second video of a successful real run and an unresolved run. Include the private evaluator comparison separately and record the review before merge.

**Merge.**

- [ ] Record the exact-head verdict, current stack bases, evaluation receipt, and operator disposition. Append P4 after P3 if requested. The operator owns merging.

## Close the program

- [ ] Check the implementation boxes only after their evidence exists. Keep intentionally unresolved cases and hardware assumptions in the final report.
- [ ] Deliver the file map, actual validation commands/results, real traces, and operator-owned stack if one was requested. Do not claim universal AC reconstruction or globally optimal adaptive sensing.

## Appendix A. Prototype evidence

Planning baseline is source HEAD `e267c7492e76e6499474e455d04c543b7f039cac`. No implementation branch or feature SHA was created. Existing user changes were present before planning and remain outside this document.

The command `.venv/bin/python -m pytest -q tests/test_isolation.py tests/test_sorties.py tests/test_planning.py tests/test_conclusion.py` passed 53 tests with three SWIG deprecation warnings. This establishes a current correctness baseline. It does not validate the proposed sensors or measure planner latency.

The executable probe at `/tmp/nomadic-outage-architecture/zero-current-probe.py` builds two real pandapower two-bus networks. An unloaded energized line with zero modeled shunt capacitance and a disconnected line both yield zero current. The output at `/tmp/nomadic-outage-architecture/zero-current-result.json` records 0.0 A for both, with the energized far bus at 1.0 pu. The fixture deliberately sets shunt capacitance to zero to isolate the ambiguity. It proves that current alone does not universally identify energization; it does not characterize every real overhead line.

Two architecture candidates and an independent comparison are stored under `/tmp/nomadic-outage-architecture/`. The first comparison predates the user's charging correction, so its forced-turnaround recommendation is superseded. Revised energy candidates are `energy-a.md` and `energy-b.md`. The final design uses explicit visits and persistent energy, not forced terminal landings.

No real TDR coupler, LiDAR calibration, private outage-reconstruction benchmark, new optimizer, or live-LLM three-method run has been validated. Their validation is implementation work listed above. Terminal or UI screenshots for those features do not yet exist.

## Appendix B. Current ownership and proposed call paths

The existing architecture has useful boundaries. The simulation is the single state writer, the manager reads a pinned snapshot, proposal acceptance is atomic, and route validation is independent of CP-SAT.

Current startup and sensing follow these paths.

```text
Simulation.__init__
  initial_world -> truth.build -> seed_starting_handshakes
  -> _refresh_estimate -> snapshot

world.advance_to -> settle -> _arrive -> Measuring -> _sensing
  -> TruthState.sense(LinePoint, Measurement, time, identity, repeat)
  -> Reading -> knowledge.ingest_reading / assessment.ingest_local_reading
  -> Simulation._publish -> _refresh_estimate -> needs.refresh -> snapshot

Coordinator._run_turn -> agents.run_llm_turn(sim) or run_scripted_turn(sim)
  -> ManagerOutput hypotheses/predictions -> agents._record -> RecordProposal
  -> CommitDecision -> Simulation._commit_decision
  -> decisions.evaluate_commit / apply_candidate / commit_apply
  -> needs._hypothesis_needs copies prediction.measurement -> tasks

fleet.route_pass -> planning.plan_sorties -> routes.validate_sorties
  -> sim commit -> world.apply_sorties
Coordinator.planner_step calls preview_plan only; it does not dispatch.
```

The incident builder currently switches `LINE_024` off and adds separate vegetation, hot-joint, and motion fixtures. That is not yet a general causal outage generator. The estimator consumes `.p_mw` bundled into current readings. The current charge completion restores the whole flight allowance and the route model treats the last perch as a terminal endpoint.

The proposed runtime uses these paths.

```text
Privileged episode runner
  -> truth.build_episode(public_fixture, private_episode_seed, private_noise_seed)
  -> private FaultEvent -> ProtectionAction -> post-trip AC state
  -> public World(public topology, reports-as-priors, battery, observed start evidence)

Coordinator._run_turn
  -> InvestigationContext(pin(snapshot), public_model, capabilities)
  -> agents.run_llm_turn(context) -> ManagerOutput.measurement_requests
  -> RecordProposal -> CommitDecision
  -> proposals.validate + decisions.evaluate_commit
  -> commit accepted hypothesis/request lifecycle atomically

Simulation._refresh_tasks
  -> needs.refresh(accepted requests, public belief, capabilities, public geometry)
  -> sensing.compile_requests -> tasks + explicit unavailability reasons
  -> fleet.route_pass -> planning.plan_sorties(public PlanningInput)
  -> routes.validate_sorties independently replays time, energy, geometry and access
  -> commit current executable itinerary prefix

world.advance_to / settle
  -> Transit | AirborneScanning | Perched | GroundMeasuring | Charging
  -> private Physics.acquire(executed acquisition history)
  -> immutable Reading and actual energy/charge telemetry
  -> Simulation._publish -> outage.reconstruct -> evidence/coverage refresh
  -> snapshot -> coordinator notification -> next LLM request and route pass

Privileged evaluator
  -> private episode + public final report -> scored results
  -> never feeds truth labels or benchmark scores into policy tools
```

Flight execution, energy integration, belief updates, and accepted-request state remain owned by the simulation thread. Worker processes receive serialized public planning inputs. The LLM chooses the method and investigative scope. Task compilation chooses valid geometric alternatives. The optimizer chooses assignments, paths, timing, and charge duration.

## Appendix C. Schema and interface changes

The usage starts with a public investigation context and an explicit request. A caller never needs the truth object to propose or validate work.

```python
proposal = run_llm_turn(context)
# Coordinator records and accepts the proposal through existing simulator commands.
tasks = compile_requests(public_context, accepted_requests)
plan = plan_sorties(planning_input)
validation = validate_sorties(public_context, plan)
```

These are proposed shapes, not implemented signatures.

```python
MeasurementMethod = Literal["lidar", "tdr", "contact_current"]

class RequestCommon:
    id: str
    hypothesis_ids: tuple[str, ...]
    rationale: str
    alternatives_distinguished: tuple[str, ...]
    expected_outcomes: tuple[OutcomeSpec, ...]
    evidence_ids: tuple[str, ...]
    urgency: int
    source_revision: int
    replaces_request_ids: tuple[str, ...]

class LidarRequest(RequestCommon):
    method: Literal["lidar"]
    target: CorridorSpan

class TdrRequest(RequestCommon):
    method: Literal["tdr"]
    target: TdrTarget  # circuit/conductor and allowed coupling access

class CurrentRequest(RequestCommon):
    method: Literal["contact_current"]
    target: ConductorScope  # one circuit and phase, allowed line interval

MeasurementRequest = LidarRequest | TdrRequest | CurrentRequest

class ManagerOutput:
    revisions: list[RevisionModel]
    measurement_requests: list[MeasurementRequest]
    retire_request_ids: list[str]
    recommendations: list[RecommendationModel]
    conclusion: ConclusionModel | None

class LidarPass:
    target: CorridorSpan
    trajectory: ScanTrajectory  # includes direction, standoff, height and look pose
    speed_mps: float
    sensor_profile_id: str

class ContactCurrent:
    conductor: ConductorPoint  # segment, circuit, phase, along-line offset
    integration_s: int

class TdrTest:
    access_site_id: str
    conductor_id: str
    coupling_profile_id: str
    return_path_id: str
    duration_s: int

Acquisition = LidarPass | ContactCurrent | TdrTest

class ScanVisit:
    acquisition: LidarPass

class PerchVisit:
    site: ConductorSite
    acquisitions: tuple[ContactCurrent | TdrTest, ...]
    charge_s: int  # optimizer decision; zero is normal

class ScheduledStep:
    start_s: int
    end_s: int
    operation: Transit | Scan | Land | GroundSample | Charge | Takeoff | Wait
    # Each operation carries its authoritative path, pose, site or acquisition.
    # Wait has a location and must match the actual airborne/perched state.

class SortieRequest:  # migrate the existing name in one wave
    drone_id: str
    steps: tuple[ScheduledStep, ...]
    final_perch: ConductorSite
    based_on_revision: int
    energy_model_revision: str
    sensor_model_revision: str
    evidence_dependency_ids: tuple[str, ...]

class Reading:
    id: str
    task_id: str
    request_ids: tuple[str, ...]
    drone_id: str
    acquisition: Acquisition
    acquired_start_s: int
    acquired_end_s: int
    payload: LidarPayload | TdrPayload | CurrentPayload
    quality: AcquisitionQuality
    sensor_model_revision: str
    evidence_epoch: str  # only a publicly established epoch

class OutageBelief:
    revision: int
    model_version: str
    candidate_weights: tuple[CandidateWeight, ...]
    supported_facts: tuple[SupportedFact, ...]
    unresolved_classes: tuple[EquivalentAlternatives, ...]
    model_checks: ModelChecks
    evidence_ids: tuple[str, ...]

class BatteryState:
    remaining_units: int
    capacity_units: int
    energy_model_revision: str
    observed_s: int

class MeasurementTask:
    id: str
    request_ids: tuple[str, ...]
    acquisition: Acquisition
    alternative_group_id: str
    repeat_of_task_id: str | None
    status: TaskStatus

class PlanningInput:
    snapshot_revision: int
    clock_s: int
    drones: tuple[PublicDroneState, ...]  # includes BatteryState and pose/contact
    tasks: tuple[MeasurementTask, ...]
    public_model: PublicOutageModel
    capabilities: SensorCapabilities
    belief: OutageBelief
    energy_model: EnergyModel
    deadline_s: int
```

Use discriminated Pydantic request models at the LLM boundary and immutable domain records inside the simulation. `sensing.py` owns the method catalog and target/acquisition definitions. Derive capability descriptions and UI method options from that catalog instead of maintaining separate method lists.

Predictions continue to describe expected observations. Remove their authority to create tasks. `MeasurementNeed` becomes the lifecycle of an accepted request; it no longer silently invents a method. Preserve task sharing, cancellation, repeats, and coverage reuse with explicit request provenance.

LiDAR payloads contain measured geometry summaries, uncertainties, actual inspected intervals, and missing/occluded intervals. TDR payloads contain a waveform or declared echo representation, delay/amplitude/polarity, usable range, propagation uncertainty, and coupling quality. The raw TDR result must not contain a hidden fault identifier or preselected true map location. Current payloads contain RMS amperes, one conductor identity, acquisition interval, uncertainty, and contact/saturation quality. They never contain voltage, P, Q, power direction, or all other circuits.

The public network supplies nominal topology, mapped paths, declared conductor identities, and access capabilities. The hidden scene supplies actual fault geometry and visibility. Candidate generation, route feasibility, and request compilation must not read hidden obstacle coordinates to choose a conveniently visible path.

Replace public `ScenarioConfig.seed`/`World.scenario_seed` coupling with a public fixture identifier/version and a separate policy seed. The privileged runner passes a private `EpisodeSpec` with independent episode and noise seeds only to the hidden builder. Public capability and goal versions are reproducible without revealing which private candidate was selected. Avoid fault-labelled episode identifiers.

The current-sample observation function uses the connected subspan and phase approximation at the actual attachment point. Its likelihood handles a nonnegative RMS response, zero-offset behavior, and saturation consistently with the sampler. TDR forward responses use the post-protection electrical paths, so an open protective device may conceal a downstream fault. A detected reflection from that device must not become an invented observation of the hidden fault beyond it.

Public candidate predictions are calculated from each declared candidate and nuisance regime. Shared sensor equations may be pure functions of those candidate inputs. They never receive the selected private state or its solved currents. Prediction-model approximation error belongs in calibration and mismatch checks.

Proposed interface changes include these contracts.

```python
run_llm_turn(context: InvestigationContext, ...) -> TurnResult
compile_requests(context: PublicContext,
                 requests: tuple[MeasurementRequest, ...]) -> TasksAndRejections
reconstruct(model: PublicOutageModel,
            evidence: tuple[Reading | OperationalEvidence, ...]) -> OutageBelief
value_requests(model: PublicOutageModel, belief: OutageBelief,
               tasks: tuple[MeasurementTask, ...]) -> ExperimentValues
plan_sorties(inputs: PlanningInput, ...) -> PlanningResult
validate_sorties(context: PublicContext,
                 requests: tuple[SortieRequest, ...]) -> ValidationResult
Physics.acquire(executed: ExecutedAcquisition) -> SensorSample
```

Change every caller, worker serialization payload, recording producer, test helper, and AppTest control together. The coordinator records returned LLM proposals; the agent no longer calls simulator submission through `_record`. Policies can propose only through the constrained coordinator boundary, not submit `SetInService` or inspect `evaluator_truth()`.

Scan and perch visits are the optimizer's internal building blocks. The emitted ordered steps are the single executable schedule. They carry every chosen delay, flight path, contact operation, charge interval, and wait across worker, validator, and executor boundaries. The validator checks monotonic contiguous timing, state-compatible waits, timed occupancy, dependency freshness, and the complete energy ledger. It recomputes costs rather than trusting energy values supplied by the solver. The executor rejects obsolete model versions or revalidates the remaining prefix after an interruption. A waiting gap never disappears from time or energy accounting.

Keep one energy law in `energy.py`. Initially use normalized energy rather than claiming unknown battery watt-hours. One full battery supplies 1,800 baseline airborne-equivalent seconds. Use scaled integers, round costs up and charge gains down, include approach/landing/takeoff and ground electronics, and cap energy at capacity.

```text
E_after = min(capacity, E_before - operation_cost + observed_charge_gain)
0 <= E(t) <= capacity at every operation boundary and during each operation
charge_s = 0 is permitted at every measurement landing
nominal net charging rate = capacity / 10800 seconds
```

The charging rate is a synthetic assumption to calibrate, including whether three hours is empty-to-full and how ground loads affect net gain. Start with a declared constant net rate and continuous energy accounting. Do not promise that a real harvester has that curve. Charge compatibility with TDR is part of the profile; default to sequential TDR and charging if interference is unspecified.

The ordinary state is `Perched`, not `Stranded`. `Perched -> GroundMeasuring -> Perched -> Flying` is legal without charging. `Perched -> Charging -> Perched` adds actual energy for the chosen interval. A failed handshake returns to a noncharging perch. Use `Stranded` only when departure/recovery is genuinely infeasible. LiDAR uses `AirborneScanning` with position evolving along the trajectory. Preserve locked scans or contacts and allow rerouting only at valid interruption boundaries.

## Appendix D. Files and migration ownership

The following existing files anchor the migration. New modules above are intentionally limited to physical sensing contracts, energy accounting, hidden episodes, and bounded outage inference.

| Concern | Existing files | Change |
|---|---|---|
| Hidden startup | [truth/build.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/truth/build.py:139), [truth/state.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/truth/state.py:75), [scenario.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/scenario.py:51) | Replace fixed independent incident facts with private causal episodes and separately seeded sensor noise. |
| LLM ownership | [hypotheses.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/hypotheses.py:50), [agents.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/agents.py:48), [manager_tools.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/manager_tools.py:46) | Add explicit discriminating requests and public belief/capability tools. |
| Acceptance | [proposals.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/proposals.py:112), [decisions.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/decisions.py:231), [coordinator.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/coordinator.py:147) | Validate and atomically accept request lifecycle changes with evidence dependencies. |
| Task generation | [needs.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/needs.py:63) | Remove prediction-driven method choice and midpoint/fixed-footprint assumptions. |
| Execution | [world.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/world.py:408), [sim.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/sim.py:345) | Execute scan motion, landed acquisitions, persistent battery, optional charging, and belief refresh. |
| Scheduling | [routes.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/routes.py:38), [planning.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/planning.py:60), [scoring.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/scoring.py:38) | Add visit entry/exit, energy-resource propagation, timed occupancy, optional charging, and evidence-aware costs. |
| Worker/fallback | [fleet.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/fleet.py:61), [resilience.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/resilience.py:47) | Migrate public planner inputs and validate no-charge/partial-charge fallback plans. |
| Evidence | [knowledge.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/knowledge.py:150), [assessment.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/assessment.py:54), [conclusion.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/conclusion.py:69) | Keep observed current, service inference, hazard evidence, and charge availability distinct. |
| Obsolete estimation | [estimation.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/estimation.py:287) and its consistency/calibration consumers | Remove measured-P observability from the three-method mode; delete unused paths after migrating callers. |
| Evaluation | [evaluation.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/evaluation.py:352), [test_isolation.py](/Users/pc/Documents/repos/nomadic_challenge/tests/test_isolation.py:1) | Separate policy/evaluator capabilities and add paired hidden-world, ambiguity, quality, and cost checks. |
| Presentation | [app.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/app.py), [views.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/views.py), [map_view.py](/Users/pc/Documents/repos/nomadic_challenge/src/grid_ops/map_view.py), [frontend-prototype.html](/Users/pc/Documents/repos/nomadic_challenge/frontend-prototype.html:404), [e2e/drive.py](/Users/pc/Documents/repos/nomadic_challenge/e2e/drive.py:1) | Show actual methods, moving scans, evidence uncertainty, battery continuity, and optional charge. |

Preserve the public hidden-truth boundary, immutable evidence, integer along-line positions, interval union arithmetic, authoritative simulator writer, atomic decisions, and independent route validator. Replace the incompatible assumptions rather than maintaining two production ways to create measurement work.

## Appendix E. Reconstruction target and optimization objective

The initial private episode contains a causal fault, its protective response, the resulting effective topology, and a solved post-trip operating state. Public storm reports are priors, not verified fault locations. The agent sees nominal topology and only evidence produced by actual acquisitions or explicit operational events. Remove direct startup knowledge of the selected switched line. Public starting charge evidence must itself be declared observed evidence, not a view of all circuits.

Reconstruction means supported fault intervals, compatible open/tripped connectivity, and affected-supply claims within a declared synthetic network model. It does not mean full voltage, P/Q, flow direction, or every line's energized state. Current magnitude and TDR/LiDAR observations do not uniquely determine those quantities in general. For large or observationally equivalent cases, the correct reconstruction includes unknowns and alternatives.

Start with a finite one-primary-fault family, declared protective responses, and a few public nuisance load/propagation regimes. Include an unrelated visible hazard and a live-unloaded branch. Generate all tiny-case candidates from public inputs. Expand to grouped/hierarchical full-map candidates only with visible resolution and truncation limits. Held-out multi-fault and mismatch cases test whether conditional confidence becomes misleading.

Use deterministic likelihood-based inference. The LLM reasons about what to test next and explains the evidence. It cannot promote an unsupported narrative into a measured electrical fact. Confidence is conditional on the declared model and must be calibrated across withheld episodes. An `unmodelled` label alone is not a guarantee that all unknown causes will be detected.

Freeze a reconstruction goal and evidence threshold before comparing policies. The threshold and localization tolerance are benchmark settings, not real operational standards. For the initial noiseless identifiable fixtures, require the correct interval and topology class exactly at the declared cell resolution. For noisy cases, use a proposed 95% supported-fact threshold only after calibration, and retain all materially compatible alternatives. The evaluator checks truth after the agent's stopping decision; it never tells the agent that the answer is correct.

The optimization problem is adaptive experimental design with vehicle routing and energy resources. Its policy-level target is the minimum expected cost to reach the predeclared evidence goal before the deadline. A cost vector includes acquisition attempts, elapsed decision time, energy, and charge waiting. Use a primary acquisition-count objective, then elapsed time, then energy among equally qualifying plans. Urgent hazard response and hard feasibility can override diagnostic efficiency.

Minimizing steps alone would choose zero measurements, so the evidence goal is a required constraint. In the bounded approximation, prioritize mandatory urgent work, choose a minimum-cost batch that reaches the forecast evidence target when possible, and otherwise maximize unique expected discrimination under the same budget. Re-evaluate joint or conditioned value when requests share information. Do not assign extra value to duplicate hypotheses, overlapping scans, or correlated repeats.

CP-SAT schedules the LLM-authorized tasks. Model transit from each visit's exit to the next entry, actual LiDAR traversal, approach/landing/takeoff, ground sensing, optional charge duration, time-window occupancy, energy before and after each operation, final safe perching, and the four-hour horizon. Every hop consumes the same persistent battery resource. Bound candidate visits and charge events, require positive operation time, and prevent gratuitous repeated cycles.

For unknown charge availability, assume zero energy gain until a handshake succeeds unless an explicit uncertainty policy permits otherwise. Every charging-dependent plan needs a feasible no-charge continuation, including previously known sites whose evidence can expire or whose power can fail. A continuation may stay on a mechanically safe perch; it must not claim that the drone can launch without the needed energy. Reconfirm actual gained energy before any departure that depends on charging. Check freshness at the scheduled interval and replan after a stale or failed handshake. Chargeability and contact suitability are separate. A dead-line contact is possible with adequate energy; departure is not conditioned on a full charge or energized status.

Execute the current validated prefix and replan as evidence arrives. Anticipate the next flight/charge opportunity, but do not pretend future uncertain measurement outcomes are already known. CP-SAT optimality applies only to its finite deterministic scheduling model. It does not prove that the whole adaptive LLM investigation uses the fewest possible measurements. Tiny adaptive oracles and paired-policy experiments measure that separate question.

Count one acquisition attempt per predeclared task, including failed and repeated attempts. LiDAR sample chunks are not separate steps. Fix maximum scan extent, sample rate, and splitting rules before benchmarking so one policy cannot game the count by emitting larger or smaller reading objects. Also report physical visits, launches, scan length, energy, and elapsed time.

## Appendix F. Validation gates and alternatives

Apply Model the Domain to requests, acquisitions, and energy-bearing states. Apply Test Behavior, Not Implementation to sensor scope, causal conclusions, and battery continuity. Apply Sequence Work into Verifiable Units to P1 through P4. Apply Prove It Works to actual driver traces and recorded evidence rather than successful schema parsing alone.

The verification gates are these.

| Gate | Required evidence |
|---|---|
| Isolation | Import checks, recursive public serialization checks, and paired-world comparisons. Perturb private state/seed without changing public priors; differences require acquired evidence. Check errors, logs, previews, recordings, worker payloads, and policy contexts. |
| Sensor fidelity | Analytic FOV/range/occlusion fixtures, interrupted coverage, one-conductor current, TDR propagation/coupling/branch fixtures, and forbidden-field tests. |
| Energy fidelity | Multiple contacts without charge, cumulative airborne-energy exhaustion, landed sensing drain, positive takeoff costs, partial charge, charge failure, dead-line departure, capacity clipping, and recovery reserve. |
| Scheduling | Exact tiny fixed-task enumeration, optional-charge choices, timed shared-site occupancy, independent route replay, stale inputs, solver timeout, and fallback. |
| Reconstruction | Tiny identifiable cases, indistinguishable cases, confounders, model mismatch, quality-conditioned negatives, correlated repeats, and no direct answer-key access. |
| LLM agency | Real model transcript linking selected method and scope to validated requests, assigned missions, readings, and a changed subsequent request. Scripted and replayed runs stay separately labelled. |
| Quality | Zero unsupported all-clears in deterministic ambiguity fixtures. On held-out noisy cases, report empirical supported-fact precision, localization interval coverage/width, confidence calibration, fault recall, and unknown fraction. Use interval estimates; 30 episodes cannot certify rare-error rates. |
| Efficiency | Paired hidden episodes and budgets. Compare at matched quality and ambiguity handling. Report acquisitions, flights, energy, charge waiting, time, and failures. Do not claim improvement when the difference is statistically or operationally inconclusive. |

Prefer the small explicit-request architecture as the integration base, with a bounded public candidate model and likelihood checks for meaningful reconstruction. The independent design review favored this combination over replacing the coordinator with a general POMDP engine.

Reject a fixed heuristic method chooser because it would remove the requested LLM decision. Reject a general POMDP rewrite because it expands the modeling and integration burden without first proving sensor, energy, and route correctness. Reject full AC state reconstruction because these measurements do not generally identify it. Reject automatic full charging after a measurement landing because the user explicitly ruled it out. Reject an airborne boom or extra camera/IR sensor because that invents hardware outside the requested toolkit.

## Appendix G. Risks and reading list

TDR access and high-voltage compatibility remain hardware assumptions, not verified engineering. LiDAR sensing needs synthetic heights and obstacles because the source map does not contain a full surveyed scene. Battery and charge profiles require calibration. None of those uncertainties justifies secretly adding a sensor or an authoritative truth lookup.

The public nominal network is itself a synthetic electrical interpretation of mapping data. A candidate-model error can produce misleading conditional confidence. Report topology assumptions and unsupported cases. Do not promise exact recovery of the real Bavarian network.

The code's Python boundaries are engineering isolation, not a security sandbox for arbitrary hostile Python code. Policies must receive only explicit public interfaces. If arbitrary policy code becomes a requirement, process-level capability isolation is additional work.

Read `docs/CHALLENGE_DESCRIPTION.md`, `data/README.md`, `docs/planner/geometry-and-sensing.md`, `docs/planner/optimization.md`, and `docs/planner/evaluation.md` before editing. Record that the original challenge's forced charging rule is intentionally superseded by the user's newer instruction.

The installed architecture and verification skill sources are `/Users/pc/.codex/plugins/cache/pstack-claude/pstack/0.9.102/skills/architect/SKILL.md`, `skills/how/SKILL.md`, `skills/arena/SKILL.md`, and the principle skills named above. Resolve the latter paths against that same installed pstack skills directory. Review shared-interface and inference changes independently before implementation is declared complete.
