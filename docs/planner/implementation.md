# Implementation and acceptance

[Design index](../planner-design.md). Read for implementation scope, required integration scenarios or explicitly deferred extensions.

Related contracts: [Integration contract](workflow.md), [Verification metrics](evaluation.md).

## Implementation scope and acceptance scenarios

This remains a design, not implemented functionality. The implementation introduces the circuit registry and geometry helpers in `grid.py`; scenario physics and local fields in `truth/`; public estimation and hypothesis tests in `estimation.py`; scoped condition assessment in `assessment.py`; candidate refresh, benefit groups, CP-SAT planning and independent validation in `planning.py`; domain histories and exact state transitions in `world.py`; versioned decisions and fallback handling in `sim.py`; and snapshot-pinned agents in `agents.py`. The UI shows each hypothesis revision beside its latest finding, update reason, evidence and unresolved limits. Pending diagnostic notices remain visible when operational acceptance is blocked, with their unaccepted status explicit. Evaluation stays separate. Add OR-Tools and the test tooling as dependencies when implementing these modules.

The implementation acceptance suite must cover these behaviors. These are required future tests, not claims that the current application passes them.

| Scenario | Required outcome |
|---|---|
| New current reading, unchanged hypotheses | Covariance, marginal values and deterministic candidates refresh within active accepted needs; routes are reconsidered without an unnecessary planner LLM call or unrelated reconstruction tasks |
| Outage cause supported while most flows remain unobservable | Accept the evidence-linked diagnosis once material alternatives are addressed; retire diagnostic needs, cancel unstarted work, finish locked measurements and validate recovery routes without requiring H0 completion |
| Flow gap with no active diagnostic purpose in incident mode | No task or routing benefit is created; a relevant discriminating electrical reading remains eligible with its stated causal purpose |
| Local hazard detected without an established outage connection | Recommend the scoped hazard action, keep the outage diagnosis unresolved and request only evidence that can address the causal link or material alternatives |
| New relevant evidence after accepted diagnosis | Explained reopening may create new diagnostic needs; covariance changes alone cannot regenerate retired work |
| One drone reports while another is mid-leg and the manager takes 20 wall seconds | Fleet route pass runs without waiting, using live positions and existing explained hypotheses; the slow LLM does not hold the fleet |
| Timely explained update after drones have moved | Semantically compatible proposal rebases from live positions, battery and reservations, then atomically accepts a new bounded route pass |
| Near-real-time response measurement | Measure ingestion-to-validated-plan publication over representative seeds; target p95 at most one second and a two-second response budget, exposing misses and safety exceptions |
| Local positive, qualified negative and low-excitation negative | Supported scope, actual coverage and inconclusive evidence remain distinct; each changed hypothesis has an LLM finding and causal explanation |
| Explained revision missing, duplicated or stale | Reject missing or stale evidence and explanation records; identical retries do not duplicate revisions or side effects |
| Known switching or qualified dead voltage contradicts a live belief | Invalidate affected chargeability before routing; update public models only through observable evidence and explained accepted assumptions |
| Prior-only and minimally observable estimates | Report real observability and bad-data detectability honestly; empty suspect lists do not imply successful validation |
| Extra reading in an unrelated component | Critical measurements elsewhere remain untestable |
| Wrong topology, shared sensor bias or AC/DC mismatch | Separate model inconsistency from sensor suspicion; retain discrepancy and correlation limits in uncertainty |
| Co-located tasks and simultaneous switching/reading/charge events | Apply physical changes before sampling, retain compatible public epochs without hidden IDs, then publish one snapshot with no division by zero or duplicated assignments |
| Operator edit or new evidence during a manager turn | A changed semantic dependency rejects the stale proposal; pure motion rebases to live state, and old releases cannot release a new cycle |
| Measuring drone rerouted or its hypothesis retired | Preserve the locked attempt, remaining dwell and unique ownership; change only the executable suffix |
| Later sortie fails acceptance | No earlier drone, accepted hypothesis or task is partially mutated; validated urgent findings remain visible as pending diagnostic notices |
| Near-line drone with sparse landing samples | Include an exact eligible projection and avoid false discretization infeasibility |
| Occupied or expired landing site and Ready drone with no useful work | Enforce occupancy and evidence validity; permit waiting without an artificial recharge |
| Solver timeout, invalid model or coordinator death | Bounded failure, validated continuation or landing-only fallback, otherwise remain held with a specific cause |
| Start at T+03:58 with a 30-minute battery | Reject any route whose measurements or landing exceed T+04:00 |
| Overlapping tasks, subdivided cells or several hypotheses sharing one observation | Fixed-grid conserved utility and joint information scoring prevent reward multiplication; urgency and quantization limits remain explicit |
| COMPONENT_07 and an infeasible operating seed | Use the declared SUBSTATION_03 source; bound scenario construction and return an explicit failure when necessary |
| Healthy versus incident modes | Distinct consistent briefs and objectives; H0 reconstruction only in the explicit benchmark, original precautionary switching in incident mode, local hazards and explanation adaptation actually exercised |
| Truth leakage probe | No hidden injections, local fault coordinates, latent noise scales or evaluator scores in public snapshots or tools |
| Nominal reduced-model rank fixture | Reproduce ranks 24 and 52 and the 28-reading minimum; recalculate after topology changes |
| Tiny multi-drone exact instances | Check full allocation, ordering and landings against enumeration and report objective gaps honestly |
| Matching-model noiseless estimation | State exact-constraint or compatible-prior assumptions and recover the specified DC state; mismatched priors are tested as a bias case |
| Paired policy seeds and replay | Observation-keyed noise, recorded LLM outputs and solver budgets reproduce the applicable run; watchdog variability is labelled |

## Deferred extensions

Bayesian root-cause probabilities and exact entropy rewards require calibrated hypothesis likelihoods beyond the scoped synthetic local tests. Full AC state estimation requires a richer observable model. Continuous load profiles require time-aware estimation beyond explicit epochs. Two-sortie optimization requires joint flight and charge scheduling. A larger fault catalogue can follow the required incident fixture. None of these extensions may be represented as already available or used to justify a global grid-health claim.
