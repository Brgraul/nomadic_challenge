# Acceptance map

Every ticket in `.scratch/measurement-planner/issues` maps to the tests that prove it and to a
real end-to-end check. Unit tests run against the real `Simulation` (hidden pandapower physics,
real CP-SAT); the e2e tests (`e2e/test_app_flow.py`) drive the Streamlit console through
`streamlit.testing.v1.AppTest` with nothing mocked. Run everything with `uv run pytest -q`.

| Ticket | Owning tests | e2e |
|---|---|---|
| 01 isolate hidden physics | `tests/test_isolation.py` (import boundary, leak probes, differential probe) | console boot in every e2e |
| 02 exact line and circuit locations | `tests/test_geometry.py` | inspect-line flow (t07) |
| 03 validated measurement sorties | `tests/test_sorties.py` (zero-distance, locked dwell, deadline) | t07 |
| 04 seeded scenarios | `tests/test_scenarios.py` | t08 (healthy mode) |
| 05 public charging evidence | `tests/test_landing.py` (expiry boundary, conflict, interrupted charging, projection) | t14 |
| 06 scoped local findings | `tests/test_findings.py` | t09 |
| 07 public electrical estimates | `tests/test_estimation.py` | t07 |
| 08 consistency and uncertainty | `tests/test_consistency.py` | t08 |
| 09 manager findings | `tests/test_manager.py`, `tests/test_recordings.py` (recorded LLM turns) | t09 |
| 10 atomic decisions | `tests/test_decisions.py` | t10 |
| 11 diagnostic tasks | `tests/test_needs.py` | t11 |
| 12 shared-evidence scoring | `tests/test_scoring.py` | t12 |
| 13 fleet sortie optimisation | `tests/test_planning.py` (brute-force tiny case, honest statuses) | t13 |
| 14 reactive replanning | `tests/test_fleet.py` | t14 |
| 15 async manager and planner | `tests/test_coordinator.py`, `tests/test_sim_thread.py` | t15 |
| 16 conclude and reopen | `tests/test_conclusion.py` | t16 |
| 17 response and failure recovery | `tests/test_resilience.py` | t17 |
| 18 policy comparison and replay | `tests/test_evaluation.py` | t18 |

## Cross-cutting scenarios named by ticket 17

| Scenario | Test |
|---|---|
| simultaneous switching / sensing / expiry | `test_landing.py::test_simultaneous_contradiction_is_conflicted_not_last_writer_wins`, `test_fleet.py::test_each_relevant_event_and_command_triggers_a_pass_and_irrelevant_ones_do_not` |
| zero-distance stops | `test_sorties.py::test_zero_distance_arrival_completes_without_division_or_drain` |
| locked ownership | `test_needs.py::test_locked_measuring_attempt_finishes_even_if_its_need_retires`, `test_planning.py::test_locked_work_and_airborne_continuations_are_preserved` |
| interrupted charging | `test_landing.py::test_interrupted_charging_makes_the_circuit_unavailable_and_visible` |
| atomic rollback | `test_decisions.py`, `test_fleet.py::test_cross_drone_updates_are_all_or_nothing` |
| exact landing projection | `test_landing.py::test_landing_candidates_include_projections_and_perches` |
| deadline | `test_sorties.py::test_deadline_rejects_stop_or_landing_beyond_t_plus_4`, `test_fleet.py::test_deadline_edges_and_occupied_endpoints_are_respected`, `test_conclusion.py::test_at_the_deadline_the_console_reports_remaining_work_not_success` |

## Fault injection (ticket 17)

`tests/test_resilience.py`: solver UNKNOWN, MODEL_INVALID and timeout (injected through
`planning.plan_sorties(fault=...)`), a hung worker killed by the watchdog and restarted, a worker
that exits mid-solve, failure after hold acquisition, a stale decision command, coordinator death
(`test_coordinator.py::test_coordinator_death_leaves_a_validated_continuation_and_accepted_state_intact`).

## Known limits (not claimed as met)

- `DEAD` voltage classification is rarely reachable under the specified evidence rules.
- Manager quality with a real LLM (OpenRouter `z-ai/glm-5.3-flash`) varies; recorded turns in
  `recordings/` are the reproducible evidence, live turns are demonstrations.
- Latency is measured on the development machine over the declared seed set
  (`tests/test_resilience.py::LATENCY_SEEDS`); another machine can differ, and the console's
  Resilience tab reports its own measured numbers.

## Policy comparison (ticket 18)

`uv run python -m grid_ops.evaluation --mode incident_demo --compare --seeds 7 8 9` writes one JSON
per policy and seed under `results/`. Over seeds 7-9, 240 minutes each, in the incident demo the
agent, greedy and lookahead policies detected none of the three hidden local defects and random
detected one in three runs (readings: agent 8, greedy 10, random 10, lookahead 20). Each drone gets
about two sorties in four hours because charging takes three hours, and the defects are tiny
compared with the report areas. In the healthy benchmark, mean flow error was agent 16.8 MW,
random 16.6, greedy 19.2, lookahead 19.1, with nine intervals unsupported in every run. Nothing here
supports a claim that the agent beats the baselines or that four-hour coverage is adequate.
Reconstruction needs are staged (12 at a time) so equal-urgency work fits under the 20-attempt cap.
