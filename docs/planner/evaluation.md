# Evaluation policy

[Design index](../planner-design.md). Read for truth-isolated metrics, policy comparisons, noise replay or benchmark claims.

Related contracts: [Scenario modes](electrical-model.md), [Acceptance cases](implementation.md).

## Evaluation

The evaluator alone compares public estimates and recommendations with hidden scenario state. It runs after each scored reading batch and records final unresolved work at T+04:00. Scores are stored outside `World` and never appear in agent tools, public event payloads or diagnostic decisions.

Incident evaluation prioritizes supported outage-cause identification, the causal link rather than merely a detected hazard, material-alternative handling, time and measurements to diagnosis, and unnecessary collection after accepted completion. Flow error and observability remain supporting diagnostics; incomplete reconstruction does not count as diagnostic failure. General reconstruction completion is scored as an objective only in the explicitly selected `healthy_benchmark`. Hidden truth evaluates conclusions after the fact and never supplies the manager's stopping decision.

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
