# 18: Compare policies with isolated scoring and replay

**What to build:** Run headless four-hour exercises and compare random, greedy and agent policies with truth-isolated metrics, reproducible observations and honest limits on coverage claims.

**Blocked by:** 17: Verify response budgets and failure recovery.

**Status:** ready-for-agent

- [ ] A headless evaluation entry point selects scenario, policy and seed, runs to T+04:00 and writes results by evidence batch and simulation time.
- [ ] Random, greedy and agent policies share initialization, task caps, landing rules, sensor model and deadline; declared mode differences remain visible.
- [ ] Incident scoring prioritizes causal diagnosis, material alternatives, time and measurements to diagnosis, scoped actions and unnecessary post-completion collection.
- [ ] Healthy reconstruction scoring reports flow error, observability, detectability and calibration separately; unsupported intervals and unresolved work are counted explicitly.
- [ ] Operational and optimization results include flight/dwell/charge timing, repeats, landing failures, urgent unassigned work, fallback frequency, bounds and tiny-case gaps.
- [ ] Paired seed distributions use observation-keyed noise; replay retains LLM inputs/outputs, solver versions and budgets, and labels wall-watchdog variability.
- [ ] At least a small two-sortie lookahead baseline tests the one-sortie policy's myopic limitation before four-hour coverage claims are made.
- [ ] Scores remain outside public World state, tools and decision payloads; separate result views cannot influence live decisions or stopping rules.
