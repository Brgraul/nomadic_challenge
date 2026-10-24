# 17: Verify response budgets and failure recovery

**What to build:** Make measured response latency and failure outcomes inspectable, and demonstrate that the complete investigation remains operationally safe under representative timing and failure cases.

**Blocked by:** 16: Conclude or reopen an outage investigation safely.

**Status:** ready-for-agent

- [ ] Measure observation-ingestion to validated-plan-publication latency over a declared representative seed and event set, targeting p95 at most one second and a two-second response budget.
- [ ] Include diagnostics, refresh, construction, solver, scoring and acceptance in timings; report budget misses and safety exceptions rather than masking them with simulation holds.
- [ ] Enforce the initial shared deterministic search budget, at least 0.5 seconds reserved for non-solver stages and a solver wall cap no greater than 1.5 seconds or the remaining budget.
- [ ] Fault injection covers solver UNKNOWN, MODEL_INVALID and timeout, worker termination, coordinator death, stale commands and failure after hold acquisition.
- [ ] Cross-cutting scenarios cover simultaneous switching/sensing/expiry, zero-distance stops, locked ownership, interrupted charging, atomic rollback, exact landing projection and the deadline.
- [ ] Truth-leak probes cover snapshots, tools, logs, diagnostics, notices and command results after the full workflow is connected.
- [ ] Every required implementation acceptance scenario is mapped to its owning test or reproducible demonstration; remaining failures prevent a claim of complete acceptance.
- [ ] An operator can inspect measured latency, fallback use and the concrete action required when safe continuation is impossible.
