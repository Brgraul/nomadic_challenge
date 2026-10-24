# 13: Optimize bounded fleet sorties with CP-SAT

**What to build:** Provide revision-labelled previews and executable candidate plans that allocate ordered measurement stops and eligible landings across the available fleet within explicit search budgets.

**Blocked by:** 12: Value shared evidence without multiplying rewards.

**Status:** ready-for-agent

- [ ] CP-SAT assigns unique attempts to connected vehicle paths, preserves locked work and fixed occupancy, permits Ready waiting, and gives every airborne drone an executable landing continuation.
- [ ] Time, battery, dwell, endpoint freshness, hazards, occupancy and T+04:00 constraints use the same authoritative rules as independent validation.
- [ ] Lexicographic scoring uses bounded safe integer weights or staged solves with honest proof status; no heuristic priority multiplier silently replaces urgency ordering.
- [ ] At most three reranking rounds share the declared deterministic budget; the worker watchdog includes construction, solving and joint scoring and cannot mutate live state.
- [ ] The 20-attempt cap preserves required work and highest-urgency representation, reports every omission, and exposes PlanningBlocked when required work alone exceeds capacity.
- [ ] Current routes are trimmed, revalidated and rescored as incumbents; newly accepted candidates are no worse under the same fixed current objective.
- [ ] Results expose statuses, unassigned reasons, revisions, bounds where valid, replay settings and stop reasons; only feasible independently validated plans can commit.
- [ ] Tiny exact enumeration covers fleet allocation, ordering, shared benefits and landing choices, reporting measured objective gaps without overstating optimality.
