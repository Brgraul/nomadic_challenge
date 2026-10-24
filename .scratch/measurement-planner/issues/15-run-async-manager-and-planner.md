# 15: Run manager and planner asynchronously during flight

**What to build:** Run the manager and planner over pinned public evidence while accepted drone routes continue, and incorporate compatible explained updates through a second bounded acceptance pass.

**Blocked by:** 14: Replan the fleet immediately after relevant events.

**Status:** ready-for-agent

- [ ] The coordinator uses manager and planner roles with no per-drone LLM agents; all tools remain read-only and revision-pinned.
- [ ] Every reading batch is retained for manager assessment, including when routes are blocked; arriving evidence is coalesced into a current turn without losing trigger IDs.
- [ ] The manager owns hypotheses, urgency and recommendations; the planner proposes needs and exact sites and runs only when semantic planning needs change.
- [ ] Planner previews obey the same scoring and route rules as acceptance, explain infeasibility, and authorize no dispatch.
- [ ] A manager delayed by 20 wall seconds cannot hold a valid fleet; simulation time and reactive replanning continue.
- [ ] Compatible proposals rebase from live state, incompatible turns are archived as superseded and regenerated, and old cycles cannot release new holds.
- [ ] Coordinator timeout or death leaves a validated operational continuation or explicit safety exception; accepted state never depends on an unfinished LLM call.
- [ ] The console distinguishes active routes, pending reasoning and proposed versus accepted explanations; asynchronous integration tests verify these behaviors.
