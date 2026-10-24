# 14: Replan the fleet immediately after relevant events

**What to build:** React to observations and operational changes by reconsidering the entire available fleet immediately, using live positions and accepted diagnostic meaning without waiting for an LLM.

**Blocked by:** 13: Optimize bounded fleet sorties with CP-SAT.

**Status:** ready-for-agent

- [ ] Observable batches refresh evidence and affected diagnostics, publish a snapshot and notify the coordinator before the bounded route pass, even when routing will fail.
- [ ] Readings, dispositions, known switching, accepted model or policy changes, evidence expiry, availability and manual requests trigger the appropriate refresh and route checks.
- [ ] Flying drones can divert from interpolated positions with spent battery preserved; Measuring drones finish their locked attempt and receive only a new suffix.
- [ ] Charging and stranded drones retain positions, occupancy and timers; a no-benefit diversion is discouraged only after feasibility and diagnostic benefit.
- [ ] Every proposed fleet continuation validates atomically; solver timeout, invalid output or no improvement uses a revalidated incumbent or landing-only recovery.
- [ ] If no complete safe continuation exists, retain a bounded explicit hold with a specific operator-visible cause; unsafe routes never release it.
- [ ] Route-only decisions do not invent hypothesis revisions. Event and route-publication timestamps support end-to-end latency measurement.
- [ ] Regressions exercise cross-drone mid-leg updates, locked measurements, expiry, occupied endpoints, deadline edges and solver failures.
