# 05: Base landing eligibility on public charging evidence

**What to build:** Authorize landings from fresh circuit-specific observations and declared policy, so route acceptance never relies on hidden energization or a combined line-status label.

**Blocked by:** 04: Run seeded incident and healthy scenarios.

**Status:** ready-for-agent

- [ ] Public circuit energization, chargeability, fault assessment and inspection coverage are independent records with evidence provenance.
- [ ] Voltage classification uses the specified uncertainty thresholds; confirmed chargeability additionally requires supported current of at least 5 A or a successful charging handshake.
- [ ] Starting perches have explicit handshake observations. Successful telemetry renews only the observed circuit while perched.
- [ ] Contradictory qualified evidence, known switching and scheduled evidence expiry invalidate dependent endpoints before routing; zero current alone does not establish de-energization.
- [ ] Default endpoints remain freshly confirmed through predicted arrival, hazard-free and compatible with retained plausible models; any optional uncertain-endpoint policy is explicit and cannot override known hazards or unavailability.
- [ ] Landing candidates include current perches, task sites, sampled points and exact feasible projections from drone positions and candidate final stops.
- [ ] Occupancy includes charging and stranded drones and planned endpoints; unproved handovers are rejected. Ready drones may wait without a flight or recharge.
- [ ] Operator views explain endpoint eligibility, expiry, conflicts and interrupted charging; regressions cover expiry boundaries and sparse landing grids.
