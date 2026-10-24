# 01: Isolate hidden physics from public state

**What to build:** Preserve the existing dispatch experience while making public observations the only source of operational knowledge. Keep hidden physics and truth inspection behind a separate simulation/evaluation boundary.

**Blocked by:** None (can start immediately).

**Status:** ready-for-agent

- [ ] Existing operator dispatch, simulation controls and public map continue to work after the prefactor.
- [ ] Hidden energization and scenario state are absent from public World state, immutable agent snapshots, ordinary logs, diagnostics and command results.
- [ ] The simulation alone owns mutable operational state; consumers receive public snapshots rather than handles to hidden physics.
- [ ] Any truth inspection is explicitly separated from the operational view and cannot feed agent tools or decisions.
- [ ] Import-boundary checks and data-flow leakage probes cover the public surfaces, including nested payloads.
