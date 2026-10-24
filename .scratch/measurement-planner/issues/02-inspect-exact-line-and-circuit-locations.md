# 02: Inspect exact line and circuit locations

**What to build:** Let operators identify precise inspection and landing locations along mapped lines, with explicit circuit identities and separate inspection footprints.

**Blocked by:** 01: Isolate hidden physics from public state.

**Status:** ready-for-agent

- [ ] Build the public synthetic circuit registry from every ordered voltage-list entry, preserving repeated voltages; allocate additional circuits round-robin and reject contradictory counts.
- [ ] Each circuit exposes its nominal voltage, endpoints, declared electrical parameters and rating without hidden realized values.
- [ ] Along-line points resolve consistently to map coordinates; nearest projections and clipped report-area samples include endpoints and boundaries.
- [ ] Stable site identity uses registered exact positions and integer-millimetre offsets rather than formatted floating-point coordinates.
- [ ] The operator can select and inspect exact sites and circuits on the map; local inspection sampling and landing sampling remain distinct.
- [ ] Geometry verification covers long and bent lines, offset bounds, clipping and the declared distance/interpolation tolerance.
