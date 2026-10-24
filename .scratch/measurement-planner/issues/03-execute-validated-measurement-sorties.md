# 03: Execute validated multi-stop measurement sorties

**What to build:** Allow an operator to request a complete measurement sortie that flies through exact stops, emits readings after dwell, and lands safely. Validate the entire fleet decision before changing any drone.

**Blocked by:** 02: Inspect exact line and circuit locations.

**Status:** ready-for-agent

- [ ] Manual sorties execute ordered measurement stops and an exact circuit landing, using the physics observation interface and recording immutable readings.
- [ ] Ready, Flying, Measuring, Charging and Stranded transitions preserve exact positions, remaining airtime and task histories; only completed charging restores full battery.
- [ ] Travel and dwell use the shared conservative seconds model; zero-distance arrivals complete without division by zero or artificial travel drain.
- [ ] Advance processes transitions at their exact timestamps to a fixed point, applying physical changes before charging consequences and sensing; hidden-only changes produce no invented public events.
- [ ] Attempts have unique ownership and explicit pending, reserved, measuring, completed, cancelled or failed states. Repeats create new attempts linked to earlier evidence.
- [ ] In-progress dwell remains locked during route replacement, including retirement of its hypothesis; its remaining dwell is counted exactly once.
- [ ] Independent validation recomputes every route from authoritative geometry and state, checks sensor capability and battery, and rejects any stop or landing beyond T+04:00.
- [ ] An invalid later sortie leaves every earlier drone unchanged; the console shows routes, tasks, readings, dwell and rejection reasons.
