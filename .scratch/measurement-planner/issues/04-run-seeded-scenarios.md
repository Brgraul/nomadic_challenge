# 04: Run seeded incident and healthy scenarios

**What to build:** Offer reproducible incident_demo and healthy_benchmark exercises with briefs that match their objectives, hidden electrical physics, and numeric sensor observations.

**Blocked by:** 03: Execute validated multi-stop measurement sorties.

**Status:** ready-for-agent

- [ ] Incident mode retains all six reports and LINE_024 precautionary switching without claiming that the protection event is a confirmed trip there; healthy mode explicitly selects counterfactual reconstruction.
- [ ] AC truth uses explicit voltage buses, transformers and registry circuits, declared source selection, bounded seeded injections and parameter variation; COMPONENT_07 uses SUBSTATION_03.
- [ ] Unsupplied islands remain explicitly de-energized without an invented replacement source. Construction and electrical transitions require finite converged states and the declared scenario limits.
- [ ] Construction tries at most 100 operating points, validates starting charging capability, and returns ScenarioInfeasible with useful diagnostics when constraints cannot be met.
- [ ] Electrical readings contain circuit-resolved terminal-equivalent quantities with documented sign and unit conventions, public calibration covariance and uncertainty metadata.
- [ ] Local fixtures include LINE_058 vegetation, qualified clean evidence, LINE_029 thermal excitation and an inconclusive low-load case, motion gust conditions and contradictory switching evidence.
- [ ] Observation noise is keyed by scenario seed, canonical observation identity, kind, epoch and repeat index; shared-calibration fixtures retain their declared correlations.
- [ ] Scenario selection and resulting public observations are visible in the console; hidden fault coordinates, injections, noise realizations and physical epoch IDs remain private.
