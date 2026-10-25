# Standing orders
Implement docs/three-method-outage-plan.md in P1, P2, P3, P4 order. Preserve existing user edits. No publishing, deployment, merge, or scheduled automation. Require real evidence and mark unsupported verification honestly.

- [x] The operator's gates come first.
- [ ] Spawn one owner per PR, and keep its work where a replacement can find it.
  Local isolated checkout per implementation unit. skip: publishing PRs was explicitly excluded.
- [ ] Run owners in parallel off trunk, never stacked,
  skip: plan requires dependent P1-P4 sequence and serialized schema edits.
- [ ] Swarm-verify once.
- [ ] The operator merges only what you verified, tested on the trunk it lands on.
  skip: no merge requested.
- [ ] Run the root layer.
  skip: hourly automation explicitly excluded for bounded implementation.
- [ ] Stand down at once on the operator's stop.
- [x] Ground
- [x] Sketch
- [x] Agree
- [ ] Implement
- [ ] Scrap
  n/a unless architecture repeatedly fails.
- [x] Blocking first steps. Read contracts and record baseline before implementation.
- [ ] Independent workstreams. Read-only design and verification may parallelize.
- [ ] Shared mutable state. One writer owns each dependent schema migration.
- [ ] Smallest safe decomposition. Four ordered implementation units prevent schema races.
- [ ] P1 private episodes, sensing, energy, verification driver.
- [ ] P2 requests, itinerary routing, persistent execution, UI migration.
- [ ] P3 public reconstruction, experiment value, honest completion.
- [ ] P4 evaluation, recordings, presentation, end-to-end evidence.
