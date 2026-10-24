# 07: Publish public electrical estimates and observability

**What to build:** Update public reduced-model electrical estimates after accepted electrical observations while clearly separating conditional estimates from quantities established by real measurements.

**Blocked by:** 04: Run seeded incident and healthy scenarios.

**Status:** ready-for-agent

- [ ] The reduced DC model uses declared circuit allocation, reactances, 100 MVA base, signed aggregate-flow rows, exact zero-injection constraints and references per connected component.
- [ ] Estimation uses accepted same-epoch aggregate from-terminal P, public covariance including correlations, and declared injection priors with generation-positive signs.
- [ ] A rank-revealing solve eliminates references and exact constraints without explicitly inverting ill-conditioned normal matrices; invalid or singular inputs return EstimateUnavailable.
- [ ] Startup is labelled prior-only. Observability excludes pseudo-measurements and repeated rows do not add structural rank.
- [ ] Known topology changes rebuild references and observability; de-energization alone does not silently locate an open switch.
- [ ] Public diagnostics and console views update before evidence is presented for manager reasoning; unchanged electrical inputs avoid unnecessary electrical solves.
- [ ] The nominal fixture reproduces zero-injection rank 24, total rank 52 and the independent 28-reading minimum at the declared tolerance; changed topology recalculates the result.
- [ ] Matching-model noiseless recovery states its exact-constraint or compatible-prior assumptions, and mismatched priors are verified as a bias case.
