# 16: Conclude or reopen an outage investigation safely

**What to build:** Stop diagnostic collection once public evidence supports an outage cause and next action, recover the fleet safely, and reopen only for relevant new evidence or an operator request.

**Blocked by:** 15: Run manager and planner asynchronously during flight.

**Status:** ready-for-agent

- [ ] Incident completion requires an evidence-supported causal connection, addressed material alternatives, a scoped recommendation and explicit remaining uncertainty.
- [ ] A detected local hazard alone cannot complete outage diagnosis; urgent hazard action remains available while the causal investigation stays unresolved.
- [ ] Incomplete global flow observability or reconstruction does not prevent incident completion and cannot independently reopen it.
- [ ] Accepted explained completion retires diagnostic needs, cancels unsupported unstarted work, finishes locked attempts and commits validated recovery routes atomically.
- [ ] Ambiguous causes remain unresolved with a discriminating-reading or field-inspection recommendation; an operator may explicitly accept stated unresolved limits.
- [ ] Relevant evidence or an operator request reopens a concluded case through an explained revision; covariance changes alone do not regenerate retired work.
- [ ] Electrical operating assessment, local investigation and benchmark reconstruction retain separate scoped results; benchmark completion enforces its observability, uncertainty, consistency and detectability requirements.
- [ ] At the deadline the console reports remaining work and unmet checks rather than success, and integration tests cover completion with most flows unobserved and later reopening.
