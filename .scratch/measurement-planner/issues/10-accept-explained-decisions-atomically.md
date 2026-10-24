# 10: Accept explained decisions atomically

**What to build:** Accept an explained hypothesis update, its candidate model and operational consequences as one validated decision, while retaining inspectable proposed findings when acceptance fails.

**Blocked by:** 05: Base landing eligibility on public charging evidence; 09: Generate evidence-linked manager findings.

**Status:** ready-for-agent

- [ ] Versioned commands carry decision identity, source snapshot/cycle, semantic dependencies and before values; retries return stored outcomes without repeating effects.
- [ ] Changed evidence, epochs, models, sensor assumptions, hypotheses, manual intent or relevant policy reject incompatible proposals. Pure motion rebases operational calculations from live state.
- [ ] Model changes are staged, independently tested and accepted with their explained revisions; enqueueing a request never changes the accepted model.
- [ ] Candidate hypotheses, explanations, model, needs, tasks and all routes remain private until every validation succeeds; failure leaves accepted state unchanged.
- [ ] Validated proposed findings and independently checked recommendations appear as PROPOSED_NOT_ACCEPTED notices even when routing is blocked, with acceptance, rejection and supersession history.
- [ ] Only the current cycle and accepted decision can release its hold; late releases are rejected and operator pause remains independent.
- [ ] Rejection after acquiring a hold attempts the same validated route-only or landing continuation and releases when safe; rejection before a hold preserves the current valid continuation.
- [ ] Tests cover stale explanations, changed before values, duplicate commands, later-sortie rejection and failed acceptance without partial mutations.
