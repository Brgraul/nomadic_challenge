# 06: Show scoped local findings and coverage

**What to build:** Turn local sensor observations into inspectable coverage and condition findings that distinguish a detected hazard, a qualified negative and an inconclusive test.

**Blocked by:** 04: Run seeded incident and healthy scenarios.

**Status:** ready-for-agent

- [ ] Readings retain numeric payloads, actual inspected subintervals, visibility, operating conditions, evidence IDs and public epoch.
- [ ] Coverage uses clipped interval unions; overlapping inspections do not multiply inspected length, and occluded or uninspected intervals remain visible.
- [ ] Negative evidence contradicts only the tested scope under adequate detection conditions, including the declared 95% detection requirement.
- [ ] Thermal negatives without adequate current and motion negatives outside qualifying gust windows remain inconclusive rather than clearing defects.
- [ ] Scoped condition results distinguish PASS, FAIL, INDETERMINATE and NOT_SUPPORTED and cite assumptions and evidence.
- [ ] The console shows observed coverage and scoped hazard diagnostics without claiming that a local defect caused the outage or that clean flow proves mechanical integrity.
- [ ] Fixture tests exercise positive, qualified negative, inconclusive and overlapping observations and preserve untested regional alternatives.
