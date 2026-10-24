from edit import section,swap
section('## Replanning rules','### Task and observation lifecycle', '''## Replanning rules

| Drone state | Participation | Effective start | Remaining budget |
|---|---|---|---|
| Ready | Wait or start one sortie | Exact current perch at current clock | Current full battery |
| Flying | Replan the unflown suffix | Exact airborne position at current clock | Current `battery_min` |
| Measuring | Preserve locked attempt and replan its continuation | Exact stop at dwell completion | Current battery minus remaining dwell |
| Charging | Fixed occupancy until the required charge completes | Not a solver vehicle | No early departure |
| Stranded | Fixed occupancy until charging capability returns and full charging completes | Not a solver vehicle | No unauthorized relaunch |

A measurement in progress finishes unless a recorded physical interruption makes it fail. Dwell and recharge are never shortened by a replan. Keep completed and failed work in history. Trim and revalidate a current route before using it as an incumbent; CP-SAT receives valid hints, not a guarantee of improving local search. Compare returned candidates under the same current objective and retain a still-valid continuation if it scores better. Switching is a lower objective tier than unique diagnostic benefit and cannot block a necessary feasibility change.

All dispatched drones must land by T+04:00 and then complete the required 180-minute recharge before any later departure. A Ready drone with no useful feasible sortie stays perched without a fictitious flight. Replanning does not move charging or stranded drones, erase their occupancy or reset their timers. An interrupted charge starts over only when actual charging capability returns.

''')
swap('''    assessment: HypothesisAssessment
    priority: int''','''    assessment: HypothesisAssessment
    lifecycle: HypothesisLifecycle
    priority: int''')
swap('''    hypotheses: dict[str, Hypothesis]
    revision: SnapshotRevision''','''    hypotheses: dict[str, Hypothesis]
    predictions: dict[str, Prediction]
    measurement_needs: dict[str, MeasurementNeed]
    benefit_groups: dict[str, BenefitGroup]
    revision: SnapshotRevision''')
swap('''def observability_gaps(network: Network, est: GridEstimate, k: int) -> list[MeasurementTask]: ...''','''def observability_gaps(model: ElectricalModel, est: GridEstimate,
                       k: int) -> list[MeasurementNeed]: ...''')
swap('''The manager owns urgency 1 to 5 on each investigation need.''','''The manager owns urgency 1 to 5 on each investigation need. `Hypothesis.priority` stores that urgency; `Hypothesis.lifecycle` is `ACTIVE` or `RETIRED`, independent of evidential assessment.''')
swap('''It removes redundant or invalid candidates, preserves in-progress work, generates baseline and confirmation candidates, and recomputes conditional information values.''','''It removes redundant or invalid candidates, preserves in-progress work, generates baseline and confirmation candidates, and recomputes conditional information values. If an estimate is unavailable, generate labelled topology-based or diagnostic candidates without fabricated covariance gains.''')
swap('''The revision records the decision and source snapshot, hypothesis ID, full before and after values, actual trigger event IDs, all contributing evidence IDs, and the latest measurement IDs and timestamps in that evidence.''','''The revision records the decision and source snapshot, hypothesis ID, full before and after values, actual trigger event IDs, all contributing evidence IDs, and the latest measurement IDs and timestamps in that evidence. Every measurement-triggered revision must identify the current triggering batch and explicitly state what its latest observation added; older supporting evidence cannot be substituted for the new finding.''')
swap('''H1's stored LLM explanation reads, for example:''','''H1's stored LLM explanation, with H1-alt creation explained separately, reads for example:''')
