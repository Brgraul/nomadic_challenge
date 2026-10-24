# Tasks and route acceptance

[Design index](../planner-design.md). Read for in-progress work, reservations, repeats, battery limits, exact landings, occupancy or the deadline.

Related contracts: [Drone phases](drone-state.md), [Route objective](optimization.md).

## Replanning rules

| Drone state | Participation | Effective start | Remaining budget |
|---|---|---|---|
| Ready | Wait or start one sortie | Exact current perch at current clock | Current full battery |
| Flying | Replan the unflown suffix | Exact airborne position at current clock | Current `battery_min` |
| Measuring | Preserve locked attempt and replan its continuation | Exact stop at dwell completion | Current battery minus remaining dwell |
| Charging | Fixed occupancy until the required charge completes | Not a solver vehicle | No early departure |
| Stranded | Fixed occupancy until charging capability returns and full charging completes | Not a solver vehicle | No unauthorized relaunch |

A measurement in progress finishes unless a recorded physical interruption makes it fail. Dwell and recharge are never shortened by a replan. Keep completed and failed work in history. Trim and revalidate a current route before using it as an incumbent; CP-SAT receives valid hints, not a guarantee of improving local search. Compare returned candidates under the same current objective and retain a still-valid continuation if it scores better. Switching is a lower objective tier than unique diagnostic benefit and cannot block a necessary feasibility change.

All dispatched drones must land by T+04:00 and then complete the required 180-minute recharge before any later departure. A Ready drone with no useful feasible sortie stays perched without a fictitious flight. Replanning does not move charging or stranded drones, erase their occupancy or reset their timers. An interrupted charge starts over only when actual charging capability returns.

## Task and observation lifecycle

A site has a stable registry ID and exact unrounded coordinates. Serialized local offsets use integer millimetres so identity does not depend on formatting a floating-point value. For electrical ranking, map all offsets observing the same circuit aggregate to one canonical observation group; distinct spatial site IDs do not create distinct state equations.

`PENDING -> RESERVED` occurs only when an accepted plan assigns the attempt to one drone. `RESERVED -> MEASURING` occurs on arrival. Completion stores one immutable reading and marks the attempt `COMPLETED`. Replanning may release an unstarted reservation back to `PENDING`, cancel it with a reason, or reassign it atomically. A failed attempt records `FAILED`; confirmation creates a new attempt rather than rewriting the failed or completed one. Histories and completed-task metadata are never removed by a replacement open-task list.

A `MEASURING` attempt is locked to its drone until completion or a recorded physical interruption. It is excluded from the optional solver set and cannot be assigned again. For that drone's continuation, start at its current stop at `world.clock + remaining_dwell`, with `battery_min - remaining_dwell`. Route acceptance includes the locked dwell and suffix exactly once. Even if its hypothesis is retired, the in-progress observation finishes; its subsequent interpretation may be inconclusive. Outstanding reservations and their dependencies are retained in each decision snapshot.

A repeat task requires a purpose and links its earlier reading or diagnostic. Its noise independence is assessed by the sensor model; repeated measurements do not automatically provide new structural rank or remove shared calibration bias. Full reading metadata includes position, kind, circuit payload, epoch, covariance and detection conditions, so a later explanation remains meaningful after the task leaves the open set.

Every incident diagnostic task's `purpose` identifies its active measurement need, the causal uncertainty being tested and how the result could affect the diagnosis or next action. Revalidate that association during refresh and route acceptance. On accepted retirement of a need, cancel its unstarted attempts unless another active need independently justifies them, and remove them before comparing a retained route with new candidates. A completed diagnosis therefore cannot retain obsolete stops merely because they still earn flow or coverage utility. Locked work and validated recovery routes remain required.

## Route acceptance invariants

Every accepted plan accounts for each `Ready` drone as either waiting at its present perch or taking a sortie, and every `Flying` or `Measuring` drone with exactly one executable continuation. Charging and stranded drones retain their state and occupy their actual perches; they are not silently offered as available vehicles. A ready drone may wait without taking off, consuming battery or restarting a charge.

Validate site and circuit existence, offset bounds, sensor capability, unique attempt ownership, locked work, exact landing geometry, current landing policy and freshness through predicted arrival. Known hazards invalidate their affected landing intervals. Check battery and remaining dwell, every stop's completion time, and landing by T+04:00. There is no hidden post-deadline flight in evaluation. A route-only fallback must pass the same checks.

The landing-site candidate set includes sampled points, existing drone perches, task locations, and nearest eligible projections from each current drone position and each candidate final measurement stop onto every reachable segment. All have explicit circuit identity. Sampling alone must not create a false inability to land. Use a conservative common seconds model: ceil each nonzero travel and dwell duration, floor remaining battery seconds, and use the same durations in optimization, validation and simulation.

The exercise allows at most one drone at a registered landing point and circuit. Include charging and stranded occupancy, reserve planned landing occupancy through the next departure or scenario end, and conservatively exclude a point whose occupant has no accepted departure. Separate exact points on a long line may host separate drones; this is a declared simplified occupancy model. If route timing is not modeled precisely enough to prove a handover, keep that point unavailable rather than double-book it.
