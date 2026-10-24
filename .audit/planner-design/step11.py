from edit import swap,insert_before,section,record
swap('''    perch_segment: str          # believed-live line it ends on
    airtime_minutes: float''','''    available_from: float
    locked_dwell_minutes: float
    landing: LandingSite
    landing_clock: float
    airtime_minutes: float''')
swap('''    sorties: tuple[Sortie, ...]
    unassigned: tuple[str, ...]

def plan_sorties(world: World, network: Network, tasks: Sequence[MeasurementTask],
                 current: Plan | None) -> Plan: ...''','''    source_revision: SnapshotRevision
    sorties: tuple[Sortie, ...]
    waiting_drone_ids: tuple[str, ...]
    unassigned: tuple[UnassignedTask, ...]
    landing_policy_revision: str

def plan_sorties(inputs: PlanningInput, current: Plan | None) -> PlanningResult: ...''')
swap('''# Network gains:''','''@dataclass(frozen=True)
class LandingSite:
    point: LinePoint
    circuit_id: str

# Network gains:''')
swap('''    perch: LinePoint            # every route ends on a perch, by construction''','''    perch: LandingSite''')
s=__import__('edit').DOC.read_text().replace('    perch: LinePoint\n','    perch: LandingSite\n')
__import__('edit').DOC.write_text(s)
swap('''- **A route always ends on a perch.** `Route` has a required `perch` field, so a route that leaves a drone in the air can't be built.
- **The sim checks the battery when it commits a plan.** `commit` rejects the whole plan if any sortie's flight and dwell time exceeds that drone's `battery_min`. The solver should never produce such a plan, so this check guards the boundary. It isn't a fallback.''','''- **A route names an exact landing point and circuit.** This makes the intended endpoint explicit but does not prove reachability or chargeability. `validate_plan` checks those conditions separately.
- **Acceptance validates every route before any mutation.** Recalculate flight, locked dwell, task dwell and landing time from authoritative coordinates, speed and current battery. A rejected sortie rejects the complete candidate decision without changing any drone. Do not trust the solver's reported airtime.''')
swap('''- **Position in flight is interpolated** between `origin` and `route.target` by `(clock - departed) / (arrives - departed)`. The interpolation is linear in lon/lat. Over 30 km at this latitude the error should be negligible, though that's a guess and not measured. At stops and perches the position is the exact point on the line.''','''- **Position follows the common geometry and time model.** For a positive-duration leg, interpolate from `origin` to `route.target` using elapsed flight fraction. A zero-distance arrival is processed immediately. At stops and landing sites, resolve the exact registered point. Geometry error is bounded by the configured route-validation tolerance rather than assumed negligible.''')
insert_before('## Data shape', '''### Route acceptance invariants

Every accepted plan accounts for each `Ready` drone as either waiting at its present perch or taking a sortie, and every `Flying` or `Measuring` drone with exactly one executable continuation. Charging and stranded drones retain their state and occupy their actual perches; they are not silently offered as available vehicles. A ready drone may wait without taking off, consuming battery or restarting a charge.

Validate site and circuit existence, offset bounds, sensor capability, unique attempt ownership, locked work, exact landing geometry, current landing policy and freshness through predicted arrival. Known hazards invalidate their affected landing intervals. Check battery and remaining dwell, every stop's completion time, and landing by T+04:00. There is no hidden post-deadline flight in evaluation. A route-only fallback must pass the same checks.

The landing-site candidate set includes sampled points, existing drone perches, task locations and each drone's nearest eligible projection on each reachable segment. All have explicit circuit identity. Sampling alone must not create a false inability to land. Use a conservative common seconds model: ceil each nonzero travel and dwell duration, floor remaining battery seconds, and use the same durations in optimization, validation and simulation.

The exercise allows at most one drone at a registered landing point and circuit. Include charging and stranded occupancy, reserve planned landing occupancy through the next departure or scenario end, and conservatively exclude a point whose occupant has no accepted departure. Separate exact points on a long line may host separate drones; this is a declared simplified occupancy model. If route timing is not modeled precisely enough to prove a handover, keep that point unavailable rather than double-book it.
''')
record(11,13,'Preserve exact circuit landing sites and require complete independently validated routes','docs/planner-design.md#route-acceptance-invariants')
