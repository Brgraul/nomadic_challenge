# Geometry, sensors and coverage

[Design index](../planner-design.md). Read for line positions, local footprints, sensor payloads, terminal-equivalent sensing, noise or negative evidence.

Related contracts: [Circuit registry](electrical-model.md), [Noise use](estimation.md).

## Positions on the line

Today every flight goes to a segment midpoint. That is a poor stand-in for a position. LINE_024 is 26.8 km long, so its midpoint is about 13 km from either end, and 13 of the 64 segments are longer than 5 km (measured from `grid_segments.geojson`). The design replaces midpoints with points along the line.

```python
LonLat = tuple[float, float]

@dataclass(frozen=True)
class LinePoint:
    """A point on a mapped line, measured along its path from from_node."""
    segment_id: str
    offset_km: float

@dataclass(frozen=True)
class LandingSite:
    point: LinePoint
    circuit_id: str

# Network gains:
#   position_of(point: LinePoint) -> LonLat
#   nearest_point(segment_id: str, near: LonLat) -> LinePoint
#   sample_points(segment_id: str, every_km: float, inside: Area | None) -> list[LinePoint]
```

- **Local tasks inspect bounded intervals.** Place stops no more than 1 km apart for the assumed 0.5 km along-line footprint when complete coverage is intended. Include clipped report-area boundaries and segment endpoints. A 2 km spacing would leave 1 km blind intervals and is insufficient for clearing a corridor.
- **Landing candidates use a separate discretization.** A coarse 2 km landing grid is augmented by exact task sites, current perches and nearest feasible projections. Eligibility uses circuit-specific chargeability, hazards, expiry and occupancy. Local inspection spacing is independent of landing-point sampling.
- **Electrical and local evidence have different scopes.** `CURRENT` and `VOLTAGE_PRESENCE` use the declared terminal-equivalent abstraction for each circuit. Local measurements support only their actual inspected interval and stated detection conditions. Their range does not turn a local negative into evidence about the entire line.

## Local coverage and negative evidence

For each accepted local reading, store the nominal footprint clipped to its segment, the actual inspected subintervals, sensor kind, epoch, condition flags and evidence ID. Compute union coverage by interval arithmetic, not by multiplying stop count by range. Overlapping footprints contribute new coverage only for their difference from existing valid coverage. Report occluded and uninspected intervals explicitly.

A clean reading contradicts a local prediction only where that sensor's likelihood model provides adequate detection probability under the recorded conditions. The synthetic local model uses Gaussian clearance error with 0.05 m standard deviation, temperature error with 1 degree Celsius standard deviation and motion-amplitude error with 0.02 m standard deviation. Each payload reports the most adverse quantity over its actually inspected interval. Environmental baselines and excitation flags are part of the observation model. The fixture defines likelihoods for the relevant threshold events; use a 95% detection requirement before treating a qualified negative as evidence against the scoped claim.

A single qualified negative can lower support but does not automatically refute an entire report-area hypothesis. Refutation requires the hypothesis's declared spatial and operating-condition test coverage. Thermal defect tests require current above the fixture's excitation threshold; intermittent-motion tests require a qualifying gust window. Insufficient excitation or visibility yields an inconclusive result and an explicit follow-up need. Retire only the tested scoped claim, retaining untested alternatives with explained revisions.

## Sensing

`MeasurementPayload` is a tagged union with a numeric schema per measurement kind. `CURRENT` contains one `CircuitFlow` per resolved synthetic circuit. `VOLTAGE_PRESENCE` contains voltage values by circuit ID. Local payloads contain clearance in metres, conductor temperature in degrees Celsius, or motion amplitude in metres, with the inspected interval and environmental/operating-condition flags. Raw readings are immutable; human-readable summaries do not replace numbers or uncertainty metadata.

## Electrical observation abstraction

For this exercise, a drone at any point on an uninterrupted segment obtains terminal-equivalent P, Q, current and voltage for each circuit, normalized to that circuit's `from_node`. This is an explicit synthetic sensing abstraction, not a claim that AC quantities are constant along the line or that a real drone can perform remote terminal metering. `current_a` is the from-terminal magnitude, not pandapower's maximum-end `i_ka`. Positive P and Q use the from-bus-into-line convention. Direct electrical operating checks are scoped to those represented terminal quantities; unmeasured far-end limits remain unsupported.

The truth stores both terminal powers and currents, losses, bus voltages and circuit ratings. The estimator compares aggregate measured from-terminal P with its lossless reduced prediction and includes the discrepancy allowance. If a later sensor implementation measures local fields instead, it must define the position-dependent observation function and revise the estimator and calibration together. See [pandapower's terminal quantities and line model](https://pandapower.readthedocs.io/en/v3.5.6/elements/line.html).

## Noise and detection conditions

The baseline electrical noise is zero-mean Gaussian. Each circuit's public sensor configuration supplies fixed calibration references `I_ref_a`, `P_ref_mw`, `Q_ref_mvar` and nominal kV before observations begin. Initial references are 100 A, 100 MW and 100 Mvar. Standard deviations are `max(0.01 * I_ref_a, 1 A)`, `max(0.02 * P_ref_mw, 0.5 MW)`, `max(0.02 * Q_ref_mvar, 0.5 Mvar)` and `max(0.01 * nominal_kv, 0.1 kV)`. These percentages describe calibration references, not hard plus-or-minus bounds or a percentage of the hidden actual flow. Return this public covariance; it contains no latent signal-dependent scale that could reveal truth. The base fixture assumes independent circuits, quantities and observation attempts; dedicated fixtures introduce a recorded shared calibration component and correlated errors to verify that repeating a reading cannot remove that component.

The variance of summed circuit P is `ones.T @ R_P @ ones`, not 2% of the aggregate, especially when flows cancel. Planning and estimation use the same public calibration covariance for an unmeasured candidate. Changing references or correlations is a versioned sensor-model change and requires recalibration.

Voltage decisions use the returned 95% interval: a lower bound above 10% of nominal supports `LIVE`, and an upper bound below 2% supports `DE_ENERGIZED`. Intermediate intervals remain inconclusive. These are exercise classification thresholds. Chargeability additionally needs the declared current threshold, evaluated on the lower uncertainty bound. Directly contradictory qualified evidence invalidates charging assumptions even before causal diagnosis.

The sensor model declares a 0.5 km maximum along-line footprint for local sensors. Range is not proof of detection probability. Store the actual observed interval, visibility, thermal excitation and other validity flags. Local likelihoods and fixture values are specified in the scenario section. A clean low-load thermal reading or a motion reading outside the relevant gust conditions is inconclusive about an intermittent defect. A cold or de-energized line may retain heat or be warmed by the environment.
