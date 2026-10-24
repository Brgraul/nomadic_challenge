from .types import SegmentHypothesis


def build_prompt(h: SegmentHypothesis) -> str:
    v = h.vegetation
    imp = h.impact
    s = h.sensor_readings

    # --- Vegetation block ---
    veg_lines = [
        f"Forest overlap within 50m buffer: {v.forest_overlap_pct:.0f}%",
    ]
    if v.nearest_forest_m is not None:
        veg_lines.append(f"Distance to nearest forest: {v.nearest_forest_m:.0f}m")
    if v.max_canopy_height_m is not None:
        veg_lines.append(f"Max canopy height within 50m: {v.max_canopy_height_m:.1f}m")
    if v.mean_canopy_height_m is not None:
        veg_lines.append(f"Mean canopy height: {v.mean_canopy_height_m:.1f}m")
    if v.estimated_clearance_m is not None:
        veg_lines.append(f"Estimated electrical clearance: {v.estimated_clearance_m:.1f}m")
        if v.max_canopy_height_m is not None:
            gap = v.estimated_clearance_m - v.max_canopy_height_m
            veg_lines.append(f"Tree-to-wire gap: {gap:.1f}m ({'NEGATIVE — contact possible' if gap < 0 else 'positive — clearance maintained under calm conditions'})")
    veg_lines.append(f"Dominant species: {v.dominant_species} ({'high fragility — shallow roots' if v.dominant_species == 'coniferous' else 'lower fragility' if v.dominant_species == 'deciduous' else 'mixed fragility'})")
    veg_lines.append(f"Wind exposure score: {v.wind_exposure_score:.2f} (0=sheltered, 1=fully upwind of assumed SW storm wind)")
    veg_text = "\n".join(f"  {l}" for l in veg_lines)

    # --- Impact block ---
    critical = imp.downstream_hospitals + imp.downstream_water_works + imp.downstream_care_homes
    imp_lines = [
        f"Downstream households: {imp.downstream_households:,}",
        f"Downstream population: {imp.downstream_population:,}",
        f"Downstream power plants: {imp.downstream_renewable_plants} units, {imp.downstream_power_plants_mw:.1f} MW",
        f"Critical infrastructure: {critical} (hospitals={imp.downstream_hospitals}, water={imp.downstream_water_works}, care homes={imp.downstream_care_homes})",
    ]
    imp_text = "\n".join(f"  {l}" for l in imp_lines)

    # --- Sensor block ---
    sensor_parts = []
    if s.line_live is not None:
        sensor_parts.append(f"line_live={s.line_live}")
    if s.vibration_anomaly is not None:
        sensor_parts.append(f"vibration_anomaly={s.vibration_anomaly}")
    if s.sag_angle_anomaly is not None:
        sensor_parts.append(f"sag_angle_anomaly={s.sag_angle_anomaly}")
    if s.pd_detected is not None:
        sensor_parts.append(f"partial_discharge={s.pd_detected}")
    if s.baseline_comparison:
        sensor_parts.append(f"baseline: {s.baseline_comparison}")
    sensor_text = ", ".join(sensor_parts) if sensor_parts else "No sensor data yet."

    # --- Reports block ---
    report_text = (
        "No reports reference this segment."
        if not h.reports
        else "\n".join(
            f"  [{r.id}] {r.type}, confidence={r.confidence}: {r.description}"
            for r in h.reports
        )
    )

    return f"""You are evaluating a power grid segment for vegetation-caused faults after a windstorm.
The focus hypothesis is: a tree has contacted or is about to contact the conductor on this segment.

SEGMENT: {h.segment_id}
Voltage: {h.voltage_kv} kV | Length: {h.length_km:.1f} km | Live: {h.line_is_live} | In storm footprint: {h.in_storm_footprint}

VEGETATION RISK (from OSM forests, canopy height map, DLR species map):
{veg_text}

DOWNSTREAM IMPACT (from Zensus 2022, Marktstammdatenregister, OSM critical infra):
{imp_text}

SENSOR READINGS:
{sensor_text}

INCOMING REPORTS:
{report_text}

NOTES: {h.notes or 'None.'}

---

Return four probabilities from 0.0 to 1.0:

1. vegetation_contact_probability
   Probability that a tree is currently touching or is imminently about to contact the conductor.
   High (→1.0): forest overlaps buffer, canopy height exceeds or approaches clearance, coniferous species, high wind exposure, vibration/sag anomaly detected.
   Low (→0.0): no forest nearby, trees clearly shorter than clearance, deciduous/bare species, sheltered from wind, sensors all normal.

2. impact_probability
   Probability that failure of this segment causes significant customer or infrastructure impact.
   High (→1.0): thousands of households downstream, critical infrastructure (hospital, water works), large renewable plant.
   Low (→0.0): few or no households downstream, no critical infra, negligible power plant capacity.

3. report_credibility
   Probability that incoming reports credibly and specifically point to this segment as the fault location.
   High (→1.0): medium/high-confidence report covers a narrow area containing this segment.
   Low (→0.0): no reports, reports too broad, low confidence, or already explained by another cause.

4. innocent_explanation
   Probability that a benign explanation fully accounts for all observations on this segment.
   High (→1.0): all anomalies explained by planned switching, confirmed generator test, or expected weather with no damage.
   Low (→0.0): at least one anomaly remains unexplained after ruling out known benign causes.
"""
