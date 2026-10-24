from .types import SegmentHypothesis


def build_prompt(h: SegmentHypothesis) -> str:
    """Build the Gemini prompt for a single segment hypothesis."""

    report_text = (
        "No reports reference this segment."
        if not h.reports
        else "\n".join(
            f"  [{r.id}] type={r.type}, confidence={r.confidence}: {r.description}"
            for r in h.reports
        )
    )

    s = h.sensor_readings
    sensor_parts = []
    if s.line_live is not None:
        sensor_parts.append(f"line_live={s.line_live}")
    if s.vibration_anomaly is not None:
        sensor_parts.append(f"vibration_anomaly={s.vibration_anomaly}")
    if s.sag_angle_anomaly is not None:
        sensor_parts.append(f"sag_angle_anomaly={s.sag_angle_anomaly}")
    if s.pd_detected is not None:
        sensor_parts.append(f"pd_detected={s.pd_detected}")
    if s.baseline_comparison:
        sensor_parts.append(f"baseline: {s.baseline_comparison}")
    sensor_text = ", ".join(sensor_parts) if sensor_parts else "No sensor data available."

    v = h.vegetation
    veg_parts = [f"forest_cover_nearby={v.forest_cover_nearby}"]
    if v.estimated_tree_height_m is not None:
        veg_parts.append(f"tree_height={v.estimated_tree_height_m}m")
    if v.estimated_line_clearance_m is not None:
        veg_parts.append(f"line_clearance={v.estimated_line_clearance_m}m")
    veg_parts.append(f"wind_alignment={v.wind_alignment}")
    veg_parts.append(f"tree_fragility={v.tree_species_fragility}")
    veg_text = ", ".join(veg_parts)

    return f"""You are evaluating whether an operator crew should be dispatched to inspect a power grid segment after a windstorm.

SEGMENT: {h.segment_id}
Voltage: {h.voltage_kv} kV | Length: {h.length_km:.1f} km | Live: {h.line_is_live} | In storm footprint: {h.in_storm_footprint}

VEGETATION:
{veg_text}

SENSOR READINGS:
{sensor_text}

INCOMING REPORTS:
{report_text}

NOTES: {h.notes or 'None.'}

---

Rate each of the following as a probability from 0.0 to 1.0:

1. vegetation_risk: Could a tree physically contact the conductor on {h.segment_id}?
   - High (→1.0) if: forest nearby, trees taller than clearance, upwind, fragile species (spruce).
   - Low (→0.0) if: no forest, trees clearly too short, downwind, stable bare deciduous trees.

2. sensor_anomaly: Do sensor readings show an anomaly inconsistent with normal post-storm conditions?
   - High (→1.0) if: vibration, sag, or partial discharge deviates significantly from pre-storm baseline.
   - Low (→0.0) if: all readings normal or no sensor data (treat missing data as uncertainty ~0.5).

3. report_credibility: Do the incoming reports specifically and credibly implicate {h.segment_id}?
   - High (→1.0) if: medium/high-confidence report covers a narrow area containing this segment.
   - Low (→0.0) if: reports are too broad, low confidence, zero reports, or already explained by another cause.

4. innocent_explanation: Is there a full benign explanation for all observations?
   - High (→1.0) if: all observations explained by planned switching (e.g. LINE_024), confirmed generator test, or expected weather with no damage.
   - Low (→0.0) if: at least one anomaly remains unexplained after ruling out known benign causes.

Return a JSON object with these four float fields only."""
