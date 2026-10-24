"""Plain-data views of a public World for the console and for tests."""

from __future__ import annotations

from grid_ops.world import (
    DWELL_MINUTES,
    Charging,
    Flying,
    Measuring,
    Ready,
    Stranded,
    World,
    format_clock,
)


def _t(seconds: int) -> str:
    return format_clock(seconds / 60)


def drone_rows(world: World) -> list[dict]:
    rows = []
    for d in world.drones.values():
        match d.state:
            case Ready(perch) | Stranded(perch):
                where, until = perch.circuit_id, ""
            case Charging(perch, until_s):
                where, until = perch.circuit_id, f"charged {_t(until_s)}"
            case Flying(arrives_s=a, route=r):
                where = f"→ {len(r.stops)} stop(s) then {r.perch.circuit_id}"
                until = f"arrives {_t(a)}"
            case Measuring(stop=s, until_s=u, route=r):
                where = f"{s.kind} @ {s.point.segment_id}"
                until = f"dwell ends {_t(u)}"
        rows.append(
            {
                "drone": d.id,
                "phase": d.phase,
                "battery_min": round(d.battery_min, 1),
                "where": where,
                "next": until,
            }
        )
    return rows


def route_rows(world: World) -> list[dict]:
    rows = []
    for d in world.drones.values():
        match d.state:
            case Flying(route=r):
                locked = ""
            case Measuring(stop=s, until_s=u, route=r):
                locked = f"locked {s.task_id} until {_t(u)} ({(u - world.clock_s) // 60}m left)"
            case _:
                continue
        rows.append(
            {
                "drone": d.id,
                "locked": locked,
                "stops": " → ".join(
                    f"{s.task_id}:{s.kind}@{s.point.segment_id}/{s.point.offset_km:.3f}km"
                    for s in r.stops
                ),
                "landing": f"{r.perch.circuit_id} @ {r.perch.point.offset_km:.3f} km",
            }
        )
    return rows


def task_rows(world: World) -> list[dict]:
    return [
        {
            "task": t.id,
            "site": t.site_id,
            "kind": t.kind,
            "dwell_min": DWELL_MINUTES[t.kind],
            "status": t.status,
            "drone": t.assigned_drone_id or "",
            "repeat_of": t.repeat_of_reading_id or "",
            "needs": ", ".join(t.need_ids),
            "group": t.group_id,
            "note": t.note,
            "purpose": t.purpose,
        }
        for t in world.tasks.values()
    ]


def reading_rows(world: World) -> list[dict]:
    return [
        {
            "reading": r.id,
            "time": _t(r.clock_s),
            "drone": r.drone_id,
            "task": r.task_id,
            "site": r.site_id,
            "kind": r.kind,
            "epoch": r.operating_epoch,
            "values": r.summary,
            "repeat": r.repeat_index,
            "conditions": ", ".join(f"{k}={v:.3g}" for k, v in r.conditions),
            "inspected_m": round(sum(b - a for a, b in r.inspected) / 1000, 1)
            if r.inspected
            else "",
            "sensor_model": r.sensor_model_revision,
            "covariance_dim": len(r.covariance.names) if r.covariance else "",
        }
        for r in world.readings.values()
    ]


def rejection_rows(world: World) -> list[dict]:
    return [
        {"time": _t(r.clock_s), "by": r.author, "reason": reason}
        for r in reversed(world.rejections)
        for reason in r.reasons
    ]


def known_switching_rows(world: World) -> list[dict]:
    return [
        {"segment": seg, "reported_at": _t(t)} for seg, t in sorted(world.known_switching.items())
    ]


def endpoint_rows(world: World) -> list[dict]:
    """Per-circuit landing eligibility with its reasons, expiry, conflicts and occupant."""
    from grid_ops import knowledge

    occupant = {}
    for d in world.drones.values():
        if isinstance(d.state, Charging | Stranded):
            occupant[d.state.perch.circuit_id] = f"{d.id} ({d.phase})"
    rows = []
    for cid, ch in world.knowledge.chargeability.items():
        ck = world.knowledge.circuits[cid]
        if ch.state == "unknown" and ck.energization == "unknown" and cid not in occupant:
            continue
        from grid_ops.geometry import LandingSite, LinePoint

        site = LandingSite(LinePoint(cid.split("/")[0], 0), cid)
        verdict = knowledge.assess_landing(world, site, world.clock_s)
        rows.append(
            {
                "circuit": cid,
                "energization": ck.energization,
                "chargeability": ch.state,
                "expires": _t(ch.expires_s) if ch.expires_s is not None else "",
                "eligible_now": verdict.eligible,
                "why": "; ".join(verdict.reasons),
                "conflict": ", ".join(ck.discrepancy_ids),
                "occupant": occupant.get(cid, ""),
                "evidence": ", ".join(ch.evidence_ids),
            }
        )
    return rows


def coverage_rows(world: World) -> list[dict]:
    """Union coverage per segment and kind, with occluded and uninspected lengths."""
    from grid_ops import assessment

    rows = []
    for seg, kind in sorted({(c.segment_id, c.kind) for c in world.coverage}):
        scope = assessment.SpanRef(seg, 0, world.segment_lengths[seg])
        rep = assessment.coverage_report(world, scope, kind)
        rows.append(
            {
                "segment": seg,
                "kind": kind,
                "stops": rep.stop_count,
                "inspected_km": round(rep.inspected_mm / 1e6, 3),
                "occluded_km": round(sum(i.end_mm - i.start_mm for i in rep.occluded) / 1e6, 3),
                "uninspected_km": round(
                    sum(i.end_mm - i.start_mm for i in rep.uninspected) / 1e6, 3
                ),
                "of_km": round(rep.scope_mm / 1e6, 3),
            }
        )
    return rows


def assessment_rows(world: World) -> list[dict]:
    from grid_ops import assessment

    return [
        {
            "scope": (
                f"{a.scope.segment_id} {a.scope.start_mm / 1e6:.3f}-{a.scope.end_mm / 1e6:.3f} km"
            ),
            "condition": a.condition,
            "verdict": a.verdict,
            "evidence": ", ".join(a.evidence_ids),
            "assumptions": "; ".join(a.assumptions),
            "explanation": a.explanation,
        }
        for a in assessment.segment_assessments(world)
    ]


def hazard_rows(world: World) -> list[dict]:
    return [
        {
            "hazard": h.id,
            "segment": h.segment_id,
            "interval_km": f"{h.start_mm / 1e6:.3f}-{h.end_mm / 1e6:.3f}",
            "verdict": h.verdict,
            "evidence": ", ".join(h.evidence_ids),
            "note": "; ".join(h.assumptions),
        }
        for h in world.hazards
    ]


def estimate_rows(world: World) -> list[dict]:
    est = world.estimate
    if est is None or not hasattr(est, "flows"):
        return []
    return [
        {
            "segment": seg,
            "p_mw": round(f.p_mw, 2),
            "conditional_std_mw": round(f.conditional_std_mw, 2),
            "observable": f.observable,
            "basis": "real measurement" if f.observable else "prior-conditioned",
        }
        for seg, f in est.flows.items()
    ]


def estimate_summary(world: World) -> list[str]:
    est = world.estimate
    if est is None:
        return ["No estimate yet."]
    if not hasattr(est, "flows"):
        return [f"EstimateUnavailable: {est.reason}", *est.diagnostics]
    label = (
        "PRIOR-ONLY: no accepted measurements; every value is a conditional prior estimate."
        if est.prior_only
        else f"{len(est.accepted_reading_ids)} accepted aggregate-P reading(s)."
    )
    return [
        label,
        f"Model {est.model_revision}, epoch {est.operating_epoch}: {est.n_unknowns} angle "
        f"unknowns, zero-injection rank {est.zero_injection_rank}, total rank {est.total_rank}.",
        f"Observable flows (pinned by real readings alone): {len(est.observable_segments)}.",
        *est.assumptions,
        *est.diagnostics,
    ]


def consistency_summary(world: World) -> list[str]:
    rep = world.consistency
    if rep is None:
        return ["No consistency report yet."]
    lines = [
        f"Global test: {rep.global_status} (degrees of freedom {rep.degrees_of_freedom}"
        + (
            f", chi2 {rep.global_chi2:.2f}, p {rep.global_p_value:.4f})"
            if rep.global_chi2 is not None
            else ")"
        ),
        rep.suspect_meaning,
    ]
    if rep.component_dof:
        lines.append(
            "Redundancy by component: "
            + ", ".join(f"{k}={v}" for k, v in sorted(rep.component_dof.items()))
        )
    for s in rep.critical_sets:
        lines.append("Critical set (an error among these cannot be localised): " + ", ".join(s))
    for e in rep.explanations:
        lines.append(f"Candidate explanation [{e.label}]: {e.support}")
    est = world.estimate
    env = world.envelope
    if env is not None:
        lines.append(
            f"Discrepancy envelope {env.version}: {len(env.calibration_seeds)} calibration + "
            f"{len(env.validation_seeds)} held-out validation seeds; domain: {env.domain}"
        )
    if est is not None and hasattr(est, "radius_unavailable") and est.radius_unavailable:
        lines.append(
            f"95% radius unavailable for {len(est.radius_unavailable)} of {len(est.flows)} flows "
            "(unsupported domain/topology or weak calibration); these cannot satisfy "
            "reconstruction completion."
        )
    return lines


def consistency_rows(world: World) -> list[dict]:
    rep = world.consistency
    if rep is None:
        return []
    return [
        {
            "reading": c.reading_id,
            "segment": c.segment_id,
            "status": c.testability,
            "normalised_residual": None
            if c.normalised_residual is None
            else round(c.normalised_residual, 2),
            "candidate": c.candidate,
            "note": c.note,
        }
        for c in rep.checks
    ]


def uncertainty_rows(world: World) -> list[dict]:
    est = world.estimate
    if est is None or not hasattr(est, "flows"):
        return []
    return [
        {
            "segment": seg,
            "p_mw": round(f.p_mw, 2),
            "conditional_std_mw": round(f.conditional_std_mw, 2),
            "discrepancy_bound_mw": None
            if f.discrepancy_bound_mw is None
            else round(f.discrepancy_bound_mw, 2),
            "radius_95_mw": None
            if f.uncertainty_radius_95_mw is None
            else round(f.uncertainty_radius_95_mw, 2),
            "radius_status": "published"
            if f.uncertainty_radius_95_mw is not None
            else "unavailable: " + (est.radius_unavailable or {}).get(seg, "unsupported"),
        }
        for seg, f in est.flows.items()
    ]


def disposition_rows(world: World) -> list[dict]:
    return [
        {
            "time": _t(d.clock_s),
            "reading": d.reading_id,
            "action": d.action,
            "epoch": f"{d.from_epoch}->{d.to_epoch}" if d.to_epoch else "",
            "reason": d.reason,
            "by": d.author,
        }
        for d in world.dispositions
    ]


def proposal_rows(world: World) -> list[dict]:
    return [
        {
            "finding": p.id,
            "author": p.author,
            "snapshot": f"r{p.snapshot_revision}",
            "status": p.status,
            "revisions": len(p.output.revisions),
            "recommendations": len(p.output.recommendations),
            "note": p.history[-1][1][:160] if p.history else "",
        }
        for p in world.proposals
    ]


def revision_rows(world: World) -> list[dict]:
    rows = []
    for p in world.proposals:
        for r in p.output.revisions:
            rows.append(
                {
                    "finding": p.id,
                    "status": p.status,
                    "hypothesis": r.hypothesis_id,
                    "change": ", ".join(r.change_kinds),
                    "assessment": f"{r.before.assessment if r.before else '-'} -> "
                    f"{r.after.assessment}",
                    "urgency": f"{r.before.priority if r.before else '-'} -> {r.after.priority}",
                    "scope": "; ".join(
                        f"{s.segment_id} {s.start_km:.2f}-{s.end_km:.2f} km"
                        for s in r.after.scope[:3]
                    ),
                    "finding_summary": r.finding_summary,
                    "update_reason": r.update_reason,
                    "remaining_uncertainty": r.remaining_uncertainty,
                    "trigger": f"{r.trigger_kind} "
                    f"{', '.join(r.trigger_event_ids) or r.operator_event_id or ''}",
                    "evidence": ", ".join(r.contributing_evidence_ids),
                }
            )
    return rows


def recommendation_rows(world: World) -> list[dict]:
    return [
        {
            "finding": p.id,
            "status": p.status,
            "id": rec.id,
            "action": rec.action_kind,
            "urgency": rec.urgency,
            "scope": ""
            if rec.scope is None
            else f"{rec.scope.segment_id} {rec.scope.start_km:.3f}-{rec.scope.end_km:.3f} km",
            "text": rec.text,
            "rationale": rec.rationale,
            "uncertainty": rec.uncertainty,
            "evidence": ", ".join(rec.evidence_ids),
        }
        for p in world.proposals
        for rec in p.output.recommendations
    ]


def hypothesis_rows(world: World) -> list[dict]:
    return [
        {
            "hypothesis": hid,
            "assessment": h["assessment"],
            "urgency": h["priority"],
            "lifecycle": h["lifecycle"],
            "claim": h["claim"],
            "model_change": ", ".join(h["model_change"] or []),
        }
        for hid, h in sorted(world.hypotheses.items())
    ]


def accepted_revision_rows(world: World) -> list[dict]:
    return [
        {
            "revision": r.id,
            "decision": r.decision_id,
            "hypothesis": r.hypothesis_id,
            "change": ", ".join(r.change_kinds),
            "finding": r.finding_summary,
            "reason": r.update_reason,
            "uncertainty": r.remaining_uncertainty,
            "at": _t(r.clock_s),
        }
        for r in world.hypothesis_revisions
    ]


def decision_rows(world: World) -> list[dict]:
    return [
        {
            "decision": o.decision_id,
            "outcome": o.outcome,
            "cycle": str(o.cycle_id) if o.cycle_id is not None else "no hold",
            "fleet": o.fallback,
            "reasons": "; ".join(o.reasons)[:200],
            "at": _t(o.clock_s),
        }
        for o in world.decision_outcomes.values()
    ]


def hold_text(world: World) -> str:
    h = world.hold
    if h is None:
        return "No acceptance hold."
    return (
        f"HOLD cycle {h.cycle_id} since {_t(h.since_s)} ({h.decision_id}): {h.reason}. "
        "Simulated time is frozen until a safe continuation is accepted."
    )


def notice_rows(world: World) -> list[dict]:
    """Pending findings shown beside raw observations, whatever route acceptance did."""
    return [
        {
            "finding": p.id,
            "status": p.status,
            "summary": p.output.summary,
            "history": " | ".join(f"{_t(t)} {n}" for t, n in p.history)[:300],
        }
        for p in world.proposals
        if p.status in ("PROPOSED_NOT_ACCEPTED", "SUPERSEDED", "REJECTED")
    ]


def need_rows(world: World) -> list[dict]:
    return [
        {
            "need": n.id,
            "status": n.status,
            "purpose": n.purpose,
            "kind": n.kind,
            "scope": f"{n.scope.segment_id} {n.scope.start_mm / 1e6:.2f}-"
            f"{n.scope.end_mm / 1e6:.2f} km",
            "urgency": n.urgency,
            "uncertainty": n.causal_uncertainty,
            "distinguishes": n.alternatives_distinguished,
            "could_change": n.result_effect,
            "repeat_of": n.repeat_of_reading_id or "",
            "retired": n.retired_reason,
        }
        for n in world.needs.values()
    ]


def unavailable_rows(world: World) -> list[dict]:
    return [{"need": u.need_id, "why_no_work": u.reason} for u in world.unavailable_work]


def task_log_rows(world: World) -> list[dict]:
    return [{"time": _t(t), "change": n} for t, n in world.task_log[-30:]]


def score_rows(world: World) -> list[dict]:
    return [
        {
            "task": r["task"],
            "eligible": r["eligible"],
            "owner_urgency": r["owner_urgency"],
            "points_by_urgency_5_to_1": " / ".join(str(v) for v in r["tier_points_5_to_1"]),
            "notes": r["notes"],
        }
        for r in world.score_preview
    ]


SCORE_ASSUMPTIONS = (
    "Synthetic units, compared lexicographically: urgency tiers 5 to 1, then risk, switching, "
    "airtime. Coverage = 1 point per metre of undecided scope on a fixed 10 m grid; shared "
    "evidence is counted once and owned by the highest accepted urgency; confirmation = 500 per "
    "group; electrical = 1000 for removing all targeted conditional variance. Not a calibrated "
    "causal entropy; a preview, not an authorisation."
)


def plan_candidate_rows(result) -> list[dict]:
    return [
        {
            "rank": i + 1,
            "source": c.source,
            "tiers_5_to_1": c.score.tiers,
            "risk": c.score.risk,
            "switching": c.score.switching,
            "airtime_min": round(c.score.airtime_s / 60, 1),
            "sorties": "; ".join(
                f"{r.drone_id}: {' > '.join(s.task_id for s in r.stops) or 'land'} -> "
                f"{r.landing.circuit_id}@{r.landing.point.offset_km:.2f}km"
                for r in c.sorties
            )
            or "(no flights)",
        }
        for i, c in enumerate(result.ranked)
    ]


def plan_stage_rows(result) -> list[dict]:
    return [
        {
            "round": s.round,
            "stage": s.name,
            "status": s.status,
            "value": s.objective,
            "bound": s.bound,
            "wall_ms": round(s.wall_s * 1000, 1),
        }
        for s in result.stages
    ]


def plan_unassigned_rows(result) -> list[dict]:
    return [{"task": u.task_id, "why": u.reason} for u in (*result.unassigned, *result.omitted)]


def route_pass_rows(world: World) -> list[dict]:
    return [
        {
            "pass": p.id,
            "at": _t(p.sim_clock_s),
            "trigger": p.trigger,
            "events": ", ".join(p.event_ids),
            "outcome": p.outcome,
            "detail": p.detail,
            "solver": p.solver,
            "latency_ms": round(p.latency_s * 1000),
            "planner_ms": round(p.planner_s * 1000),
        }
        for p in world.route_passes[-30:]
    ]


def coordinator_turn_rows(world: World) -> list[dict]:
    return [
        {
            "turn": t.id,
            "status": t.status,
            "pending_reasoning": t.status == "running",
            "triggers": ", ".join(t.trigger_ids) or "-",
            "snapshot": f"r{t.snapshot_revision}",
            "finding": t.finding_id or "",
            "note": t.note,
            "wall_s": t.wall_s,
        }
        for t in world.coordinator_turns
    ]


def planner_preview_rows(world: World) -> list[dict]:
    return [
        {
            "preview": p.id,
            "snapshot": f"r{p.snapshot_revision}",
            "summary": p.summary,
            "blocked": p.blocked or "",
            "unassigned": len(p.unassigned),
            "authorises_dispatch": p.authorises_dispatch,
        }
        for p in world.planner_previews[-10:]
    ]


def investigation_lines(world: World) -> list[str]:
    from grid_ops import conclusion

    inv = world.investigation
    lines = [f"Incident investigation: {inv.status}"]
    if inv.conclusion and "causal_statement" in inv.conclusion:
        lines.append(f"Concluded: {inv.conclusion['causal_statement']}")
        lines.append(f"Remaining uncertainty: {inv.conclusion['remaining_uncertainty']}")
    if inv.unresolved:
        lines.append(f"UNRESOLVED: {inv.unresolved}")
    for f in inv.reopen_flags:
        lines.append(f"Relevant new evidence since conclusion: {f}")
    rep = conclusion.investigation_report(world)
    lines.append(f"Operating assessment: {rep['operating_assessment']}")
    lines.append(f"Local investigation: {rep['local_investigation']}")
    if isinstance(rep["benchmark"], list):
        for name, met, detail in rep["benchmark"]:
            lines.append(f"Benchmark check {name}: {'MET' if met else 'UNMET'} - {detail}")
    if "deadline" in rep:
        d = rep["deadline"]
        lines.append(
            f"Deadline T+04:00: {d['outcome']}; remaining work: {len(d['remaining_work'])} "
            f"attempt(s); unmet checks: {', '.join(d['unmet_checks']) or 'none'}"
        )
    for t, n in inv.history:
        lines.append(f"{_t(t)} {n}")
    return lines


def resilience_lines(world) -> list[str]:
    """Measured response latency, fallback use and the action required when no safe continuation
    exists. Read-only: derived from recorded route passes."""
    from grid_ops import resilience

    rep = resilience.latency_report(world.route_passes)
    lines = [f"Latency: {rep.summary()}"]
    if rep.n:
        lines.append(
            "Mean stage time: "
            + ", ".join(f"{k.replace('_', ' ')} {v:.3f}s" for k, v in rep.stage_means_s.items())
        )
    for pid, why in rep.fallbacks:
        lines.append(f"Fallback used in {pid}: {why}")
    for pid in rep.budget_misses:
        lines.append(f"BUDGET MISS: {pid} exceeded the response budget")
    actions = resilience.operator_actions(world)
    lines += [f"ACTION REQUIRED: {a}" for a in actions] or ["No operator action required."]
    return lines
