"""Streamlit operator console: live drone map plus controls for the simulation and agents."""

from __future__ import annotations

import os
import threading

import streamlit as st

from grid_ops.env import load_env
from grid_ops.evaluation import energized_segments
from grid_ops.geometry import MM_PER_KM, LandingSite, LinePoint
from grid_ops.grid import load_network
from grid_ops.map_view import build_deck
from grid_ops.routes import SortieRequest, StopRequest
from grid_ops.scenario import ScenarioConfig, ScenarioInfeasible, ScenarioMode
from grid_ops.sim import (
    AcceptLimits,
    CommitDecision,
    CommitSorties,
    Dispatch,
    ExcludeReading,
    Log,
    ReassignEpoch,
    RegisterSite,
    ReinstateReading,
    RejectFinding,
    ReopenInvestigation,
    SetInService,
    SetLandingPolicy,
    Simulation,
)
from grid_ops.views import (
    SCORE_ASSUMPTIONS,
    accepted_revision_rows,
    assessment_rows,
    consistency_rows,
    consistency_summary,
    coordinator_turn_rows,
    coverage_rows,
    decision_rows,
    disposition_rows,
    drone_rows,
    endpoint_rows,
    estimate_rows,
    estimate_summary,
    hazard_rows,
    hold_text,
    hypothesis_rows,
    investigation_lines,
    known_switching_rows,
    need_rows,
    notice_rows,
    plan_candidate_rows,
    plan_stage_rows,
    plan_unassigned_rows,
    planner_preview_rows,
    proposal_rows,
    reading_rows,
    recommendation_rows,
    rejection_rows,
    resilience_lines,
    revision_rows,
    route_pass_rows,
    route_rows,
    score_rows,
    task_log_rows,
    task_rows,
    unavailable_rows,
    uncertainty_rows,
)
from grid_ops.world import Measurement, format_clock

REFRESH_SECONDS = 4
load_env()

st.set_page_config(page_title="Grid Ops", layout="wide")


@st.cache_resource
def get_coordinators(_sim: Simulation, key: str) -> dict:
    return {}


@st.cache_resource
def get_sim(mode: str, seed: int) -> Simulation:
    sim = Simulation(load_network(), scenario=ScenarioConfig(ScenarioMode(mode), seed))
    sim.start()
    return sim


def run_llm_turn_in_background(sim: Simulation) -> None:
    from grid_ops import agents

    def work() -> None:
        try:
            agents.run_llm_turn(sim, record=True)
        except Exception as e:
            sim.submit(Log("manager-llm", f"Manager turn failed: {e}"))

    threading.Thread(target=work, name="manager-llm", daemon=True).start()


with st.sidebar:
    st.header("Scenario")
    mode = st.selectbox("Mode", [m.value for m in ScenarioMode], key="scenario_mode")
    seed = int(st.number_input("Seed", 0, 10_000, 7, key="scenario_seed"))

try:
    sim = get_sim(mode, seed)
except ScenarioInfeasible as e:
    st.error(f"Scenario infeasible: {e}")
    st.json(e.diagnostics, expanded=False)
    st.stop()
network = sim.network
segment_ids = sorted(network.segments)

with st.sidebar:
    st.header("Simulation")
    sim.minutes_per_second = st.slider("Sim minutes per real second", 0.5, 30.0, 2.0, 0.5)
    sim.paused = st.toggle("Paused", value=False)
    evaluator_overlay = st.toggle("Evaluator overlay: true energization", value=False)
    if st.button("Reset simulation"):
        sim.stop()
        st.cache_resource.clear()
        st.rerun()

    st.header("Manual dispatch")
    drone_id = st.selectbox("Drone", sorted(sim.snapshot.drones))
    target = st.selectbox("Target segment", segment_ids)
    if st.button("Fly"):
        try:
            st.toast(sim.submit(Dispatch(drone_id, target, "operator")).result(timeout=5))
        except Exception as e:
            st.toast(f"Rejected: {e}")

    uncertain = st.toggle(
        "Allow uncertain landing endpoints (risk)",
        key="allow_uncertain",
    )
    if uncertain != sim.snapshot.landing_policy.allow_uncertain:
        sim.submit(SetLandingPolicy(uncertain, "operator"))

    st.header("Inspect line location")
    inspect_segment = st.selectbox("Line", segment_ids, key="inspect_segment")
    length_mm = sim.geometry.length_mm(inspect_segment)
    offset_km = st.slider(
        "Offset along line (km)",
        0.0,
        length_mm / MM_PER_KM,
        min(length_mm / MM_PER_KM / 2, length_mm / MM_PER_KM),
        0.001,
        key=f"inspect_offset_{inspect_segment}",
        format="%.3f",
    )
    snap_to = st.selectbox("Snap to nearest point from", ["(none)", *sorted(sim.snapshot.drones)])
    if snap_to != "(none)":
        snapped = sim.geometry.nearest_point(inspect_segment, sim.snapshot.drones[snap_to].position)
        st.caption(f"Nearest point to {snap_to}: {snapped.offset_km:.3f} km")
        if st.button("Use snapped offset"):
            st.session_state[f"inspect_offset_{inspect_segment}"] = snapped.offset_km
            st.rerun()
    preview_point = LinePoint(inspect_segment, round(offset_km * MM_PER_KM))
    if st.button("Register site"):
        try:
            st.toast(
                sim.submit(
                    RegisterSite(inspect_segment, preview_point.offset_mm, "operator")
                ).result(timeout=5)
            )
        except Exception as e:
            st.toast(f"Rejected: {e}")

    st.header("Sortie builder")
    drafts: dict = st.session_state.setdefault("drafts", {})
    draft_drone = st.selectbox("Sortie drone", sorted(sim.snapshot.drones), key="draft_drone")
    draft = drafts.setdefault(draft_drone, {"stops": [], "landing": None})
    kind = st.selectbox("Measurement", [m.value for m in Measurement], key="draft_kind")
    circuit_ids = [c.id for c in sim.circuits.by_segment[inspect_segment]]
    circuit_id = st.selectbox("Landing circuit", circuit_ids, key="draft_circuit")
    b1, b2 = st.columns(2)
    if b1.button("Add stop here"):
        draft["stops"].append((inspect_segment, preview_point.offset_mm, kind))
    if b2.button("Land here"):
        draft["landing"] = (inspect_segment, preview_point.offset_mm, circuit_id)
    for drone_key, d in drafts.items():
        if d["stops"] or d["landing"]:
            st.caption(
                f"{drone_key}: "
                + " → ".join(f"{k}@{s}/{mm / MM_PER_KM:.3f}km" for s, mm, k in d["stops"])
                + (
                    f" ⇒ land {d['landing'][2]} {d['landing'][1] / MM_PER_KM:.3f}km"
                    if d["landing"]
                    else " ⇒ (no landing yet)"
                )
            )
    c1, c2 = st.columns(2)
    if c1.button("Clear drafts"):
        drafts.clear()
        st.rerun()
    if c2.button("Submit sorties"):
        requests = tuple(
            SortieRequest(
                drone_key,
                tuple(StopRequest(LinePoint(s, mm), Measurement(k)) for s, mm, k in d["stops"]),
                LandingSite(LinePoint(d["landing"][0], d["landing"][1]), d["landing"][2]),
            )
            for drone_key, d in drafts.items()
            if d["landing"]
        )
        try:
            st.toast(sim.submit(CommitSorties(requests, "operator")).result(timeout=5))
            drafts.clear()
        except Exception as e:
            st.toast(f"Rejected: {e}")

    st.header("Reading disposition")
    reading_ids = sorted(sim.snapshot.readings)
    if reading_ids:
        picked = st.selectbox("Reading", reading_ids, key="disp_reading")
        reason = st.text_input("Reason", key="disp_reason")
        epoch_to = st.text_input("New epoch", "E1", key="disp_epoch")
        d1, d2, d3 = st.columns(3)
        for col, label, make in (
            (d1, "Exclude", lambda: ExcludeReading(picked, reason, "operator")),
            (d2, "Reinstate", lambda: ReinstateReading(picked, reason, "operator")),
            (d3, "Reassign", lambda: ReassignEpoch(picked, epoch_to, reason, "operator")),
        ):
            if col.button(label, key=f"disp_{label}"):
                try:
                    st.toast(sim.submit(make()).result(timeout=5))
                except Exception as e:
                    st.toast(f"Rejected: {e}")
    else:
        st.caption("No readings yet.")

    st.header("Scenario control (hidden)")
    switched = st.selectbox("Segment", segment_ids, key="truth_segment")
    in_service = st.toggle("In service", value=True)
    if st.button("Apply switching"):
        st.toast(sim.submit(SetInService(switched, in_service, "scenario")).result(timeout=5))

    st.header("Fleet")
    sim.auto_replan = st.toggle("Autonomous replanning on events", value=True, key="auto_replan")
    st.header("Planner")
    if st.button("Preview plan"):
        st.session_state["plan_preview"] = sim.preview_plan()
    plan_now = st.session_state.get("plan_preview")
    best = None if plan_now is None else plan_now.best
    if best is not None and best.sorties and st.button("Commit best plan"):
        try:
            st.toast(sim.submit(CommitSorties(best.sorties, "planner")).result(timeout=10))
        except Exception as e:
            st.toast(f"Rejected: {e}")

    st.header("Decisions")
    pending = [p.id for p in sim.snapshot.proposals if p.status == "PROPOSED_NOT_ACCEPTED"]
    if pending:
        finding = st.selectbox("Finding", pending, key="decide_finding")
        why = st.text_input("Rejection reason", key="decide_reason")
        e1, e2 = st.columns(2)
        if e1.button("Accept"):
            pf = next(p for p in sim.snapshot.proposals if p.id == finding)
            did = f"D{len(sim.snapshot.decision_outcomes) + 1:03d}-{finding}"
            try:
                st.toast(
                    sim.submit(CommitDecision(did, finding, pf.semantic_key, "operator")).result(
                        timeout=15
                    )
                )
            except Exception as e:
                st.toast(f"Rejected: {e}")
        if e2.button("Reject"):
            try:
                st.toast(sim.submit(RejectFinding(finding, why, "operator")).result(timeout=5))
            except Exception as e:
                st.toast(f"Rejected: {e}")
    else:
        st.caption("No pending findings.")

    st.header("Investigation")
    limits = st.text_input("Accept stated limits", key="accept_limits")
    i1, i2 = st.columns(2)
    if i1.button("Accept limits") and limits:
        try:
            st.toast(sim.submit(AcceptLimits(limits, "operator")).result(timeout=5))
        except Exception as e:
            st.toast(f"Rejected: {e}")
    if i2.button("Reopen"):
        try:
            st.toast(
                sim.submit(ReopenInvestigation("operator request", "operator")).result(timeout=5)
            )
        except Exception as e:
            st.toast(f"Rejected: {e}")

    st.header("Autonomous manager")
    modes = (
        ["off", "scripted", "LLM"] if os.environ.get("OPENROUTER_API_KEY") else ["off", "scripted"]
    )
    chosen = st.selectbox("Asynchronous manager + planner", modes, key="auto_manager")
    coords = get_coordinators(sim, f"{mode}-{seed}")
    running = coords.get("running")
    if chosen == "off" and running is not None:
        running[1].stop()
        coords.pop("running")
    elif chosen != "off" and (running is None or running[0] != chosen):
        if running is not None:
            running[1].stop()
        from grid_ops import agents
        from grid_ops.coordinator import Coordinator

        fn = (
            agents.run_scripted_turn
            if chosen == "scripted"
            else (
                lambda s, trigger_ids=(): agents.run_llm_turn(
                    s, trigger_ids=trigger_ids, record=True
                )
            )
        )
        co = Coordinator(sim, manager_fn=fn)
        co.start()
        coords["running"] = (chosen, co)
    st.caption("Runs on pinned snapshots while the fleet keeps flying; never gates routing.")

    st.header("Manager")
    st.caption("Proposes findings from a pinned read-only snapshot. Nothing is accepted here.")
    if st.button("Manager turn (scripted, may conclude)"):
        from grid_ops import agents

        turn = agents.run_scripted_turn(sim, conclude=True)
        st.toast("Proposal recorded." if not turn.errors else f"Rejected: {turn.errors[0]}")
    if st.button("Manager turn (scripted)"):
        from grid_ops import agents

        turn = agents.run_scripted_turn(sim)
        st.toast("Proposal recorded." if not turn.errors else f"Rejected: {turn.errors[0]}")
    if not os.environ.get("OPENROUTER_API_KEY"):
        st.caption("Set OPENROUTER_API_KEY in .env to enable the LLM manager.")
    elif st.button("Manager turn (LLM)"):
        run_llm_turn_in_background(sim)
        st.toast("LLM manager turn started.")


@st.fragment(run_every=REFRESH_SECONDS)
def live_view(evaluator_overlay: bool, preview_point: LinePoint) -> None:
    world = sim.snapshot
    state = "paused" if sim.paused else f"{sim.minutes_per_second:g} sim min / s"
    st.subheader(f"{format_clock(world.clock)}  ·  {state}")

    map_col, side_col = st.columns([3, 1])
    with map_col:
        st.pydeck_chart(
            build_deck(
                network,
                world,
                energized_segments(sim) if evaluator_overlay else None,
                sim.geometry,
                preview_point,
            ),
            height=640,
            key="map",
        )
    with side_col:
        st.dataframe(drone_rows(world), hide_index=True)
        st.caption("Circuits on selected line")
        st.dataframe(
            [
                {
                    "circuit": c.id,
                    "kV": c.nominal_kv,
                    "from": c.from_node,
                    "to": c.to_node,
                    "x_pu": round(c.x_pu, 4),
                    "rating_A": c.max_current_a,
                }
                for c in sim.circuits.by_segment[preview_point.segment_id]
            ],
            hide_index=True,
        )
        st.caption(
            "Registered sites: " + (", ".join(sorted(world.sites)) if world.sites else "none")
        )
        st.caption("Event log")
        st.text(
            "\n".join(
                f"{format_clock(e.clock)} {e.author}: {e.text}" for e in reversed(world.log[-40:])
            )
        )

    (
        brief_tab,
        landing_tab,
        coverage_tab,
        estimates_tab,
        consistency_tab,
        findings_tab,
        plan_tab,
        fleet_tab,
        investigation_tab,
        coordinator_tab,
        resilience_tab,
        routes_tab,
        tasks_tab,
        readings_tab,
        rejections_tab,
    ) = st.tabs(
        [
            "Brief",
            "Landing",
            "Coverage",
            "Estimates",
            "Consistency",
            "Findings",
            "Plan",
            "Fleet",
            "Investigation",
            "Coordinator",
            "Resilience",
            "Routes",
            "Tasks",
            "Readings",
            "Rejections",
        ]
    )
    with fleet_tab:
        st.caption(
            "Immediate route passes after relevant events. They never wait for the manager and "
            "never change hypotheses. A HOLD means no complete safe continuation exists."
        )
        st.warning(hold_text(world)) if world.hold else st.caption(hold_text(world))
        st.dataframe(route_pass_rows(world), hide_index=True)
    with investigation_tab:
        for line in investigation_lines(world):
            st.markdown(f"- {line}")
    with resilience_tab:
        for line in resilience_lines(world):
            st.markdown(f"- {line}")
    with coordinator_tab:
        st.caption(
            "Active routes are in the Routes tab. Pending reasoning is a manager turn still "
            "running; proposals stay PROPOSED_NOT_ACCEPTED until an accepted decision."
        )
        st.dataframe(coordinator_turn_rows(world), hide_index=True)
        st.caption("Planner previews (authorise nothing; only on semantic planning changes)")
        st.dataframe(planner_preview_rows(world), hide_index=True)
    with plan_tab:
        st.caption(
            "Planner preview from the latest snapshot. It shows feasibility, scores and what is "
            "left out; it authorises no dispatch until a validated plan is committed."
        )
        plan = st.session_state.get("plan_preview")
        if plan is not None:
            st.markdown(
                f"- Snapshot r{plan.revision} at {format_clock(plan.clock_s / 60)}; "
                f"stop reason: {plan.stop_reason}"
            )
            st.markdown(
                f"- Replay settings: {plan.replay['solver']}, seed {plan.replay['seed']}, "
                f"{plan.replay['workers']} worker, deterministic budget "
                f"{plan.replay['deterministic_budget']}, wall cap {plan.replay['wall_cap_s']} s"
            )
            if plan.blocked:
                st.error(f"PlanningBlocked: {plan.blocked.reason}")
            st.dataframe(plan_candidate_rows(plan), hide_index=True)
            st.dataframe(plan_stage_rows(plan), hide_index=True)
            st.caption("Unassigned and omitted work")
            st.dataframe(plan_unassigned_rows(plan), hide_index=True)
    with findings_tab:
        st.caption(
            "Manager proposals: PROPOSED_NOT_ACCEPTED until an explained decision accepts them. "
            "Urgency is the manager's operational priority, not a probability."
        )
        st.warning(hold_text(world)) if world.hold else st.caption(hold_text(world))
        st.dataframe(proposal_rows(world), hide_index=True)
        st.dataframe(revision_rows(world), hide_index=True)
        st.caption("Accepted hypotheses")
        st.dataframe(hypothesis_rows(world), hide_index=True)
        st.dataframe(accepted_revision_rows(world), hide_index=True)
        st.caption("Decision outcomes")
        st.dataframe(decision_rows(world), hide_index=True)
        st.caption("Pending notices (shown even when routing is blocked)")
        st.dataframe(notice_rows(world), hide_index=True)
        st.caption("Scoped recommendations")
        st.dataframe(recommendation_rows(world), hide_index=True)
    with consistency_tab:
        for line in consistency_summary(world):
            st.markdown(f"- {line}")
        st.dataframe(consistency_rows(world), hide_index=True)
        st.caption("Uncertainty: conditional std and the discrepancy envelope are separate")
        st.dataframe(uncertainty_rows(world), hide_index=True)
        st.caption("Reading dispositions (readings themselves are never altered)")
        st.dataframe(disposition_rows(world), hide_index=True)
    with estimates_tab:
        for line in estimate_summary(world):
            st.markdown(f"- {line}")
        st.caption(f"Electrical solves so far: {world.estimation_solves}")
        st.dataframe(estimate_rows(world), hide_index=True)
        st.caption("Estimate history")
        st.text("\n".join(world.estimation_log[-10:]))
    with coverage_tab:
        st.caption(
            "Observed local coverage (union of inspected intervals) and scoped condition "
            "results. A detected hazard is not shown to have caused the outage, and clean "
            "readings do not prove mechanical integrity."
        )
        st.dataframe(coverage_rows(world), hide_index=True)
        st.dataframe(assessment_rows(world), hide_index=True)
        st.caption("Detected hazards (block landing on their interval)")
        st.dataframe(hazard_rows(world), hide_index=True)
    with landing_tab:
        st.caption(
            f"Landing policy: {world.landing_policy.name}. Endpoints need fresh, circuit-specific "
            "charging evidence; hazards and known unavailability are never overridden."
        )
        st.dataframe(endpoint_rows(world), hide_index=True)
    with brief_tab:
        st.caption(f"Scenario {world.scenario_mode} · seed {world.scenario_seed}")
        for line in world.brief:
            st.markdown(f"- {line}")
        st.caption("Known switching (public reports)")
        st.dataframe(known_switching_rows(world), hide_index=True)
    with routes_tab:
        st.dataframe(route_rows(world), hide_index=True)
    with tasks_tab:
        st.caption(
            "Tasks exist only for accepted diagnostic needs (and, in healthy_benchmark, "
            "reconstruction targets). Reservations change only through accepted decisions."
        )
        st.dataframe(need_rows(world), hide_index=True)
        st.dataframe(task_rows(world), hide_index=True)
        st.caption("Plan scoring preview")
        st.caption(SCORE_ASSUMPTIONS)
        st.dataframe(score_rows(world), hide_index=True)
        st.caption("Work that is unavailable, and why")
        st.dataframe(unavailable_rows(world), hide_index=True)
        st.caption("Refresh log")
        st.dataframe(task_log_rows(world), hide_index=True)
    with readings_tab:
        st.dataframe(reading_rows(world), hide_index=True)
    with rejections_tab:
        st.dataframe(rejection_rows(world), hide_index=True)


live_view(evaluator_overlay, preview_point)
