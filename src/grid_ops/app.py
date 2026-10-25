"""Streamlit operator console: live drone map plus controls for the simulation and agents.

Three-method console: requests, compiled tasks, payload-specific readings, itinerary routes
and battery continuity. Electrical inference (estimates, consistency, envelopes, MW) and
truth-level switching controls are not part of this console.
"""

from __future__ import annotations

import os
import threading

import streamlit as st

from grid_ops.env import load_env
from grid_ops.geometry import MM_PER_KM, LinePoint
from grid_ops.grid import load_network
from grid_ops.map_view import build_deck
from grid_ops.proposals import ProposedStatus
from grid_ops.scenario import ScenarioConfig, ScenarioInfeasible, ScenarioMode
from grid_ops.sim import (
    AcceptLimits,
    CommitDecision,
    CommitSorties,
    Dispatch,
    ExcludeReading,
    Log,
    ReassignEpoch,
    RecordProposal,
    RegisterSite,
    ReinstateReading,
    RejectFinding,
    ReopenInvestigation,
    SetLandingPolicy,
    Simulation,
)
from grid_ops.views import (
    SCORE_ASSUMPTIONS,
    accepted_revision_rows,
    alternative_rows,
    assessment_rows,
    coordinator_turn_rows,
    coverage_rows,
    decision_rows,
    disposition_rows,
    drone_rows,
    endpoint_rows,
    experiment_rows,
    fact_rows,
    hazard_rows,
    hold_text,
    hypothesis_rows,
    investigation_lines,
    known_switching_rows,
    notice_rows,
    plan_candidate_rows,
    plan_stage_rows,
    plan_unassigned_rows,
    planner_preview_rows,
    proposal_rows,
    reading_rows,
    recommendation_rows,
    reconstruction_lines,
    region_rows,
    rejection_rows,
    request_rows,
    resilience_lines,
    revision_rows,
    route_pass_rows,
    route_rows,
    score_rows,
    task_log_rows,
    task_rows,
    unavailable_rows,
)
from grid_ops.world import format_clock

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


def record_manager_turn(sim: Simulation, result, author: str) -> str:
    """Record a validated (or rejected) manager output as a proposal. It changes no state."""
    status = ProposedStatus.REJECTED if result.errors else ProposedStatus.PROPOSED_NOT_ACCEPTED
    note = "; ".join(result.errors) if result.errors else "validated against the pinned snapshot"
    from grid_ops.coordinator import decisions_semantic_key

    return sim.submit(
        RecordProposal(
            author,
            sim.snapshot.revision,
            result.output,
            status,
            note,
            decisions_semantic_key(sim.snapshot, result.output),
        )
    ).result(timeout=15)


def run_llm_turn_in_background(sim: Simulation) -> None:
    from grid_ops import agents, manager_tools

    def work() -> None:
        try:
            tb = manager_tools.build_toolbox(sim.snapshot, sim.network, sim.circuits)
            result = agents.run_llm_turn(tb)
            record_manager_turn(sim, result, "manager-llm")
        except Exception as e:
            sim.submit(Log("manager-llm", f"Manager turn failed: {e}"))

    threading.Thread(target=work, name="manager-llm", daemon=True).start()


def scripted_manager_turn(sim: Simulation, conclude: bool) -> None:
    from grid_ops import agents, manager_tools

    tb = manager_tools.build_toolbox(sim.snapshot, sim.network, sim.circuits)
    result = agents.run_scripted_turn(tb, conclude=conclude)
    fid = record_manager_turn(sim, result, "manager-scripted")
    if result.errors:
        st.toast(f"Rejected: {result.errors[0]}")
    else:
        st.toast(f"Proposal {fid} recorded.")


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
    if st.button("Reset simulation"):
        sim.stop()
        st.cache_resource.clear()
        st.rerun()

    st.header("Manual dispatch")
    drone_id = st.selectbox("Drone", sorted(sim.snapshot.drones))
    target = st.selectbox("Target segment", segment_ids)
    if st.button("Dispatch (landing-only sortie)"):
        try:
            st.toast(sim.submit(Dispatch(drone_id, target, "operator")).result(timeout=10))
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

    st.header("Planner")
    if st.button("Preview plan"):
        st.session_state["plan_preview"] = sim.preview_plan()
    plan_now = st.session_state.get("plan_preview")
    best = None if plan_now is None else plan_now.best
    if best is not None and best.sorties and st.button("Commit best plan"):
        try:
            st.toast(sim.submit(CommitSorties(best.sorties, "planner")).result(timeout=15))
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
                        timeout=30
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

    st.header("Fleet")
    sim.auto_replan = st.toggle("Autonomous replanning on events", value=True, key="auto_replan")

    st.header("Autonomous manager")
    modes = ["off", "scripted", "scripted (reconstruction-driven)"]
    if os.environ.get("OPENROUTER_API_KEY"):
        modes.append("LLM")
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

        def adaptive(tb, trigger_ids=()):
            return agents.run_adaptive_scripted_turn(tb, trigger_ids, conclude=True)

        def language_model(tb, trigger_ids=()):
            return agents.run_llm_turn(tb, trigger_ids=trigger_ids, record=True)

        adaptive.author = "manager-scripted (reconstruction-driven)"
        language_model.author = "manager-llm"
        fn = {
            "scripted": agents.run_scripted_turn,
            "scripted (reconstruction-driven)": adaptive,
        }.get(chosen, language_model)
        co = Coordinator(sim, manager_fn=fn)
        co.start()
        coords["running"] = (chosen, co)
        if chosen != "scripted" and not sim.snapshot.requests:
            # these managers ask for their first measurement before any reading exists
            threading.Thread(target=co.run_turn_now, name="manager-startup", daemon=True).start()
    st.caption("Runs on pinned snapshots while the fleet keeps flying; never gates routing.")

    st.header("Manager")
    st.caption("Proposes findings and measurement requests from a pinned read-only snapshot.")
    if st.button("Manager turn (scripted, may conclude)"):
        scripted_manager_turn(sim, conclude=True)
    if st.button("Manager turn (scripted)"):
        scripted_manager_turn(sim, conclude=False)
    if not os.environ.get("OPENROUTER_API_KEY"):
        st.caption("Set OPENROUTER_API_KEY in .env to enable the LLM manager.")
    elif st.button("Manager turn (LLM)"):
        run_llm_turn_in_background(sim)
        st.toast("LLM manager turn started.")


@st.fragment(run_every=REFRESH_SECONDS)
def live_view(preview_point: LinePoint) -> None:
    world = sim.snapshot
    state = "paused" if sim.paused else f"{sim.minutes_per_second:g} sim min / s"
    st.subheader(f"{format_clock(world.clock)}  ·  {state}")

    map_col, side_col = st.columns([3, 1])
    with map_col:
        st.pydeck_chart(
            build_deck(network, world, sim.geometry, preview_point),
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
        requests_tab,
        tasks_tab,
        readings_tab,
        routes_tab,
        fleet_tab,
        landing_tab,
        coverage_tab,
        reconstruction_tab,
        findings_tab,
        plan_tab,
        coordinator_tab,
        resilience_tab,
        investigation_tab,
    ) = st.tabs(
        [
            "Brief",
            "Requests",
            "Tasks",
            "Readings",
            "Routes",
            "Fleet",
            "Landing",
            "Coverage",
            "Reconstruction",
            "Findings",
            "Plan",
            "Coordinator",
            "Resilience",
            "Investigation",
        ]
    )
    with reconstruction_tab:
        st.caption(
            "A deterministic posterior over a declared candidate family, built only from executed "
            "readings and operational events. Probabilities are conditional on that model; the "
            "manager cannot write them, and zero current never certifies a de-energized line."
        )
        for line in reconstruction_lines(world):
            st.markdown(f"- {line}")
        st.caption("Supported facts: observed current, service, fault geometry, charge")
        st.dataframe(fact_rows(world), hide_index=True)
        st.caption("Alternatives the evidence cannot yet separate")
        st.dataframe(alternative_rows(world), hide_index=True)
        st.caption("Where the fault could be, and what has been inspected")
        st.dataframe(region_rows(world), hide_index=True)
        st.caption("Ranked measurements by expected discrimination, and what the optimizer sees")
        st.dataframe(experiment_rows(world), hide_index=True)
    with requests_tab:
        st.caption(
            "Accepted measurement requests (lidar, TDR, contact current) with their compiled "
            "tasks. An ACTIVE request with no completed valid attempt is explicitly unresolved."
        )
        st.dataframe(request_rows(world), hide_index=True)
        st.caption("Requests that compiled to no executable work, and why")
        st.dataframe(unavailable_rows(world), hide_index=True)
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
    with coverage_tab:
        st.caption(
            "What each lidar scan inspected and missed, plus scoped condition results. A "
            "detected hazard is not shown to have caused the outage, and clean readings do not "
            "prove mechanical integrity."
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
            "Tasks exist only for accepted measurement requests. Reservations change only "
            "through accepted sorties."
        )
        st.dataframe(task_rows(world), hide_index=True)
        st.caption("Plan scoring preview")
        st.caption(SCORE_ASSUMPTIONS)
        st.dataframe(score_rows(world), hide_index=True)
        st.caption("Refresh log")
        st.dataframe(task_log_rows(world), hide_index=True)
    with readings_tab:
        st.dataframe(reading_rows(world), hide_index=True)
        st.caption("Reading dispositions (readings themselves are never altered)")
        st.dataframe(disposition_rows(world), hide_index=True)
        st.dataframe(rejection_rows(world), hide_index=True)


live_view(preview_point)