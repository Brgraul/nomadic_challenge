"""Streamlit operator console: live drone map plus controls for the simulation and agents."""

from __future__ import annotations

import os
import threading

import streamlit as st

from grid_ops.grid import load_network
from grid_ops.map_view import build_deck
from grid_ops.sim import Dispatch, Log, SetInService, Simulation
from grid_ops.world import format_clock

REFRESH_SECONDS = 4

st.set_page_config(page_title="Grid Ops", layout="wide")


@st.cache_resource
def get_sim() -> Simulation:
    sim = Simulation(load_network())
    sim.start()
    return sim


@st.cache_resource
def get_agent_runner(_sim: Simulation) -> dict:
    from grid_ops.agents import build_master

    return {"run": build_master(_sim), "busy": False}


def run_agent_in_background(runner: dict, sim: Simulation, instruction: str) -> None:
    def work() -> None:
        runner["busy"] = True
        try:
            runner["run"](instruction)
        except Exception as e:
            sim.submit(Log("master", f"Agent run failed: {e}"))
        finally:
            runner["busy"] = False

    threading.Thread(target=work, name="master-agent", daemon=True).start()


sim = get_sim()
network = sim.network
segment_ids = sorted(network.segments)

with st.sidebar:
    st.header("Simulation")
    sim.minutes_per_second = st.slider("Sim minutes per real second", 0.5, 30.0, 2.0, 0.5)
    sim.paused = st.toggle("Paused", value=False)
    color_by_truth = st.toggle("Color lines by ground-truth energization", value=False)
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

    st.header("Ground truth")
    switched = st.selectbox("Segment", segment_ids, key="truth_segment")
    in_service = st.toggle("In service", value=True)
    if st.button("Apply switching"):
        st.toast(sim.submit(SetInService(switched, in_service, "scenario")).result(timeout=5))

    st.header("Master agent")
    instruction = st.text_area(
        "Instruction", "Review the reports and decide where each drone should go first."
    )
    if not os.environ.get("ANTHROPIC_API_KEY"):
        st.caption("Set ANTHROPIC_API_KEY to enable agents.")
    else:
        runner = get_agent_runner(sim)
        if st.button("Run master agent", disabled=runner["busy"]):
            run_agent_in_background(runner, sim, instruction)
            st.toast("Master agent started.")


@st.fragment(run_every=REFRESH_SECONDS)
def live_view(color_by_truth: bool) -> None:
    world = sim.snapshot
    state = "paused" if sim.paused else f"{sim.minutes_per_second:g} sim min / s"
    st.subheader(f"{format_clock(world.clock)}  ·  {state}")

    map_col, side_col = st.columns([3, 1])
    with map_col:
        st.pydeck_chart(build_deck(network, world, color_by_truth), height=640, key="map")
    with side_col:
        st.dataframe(
            [
                {"drone": d.id, "phase": d.phase, "where": str(d.state)}
                for d in world.drones.values()
            ],
            hide_index=True,
        )
        st.caption("Event log")
        st.text(
            "\n".join(
                f"{format_clock(e.clock)} {e.author}: {e.text}" for e in reversed(world.log[-40:])
            )
        )


live_view(color_by_truth)
