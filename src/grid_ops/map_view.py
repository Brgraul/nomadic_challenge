"""pydeck map of the network, report areas and drones for one World snapshot."""

from __future__ import annotations

import pydeck as pdk

from grid_ops.grid import Network
from grid_ops.world import Flying, LineStatus, World, format_clock

BELIEF_COLORS = {
    LineStatus.UNKNOWN: [140, 140, 140],
    LineStatus.LIVE: [46, 160, 67],
    LineStatus.DE_ENERGIZED: [30, 30, 30],
    LineStatus.SUSPECTED: [230, 150, 20],
    LineStatus.FAULT_CONFIRMED: [210, 40, 40],
    LineStatus.CLEARED: [40, 110, 220],
}
TRUTH_COLORS = {True: [46, 160, 67], False: [210, 40, 40]}
PHASE_COLORS = {
    "ready": [40, 110, 220],
    "flying": [150, 60, 200],
    "charging": [46, 160, 67],
    "stranded": [210, 40, 40],
}

VIEW = pdk.ViewState(longitude=10.665, latitude=48.235, zoom=10)


def build_deck(network: Network, world: World, color_by_truth: bool = False) -> pdk.Deck:
    areas = [
        {
            "polygon": [list(p) for p in area.polygon],
            "fill": [120, 120, 220, 18]
            if area.kind == "synthetic_storm_footprint"
            else [240, 140, 40, 40],
            "tooltip": area.label,
        }
        for area in network.areas.values()
        if area.id != "SYN_REPORT_01"
    ]
    segments = [
        {
            "path": [list(p) for p in seg.path],
            "color": TRUTH_COLORS[seg.id in world.energized]
            if color_by_truth
            else BELIEF_COLORS[world.belief[seg.id]],
            "width": 4 if seg.voltage_kv >= 380 else 2,
            "tooltip": f"{seg.id} · {seg.voltage_kv:.0f} kV · {world.belief[seg.id]}",
        }
        for seg in network.segments.values()
    ]
    nodes = [
        {
            "position": list(node.position),
            "radius": 6 if node.type == "abstract_substation" else 3,
            "tooltip": f"{node.id} · {node.type}" + (f" · {node.name}" if node.name else ""),
        }
        for node in network.nodes.values()
    ]
    drones = [
        {
            "id": drone.id,
            "label": drone.id.removeprefix("DRONE_"),
            "position": list(drone.position),
            "color": PHASE_COLORS[drone.phase],
            "tooltip": f"{drone.id} · {drone.phase} · {format_clock(world.clock)}",
        }
        for drone in world.drones.values()
    ]
    flights = [
        {"from": list(d.position), "to": list(d.state.target)}
        for d in world.drones.values()
        if isinstance(d.state, Flying)
    ]

    layers = [
        pdk.Layer(
            "PolygonLayer",
            areas,
            id="areas",
            get_polygon="polygon",
            get_fill_color="fill",
            get_line_color=[240, 140, 40, 120],
            line_width_min_pixels=1,
            pickable=True,
        ),
        pdk.Layer(
            "PathLayer",
            segments,
            id="segments",
            get_path="path",
            get_color="color",
            get_width="width * 20",
            width_min_pixels=2,
            pickable=True,
            auto_highlight=True,
        ),
        pdk.Layer(
            "ScatterplotLayer",
            nodes,
            id="nodes",
            get_position="position",
            get_radius="radius * 40",
            radius_min_pixels=2,
            radius_max_pixels=6,
            get_fill_color=[60, 60, 60],
            pickable=True,
        ),
        pdk.Layer(
            "LineLayer",
            flights,
            id="flights",
            get_source_position="from",
            get_target_position="to",
            get_color=[150, 60, 200],
            get_width=2,
        ),
        pdk.Layer(
            "ScatterplotLayer",
            drones,
            id="drones",
            get_position="position",
            get_radius=400,
            radius_min_pixels=7,
            radius_max_pixels=12,
            get_fill_color="color",
            get_line_color=[255, 255, 255],
            stroked=True,
            line_width_min_pixels=2,
            pickable=True,
        ),
        pdk.Layer(
            "TextLayer",
            drones,
            id="drone-labels",
            get_position="position",
            get_text="label",
            get_size=12,
            get_color=[255, 255, 255],
        ),
    ]
    return pdk.Deck(
        layers=layers,
        initial_view_state=VIEW,
        map_style=pdk.map_styles.CARTO_LIGHT,
        tooltip={"text": "{tooltip}"},
    )
