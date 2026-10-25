"""pydeck map of the network, report areas and drones for one World snapshot.

Three-method rendering: drones show their itinerary phase and battery; inspected lidar
coverage is drawn from the recorded coverage records.
"""

from __future__ import annotations

import pydeck as pdk

from grid_ops.geometry import Geometry, Interval, LinePoint
from grid_ops.grid import Network
from grid_ops.views import reconstruction_status
from grid_ops.world import Executing, LineStatus, World, format_clock, position_at

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
    "perched": [40, 110, 220],
    "executing": [240, 190, 20],
    "stranded": [210, 40, 40],
}

VIEW = pdk.ViewState(longitude=10.665, latitude=48.235, zoom=10)


def build_deck(
    network: Network,
    world: World,
    geometry: Geometry | None = None,
    preview: LinePoint | None = None,
) -> pdk.Deck:
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
    inferred = reconstruction_status(world)

    def shown(seg_id: str) -> LineStatus:
        """The operator's label wins; otherwise the reconstruction's (never 'de-energized')."""
        status = world.belief[seg_id]
        return inferred.get(seg_id, status) if status is LineStatus.UNKNOWN else status

    segments = [
        {
            "path": [list(p) for p in seg.path],
            "color": BELIEF_COLORS[shown(seg.id)],
            "width": 4 if seg.voltage_kv >= 380 else 2,
            "tooltip": f"{seg.id} · {shown(seg.id)}"
            + (" (reconstruction)" if shown(seg.id) is not world.belief[seg.id] else ""),
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
            "tooltip": (
                f"{drone.id} · {drone.phase} · battery "
                f"{round(100 * drone.battery.remaining_units / drone.battery.capacity_units)}% · "
                f"{format_clock(world.clock)}"
            ),
        }
        for drone in world.drones.values()
    ]
    flights = []
    for d in world.drones.values():
        match d.state:
            case Executing(itinerary=it, step_index=i):
                remaining = it.steps[i:]
            case _:
                continue
        here = d.position
        for step in remaining:
            anchor = position_at(step, 1.0)
            flights.append({"from": list(here), "to": list(anchor)})
            here = anchor

    site_rows = [
        {
            "position": list(site.position),
            "tooltip": f"{site.id} · exact site",
        }
        for site in world.sites.values()
    ]
    footprints = []
    preview_rows = []
    if geometry is not None:
        for site in world.sites.values():
            iv: Interval = geometry.footprint(site.point)
            footprints.append({"path": [list(p) for p in geometry.path_of(iv)]})
        if preview is not None:
            iv = geometry.footprint(preview)
            footprints.append({"path": [list(p) for p in geometry.path_of(iv)]})
            preview_rows.append(
                {
                    "position": list(geometry.position_of(preview)),
                    "tooltip": f"{preview.segment_id} @ {preview.offset_km:.3f} km (unregistered)",
                }
            )

    inspected_rows = []
    hazard_paths = []
    if geometry is not None:
        for cov in world.coverage:
            for iv in cov.inspected:
                inspected_rows.append(
                    {
                        "path": [list(p) for p in geometry.path_of(iv)],
                        "tooltip": f"{cov.reading_id} inspected",
                    }
                )
        for hz in world.hazards:
            iv = Interval(hz.segment_id, hz.start_mm, hz.end_mm)
            hazard_paths.append(
                {
                    "path": [list(p) for p in geometry.path_of(iv)],
                    "tooltip": f"{hz.id} {'; '.join(hz.assumptions)}",
                }
            )

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
            "PathLayer",
            inspected_rows,
            id="inspected",
            get_path="path",
            get_color=[40, 170, 90, 230],
            get_width=50,
            width_min_pixels=5,
            pickable=True,
        ),
        pdk.Layer(
            "PathLayer",
            hazard_paths,
            id="hazards",
            get_path="path",
            get_color=[220, 30, 30, 255],
            get_width=70,
            width_min_pixels=8,
            pickable=True,
        ),
        pdk.Layer(
            "PathLayer",
            footprints,
            id="footprints",
            get_path="path",
            get_color=[255, 200, 0, 200],
            get_width=60,
            width_min_pixels=6,
        ),
        pdk.Layer(
            "ScatterplotLayer",
            site_rows,
            id="sites",
            get_position="position",
            get_radius=150,
            radius_min_pixels=5,
            get_fill_color=[255, 200, 0],
            get_line_color=[0, 0, 0],
            stroked=True,
            line_width_min_pixels=1,
            pickable=True,
        ),
        pdk.Layer(
            "ScatterplotLayer",
            preview_rows,
            id="preview",
            get_position="position",
            get_radius=150,
            radius_min_pixels=6,
            get_fill_color=[255, 255, 255],
            get_line_color=[255, 120, 0],
            stroked=True,
            line_width_min_pixels=2,
            pickable=True,
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