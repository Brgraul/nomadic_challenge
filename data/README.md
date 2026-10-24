# Grid data package

## Start here

This package shows mapped power lines in Bavaria and adds a fictional windstorm scenario. The line locations come from real mapping data. The storm and reports are invented: they do not describe actual damage or outages.

`network_map.png` is a quick overview. The CSV and GeoJSON files contain the full detail. Public reports are approximate and may be incomplete or wrong; they are not verified measurements or exact fault locations. Organizer-only fault information is not included here.

## Files you can use

- `network_map.png` — overview map, prepared for A3 landscape printing.
- `grid_segments.csv` / `grid_segments.geojson` — 64 mapped line sections. Each has a `segment_id` such as `LINE_001`.
- `grid_nodes.csv` / `grid_nodes.geojson` — 59 marked line ends, connection locations, and substation locations. Each has a `node_id`.
- `initial_information.csv` — six fictional reports. `information_id` identifies a report; `information_type` describes what kind of report it is.
- `scenario_context.geojson` — the fictional storm area and broad report areas. These areas are not exact event locations.
- `scenario_public_config.json` — public scenario settings and report text used to create the context files.

The network has seven `component_id` groups. A component is simply a group of lines joined in this mapped dataset; it does not show which homes receive electricity.

## Opening the files

- Open `network_map.png` in any image viewer. It is an overview, not a substitute for the detailed files.
- Open CSV files in a spreadsheet as **UTF-8, comma-separated**. Keep IDs as text. A blank value means “not provided,” not zero. Some fields with multiple values use semicolons in CSV.
- Open GeoJSON files in QGIS or another mapping program. They use WGS84 / EPSG:4326: coordinates are longitude and latitude in degrees. GeoJSON stores multiple values as arrays. Segment geometry is also provided as optional WKT in `grid_segments.csv`.
- `geojson.io` can preview GeoJSON, but only upload files from this participant folder to a public website. Never upload organizer-only files.

## What the map can—and cannot—tell you

`grid_segments` contains lines; `grid_nodes` contains marked locations where lines end or are represented as connected. Use a segment's `from_node` and `to_node` values to find its two node IDs. A node's `connected_segments` lists segments that refer to it.

Some nodes represent substations: places where electricity may be routed or its voltage changed. Several lines entering the same substation do **not** prove that those lines connect directly inside it. The internal connections are unknown. A substation marker may also sit a little away from the actual line ends; the marker shows the station area, not an exact electrical connection point.

Other limits to keep in mind:

- Lines that cross on the map are not necessarily joined.
- A gap between mapped lines is not proof of an outage.
- Lines cut off at the edge of this study area may continue beyond it; the cut point is not a real line end.
- The seven mapped components are not known outage areas or separate electricity systems.
- The files do not show power flow or which customers are supplied by a line. Real electricity routes and customer supply are unknown.

## A few field notes

- For line sections, `voltage_kv` is the highest voltage recorded in the source, in kilovolts (kV). `voltages_kv` lists all recorded voltage values. Node values can combine information from several lines; they do not prove those lines are electrically connected. A voltage value does not show how much charging power is available.
- `circuits` is the source's count of electrical paths on a line. `cables` is a source tag about cables/conductors. Neither field is a capacity measurement, and either may be blank.
- In this fictional scenario, a sensor can recharge only if a line is live and can supply enough power. A voltage listed in the data does not confirm either condition. Do not use this package to plan real flights or work near power lines.

## Source and credit

The grid data is **© OpenStreetMap contributors**, shared under the [Open Database License (ODbL) 1.0](https://opendatacommons.org/licenses/odbl/1-0/). This adapted dataset uses the same license. See [OpenStreetMap copyright](https://www.openstreetmap.org/copyright) for details.

The source file was provided as an OpenInfrastructureMap/OpenStreetMap extract. It does not include an export history, so we cannot confirm when the data was collected or how it was exported. The fictional storm, reports, and faults were created for this exercise; they are not OpenStreetMap data.
