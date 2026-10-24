# Nomadic JEV — Vegetation Fault Decision Engine

A CLI that evaluates power grid segments for vegetation-caused faults after a windstorm, and decides whether to dispatch an operator crew, send a drone, or deprioritize.

---

## How it works

The system uses Gemini (structured output via Pydantic) to produce calibrated probabilities for each segment. Code then combines those probabilities into a dispatch priority and decision.

### Step-by-step flow

```
1. Meta-agent forms hypotheses on which segments may have a tree fault
2. Gemini evaluates each segment → produces 4 probabilities
3. Code computes priority = P(tree_contact) × P(impact)
4. Drones are routed to highest-priority segments
5. New sensor data comes in → Gemini re-evaluated → probabilities shift
6. False positives are caught before a crew is dispatched
7. Hypothesis confirmed at threshold → crew dispatched
```

---

## What Gemini produces

For each segment, Gemini returns 4 probabilities (0.0 – 1.0):

| Probability | Meaning |
|---|---|
| `P(tree_contact)` | A tree is touching or about to contact the conductor |
| `P(impact)` | Segment failure causes significant customer/infrastructure impact |
| `P(report_credibility)` | Incoming reports specifically implicate this segment |
| `P(innocent_explanation)` | A benign cause (planned switching, generator test) explains everything |

Code then computes:
```
priority = P(tree_contact) × P(impact)
```

### Decision thresholds

| Priority | Evidence count | Decision |
|---|---|---|
| ≥ 0.64 | ≥ 2 | `DISPATCH_CREW` |
| 0.16 – 0.64 | any | `MEASURE_MORE` |
| < 0.16 | any | `DEPRIORITIZE` |

---

## Vegetation inputs to Gemini

These fields are computed from real GIS data and fed to Gemini before it makes its judgment.

| Gemini input field | Where it comes from |
|---|---|
| `forest_overlap_pct` | OSM forests from [overpass-turbo.eu](https://overpass-turbo.eu) |
| `max_canopy_height_m` | Meta canopy height map ([AWS](https://registry.opendata.aws/dataforgood-fb-forests)) or Bavarian laser scans ([geodaten.bayern.de](https://geodaten.bayern.de/opengeodata)) |
| `dominant_species` | DLR tree species map ([geoservice.dlr.de](https://geoservice.dlr.de/web/datasets/treespecies_de_2022)) |
| `wind_exposure_score` | Computed from line direction (`grid_segments.geojson`) + assumed SW wind (225°) |
| `estimated_clearance_m` | Derived from voltage class (engineering standard) |

## Impact inputs to Gemini

| Gemini input field | Where it comes from |
|---|---|
| `downstream_households` | [Zensus 2022](https://zensus2022.de) 100m grid cells → assigned to nearest substation |
| `downstream_power_plants_mw` | [Marktstammdatenregister](https://www.marktstammdatenregister.de) → filter Landkreis Augsburg → export CSV |
| `downstream_hospitals` / `water_works` / `care_homes` | OSM via [overpass-turbo.eu](https://overpass-turbo.eu) → assigned to nearest substation |

---

## Priority formula

```
priority = vegetation_risk × P(impact) / flight_time
         = P(tree_contact) × P(impact) / flight_time
```

Where `P(tree_contact)` is the vegetation risk score produced by Gemini from the GIS inputs above.

---

## Project structure

```
jev-cli/
├── src/
│   ├── types.py       # Input/output dataclasses
│   ├── questions.py   # Gemini prompt builder
│   ├── evaluate.py    # Gemini call + decision logic
│   └── cli.py         # CLI entrypoint
├── hypotheses_example.json   # Example input with 4 segments
└── README.md
```

---

## Setup

```bash
python3 -m venv .venv
.venv/bin/pip install google-genai pydantic
export GEMINI_API_KEY=your_key_here
```

## Run

```bash
# From a JSON file
.venv/bin/python3 -m src.cli --file hypotheses_example.json

# Inline
.venv/bin/python3 -m src.cli --inline '{"segment_id": "LINE_026", ...}'

# Full JSON output
.venv/bin/python3 -m src.cli --file hypotheses_example.json --json-out
```

---

## Example output

```
Segment        P(tree)  P(impact)  Priority   Report  Innocent  Decision
LINE_026        0.76      0.82      0.623       0.42      0.38  MEASURE_MORE
LINE_024        0.78      0.50      0.390       0.82      0.20  MEASURE_MORE
LINE_004        0.18      0.92      0.166       0.02      0.95  MEASURE_MORE
LINE_002        0.05      0.25      0.013       0.15      0.85  DEPRIORITIZE
```
