# P2 review-gate evidence (captured 2026-10-25 via Orca computer-use, Safari, real Streamlit app)

App: `uv run streamlit run src/grid_ops/app.py --server.headless true --server.port 8501`
on the real bavaria fixture, incident_demo, seed 7. Driven through the visible Safari window
with accessibility clicks; screenshots captured by Orca (Screen Recording + Accessibility
permissions granted). All artifacts are unedited captures.

## Screenshots

- `01-boot.png` — app boot: sidebar scenario/simulation controls, Brief tab, manager panel.
- `02-manager-turn.png` — after "Manager turn (scripted)": proposal PF0001 pending in Decisions.
- `03-accepted.png` — operator accepted PF0001; finding combo, log shows
  "operator: Decision ... accepted".
- `04-requests-decisions.png` — decisions panel and reading-disposition controls post-accept.
- `05-requests.png` — Requests tab: REQ-C-SUP (contact_current, LINE_024/C1 phase A,
  0.000-26.838 km, urgency 4) and REQ-L-VTG (lidar, LINE_025 span, urgency 3), both ACTIVE,
  with compiled tasks. This is the explicit-request panel required by the plan.
- `06-preview-plan.png` — map at T+00:48 with all five drone markers, the Planner/Decisions/
  Investigation sidebar and the event log (RP0001 kept-incumbent, evidence expiry). The planner
  preview bullets themselves were visible in the Plan tab during the session; this capture shows
  the map view around it.
- `07-reset.png` — simulation reset, paused at T+00:00.
- `08-executing.png` — event log during the executed sortie: RP0001 accepted-plan -> DRONE_3
  took off from LINE_026/C1 -> landed on LINE_024/C1, handshake succeeded -> Reading R0001
  (contact_current): rms 80.9 A +/- 1.0 -> RP0002/RP0003 kept-incumbent. No charging wait.
- `09-map-t0400.png` — map with all five drones perched and the autonomous manager's proposal
  in the log (closed-loop take reached the T+04:00 deadline).

## Video

- `p2-runtime-video.mp4` (48 s) — first take: startup -> proposal -> accept -> executed
  contact-current sortie visible in the event log; frames also show the Requests/Tasks tables.
- `p2-closed-loop-video.mp4` (60 s) — second take with the autonomous manager set to
  "scripted": the manager proposes PF0001; the app's async coordinator records the proposal
  but does not auto-accept, so the drones stay perched while the run proceeds to T+04:00.
  The accepted-plan execution evidence is in the first video and `08-executing.png`.
- `frames/` — the raw captured frames (f01..f30 per take).

## Honest limitations

- The Streamlit speed slider could not be driven beyond 2 sim-min/s via accessibility
  (set-value and drag did not take), so both videos run at 2x; the executed sortie takes
  ~9 sim-minutes and is visible in the log and Requests/Tasks tables, not as a moving drone
  on the map at high speed.
- The second take shows proposal-without-execution because the app's autonomous manager
  records proposals without accepting them; execution was driven manually in the first take.
- The main content area occasionally renders blank mid-rerender in captures; the sidebar log
  and tables remain the authoritative visible evidence.
- These are operator-facing interaction-review artifacts for the P2 review gate; the
  machine-checkable evidence remains in `.audit/p2-lanes/runtime|perf` receipts and the
  test suite.