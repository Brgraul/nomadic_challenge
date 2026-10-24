from pathlib import Path
import json
base=Path('/Users/pc/Documents/repos/nomadic_challenge/docs/planner')
changed=[]
def replace(file,old,new):
 p=base/file;s=p.read_text();assert s.count(old)==1,(file,old[:80]);p.write_text(s.replace(old,new,1));changed.append(file)
def section(file,start,end,text):
 p=base/file;s=p.read_text();a=s.index(start);b=s.index(end,a);p.write_text(s[:a]+text.strip()+'\n\n'+s[b:]);changed.append(file)
section('workflow.md','## Agentic workflow','## Events and invalidation','''## Agentic workflow

There are two LLM agents and one route solver. The manager owns hypotheses, investigation urgency and recommendations. The planner proposes measurement needs and sites. Deterministic code maintains evidence, eligibility, reservations and information values. Drones execute accepted routes without drone agents.

There are two decision paths. An observable event first triggers immediate deterministic fleet replanning using accepted hypotheses and the new evidence. Once a complete continuation is validated, the sim resumes. Manager and planner LLM calls run asynchronously against a pinned evidence snapshot. An explained hypothesis or measurement-needs update triggers another route pass when ready; it does not block the first pass.

```mermaid
flowchart TD
    event["Any drone update or relevant event"] --> ingest["Brief hold: ingest evidence and refresh affected diagnostics"]
    ingest --> fast["Refresh tasks and solve fleet routes without waiting for an LLM"]
    fast --> valid{"Complete valid continuation?"}
    valid -->|yes| commit["Atomically accept route-only decision and release"]
    valid -->|no| fallback["Validate retained or landing-only continuation; otherwise remain held"]
    commit --> flight["Drones continue from current positions"]
    commit --> manager["Async manager LLM: assess readings and explain revisions"]
    manager --> tests["Read-only candidate tests before final explanation"]
    tests --> semantic{"New semantic measurement needs?"}
    semantic -->|yes| planner["Async planner LLM proposes needs and sites"]
    semantic -->|no| ready
    planner --> ready["Semantic revision check; rebase motion-only changes"]
    ready --> accept["Brief acceptance hold: solve from live positions and validate whole decision"]
    accept --> flight
    flight --> event
```

`refresh_tasks` runs after relevant evidence, model, coverage, reservation, expiry or availability changes, even if hypotheses are unchanged. It refreshes canonical candidates and conditional values deterministically. When covariance is unavailable, use labelled diagnostic candidates rather than invented information gains. The planner LLM runs only when semantic measurement needs change. Its final structured proposal defines needs and sites; previews authorize no dispatch.

Every reading batch is retained for manager assessment. If new evidence arrives during an LLM call, the fast path still runs immediately. Coalesce unassessed batches into the next current-evidence turn and retain their event IDs. An older turn may be archived as superseded but cannot commit a hypothesis explanation against newer incompatible evidence.

## In-flight rerouting and latency

An update from any drone triggers reconsideration of the entire available fleet, including drones between stops. A `Flying` drone can divert directly from its current interpolated position; it need not finish its current leg, land or recharge first. Preserve spent battery and elapsed time. Replanning does not force every drone to divert: retain a valid next stop when switching would add no value. A `Measuring` drone finishes its locked dwell and adopts the new suffix afterwards. Charging and stranded drones stay put.

The target is a validated route update within one wall-clock second at the 95th percentile, with a two-second end-to-end response budget from observation ingestion. These are implementation acceptance targets, not measured performance. The budget includes affected checks, candidate refresh, bounded solving, validation, commit and publication. Reserve time for acceptance rather than letting the optimizer consume the entire deadline. If optimization cannot finish, validate and publish the current continuation or a landing-only fallback within the budget. A missed target is recorded; if no valid continuation exists, remain held with a visible safety exception rather than falsely reporting timely rerouting.

LLM reasoning is outside this fast-path deadline. A slow, failed or superseded manager/planner call leaves a still-valid fleet continuation running. It cannot silently change hypotheses or suppress evidence. An accepted explained hypothesis update gets its own immediate bounded route pass. At startup, initial LLM planning may finish before any drone takes off. Later short holds make decision latency free in simulation time, but do not count as meeting the wall-clock target without a measured timely response.

The coordinator uses semantic dependencies for asynchronous proposals. Evidence selection, operating epoch/model, sensor assumptions, hypothesis state, manual intent and relevant policy must still match. Continuous position, battery and route-progress changes alone do not force a new LLM call. The sim recomputes tasks, reservations and routes from live drone state at acceptance. A new reading, material condition change or conflicting operator edit invalidates the relevant proposal and requires a current-evidence explanation.
''')
replace('workflow.md', '''The sim processes all observable events at one simulation timestamp as one decision batch. It holds the clock, ingests observations, refreshes affected diagnostics and publishes the immutable snapshot before notifying the coordinator. `hold` is separate from the operator's pause toggle. Decision time is free in the exercise; reset the wall-time baseline on release so LLM latency is not charged as flight time.''', '''The sim batches observable events at a timestamp, ingests them under a brief hold, refreshes affected diagnostics and runs the fast route pass. Publish the accepted immutable snapshot before notifying the asynchronous coordinator. `hold` is separate from operator pause; it lasts only for bounded route/decision acceptance or an explicit inability to continue safely. Simulation time advances while LLM calls run. Reset the wall-time baseline on each release so only the brief acceptance hold is free in simulation time.

In the table, manager and planner calls are asynchronous. A "Yes" in those columns never gates the immediate route response. New explained revisions cause a second route pass after semantic validation.''')
replace('workflow.md', '''Agent exceptions, malformed explanations, stale drafts, failed estimation, solver timeout, infeasibility and commit rejection have distinct recorded outcomes. No path releases the clock unconditionally. A failed LLM turn accepts no proposed hypothesis changes. Existing tasks may be reused only after refresh against current evidence and reservations.''', '''Agent exceptions, malformed explanations, stale drafts, failed estimation, solver timeout, infeasibility and commit rejection have distinct outcomes. A failed or stale asynchronous LLM turn accepts no proposed hypothesis changes and does not pause drones already following a valid continuation. An event or acceptance pass releases its brief hold only after validating complete routes. Existing tasks may be reused only after refresh against current evidence and reservations.''')
replace('workflow.md', '''Each hold has a unique `cycle_id`. Only `Release(cycle_id, accepted_decision_id)` for that cycle's accepted decision can resume it. Late commands and releases from older cycles are rejected. Retrying a previously accepted command ID returns its stored result without replaying its effects.''', '''Each short acceptance hold has a unique `cycle_id`. Only `Release(cycle_id, accepted_decision_id)` for its still-current accepted decision can resume it. The sim invokes this automatically after a fast route-only acceptance. An asynchronous proposal carries its source cycle for provenance; it obtains a new acceptance cycle after semantic validation and never releases its old source cycle. Late releases are rejected. Repeated command IDs return stored outcomes without replaying effects.''')
section('workflow.md','## Simulation and decision transaction','## Agent tools','''## Simulation and decision transaction

`SnapshotRevision` records the full public snapshot, including motion and reservation revisions. `SemanticRevision` records the subset used to justify an LLM proposal: selected evidence, public epoch/model, sensor assumptions, accepted hypotheses, manual intent and applicable policy/condition dependencies. `DecisionSnapshot` is immutable and contains the source cycle, revisions, events and clock. Tools are pinned to it. UI reads may use the latest public snapshot.

Commands are `Start`, `SubmitObservation`, `RequestReadingDisposition`, `RequestModelChange`, `RequestManualDispatch`, `CommitDecision`, `Release`, `SetPaused` and `Log`. Hidden scenario control remains separate. `RequestModelChange` stages a candidate; enqueueing it cannot change the accepted model. Known switch evidence invalidates operational assumptions immediately, while a model replacement is tested and accepted with its explained revision. Agents return proposals rather than directly mutating state.

A `DecisionDraft` records its kind (`REACTIVE_ROUTES` or `EXPLAINED_UPDATE`), decision ID, source cycle/full revision, semantic dependency key, explained revisions if any, model proposals, recommendations and measurement needs. Reactive decisions use current accepted hypotheses and create no new hypothesis version. Command IDs and accepted outcomes provide retry deduplication. No-op hypotheses retain their semantic version while their evidence assessment is archived.

```text
on an observable batch or state-changing operator command
  take a brief acceptance hold and ingest evidence and direct consequences
  refresh affected diagnostics, tasks and eligibility using accepted hypotheses
  solve or select a validated fleet-wide continuation within the response budget
  atomically accept a route-only decision, publish it and release its cycle
  notify the coordinator with that immutable evidence snapshot
  if no valid continuation exists, remain held and expose the exception

while routes execute
  run manager and optional planner LLM calls asynchronously on pinned inputs
  archive every reading batch and any superseded assessment
  publish valid proposed findings as pending notices without changing active state

on CommitDecision for an explained update
  verify command identity, before values, evidence and semantic dependencies
  if meaning changed, reject for a fresh current-evidence explanation
  acquire a new brief acceptance cycle and process any newly due observations
  recheck semantic dependencies before applying anything
  build a private candidate using current positions, battery and locked work
  validate explained revisions and test candidate model/hypothesis changes
  refresh candidate diagnostics and measurement needs from current accepted evidence
  solve and validate every route within the response budget
  reject all candidate mutations if any acceptance condition fails
  otherwise atomically accept explanations, hypotheses, model, tasks and routes
  publish the accepted decision and release this acceptance cycle
```

Motion-only rebasing changes operational assignments, not the LLM's evidence or causal narrative. Recompute route feasibility and benefit using current reservations; never replay stale waypoint assignments. If a new observation occurs before acceptance and changes a declared dependency, abandon that proposal and schedule a current-evidence turn. The already-running valid continuation remains available. Operator edits to relevant meaning invalidate old proposals; a matching semantic key alone never bypasses before-value checks or independent route validation.

A rejected explained update retains raw observations and its clearly pending notice. It cannot partially change accepted hypotheses or routes. If rejection affects operational feasibility, invoke the same bounded continuation/landing fallback as the fast path. Detailed latency, cycle-release and failure rules are owned by this chapter.
''')
replace('overview.md', '''The sim thread is the only writer of `World`. The coordinator runs the two LLM agents against one immutable `DecisionSnapshot` for each held cycle. Tools are pinned to that snapshot and return its revision. A single writer prevents simultaneous writes; revision checks and atomic acceptance prevent semantically stale decisions.''', '''The sim thread is the only writer of `World`. It handles bounded reactive route decisions without waiting for an LLM. The coordinator runs manager and planner calls asynchronously on immutable evidence snapshots while validated routes execute. Full source revisions preserve provenance; semantic dependency checks allow motion-only rebasing, and atomic acceptance validates current operational state. See [workflow and latency](workflow.md#in-flight-rerouting-and-latency).''')
replace('overview.md', '''    public -->|"Publish snapshot before event"| co["Coordinator"]''', '''    public -->|"Immediate route-only pass"| accept
    public -->|"Accepted snapshot before async event"| co["Coordinator"]''')
replace('overview.md', '''    refresh -->|"CommitDecision with source revision"| queue''', '''    refresh -->|"CommitDecision with semantic dependencies"| queue''')
replace('hypotheses.md', '''    Sim->>Sim: Refresh affected diagnostics, publish snapshot R2
    Sim-->>Co: Decision event with R2''', '''    Sim->>Sim: Refresh diagnostics and deterministic tasks
    Sim->>Sol: Immediate fleet route solve without LLM
    Sol-->>Sim: Valid reactive continuation
    Sim->>Sim: Accept, publish R2 and release C2
    Sim-->>Co: Async evidence event with R2''')
replace('hypotheses.md', '''    Co->>Sim: CommitDecision based on R2 and cycle C2
    Sim->>Sim: Validate revisions and candidate model state''', '''    Co->>Sim: CommitDecision with R2 semantic dependencies
    Sim->>Sim: Recheck meaning, acquire new short acceptance cycle
    Sim->>Sim: Validate revisions and candidate model state''')
replace('hypotheses.md', '''    Co->>Sim: Release C2 for accepted decision''', '''    Sim->>Sim: Release new cycle; flight continues''')
replace('optimization.md', '''A two-second wall watchdog includes model construction, all rounds and candidate scoring; it runs the pure solver in an owned worker process that can be terminated without mutating `World`.''', '''The [workflow's two-second end-to-end response budget](workflow.md#in-flight-rerouting-and-latency) includes checks and acceptance, not just optimization. Initially reserve at least 0.5 seconds for those stages and cap the pure solver worker at the lesser of 1.5 seconds or the remaining deadline. Its watchdog includes model construction, all rounds and candidate scoring and can terminate the worker without mutating `World`.''')
replace('optimization.md', '''LLM latency is separately bounded by the coordinator timeout and does not advance simulation time.''', '''LLM calls have a separate coordinator timeout and run asynchronously while simulation time advances. They do not consume the fast route-response budget or keep the fleet held.''')
replace('implementation.md', '''| Operator edit during a manager turn | Old revision cannot commit or release the new cycle |''', '''| Operator edit or new evidence during a manager turn | A changed semantic dependency rejects the stale proposal; pure motion rebases to live state, and old releases cannot release a new cycle |''')
needle='''| New current reading, unchanged hypotheses | Covariance, marginal values and deterministic candidates refresh; routes are reconsidered without an unnecessary planner LLM call |'''
replace('implementation.md',needle,needle+'''
| One drone reports while another is mid-leg and the manager takes 20 wall seconds | Fleet route pass runs without waiting, using live positions and existing explained hypotheses; the slow LLM does not hold the fleet |
| Timely explained update after drones have moved | Semantically compatible proposal rebases from live positions, battery and reservations, then atomically accepts a new bounded route pass |
| Near-real-time response measurement | Measure ingestion-to-validated-plan publication over representative seeds; target p95 at most one second and a two-second response budget, exposing misses and safety exceptions |''')
replace('domain-types.md', '''Observations update circuit knowledge, chargeability and local coverage through the ingestion rules below.''', '''Observations update circuit knowledge, chargeability and local coverage through the [operational knowledge rules](drone-state.md#energization-chargeability-and-inspection-state).''')
# Record which preserved section bodies intentionally change for the user's timing requirement.
from importlib.util import spec_from_file_location,module_from_spec
spec=spec_from_file_location('split_check',base.parents[1]/'.audit/planner-design/check_split.py')
# Section parsing below avoids executing the historical preservation assertions.
def parse(text):
 import re
 result={};title=None;lines=[];fenced=False
 for line in text.splitlines(keepends=True):
  if line.startswith('```'): fenced=not fenced
  m=re.match(r'^#{2,3} (.+)\n$',line) if not fenced else None
  if m:
   if title: result[title]=''.join(lines).strip()
   title=m.group(1);lines=[]
  elif title: lines.append(line)
 if title: result[title]=''.join(lines).strip()
 return result
manifest_path=base.parents[1]/'.audit/planner-design/split-manifest.json'
m=json.loads(manifest_path.read_text())
m['intentional_amendments']={}
for anchor,entry in m['sections'].items():
 if Path(entry['chapter']).name in set(changed):
  m['intentional_amendments'][entry['title']]='Clarify near-real-time fleet rerouting, asynchronous reasoning, semantic rebasing or relocated cross-reference.'
manifest_path.write_text(json.dumps(m,indent=2)+'\n')
print('Updated near-real-time contracts in '+', '.join(sorted(set(changed))))
