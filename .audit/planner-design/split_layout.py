from pathlib import Path
import re
import os
import json
import hashlib

base=Path('/Users/pc/Documents/repos/nomadic_challenge')
source=base/'docs/planner-design.md'
original=source.read_text()
audit=base/'.audit/planner-design'
(audit/'pre-split.md').write_text(original)
sections=[]
current=None
fenced=False
for line in original.splitlines(keepends=True):
    if line.startswith('```'): fenced=not fenced
    match=re.match(r'^(#{2,3}) (.+)\n$',line) if not fenced else None
    if match:
        if current: sections.append(current)
        current={'title':match.group(2),'body':[]}
    elif current:
        current['body'].append(line)
if current: sections.append(current)
chapters={
 'overview':('Overview and architecture','Read for scope, assumptions, ownership or module boundaries.',[
  'Premises','Architecture']),
 'workflow':('Workflow and events','Read for manager/planner calls, fleet-wide rerouting, event invalidation, holds or decision acceptance.',[
  'Agentic workflow','Events and invalidation','Failure policy','Simulation and decision transaction','Agent tools']),
 'validation':('Validation and completion','Read for grid plausibility checks, supported conclusions, investigation completion or operator actions.',[
  'Validation contract','Completion and operator actions']),
 'hypotheses':('Hypotheses and explanations','Read for physical predictions, evidence interpretation, LLM-written revision reasons or the worked feedback example.',[
  'Hypotheses and physical predictions','Explained hypothesis revisions','Example request']),
 'tasks-and-routes':('Tasks and route acceptance','Read for in-progress work, reservations, repeats, battery limits, exact landings, occupancy or the deadline.',[
  'Replanning rules','Task and observation lifecycle','Route acceptance invariants']),
 'domain-types':('Domain type sketches','Read when changing a shared record, command payload, public snapshot or physics interface.',[
  'Data shape','World and ground truth']),
 'geometry-and-sensing':('Geometry, sensors and coverage','Read for line positions, local footprints, sensor payloads, terminal-equivalent sensing, noise or negative evidence.',[
  'Positions on the line','Local coverage and negative evidence','Sensing','Electrical observation abstraction','Noise and detection conditions']),
 'drone-state':('Drone state and simulation time','Read for drone transitions, energization/charging knowledge, evidence freshness, same-time events or hidden physical scheduling.',[
  'State model','Drone state machine','Energization, chargeability and inspection state','Time and simultaneous transitions']),
 'electrical-model':('Electrical model and scenarios','Read for synthetic truth, incident fixtures, circuit allocation, DC reduction, topology assumptions or the minimum sampling calculation.',[
  'Power flow ground truth and scenarios','Local incident fixtures','Explicit electrical reduction','Minimum sampling']),
 'estimation':('State estimation and uncertainty','Read for WLS, observability, critical sets, residual tests, priors, calibration or model discrepancy.',[
  'State estimation','Conditional uncertainty and model discrepancy','Bad-data interpretation']),
 'optimization':('Optimization and solver guarantees','Read for shared-evidence rewards, urgency, CP-SAT constraints, search budgets, fallbacks or the sortie arithmetic example.',[
  'Optimization model and objective','Benefit ownership','Reward order','Routing constraints','Solver guarantees and search budget','Example sortie']),
 'evaluation':('Evaluation','Read for truth-isolated metrics, policy comparisons, noise replay or benchmark claims.',[
  'Evaluation']),
 'implementation':('Implementation and acceptance','Read for implementation scope, required integration scenarios or explicitly deferred extensions.',[
  'Implementation scope and acceptance scenarios','Deferred extensions']),
}
lookup={s['title']:s for s in sections}
assigned=[name for _,_,titles in chapters.values() for name in titles]
assert len(assigned)==len(set(assigned))
assert set(assigned)==set(lookup),(set(lookup)-set(assigned),set(assigned)-set(lookup))
def slug(title): return re.sub(r'[^a-z0-9 -]','',title.lower()).replace(' ','-')
anchor_map={slug(title):f'planner/{key}.md#{slug(title)}' for key,(_,_,titles) in chapters.items() for title in titles}
outdir=base/'docs/planner'
assert not outdir.exists(),'destination exists; inspect before running'
outdir.mkdir()
manifest={'source_sha256':hashlib.sha256(original.encode()).hexdigest(),'sections':{},'chapters':{}}
for key,(title,trigger,titles) in chapters.items():
    path=outdir/f'{key}.md'
    text=f'# {title}\n\n[Design index](../planner-design.md). {trigger}\n\n'
    for section_title in titles:
        body=''.join(lookup[section_title]['body']).rstrip()
        def relocate(match):
            label,target=match.groups()
            if target.startswith(('https://','http://','#')): return match.group(0)
            oldpath,sep,fragment=target.partition('#')
            targetpath=(source.parent/oldpath).resolve()
            newpath=os.path.relpath(targetpath,path.parent)
            return f'[{label}]({newpath}'+(f'#{fragment}' if sep else '')+')'
        body=re.sub(r'\[([^\]]+)\]\(([^)]+)\)',relocate,body)
        text+=f'## {section_title}\n\n'+body+'\n\n'
        manifest['sections'][slug(section_title)]={'title':section_title,'chapter':str(path.relative_to(base))}
    path.write_text(text.rstrip()+'\n')
    manifest['chapters'][str(path.relative_to(base))]={'sections':titles,'lines':len(text.splitlines())}
index='''# Measurement planner design\n\nThis design turns root-cause hypotheses and drone measurements into updated investigation routes. It is a specification; the proposed functionality is not implemented yet.\n\nRead this index first, then load only the chapter for the task. Follow a linked dependency when changing its contract. A complete design audit uses the [validation report](planner-design-validation.md).\n\n| Work on | Read |\n|---|---|\n'''
for key,(title,trigger,titles) in chapters.items():
    index+=f'| {trigger.removeprefix("Read for ").removeprefix("Read when ").rstrip(".")} | [{title}](planner/{key}.md) |\n'
index+='''\n## Shared contracts\n\n- Public evidence and declared assumptions drive decisions; hidden truth remains isolated.\n- Every accepted hypothesis revision includes an LLM-written finding and update reason linked to evidence.\n- Readings trigger fleet-wide reconsideration, including routes of airborne drones; in-progress measurements retain their reservation.\n- Validate complete decisions and routes before changing accepted operational state.\n\nFor cross-cutting changes, check the owning chapter plus the contracts it references. Keep detailed definitions in their owning chapter instead of copying them into this index.\n'''
source.write_text(index)
report=base/'docs/planner-design-validation.md'
s=report.read_text()
def move_report_link(match):
    label,anchor=match.groups()
    assert anchor in anchor_map,anchor
    return f'[{label}]({anchor_map[anchor]})'
s=re.sub(r'\[([^\]]+)\]\(planner-design\.md#([^)]+)\)',move_report_link,s)
s=s.replace('The revised artifact is [planner-design.md](planner-design.md).','The design starts at [planner-design.md](planner-design.md), which routes readers to the focused chapters in `docs/planner/`.')
s=s.replace('The reviewed design\'s hash matches the preserved verification metadata.','The reviewed monolithic snapshot is preserved as `pre-split.md`; the split manifest and preservation check cover the current chapter layout.')
report.write_text(s)
(audit/'split-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps({'chapters':len(chapters),'preserved_sections':len(sections),'index_lines':len(index.splitlines()),'chapter_lines':{k:v['lines'] for k,v in manifest['chapters'].items()}},indent=2))
