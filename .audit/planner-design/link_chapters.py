from pathlib import Path
import re
import json
import hashlib
import os
from datetime import datetime,timezone

root=Path('/Users/pc/Documents/repos/nomadic_challenge')
audit=root/'.audit/planner-design'
related={
 'overview': [('workflow','Decision acceptance'),('domain-types','Shared records')],
 'workflow': [('hypotheses','Explained revisions'),('tasks-and-routes','Route invariants'),('drone-state','Event timing')],
 'validation': [('estimation','Supported electrical diagnostics'),('workflow','Check triggers')],
 'hypotheses': [('geometry-and-sensing','Evidence and detection limits'),('workflow','Acceptance and latency')],
 'tasks-and-routes': [('drone-state','Drone phases'),('optimization','Route objective')],
 'domain-types': [('workflow','Command semantics'),('drone-state','State semantics')],
 'geometry-and-sensing': [('electrical-model','Circuit registry'),('estimation','Noise use')],
 'drone-state': [('workflow','Decision cycles'),('tasks-and-routes','Locked work')],
 'electrical-model': [('geometry-and-sensing','Observation model'),('estimation','Reduced-model inference')],
 'estimation': [('electrical-model','Model assumptions'),('validation','Completion criteria')],
 'optimization': [('tasks-and-routes','Hard invariants'),('workflow','Response deadline')],
 'evaluation': [('electrical-model','Scenario modes'),('implementation','Acceptance cases')],
 'implementation': [('workflow','Integration contract'),('evaluation','Verification metrics')],
}
for name,links in related.items():
 p=root/f'docs/planner/{name}.md';s=p.read_text()
 parts=s.split('\n\n',2)
 s=parts[0]+'\n\n'+parts[1]+'\n\nRelated contracts: '+', '.join(f'[{label}]({target}.md)' for target,label in links)+'.\n\n'+parts[2]
 s=re.sub(r'(\n## [^\n]+\n)\n\n',r'\1\n',s)
 p.write_text(s)
# Keep the LLM revision's provenance and semantic dependency key together.
p=root/'docs/planner/domain-types.md';s=p.read_text();s=s.replace('''    source_revision: SnapshotRevision
    hypothesis_id: str''','''    source_revision: SnapshotRevision
    semantic_revision: SemanticRevision
    hypothesis_id: str''');p.write_text(s)
# Label the timing requirement at the front without duplicating the contract.
p=root/'docs/planner-design.md';s=p.read_text();s=s.replace('''manager/planner calls, fleet-wide rerouting, event invalidation, holds or decision acceptance''','''near-real-time rerouting, async manager/planner calls, event invalidation or decision acceptance''');p.write_text(s)
# Compute exactly which original section bodies were deliberately amended after the lossless move.
def sections(text):
 out={};title=None;lines=[];fenced=False
 for line in text.splitlines(keepends=True):
  if line.startswith('```'): fenced=not fenced
  m=re.match(r'^#{2,3} (.+)\n$',line) if not fenced else None
  if m:
   if title:out[title]=''.join(lines).strip()
   title=m.group(1);lines=[]
  elif title:lines.append(line)
 if title:out[title]=''.join(lines).strip()
 return out

def normalize(body,path):
 def link(m):
  label,target=m.groups()
  if target.startswith(('http://','https://','#')):return m.group(0)
  name,sep,frag=target.partition('#')
  return f'[{label}]({(path.parent/name).resolve()}'+(f'#{frag}' if sep else '')+')'
 return re.sub(r'\[([^\]]+)\]\(([^)]+)\)',link,body)
manifest=json.loads((audit/'split-manifest.json').read_text())
before=sections((audit/'pre-split.md').read_text())
amended={}
for anchor,entry in manifest['sections'].items():
 p=root/entry['chapter'];body=sections(p.read_text())[entry['title']]
 if normalize(body,p)!=normalize(before[entry['title']],root/'docs/planner-design.md'):
  amended[entry['title']]={'reason':'Near-real-time reactive/async decision contract or updated cross-chapter reference.','sha256':hashlib.sha256(body.encode()).hexdigest()}
manifest['intentional_amendments']=amended
manifest['additional_sections']=['In-flight rerouting and latency']
for path,spec in manifest['chapters'].items():
 p=root/path;spec['lines']=len(p.read_text().splitlines());spec['sha256']=hashlib.sha256(p.read_bytes()).hexdigest()
(audit/'split-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
with (audit/'decisions.tsv').open('a') as f:
 for decision,why,evidence in [
 ('Split the large design into a short index and13 focused chapters','The user needs agents to load only the relevant branch of the design','docs/planner-design.md; .audit/planner-design/split-preservation-result.json'),
 ('Add reactive fleet-wide rerouting independent of async LLM reasoning','The user requires near-real-time mid-flight route updates from any drone reading','docs/planner/workflow.md#in-flight-rerouting-and-latency'),
 ]:
  f.write('\t'.join([datetime.now(timezone.utc).isoformat(),'modular design',decision,why,evidence,'applied; final modular checks pending'])+'\n')
print('Deliberately amended original sections: '+', '.join(amended))
