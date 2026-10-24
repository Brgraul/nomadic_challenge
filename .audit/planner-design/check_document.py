from pathlib import Path
import ast
import hashlib
import json
import re
import subprocess
import sys

AUDIT=Path(__file__).resolve().parent
ROOT=AUDIT.parents[1]
DOCS=ROOT/'docs'
INDEX=DOCS/'planner-design.md'
REPORT=DOCS/'planner-design-validation.md'
CHAPTERS=sorted((DOCS/'planner').glob('*.md'))
MANIFEST=json.loads((AUDIT/'split-manifest.json').read_text())

def sections(text):
    result={}
    title=None
    lines=[]
    fenced=False
    for line in text.splitlines(keepends=True):
        if line.startswith('```'):
            fenced=not fenced
        match=re.match(r'^#{2,3} (.+)\n$',line) if not fenced else None
        if match:
            if title:
                assert title not in result,title
                result[title]=''.join(lines).strip()
            title=match.group(1)
            lines=[]
        elif title:
            lines.append(line)
    if title:
        assert title not in result,title
        result[title]=''.join(lines).strip()
    return result

def normalized(body,path):
    def link(match):
        label,target=match.groups()
        if target.startswith(('http://','https://','#')):
            return match.group(0)
        name,separator,fragment=target.partition('#')
        target=(path.parent/name).resolve()
        return f'[{label}]({target}'+(f'#{fragment}' if separator else '')+')'
    return re.sub(r'\[([^\]]+)\]\(([^)]+)\)',link,body)

def anchors(path):
    headings=[]
    fenced=False
    for line in path.read_text().splitlines():
        if line.startswith('```'):
            fenced=not fenced
        match=re.match(r'^#{1,6} (.+)$',line) if not fenced else None
        if match:
            headings.append(match.group(1))
    assert len(headings)==len(set(headings)),str(path)
    return {re.sub(r'[^a-z0-9 -]','',h.lower()).replace(' ','-') for h in headings}

assert len(CHAPTERS)==13
assert len(INDEX.read_text().splitlines())<=45
assert max(len(p.read_text().splitlines()) for p in CHAPTERS)<=180
assert all(f'(planner/{p.name})' in INDEX.read_text() for p in CHAPTERS)
assert {str(p.relative_to(ROOT)) for p in CHAPTERS}==set(MANIFEST['chapters'])
all_sections={}
blocks=[]
for path in CHAPTERS:
    text=path.read_text()
    assert text.count('```')%2==0, f'unclosed code fence: {path}'
    for title,body in sections(text).items():
        assert title not in all_sections,title
        all_sections[title]=(path,body)
    for i,block in enumerate(re.findall(r'^```python\n(.*?)^```',text,re.M|re.S),1):
        ast.parse(block,filename=f'{path.name} sketch {i}')
        blocks.append(block)
assert len(blocks)==5,len(blocks)
before=sections((AUDIT/'pre-split.md').read_text())
assert set(before)|set(MANIFEST['additional_sections'])==set(all_sections)
preserved=0
amended=0
for title,old_body in before.items():
    path,new_body=all_sections[title]
    if title in MANIFEST['intentional_amendments']:
        spec=MANIFEST['intentional_amendments'][title]
        assert hashlib.sha256(new_body.encode()).hexdigest()==spec['sha256'],title
        amended+=1
    else:
        assert normalized(old_body,INDEX)==normalized(new_body,path),title
        preserved+=1
assert preserved+amended==42
historical_proof=json.loads((AUDIT/'split-preservation-result.json').read_text())
assert historical_proof['original_sections_preserved']==42
obsolete=[
    'unchanged hypotheses produce the same tasks',
    'Local search only accepts improvements',
    'ReadAssignmentFromRoutes',
    'perch_segment: str',
    'A task that already has a reading is never planned again',
    'At 40 km/h the same route takes 40 minutes, so the solver would drop the lowest-priority stop',
    'At this size (at most 5 drones and about 20 tasks) it should usually find the optimum',
    'no time_limit → deterministic',
    'Injection = 0 | 0.01 MW',
    'The solver finishes in milliseconds',
    'LLM latency is separately bounded by the coordinator timeout and does not advance simulation time',
]
combined='\n'.join(p.read_text() for p in CHAPTERS)
for old in obsolete:
    assert old not in combined,old
checked_links=0
for path in [INDEX,REPORT,AUDIT/'README.md',*CHAPTERS]:
    for label,target in re.findall(r'\[([^\]]+)\]\(([^)]+)\)',path.read_text()):
        if target.startswith(('http://','https://')):
            continue
        name,separator,fragment=target.partition('#')
        destination=(path.parent/name).resolve() if name else path
        assert destination.exists(),(path,target)
        if separator:
            assert fragment in anchors(destination),(path,target)
        checked_links+=1
closure=REPORT.read_text().split('## Requirement closure',1)[1].split('## Regressions',1)[0]
ids=[int(x) for x in re.findall(r'^\| (\d+) \|',closure,re.M)]
assert ids==list(range(1,17)),ids
rank_run=subprocess.run([sys.executable,str(AUDIT/'check_rank.py')],check=True,capture_output=True,text=True)
rank=json.loads(rank_run.stdout)
assert (rank['zero_injection_rank'],rank['full_rank'],rank['minimum_independent_readings'])==(24,52,28)
print(json.dumps({
    'index_lines':len(INDEX.read_text().splitlines()),
    'chapters':len(CHAPTERS),
    'largest_chapter_lines':max(len(p.read_text().splitlines()) for p in CHAPTERS),
    'original_sections_accounted_for':preserved+amended,
    'section_bodies_unchanged':preserved,
    'intentional_realtime_or_navigation_amendments':amended,
    'python_sketches_parsed':len(blocks),
    'local_links_and_anchors_checked':checked_links,
    'original_finding_rows':len(ids),
    'nominal_rank_check':28,
},indent=2))
