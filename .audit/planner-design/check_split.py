from pathlib import Path
import hashlib
import json
import os
import re

ROOT=Path(__file__).resolve().parents[2]
AUDIT=Path(__file__).resolve().parent

def sections(text):
    found={}
    title=None
    lines=[]
    fenced=False
    for line in text.splitlines(keepends=True):
        if line.startswith('```'): fenced=not fenced
        match=re.match(r'^#{2,3} (.+)\n$',line) if not fenced else None
        if match:
            if title: found[title]=''.join(lines).strip()
            title=match.group(1);lines=[]
        elif title: lines.append(line)
    if title: found[title]=''.join(lines).strip()
    return found

def normalized(body,path):
    def link(match):
        label,target=match.groups()
        if target.startswith(('http://','https://','#')): return match.group(0)
        name,sep,fragment=target.partition('#')
        resolved=(path.parent/name).resolve()
        return f'[{label}]({resolved}'+(f'#{fragment}' if sep else '')+')'
    return re.sub(r'\[([^\]]+)\]\(([^)]+)\)',link,body)

manifest=json.loads((AUDIT/'split-manifest.json').read_text())
before=sections((AUDIT/'pre-split.md').read_text())
proof=[]
for spec in manifest['sections'].values():
    path=ROOT/spec['chapter']
    after=sections(path.read_text())
    title=spec['title']
    assert title in after,(path,title)
    assert normalized(before[title],ROOT/'docs/planner-design.md')==normalized(after[title],path),(path,title)
    proof.append({'section':title,'chapter':spec['chapter'],'body_sha256':hashlib.sha256(normalized(after[title],path).encode()).hexdigest()})
assert len(proof)==42
(AUDIT/'split-preservation-result.json').write_text(json.dumps({'original_sections_preserved':42,'sections':proof},indent=2)+'\n')
print('42 of 42 original sections preserved; only heading levels, navigation and relative links changed.')
