from pathlib import Path
from datetime import datetime, timezone

DOC=Path('/Users/pc/Documents/repos/nomadic_challenge/docs/planner-design.md')
ROOT=Path('/private/tmp/planner-design-revision')

def section(start, end, replacement):
    s=DOC.read_text()
    a=s.index(start)
    b=s.index(end,a) if end else len(s)
    DOC.write_text(s[:a]+replacement.rstrip()+'\n\n'+s[b:])

def swap(old,new):
    s=DOC.read_text()
    if s.count(old)!=1:
        raise ValueError(f'Expected unique replacement, found {s.count(old)}: {old[:100]}')
    DOC.write_text(s.replace(old,new,1))

def insert_before(marker, content):
    swap(marker, content.rstrip()+'\n\n'+marker)

def record(rank, finding, decision, evidence):
    p=ROOT/'decisions.tsv'
    if not p.exists(): p.write_text('ts\tphase\tdecision\twhy\tevidence\tresult\n')
    values=[datetime.now(timezone.utc).isoformat(),f'rank {rank}',decision,f'Original finding {finding}',evidence,'section inspected; final cross-check pending']
    with p.open('a') as f: f.write('\t'.join(values)+'\n')
