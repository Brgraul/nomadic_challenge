import csv
import json
from collections import defaultdict
from pathlib import Path
import numpy as np
import pandapower as pp

base=Path(__file__).resolve().parents[2]
nodes=list(csv.DictReader((base/'data/grid_nodes.csv').open()))
segments=list(csv.DictReader((base/'data/grid_segments.csv').open()))
net=pp.create_empty_network()
xs={v:pp.load_std_type(net,name,'line')['x_ohm_per_km'] for v,name in [(110.,'243-AL1/39-ST1A 110.0'),(380.,'490-AL1/64-ST1A 380.0')]}
index={n['node_id']:i for i,n in enumerate(nodes)}
inc=np.zeros((len(segments),len(nodes)))
bs=[]
for j,s in enumerate(segments):
    vs=[float(v) for v in s['voltages_kv'].split(';')]
    count=int(s['circuits']) if s['circuits'] else len(vs)
    assert count>=len(vs),s['segment_id']
    circuits=vs+[vs[k%len(vs)] for k in range(count-len(vs))]
    bs.append(sum(v*v/(xs[v]*float(s['length_km'])) for v in circuits))
    inc[j,index[s['from_node']]]=1
    inc[j,index[s['to_node']]]=-1
hf=np.diag(bs)@inc
hb=inc.T@hf
refs={}
for n in nodes: refs.setdefault(n['component_id'],index[n['node_id']])
cols=[i for i in range(len(nodes)) if i not in refs.values()]
z=[i for i,n in enumerate(nodes) if n['node_type'] in ('junction','intermediate')]
F=hf[:,cols]
Z=hb[z,:][:,cols]
def norm(m):
    return m/np.maximum(np.linalg.norm(m,axis=1,keepdims=True),1e-30)
F=norm(F);Z=norm(Z)
rankz=int(np.linalg.matrix_rank(Z,tol=1e-9))
rankall=int(np.linalg.matrix_rank(np.vstack([Z,F]),tol=1e-9))
H=Z.copy(); chosen=[]
for i,s in enumerate(segments):
    candidate=np.vstack([H,F[i]])
    if np.linalg.matrix_rank(candidate,tol=1e-9)>np.linalg.matrix_rank(H,tol=1e-9):
        chosen.append(s['segment_id']);H=candidate
u,sv,vh=np.linalg.svd(Z,full_matrices=True)
null=vh[rankz:].T
known=int(np.sum(np.linalg.norm(F@null,axis=1)<1e-8))
counts={}
for component in refs:
    ni=[i for i,n in enumerate(nodes) if n['component_id']==component]
    zi=[i for i in ni if i in z]
    counts[component]=len(ni)-1-int(np.linalg.matrix_rank(norm(hb[zi,:][:,cols]),tol=1e-9)) if zi else len(ni)-1
out={'nodes':len(nodes),'segments':len(segments),'components':len(refs),'zero_injection_rank':rankz,'full_rank':rankall,'minimum_independent_readings':rankall-rankz,'achieved_by':chosen,'known_without_readings':known,'by_component':counts,'row_normalized_rank_tolerance':1e-9}
print(json.dumps(out,indent=2))
Path(__file__).with_name('rank-result.json').write_text(json.dumps(out,indent=2)+'\n')
