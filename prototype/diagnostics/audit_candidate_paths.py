from pathlib import Path
import sys,json,numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));from audit_lowcube_clearance import Auditor
scene=ROOT/'diagnostics/scene_lowcube_cutout_candidate.xml';a=Auditor(scene);p=np.load(ROOT/'diagnostics/posture_sweeps.npz');r=[]
for name in p.files:r.append(a.audit_states(name,p[name]))
(ROOT/'diagnostics/candidate_path_clearance.json').write_text(json.dumps({'candidate':str(scene),'scope':'101 samples per linear joint-space interpolation, clearance only, not a feasible balance-controlled movement','reports':r},indent=2))
for x in r: print(x['label'],[(n,d['minimum_gap_mm']) for n,d in x['groups'].items()])
