from pathlib import Path
import sys,json
import mujoco,numpy as np
ROOT=Path(__file__).resolve().parents[1]
m=mujoco.MjModel.from_xml_path(str(ROOT/'models/scene_tab5_lowcube.xml'));d=mujoco.MjData(m)
keys={n:m.key(n).qpos.copy() for n in ['STAND','SIT','FOLD']}
shell={g for g in range(m.ngeom) if m.geom(g).name.startswith('cube_') or m.geom(g).name=='tab5_collision'}
legs={g for g in range(m.ngeom) if m.geom(g).name.startswith('shell_guard_legmesh_')}
reports=[];saved={}
for a,b in [('STAND','SIT'),('STAND','FOLD'),('SIT','FOLD')]:
 states=[];hits={};examples=[]
 for k,t in enumerate(np.linspace(0,1,101)):
  q=keys[a]*(1-t)+keys[b]*t;d.qpos[:]=q;mujoco.mj_forward(m,d);states.append(q)
  for c in d.contact[:d.ncon]:
   pair={int(c.geom1),int(c.geom2)}
   if not(pair&shell and pair&legs):continue
   sg=next(iter(pair&shell));lg=next(iter(pair&legs));n=m.geom(sg).name;body=m.body(m.geom_bodyid[lg]).name;mesh=m.mesh(m.geom_dataid[lg]).name
   key=n+' / '+body+' / '+mesh
   h=hits.setdefault(key,{'first_t':float(t),'last_t':float(t),'max_penetration_mm':0.,'count':0,'point_bounds_trunk_mm':[[1e6,-1e6] for _ in range(3)]})
   h['last_t']=float(t);h['count']+=1;h['max_penetration_mm']=max(h['max_penetration_mm'],-1000*float(c.dist));p=(c.pos-d.xpos[1])@d.xmat[1].reshape(3,3)*1000
   for j in range(3):h['point_bounds_trunk_mm'][j]=[min(h['point_bounds_trunk_mm'][j][0],float(p[j])),max(h['point_bounds_trunk_mm'][j][1],float(p[j]))]
  if k in [0,25,50,75,100]:examples.append({'t':float(t),'qpos':q.tolist(),'contacts':len(d.contact)})
 reports.append({'path':f'{a}_to_{b}','interpolation':'101 linear joint-space samples; not a dynamically feasible transition','hits':hits,'examples':examples})
 saved[f'{a}_to_{b}']=np.array(states)
np.savez_compressed(ROOT/'diagnostics/posture_sweeps.npz',**saved)
(ROOT/'diagnostics/posture_sweep_contacts.json').write_text(json.dumps(reports,indent=2))
for r in reports:
 print(r['path'])
 for k,h in r['hits'].items():print(k,'t',h['first_t'],h['last_t'],'depth',round(h['max_penetration_mm'],2),'bounds',np.round(h['point_bounds_trunk_mm'],1).tolist())
