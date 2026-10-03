from pathlib import Path
import sys,json,hashlib
import mujoco,numpy as np
ROOT=Path(__file__).resolve().parents[1];OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT));from audit_lowcube_clearance import Auditor
scene=OUT/'scene_lowcube_cutout_candidate.xml';a=Auditor(scene)
q=np.load(OUT/'feet_flat_crouch_paths.npz')['qpos']
r=a.audit_states('feet_flat_crouch_181',q)
old=mujoco.MjModel.from_xml_path(str(ROOT/'models/scene_tab5_lowcube.xml'));new=a.model
checks=[]
for i in range(1,old.njnt):
 name=old.joint(i).name; j=new.joint(name).id;b=int(old.jnt_bodyid[i]);c=int(new.jnt_bodyid[j]);bodyname=old.body(b).name
 eq={f:bool(np.array_equal(getattr(old,f)[i],getattr(new,f)[j]))for f in ['jnt_type','jnt_axis','jnt_pos','jnt_range','jnt_limited']}
 eq.update({f:bool(np.array_equal(getattr(old,f)[b],getattr(new,f)[c]))for f in ['body_pos','body_quat','body_mass','body_inertia','body_ipos','body_iquat']})
 eq['same_parent_name']=old.body(old.body_parentid[b]).name==new.body(new.body_parentid[c]).name
 checks.append({'joint':name,'body':bodyname,'checks':eq})
for g in [g for g in range(old.ngeom) if old.geom(g).name.startswith('shell_guard_legmesh_')]:
 ng=new.geom(old.geom(g).name).id
 eq={f:bool(np.array_equal(getattr(old,f)[g],getattr(new,f)[ng]))for f in ['geom_pos','geom_quat','geom_size','geom_type']}
 eq['same_mesh']=old.mesh(old.geom_dataid[g]).name==new.mesh(new.geom_dataid[ng]).name
 checks.append({'geom':old.geom(g).name,'checks':eq})
output={'scene':str(scene),'candidate_model_sha256':hashlib.sha256((OUT/'lowcube_cutout_candidate.xml').read_bytes()).hexdigest(),'report':r,'mechanism_all_exactly_equal':all(all(c['checks'].values())for c in checks),'mechanism_comparison':checks,'scope':'181 bounded feet-flat kinematic configurations; convex mesh-to-box optimization independent of contact filtering. No simulation/control/stability/continuous collision guarantee.'}
(OUT/'candidate_crouch_clearance.json').write_text(json.dumps(output,indent=2))
print(json.dumps({'report':r,'mechanism_all_exactly_equal':output['mechanism_all_exactly_equal']},indent=2))
