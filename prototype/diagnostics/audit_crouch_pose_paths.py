"""Read-only kinematic crouch diagnostics. Writes diagnostic artifacts only.
Run repository .venv/bin/python prototype/diagnostics/audit_crouch_pose_paths.py
No dynamics, training, network, hardware, or source/model edits.
"""
from pathlib import Path
import sys,json,itertools,hashlib
import numpy as np
import mujoco
from scipy.optimize import brentq, linprog
from scipy.spatial import ConvexHull
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from audit_lowcube_clearance import Auditor
OUT=Path(__file__).resolve().parent
SCENE=ROOT/'models/scene_tab5_lowcube.xml'
a=Auditor(SCENE); m,d=a.model,a.data
foot_geoms=[m.geom(n).id for n in ['left_foot_collision','right_foot_collision']]
visual=[g for g in range(m.ngeom) if m.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH and m.geom_group[g]==2]
trunk=m.body('trunk_base').id
vertices={}; equations={}
for g in visual:
 mesh=int(m.geom_dataid[g])
 if mesh in vertices:continue
 start,count=m.mesh_vertadr[mesh],m.mesh_vertnum[mesh]
 v=m.mesh_vert[start:start+count].astype(float)*1000
 hull=ConvexHull(v); vertices[mesh]=v[hull.vertices];equations[mesh]=hull.equations

def worldverts(g):return vertices[int(m.geom_dataid[g])]@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]*1000

def desc(g):
 return {'geom_id':int(g),'geom':m.geom(g).name,'body':m.body(m.geom_bodyid[g]).name,'mesh':m.mesh(m.geom_dataid[g]).name}

def forward(q):d.qpos[:]=q;mujoco.mj_forward(m,d)

def q_for(k):
 def sethip(h):
  q=np.array([0,0,.12,1,0,0,0,0,0,h,k,k-h,0,0,-h,-k,h-k]);forward(q)
  return d.site_xpos[m.site('left_foot').id,0]
 h=brentq(sethip,-.8,1.5707963267948,xtol=1e-14)
 sethip(h);q=d.qpos.copy()
 q[2]-=min(worldverts(g)[:,2].min() for g in foot_geoms)/1000
 forward(q);return q

# No head joints in this Tab5 prototype. Right leg exactly negates left.
knees=np.linspace(-.00494,np.pi/2,181)
states=np.array([q_for(k) for k in knees])
keynames=['STAND','SIT','FOLD']; stock=np.array([m.key(n).qpos.copy() for n in keynames])
upstream=np.array([0,0,.060,1,0,0,0,0,0,-.4079,1.35,0,0,0,.4079,-1.35,0])
grounded_stock=[]
for q in stock:
 forward(q);r=q.copy();r[2]-=min(worldverts(g)[:,2].min() for g in foot_geoms)/1000;grounded_stock.append(r)
np.savez_compressed(OUT/'feet_flat_crouch_paths.npz',qpos=states,knee_rad=knees,phase=np.linspace(0,1,len(states)),stock_keyframe_names=keynames,stock_keyframe_qpos=stock,stock_feet_grounded_qpos=grounded_stock,upstream_sitstand_qpos=upstream,crouch_80deg_qpos=q_for(np.deg2rad(80)),crouch_85deg_qpos=q_for(np.deg2rad(85)),crouch_90deg_qpos=states[-1],joint_names=[m.joint(i).name for i in range(1,m.njnt)])
print('Saved feet_flat_crouch_paths.npz',flush=True)

# Non-adjacent meshes only: same body and direct parent-child contacts are
# intended assembly/joint interfaces. Other intersections are compared with
# STAND, since convex hulls can bridge CAD holes and intentional clearances.
def legside(g):
 b=int(m.geom_bodyid[g]);chain=[]
 while b!=trunk and b:
  chain.append(b);b=int(m.body_parentid[b])
 if m.body('yaw2roll').id in chain:return 'left'
 if m.body('bearing_roll').id in chain:return 'right'
 return 'trunk'

pairs=[]
for g,h in itertools.combinations(visual,2):
 b,c=int(m.geom_bodyid[g]),int(m.geom_bodyid[h])
 if b==c or m.body_parentid[b]==c or m.body_parentid[c]==b:continue
 sg,sh=legside(g),legside(h)
 group='leg_leg' if {sg,sh}=={'left','right'} else ('leg_trunk' if 'trunk' in [sg,sh] else 'same_leg_nonadjacent')
 pairs.append((g,h,group))

def overlap_report(q):
 forward(q)
 transforms={}
 for g in visual:
  v=worldverts(g);R=d.geom_xmat[g].reshape(3,3);p=d.geom_xpos[g]*1000
  eq=equations[int(m.geom_dataid[g])];A=eq[:,:3]@R.T;b=A@p-eq[:,3]
  transforms[g]=(v.min(0),v.max(0),A,b)
 hits=[]
 for g,h,group in pairs:
  l1,u1,A1,b1=transforms[g];l2,u2,A2,b2=transforms[h]
  if np.any(l1>u2+1e-6)or np.any(l2>u1+1e-6):continue
  A=np.r_[A1,A2];b=np.r_[b1,b2]
  # Maximal shared interior ball radius in mm; >0 certifies convex-hull
  # interior overlap, not intersection of concave original CAD surfaces.
  r=linprog([0,0,0,-1],A_ub=np.c_[A,np.ones(len(A))],b_ub=b,bounds=[(None,None)]*4,method='highs')
  if not r.success: raise RuntimeError(f'Convex-pair LP failure {g},{h}: {r.message}')
  if r.x[3]>1e-4:
   dist=float(mujoco.mj_geomDistance(m,d,g,h,.1,None)*1000)
   hits.append({'pair':[int(g),int(h)],'group':group,'a':desc(g),'b':desc(h),'shared_interior_radius_mm':float(r.x[3]),'mujoco_signed_proxy_distance_mm':dist})
 return hits

def basic(q):
 forward(q)
 ground=[]
 for g in visual:
  z=float(worldverts(g)[:,2].min())
  if z < -1e-4:ground.append({**desc(g),'minimum_world_z_mm':z})
 return {'qpos':q.tolist(),'joint_degrees':np.rad2deg(q[7:]).tolist(),'trunk_height_mm':float(q[2]*1000),'feet_site_pos_mm':[(d.site_xpos[m.site(n).id]*1000).tolist()for n in ['left_foot','right_foot']], 'feet_site_rotation':[d.site_xmat[m.site(n).id].reshape(3,3).tolist()for n in ['left_foot','right_foot']], 'feet_min_z_mm':[float(worldverts(g)[:,2].min()) for g in foot_geoms], 'ground_mesh_penetrations':ground,'com_mm':(d.subtree_com[trunk]*1000).tolist(),'original_nonadjacent_proxy_overlaps':overlap_report(q)}

report={'scene':str(SCENE),'scene_sha256':hashlib.sha256(SCENE.read_bytes()).hexdigest(),'model_sha256':hashlib.sha256((ROOT/'models/tab5_lowcube.xml').read_bytes()).hexdigest(),'conventions':{'qpos':'17 values: base xyz metres; base quaternion wxyz; left yaw roll pitch knee ankle; right yaw roll pitch knee ankle radians','trajectory':'181 kinematic configurations, knee -0.00494 to pi/2, symmetric zero yaw/roll; ankle=knee-hip_pitch; solve left foot site x=0 with root upright/base x=y=0; set root z so lower of both sole meshes touches z=0; both sole site orientations identity','timing':'phase 0..1 only; no speed, torque, contact-force, stability, or dynamic feasibility claim. qpos file generic qpos key is feet-flat crouch trajectory.','stock_grounded':'Change root z only until lowest sole vertex touches ground; this does not fix foot pitch or other penetrations.'},'scope':'All original visual meshes convex hulls. Ignore only same-body/direct parent-child assembled interfaces. Hull overlaps are conservative flags; CAD may be concave and intended interfaces may remain. Ground vertex min is direct mesh geometry.','samples':{}}
for name,q in [(n,q)for n,q in zip(keynames,stock)]+ [('UPSTREAM_SITSTAND',upstream),('CROUCH_START',states[0]),('CROUCH_80',q_for(np.deg2rad(80))),('CROUCH_85',q_for(np.deg2rad(85))),('CROUCH_90',states[-1])]:
 r=basic(q);r['new_shell_audit']=a.audit_states(name,np.array([q]));report['samples'][name]=r
 print(name,'height',r['trunk_height_mm'],'ground',len(r['ground_mesh_penetrations']),'original overlaps',[(x['group'],x['a']['mesh'],x['b']['mesh'],round(x['shared_interior_radius_mm'],3))for x in r['original_nonadjacent_proxy_overlaps']], 'shell',[(k,v['minimum_gap_mm'])for k,v in r['new_shell_audit']['groups'].items()],flush=True)
# Check all 181 diagnostic samples; parent independently audits shell clearance.
report['original_proxy_sweep']=[]
for i in range(len(states)):
 r=basic(states[i]);report['original_proxy_sweep'].append({'sample':i,'knee_degrees':float(np.rad2deg(knees[i])),'root_z_mm':float(states[i,2]*1000),'original_nonadjacent_proxy_overlaps':r['original_nonadjacent_proxy_overlaps'],'ground_mesh_penetrations':r['ground_mesh_penetrations']})
(OUT/'crouch_pose_report.json').write_text(json.dumps(report,indent=2))
print('Saved crouch_pose_report.json',flush=True)
