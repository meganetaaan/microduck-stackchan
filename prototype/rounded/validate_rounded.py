"""Independent rounded-shell sampled clearance and immutable mechanism audit.

Uses compiled MuJoCo meshes, convex hull halfspaces, and independently evaluated
separating-plane support bounds. Run with the repository .venv/bin/python.
This is sampled numerical geometry, not continuous motion/manufacturing safety.
"""
from pathlib import Path
import hashlib,itertools,json,time
import mujoco,numpy as np,scipy,trimesh
from scipy.spatial import ConvexHull
from scipy.optimize import minimize,LinearConstraint
HERE=Path(__file__).resolve().parent; ROOT=HERE.parent

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def vertices(m,d,g):
 if m.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH:
  mesh=m.geom_dataid[g];a=m.mesh_vertadr[mesh];n=m.mesh_vertnum[mesh];v=m.mesh_vert[a:a+n].astype(float)
 elif m.geom_type[g]==mujoco.mjtGeom.mjGEOM_BOX:v=np.array(list(itertools.product([-1,1],repeat=3)))*m.geom_size[g]
 else:raise ValueError(m.geom(g).name)
 t=m.body('trunk_base').id
 return (v@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]-d.xpos[t])@d.xmat[t].reshape(3,3)*1000

def invariants(o,m):
 records=[]
 def check(kind,name,a,b,fields):
  r={'kind':kind,'name':name,'checks':{f:bool(np.array_equal(getattr(o,f)[a],getattr(m,f)[b]))for f in fields}};records.append(r);return r
 names=[o.body(o.jnt_bodyid[j]).name for j in range(1,o.njnt)]+['tab5']
 for name in names:
  a=o.body(name).id;b=m.body(name).id
  r=check('body',name,a,b,['body_pos','body_quat','body_mass','body_inertia','body_ipos','body_iquat','body_gravcomp'])
  r['checks']['same_parent']=o.body(o.body_parentid[a]).name==m.body(m.body_parentid[b]).name
  ag=np.flatnonzero(o.geom_bodyid==a);bg=np.flatnonzero(m.geom_bodyid==b);r['checks']['geom_count']=len(ag)==len(bg)
  for idx,(x,y) in enumerate(zip(ag,bg)):
   r=check('geom',name+':'+(o.geom(x).name or str(idx)),x,y,['geom_pos','geom_quat','geom_size','geom_type','geom_contype','geom_conaffinity','geom_condim','geom_friction','geom_solref','geom_solimp','geom_margin','geom_gap','geom_rgba','geom_group'])
   r['checks']['same_name']=o.geom(x).name==m.geom(y).name
   if o.geom_type[x]==mujoco.mjtGeom.mjGEOM_MESH and m.geom_type[y]==mujoco.mjtGeom.mjGEOM_MESH:
    u=o.geom_dataid[x];v=m.geom_dataid[y];r['checks']['same_mesh']=o.mesh(u).name==m.mesh(v).name
    for f in ['vert','face','normal']:
     oa=getattr(o,'mesh_'+f+'adr')[u];na=getattr(m,'mesh_'+f+'adr')[v];oc=getattr(o,'mesh_'+f+'num')[u];nc=getattr(m,'mesh_'+f+'num')[v]
     r['checks']['mesh_'+f]=oc==nc and bool(np.array_equal(getattr(o,'mesh_'+f)[oa:oa+oc],getattr(m,'mesh_'+f)[na:na+nc]))
 for a in range(o.njnt):
  name=o.joint(a).name;b=m.joint(name).id;check('joint',name,a,b,['jnt_type','jnt_axis','jnt_pos','jnt_range','jnt_limited','jnt_stiffness','jnt_margin','jnt_solref','jnt_solimp','jnt_actfrcrange','jnt_actfrclimited'])
 records.append({'kind':'model','checks':{f:bool(np.array_equal(getattr(o,f),getattr(m,f)))for f in ['nq','nv','nu','neq','qpos0','key_qpos','actuator_ctrlrange','actuator_forcerange','actuator_gainprm','actuator_biasprm','actuator_gear']}})
 return {'all_exactly_equal':all(all(r['checks'].values())for r in records),'records':records}

class Auditor:
 def __init__(self,m,d,shell):
  self.m,self.d,self.shell=m,d,shell;self.trunk=m.body('trunk_base').id;self.legs=[g for g in range(m.ngeom)if m.geom(g).name.startswith('shell_guard_legmesh_')];self.panels=shell+[m.geom('tab5_collision').id];self.local={};self.stat={};self.cache={}
  for g in self.legs:
   mid=m.geom_dataid[g];a=m.mesh_vertadr[mid];n=m.mesh_vertnum[mid];v=m.mesh_vert[a:a+n].astype(float)*1000;h=ConvexHull(v);self.local[g]=(v[h.vertices],h.equations)
  for g in self.panels:
   v=vertices(m,d,g);h=ConvexHull(v);v=v[h.vertices];self.stat[g]=(v,h.equations,v.min(0),v.max(0))
 def collect(self,states):
  out=[];m,d=self.m,self.d;hits=[];shell=set(self.panels);legs=set(self.legs)
  for index,q in enumerate(states):
   d.qpos[:]=q;mujoco.mj_forward(m,d);tr=d.xmat[self.trunk].reshape(3,3);tp=d.xpos[self.trunk];sample={}
   for g in self.legs:
    r=tr.T@d.geom_xmat[g].reshape(3,3);t=(d.geom_xpos[g]-tp)@tr*1000;v,eq=self.local[g];v=v@r.T+t;A=eq[:,:3]@r.T;b=A@t-eq[:,3];sample[g]=(v,A,b,v.min(0),v.max(0))
   for c in d.contact[:d.ncon]:
    p={int(c.geom1),int(c.geom2)}
    if p&legs and p&shell:hits.append({'sample':index,'geom1':m.geom(c.geom1).name,'geom2':m.geom(c.geom2).name,'distance_mm':float(c.dist*1000)})
   out.append(sample)
  return out,hits
 @staticmethod
 def repair(x,c,A,b):
  if np.max(A@x-b)<=0:return x
  ac=A@c;ad=A@(x-c);valid=ad>0;s=min(1.,float(np.min((b-ac)[valid]/ad[valid])))
  return c+(x-c)*max(0,s*(1-1e-12))
 def solve(self,panel,leg,mesh):
  v,A,b,lo,hi=mesh;w,eq,blo,bhi=self.stat[panel];B=eq[:,:3];c=-eq[:,3];mat=np.zeros((len(A)+len(B),6));mat[:len(A),:3]=A;mat[len(A):,3:]=B;bound=np.r_[b,c];ac=v.mean(0);bc=w.mean(0);key=(panel,leg)
  def fun(x):z=x[:3]-x[3:];return z@z
  def jac(x):z=2*(x[:3]-x[3:]);return np.r_[z,-z]
  r=minimize(fun,self.cache.get(key,np.r_[ac,bc]),jac=jac,constraints=[LinearConstraint(mat,-np.inf,bound)],method='SLSQP',options={'ftol':1e-9,'maxiter':200});self.cache[key]=r.x
  x=self.repair(r.x[:3],ac,A,b);y=self.repair(r.x[3:],bc,B,c);upper=float(np.linalg.norm(y-x));aabb=float(np.linalg.norm(np.maximum(np.maximum(blo-hi,lo-bhi),0)))
  support=0.
  if upper>1e-10:
   axis=(y-x)/upper;support=float(np.min(w@axis)-np.max(v@axis))
  lower=max(support,aabb,0.)
  return {'lower_bound_mm':lower,'feasible_upper_bound_mm':upper,'duality_gap_mm':upper-lower,'optimizer_success':bool(r.success),'optimizer_status':int(r.status),'constraint_violation_mm':float(max(0,np.max(mat@r.x-bound))),'leg_closest_point_trunk_mm':x.tolist(),'shell_closest_point_trunk_mm':y.tolist()}
 def audit(self,label,states):
  transforms,hits=self.collect(states);candidates={'shell':[],'tab5':[]};m=self.m;blo=np.array([self.stat[p][2]for p in self.panels]);bhi=np.array([self.stat[p][3]for p in self.panels])
  for index,meshes in enumerate(transforms):
   lo=np.array([meshes[g][3]for g in self.legs]);hi=np.array([meshes[g][4]for g in self.legs]);delta=np.maximum(np.maximum(blo[:,None,:]-hi[None,:,:],lo[None,:,:]-bhi[:,None,:]),0);lower=np.linalg.norm(delta,axis=2)
   ps,ls=np.where(lower<20)
   for p,l in zip(ps,ls):
    panel=self.panels[p];group='tab5' if m.geom(panel).name=='tab5_collision'else'shell';candidates[group].append((float(lower[p,l]),index,panel,self.legs[l]))
  report={'label':label,'sample_count':len(states),'shell_tab5_leg_contact_count':len(hits),'contact_examples':hits[:30],'groups':{}}
  for group,items in candidates.items():
   best=20.;lower=20.;solved=0;failures=[];winner=None;maxgap=0.;nextbound=20.
   for bound,index,panel,leg in sorted(items):
    if bound>best+1e-7:nextbound=bound;break
    r=self.solve(panel,leg,transforms[index][leg]);solved+=1;lower=min(lower,r['lower_bound_mm']);maxgap=max(maxgap,r['duality_gap_mm'])
    if not r['optimizer_success']or r['constraint_violation_mm']>1e-5:failures.append({'sample':index,'panel':m.geom(panel).name,'leg':m.geom(leg).name,**r})
    if r['feasible_upper_bound_mm']<best:best=r['feasible_upper_bound_mm'];winner={'sample_index':int(index),'panel':m.geom(panel).name,'leg_geom':m.geom(leg).name,'leg_body':m.body(m.geom_bodyid[leg]).name,'qpos':states[index].tolist(),**r}
   lower=min(lower,nextbound);report['groups'][group]={'minimum_gap_lower_bound_mm':lower,'minimum_gap_upper_bound_mm':best,'winner':winner,'candidate_pairs':len(items),'optimized_pairs':solved,'max_pair_duality_gap_mm':maxgap,'optimizer_failures':failures}
   print(label,group,'bounds',lower,best,'solved',solved,'candidates',len(items),'failures',len(failures),flush=True)
  return report

def exported_geometry_check():
 import manifold3d as mf
 def load(path):
  mesh=trimesh.load(path,process=False)
  solid=mf.Manifold(mf.Mesh64(np.ascontiguousarray(mesh.vertices*1000,dtype=np.float64),np.ascontiguousarray(mesh.faces,dtype=np.uint64)))
  assert solid.status()==mf.Error.NoError,(str(path),solid.status())
  return mesh,solid
 def box(bounds):
  b=np.array(bounds,float);return mf.Manifold.cube(b[:,1]-b[:,0]).translate(b[:,0])
 visual,shell=load(HERE/'assets/rounded_shell.obj');built=json.loads((HERE/'geometry_report.json').read_text())
 hulls=[load(HERE/part['file'])[1]for part in built['parts']]
 cover=mf.Manifold.batch_boolean(hulls,mf.OpType.Add)
 voids=[box([[39.5,42],[25,67],[-46,-15]]),box([[39.5,42],[-67,-25],[-46,-15]])]
 for y in [[-67,-64.5],[64.5,67]]:voids +=[box([[-62,42],y,[-46,10]]),box([[-50,25],y,[10,35]])]
 # OBJ quantization occurs separately for the whole visual and each cell hull.
 # A 0.00001 mm L-infinity expansion diagnoses tiny exported rounding gaps;
 # it is a read-only analytical check and does not enlarge actual colliders.
 tol=0.00001;cube=mf.Manifold.cube([2*tol]*3,center=True)
 expanded=mf.Manifold.batch_boolean([h.minkowski_sum(cube)for h in hulls],mf.OpType.Add)
 return {'method':'Independent reread of exported OBJ meshes; Manifold64 union of all convex pieces, subtraction from full visual shell, and intersection with six established angular opening boxes. Numerical booleans after OBJ export precision, not formal certification.','exported_visual_watertight':bool(visual.is_watertight),'exported_visual_winding_consistent':bool(visual.is_winding_consistent),'connected_components':len(shell.decompose()),'visual_volume_mm3':shell.volume(),'exported_collider_union_volume_mm3':cover.volume(),'visual_not_covered_volume_mm3':max(0,(shell-cover).volume()),'conservative_extra_volume_mm3':max(0,(cover-shell).volume()),'visual_prior_void_overlap_mm3':sum(max(0,(shell^v).volume())for v in voids),'collider_prior_void_overlap_mm3':sum(max(0,(cover^v).volume())for v in voids),'export_coordinate_precision_mm':tol,'coverage_linf_tolerance_mm':tol,'coverage_euclidean_tolerance_bound_mm':float(np.sqrt(3)*tol),'visual_uncovered_after_tolerance_mm3':max(0,(shell-expanded).volume()),'asset_sha256':{str(f.relative_to(HERE)):sha(f)for f in [HERE/'assets/rounded_shell.obj',*[HERE/part['file']for part in built['parts']]]}}

def main():
 scene=HERE/'scene_tab5_lowcube_rounded.xml';m=mujoco.MjModel.from_xml_path(str(scene));d=mujoco.MjData(m);mujoco.mj_forward(m,d);o=mujoco.MjModel.from_xml_path(str(ROOT/'diagnostics/scene_lowcube_cutout_candidate.xml'))
 shell=[g for g in range(m.ngeom)if m.geom(g).name.startswith('rounded_shell_collision_')];legs=[g for g in range(m.ngeom)if m.geom(g).name.startswith('shell_guard_legmesh_')]
 pairs={tuple(sorted((int(m.pair_geom1[i]),int(m.pair_geom2[i]))))for i in range(m.npair)}
 missing=[(m.geom(s).name,m.geom(l).name)for s in shell for l in legs if tuple(sorted((s,l)))not in pairs and not (m.geom_contype[s]&m.geom_conaffinity[l]or m.geom_contype[l]&m.geom_conaffinity[s])]
 result={'scene':str(scene),'model_sha256':sha(HERE/'tab5_lowcube_rounded.xml'),'geometry_report_sha256':sha(HERE/'geometry_report.json'),'mujoco_version':mujoco.__version__,'scipy_version':scipy.__version__,'invariants':invariants(o,m),'shell_piece_count':len(shell),'leg_hull_count':len(legs),'shell_leg_pairs_without_collision_route':missing,'reports':[],'scope':'484 saved kinematic samples. Distances to compiled convex hulls of unchanged leg CAD, conservative per-cell shell hulls, and unchanged Tab5 box. AABB lower bounds prune pairs; each refined pair gets an independent support-plane lower bound. No continuous collision, balance/controller, strength, cable, manufacturing or hardware-safety guarantee.'}
 print('Invariants',result['invariants']['all_exactly_equal'],'missing collision pairs',len(missing),flush=True)
 out=HERE/'validation_report.json';out.write_text(json.dumps(result,indent=2));auditor=Auditor(m,d,shell);p=np.load(ROOT/'diagnostics/posture_sweeps.npz');states={k:p[k]for k in p.files};states['feet_flat_crouch_181']=np.load(ROOT/'diagnostics/feet_flat_crouch_paths.npz')['qpos']
 for label,q in states.items():result['reports'].append(auditor.audit(label,q));out.write_text(json.dumps(result,indent=2))
 result['independent_exported_geometry_check']=exported_geometry_check();out.write_text(json.dumps(result,indent=2))
 print('Wrote',out,flush=True)
if __name__=='__main__':
 import sys
 if '--geometry-only' in sys.argv:
  out=HERE/'validation_report.json';r=json.loads(out.read_text());r['independent_exported_geometry_check']=exported_geometry_check();out.write_text(json.dumps(r,indent=2));print('Updated exported geometry check')
 else:main()
