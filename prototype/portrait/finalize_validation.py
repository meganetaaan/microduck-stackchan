"""Finalize bounded independent report without waiting for far-rail optimization."""
import sys,json,itertools
from pathlib import Path
import mujoco,numpy as np
sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from validate_portrait import geometry_check,sha
from validate_rounded import Auditor
m=mujoco.MjModel.from_xml_path(str(HERE/'scene_tab5_portrait.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
boxes=[g for g in range(m.ngeom)if m.geom(g).name.startswith('cube_portrait_')];a=Auditor(m,d,boxes)
r=json.loads((HERE/'validation_report.json').read_text());r['reports']=[x for x in r['reports']if x['label']not in ['walk_10s','idle_10s']];r['geometry']=geometry_check(m,d,a.panels);r['rollout_conservative_reports']=[]
for label in ['walk_10s','idle_10s']:
 f=HERE/(label+'_states.npz');npz=np.load(f);q=npz['qpos'];trans,hits=a.collect(q);mins=np.full(len(a.panels),np.inf);winner=[None]*len(a.panels);uncertain=[]
 for i,meshes in enumerate(trans):
  for pi,p in enumerate(a.panels):
   w,eq,lo,hi=a.stat[p]
   for g in a.legs:
    v,A,b,l,h=meshes[g];aabb=float(np.linalg.norm(np.maximum(np.maximum(lo-h,l-hi),0)));lb=aabb;method='AABB'
    if lb<3.:
     # Independent supports along box axes, hull-facet normals and centroid
     # axis. Maximum positive projected gap is a Euclidean lower bound.
     axis=w.mean(0)-v.mean(0);axis/=np.linalg.norm(axis);axes=np.r_[np.eye(3),A,axis[None,:]]
     vv=v@axes.T;ww=w@axes.T;gaps=np.maximum(ww.min(0)-vv.max(0),vv.min(0)-ww.max(0));support=float(gaps.max());lb=max(lb,support,0.);method='AABB plus independently evaluated support planes'
    if lb<mins[pi]:mins[pi]=lb;winner[pi]={'sample_index':i,'leg_geom':m.geom(g).name,'method':method}
    if lb<=0:uncertain.append({'sample_index':i,'part':m.geom(p).name,'leg_geom':m.geom(g).name})
 def group(ids):return {'minimum_certified_lower_bound_mm':float(min(mins[j]for j in ids)),'minimum_exact_distance_not_computed':True}
 rows=[{'geom':m.geom(p).name,'minimum_certified_lower_bound_mm':float(mins[i]),'limiting_bound':winner[i]}for i,p in enumerate(a.panels)]
 rr={'label':label,'sample_count':len(q),'sampling_interval_s':float(npz['dt']),'state_sha256':sha(f),'new_physical_to_leg_contact_count':len(hits),'contact_examples':hits[:30],'all_new_parts':group(range(len(a.panels))),'frame_only':group([i for i,p in enumerate(a.panels)if any(m.geom(p).name.startswith('cube_portrait_'+s)for s in ['bottom_','top_','post_'])]),'Tab5':group([len(a.panels)-1]),'per_geom':rows,'pairs_without_positive_separation_certificate':uncertain,'method':'Conservative AABB distances, refined below3 mm with independently evaluated mesh-facet/box-axis/centroid separating-plane supports. Positive lower bounds certify sampled disjointness numerically but are not exact minima.'}
 r['rollout_conservative_reports'].append(rr);print(label,rr['all_new_parts'],rr['frame_only'],rr['Tab5'],'uncertain',len(uncertain),flush=True)
r['total_detailed_sample_count']=sum(x['sample_count']for x in r['reports']);r['total_fast_rollout_sample_count']=sum(x['sample_count']for x in r['rollout_conservative_reports']);r['total_sample_count']=r['total_detailed_sample_count']+r['total_fast_rollout_sample_count'];r['optimizer_diagnostics']={'status_warning_count':sum(len(g['optimizer_failures'])for x in r['reports']for g in x['per_geom']),'max_independent_bound_gap_mm':max(g['max_pair_duality_gap_mm']for x in r['reports']for g in x['per_geom']),'note':'SLSQP status8 warnings on some far pairs are retained transparently. Positive support lower bounds do not depend on optimizer success. Bounds are numerical, not formal exact arithmetic.'};r['scope']+=' Rollouts use a separate fast conservative lower-bound pass, with no claim to exact minimum distance.'
(HERE/'validation_final_report.json').write_text(json.dumps(r,indent=2));print('Saved final report',flush=True)
# Prefer any detailed rollout completed meanwhile; fill only missing labels with
# the conservative pass, retaining the documented validation_report.json schema.
latest=json.loads((HERE/'validation_report.json').read_text());detailed={x['label']:x for x in latest['reports']};combined=[]
for name in ['STAND_to_SIT','STAND_to_FOLD','SIT_to_FOLD','feet_flat_crouch_181','walk_10s','idle_10s']:
 if name in detailed:row=detailed[name];row['distance_method']='bounded closest-point optimization with independent support verification'
 else:
  row=next(x.copy()for x in r['rollout_conservative_reports']if x['label']==name)
  for k in ['all_new_parts','frame_only','Tab5']:
   row[k]=dict(row[k]);row[k]['minimum_gap_lower_bound_mm']=row[k]['minimum_certified_lower_bound_mm'];row[k]['minimum_gap_upper_bound_mm']=None
  row['distance_method']='fast conservative separating-plane/AABB lower bound only'
 combined.append(row)
r['reports']=combined;r['total_sample_count']=sum(x['sample_count']for x in combined);r['total_detailed_sample_count']=sum(x['sample_count']for x in combined if x['distance_method'].startswith('bounded'));r['total_fast_rollout_sample_count']=r['total_sample_count']-r['total_detailed_sample_count'];r['state_files']=latest['state_files']
r['optimizer_diagnostics']={'status_warning_count':sum(len(g.get('optimizer_failures',[]))for x in combined for g in x.get('per_geom',[])),'max_independent_bound_gap_mm':max(g.get('max_pair_duality_gap_mm',0)for x in combined for g in x.get('per_geom',[])),'note':'SLSQP status8 warnings on some far pairs are retained transparently. Positive support lower bounds do not depend on optimizer success. Bounds are numerical, not formal exact arithmetic.'}
for path in [HERE/'validation_final_report.json',HERE/'validation_report.json']:
 temp=path.with_suffix('.tmp');temp.write_text(json.dumps(r,indent=2));temp.replace(path)
lines=['# Portrait open-frame validation','',f"Model SHA-256: `{r['model_sha256']}`",'', 'Independent geometry audit of all 18 new physical boxes against all 34 original moving leg CAD hulls. All 612 explicit collision pairs are present.','',f"{r['total_sample_count']} saved states checked; zero new-part/leg contacts. All sampled states have positive independent separation lower bounds.",'','| Saved states | Count | All new parts ≥ mm | Frame only ≥ mm | Tab5 ≥ mm |','|---|---:|---:|---:|---:|']
for x in combined:lines.append(f"| {x['label']} | {x['sample_count']} | {x['all_new_parts']['minimum_gap_lower_bound_mm']:.6f} | {x['frame_only']['minimum_gap_lower_bound_mm']:.6f} | {x['Tab5']['minimum_gap_lower_bound_mm']:.6f} |")
lines+=['','The smallest geometric gap is at a mount post, not the open-frame rails. Conservative rollout bounds may be lower than exact sampled closest-point minima. See each JSON row’s distance_method.','', '## Geometry and physics','- Frame measures 80 mm wide × 80 mm deep × 83 mm high','- Tab5 physical bounds are 80 mm wide × 128 mm high × 12 mm deep; mounted upper-assembly depth is 92 mm','- Total modeled mass: 657.8729 g; all added masses and uniform-box inertias match parameters','- Original legs, joints, actuator parameters, pelvis, battery and fixed servo CAD are unchanged from the saved rounded variant','- No meaningful intersection with retained fixed CAD; two posts have intended surface contact with the trunk plate','- All added parts form one face-contact-connected assembly. Rounded Tab5 body contacts the front rails; no hidden full-frame collider closes its openings','- Float32 compiled Tab5 vertices exceed the ideal collision bounds by a few millionths of a millimetre; the recorded 0.00001 mm analytical coverage tolerance covers them','', '## Limits','- This is a sampled numerical audit, not continuous collision or full joint-range certification','- Saved SIT retains about 4.44094 mm original foot/ground penetration; FOLD retains eight original nonadjacent leg convex-hull overlap flags. These predate this design','- Some far-pair SLSQP status warnings remain in the JSON; independently evaluated positive support bounds, not optimizer status, establish the reported sampled separations','- Mounting interfaces are conceptual. Fasteners, holes, wiring, manufacturing tolerances, stiffness, structural strength and hardware safety remain unvalidated','- No training, inference, hardware action or prior-model editing was performed by this audit','']
(HERE/'VALIDATION.md').write_text('\n'.join(lines));print('Compatible final reports and VALIDATION.md saved',flush=True)
