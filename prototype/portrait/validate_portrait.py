"""Independent read-only portrait-frame geometry validation.

Writes only this variant's validation artifacts. No inference, network, training,
hardware, or mutation of a prior model. Reuses the previously reviewed rounded
Auditor's convex distance solver and independently evaluated support bounds.
"""
from pathlib import Path
import hashlib, itertools, json, sys
import mujoco, numpy as np, scipy, trimesh, manifold3d as mf
from scipy.spatial import ConvexHull
from scipy.optimize import linprog
HERE=Path(__file__).resolve().parent; ROOT=HERE.parent
sys.dont_write_bytecode=True
sys.path.insert(0,str(ROOT/'rounded'))
from validate_rounded import Auditor, vertices, invariants, sha

def original_invariants(o,m):
    r=invariants(o,m)
    r['records']=[x for x in r['records'] if not x.get('name','').startswith('tab5')]
    # Fixed original pelvis, battery, electronics housing, and yaw servos.
    a=o.body('trunk_base').id;b=m.body('trunk_base').id
    r['records'].append({'kind':'body','name':'trunk_base','checks':{f:bool(np.array_equal(getattr(o,f)[a],getattr(m,f)[b])) for f in ['body_pos','body_quat','body_mass','body_inertia','body_ipos','body_iquat','body_gravcomp']}})
    ga=np.flatnonzero(o.geom_bodyid==a);gb=np.flatnonzero(m.geom_bodyid==b)
    r['records'].append({'kind':'fixed_geom_count','checks':{'equal':len(ga)==len(gb)}})
    for x,y in zip(ga,gb):
        c={f:bool(np.array_equal(getattr(o,f)[x],getattr(m,f)[y])) for f in ['geom_pos','geom_quat','geom_size','geom_type','geom_contype','geom_conaffinity','geom_condim','geom_friction','geom_solref','geom_solimp','geom_margin','geom_gap','geom_group','geom_rgba']}
        u=o.geom_dataid[x];v=m.geom_dataid[y];c['mesh_name']=o.mesh(u).name==m.mesh(v).name
        for f in ['vert','face','normal']:
            i=getattr(o,'mesh_'+f+'adr')[u];j=getattr(m,'mesh_'+f+'adr')[v];n=getattr(o,'mesh_'+f+'num')[u];k=getattr(m,'mesh_'+f+'num')[v]
            c['mesh_'+f]=n==k and bool(np.array_equal(getattr(o,'mesh_'+f)[i:i+n],getattr(m,'mesh_'+f)[j:j+k]))
        r['records'].append({'kind':'fixed_geom','name':o.mesh(u).name,'checks':c})
    r['records'].append({'kind':'dynamics','checks':{f:bool(np.array_equal(getattr(o,f),getattr(m,f))) for f in ['dof_damping','dof_armature','dof_frictionloss','actuator_ctrlrange','actuator_forcerange','actuator_gainprm','actuator_biasprm','actuator_gear']}})
    r['records'].append({'kind':'gravity','checks':{'equal':bool(np.array_equal(o.opt.gravity,m.opt.gravity))}})
    r['all_exactly_equal']=all(all(x['checks'].values()) for x in r['records'])
    r['intentional_exclusions']='Tab5 rotated/repositioned with new visible portrait graphics and corresponding cuboid inertia; replacement open-frame, mount, tray and compute arrangement.'
    return r

def mesh_solid(m,d,g):
    mid=m.geom_dataid[g];a=m.mesh_faceadr[mid];n=m.mesh_facenum[mid]
    tm=trimesh.Trimesh(vertices(m,d,g),m.mesh_face[a:a+n],process=True)
    s=mf.Manifold(mf.Mesh64(np.ascontiguousarray(tm.vertices,dtype=np.float64),np.ascontiguousarray(tm.faces,dtype=np.uint64)))
    assert tm.is_watertight and s.status()==mf.Error.NoError,(g,tm.is_watertight,s.status())
    return tm,s

def box(b):
    b=np.asarray(b,float);return mf.Manifold.cube(b[:,1]-b[:,0]).translate(b[:,0])

def geometry_check(m,d,physical):
    p=json.loads((HERE/'parameters.json').read_text());built=json.loads((HERE/'geometry_report.json').read_text());bounds={};parts=[]
    for g in physical:
        n=m.geom(g).name;v=vertices(m,d,g);b=np.c_[v.min(0),v.max(0)];bounds[n]=b;bid=m.geom_bodyid[g];size=b[:,1]-b[:,0];mass=m.body_mass[bid];inertia=mass/12*np.array([size[1]**2+size[2]**2,size[0]**2+size[2]**2,size[0]**2+size[1]**2])*1e-6
        expected=next((z for z in built['parts'] if z['name']+'_collision'==n),None)
        if expected is None:expected={'bounds_xyz_mm':(np.array(p['tab5_center_xyz_mm'])[:,None]+np.array(p['tab5_size_xyz_mm'])[:,None]*np.array([[-.5,.5]])).tolist(),'mass_kg':p['tab5_mass_kg']}
        parts.append({'geom':n,'body':m.body(bid).name,'bounds_xyz_mm':b.tolist(),'mass_kg':float(mass),'inertia_kg_m2':m.body_inertia[bid].tolist(),'positive_mass_and_inertia':bool(mass>0 and np.min(m.body_inertia[bid])>0),'physical_dimensions_match_parameters':bool(np.allclose(b,expected['bounds_xyz_mm'],atol=1e-7,rtol=0)),'mass_match':bool(abs(mass-expected['mass_kg'])<1e-12),'uniform_box_inertia_matches':bool(np.allclose(m.body_inertia[bid],inertia,rtol=1e-8,atol=1e-14)),'visible_geometry_is_same_physical_box':bool(m.geom_type[g]==mujoco.mjtGeom.mjGEOM_BOX and m.geom_group[g]==0),'collision_enabled':bool(m.geom_contype[g] or m.geom_conaffinity[g])})
    # Every face/edge touched is an ideal geometric interface only.
    edges=[];overlaps=[];adj={n:[] for n in bounds}
    for a,b in itertools.combinations(bounds,2):
        delta=np.minimum(bounds[a][:,1],bounds[b][:,1])-np.maximum(bounds[a][:,0],bounds[b][:,0])
        if np.all(delta>=-1e-7) and np.count_nonzero(delta>1e-7)>=2:
            edges.append({'a':a,'b':b,'overlap_extents_mm':np.maximum(delta,0).tolist()});adj[a].append(b);adj[b].append(a)
        if np.all(delta>1e-7):overlaps.append({'a':a,'b':b,'overlap_volume_mm3':float(np.prod(delta))})
    unseen=set(bounds);components=[]
    while unseen:
        todo=[unseen.pop()];component=[]
        while todo:
            n=todo.pop();component.append(n)
            for b in adj[n]:
                if b in unseen:unseen.remove(b);todo.append(b)
        components.append(component)
    frame=[b for n,b in bounds.items() if any(n.startswith('cube_portrait_'+prefix) for prefix in ['bottom_','top_','post_'])]
    fb=np.c_[np.min([b[:,0] for b in frame],axis=0),np.max([b[:,1] for b in frame],axis=0)]
    allb=np.c_[np.min([b[:,0]for b in bounds.values()],axis=0),np.max([b[:,1]for b in bounds.values()],axis=0)]
    # Boolean exact CAD audit, including original fixed-body parts normally
    # filtered as one welded body by MuJoCo collision detection.
    fixed=[];seen=set();intersections=[];near_contact=[]
    for g in range(m.ngeom):
        if m.geom_bodyid[g]!=m.body('trunk_base').id:continue
        key=(int(m.geom_dataid[g]),d.geom_xpos[g].tobytes(),d.geom_xmat[g].tobytes())
        if key in seen:continue
        seen.add(key);tm,s=mesh_solid(m,d,g);name=m.mesh(m.geom_dataid[g]).name
        fixed.append({'geom_id':g,'mesh':name,'bounds_xyz_mm':np.c_[tm.bounds[0],tm.bounds[1]].tolist(),'watertight':bool(tm.is_watertight),'volume_mm3':float(s.volume())})
        for n,b in bounds.items():
            inter=max(0.,float((box(b)^s).volume()))
            if inter>1e-3:intersections.append({'new_geom':n,'fixed_geom_id':g,'fixed_mesh':name,'intersection_mm3':inter})
            expanded=b+np.array([[-.001,.001]]*3);touch=max(0.,float((box(expanded)^s).volume()))
            if touch>1e-5 and inter<=1e-3:near_contact.append({'new_geom':n,'fixed_geom_id':g,'fixed_mesh':name,'intersection_mm3':inter,'expanded_1micron_intersection_mm3':touch})
    tm,ts=mesh_solid(m,d,m.geom('tab5_visual').id);tabb=bounds['tab5_collision'];tabuncovered=max(0.,float((ts-box(tabb)).volume()));tol=1e-5;tabexpanded=max(0.,float((ts-box(tabb+np.array([[-tol,tol]]*3))).volume()));tabexcess=float(max(0,np.max(tabb[:,0]-tm.vertices),np.max(tm.vertices-tabb[:,1])))
    # Rounded physical body contact with front rails, not merely its box.
    tabcontacts=[]
    for n,b in bounds.items():
        if n=='tab5_collision':continue
        touch=max(0.,float((ts^box(b+np.array([[-.001,.001]]*3))).volume()))
        if touch>1e-5:tabcontacts.append({'new_geom':n,'expanded_1micron_intersection_mm3':touch})
    return {'parts':parts,'frame_outer_bounds_xyz_mm':fb.tolist(),'frame_dimensions_depth_width_height_mm':(fb[:,1]-fb[:,0]).tolist(),'upper_assembly_bounds_including_mounts_xyz_mm':allb.tolist(),'upper_assembly_dimensions_depth_width_height_mm':(allb[:,1]-allb[:,0]).tolist(),'total_model_mass_kg':float(m.body_mass.sum()),'expected_model_mass_kg':built['model_mass_kg_expected'],'expected_mass_matches':bool(abs(m.body_mass.sum()-built['model_mass_kg_expected'])<1e-12),'added_box_connection_components':components,'face_contact_edges':edges,'added_box_solid_overlaps':overlaps,'fixed_original_CAD':fixed,'fixed_CAD_solid_intersections_over_0p001_mm3':intersections,'fixed_CAD_surface_contact_candidates':near_contact,'tab5_visual_watertight':bool(tm.is_watertight),'tab5_visual_outside_bounding_collider_mm3':tabuncovered,'tab5_max_compiled_vertex_bound_excess_mm':tabexcess,'tab5_coverage_linf_numerical_tolerance_mm':tol,'tab5_visual_outside_collider_after_numerical_tolerance_mm3':tabexpanded,'tab5_visual_to_frame_contact_candidates':tabcontacts,'notes':['Frame holes remain open because each visible structural member is itself the active box collider. No full-frame bounding collider is present.','Box connectivity is ideal surface contact. Mounting screws, holes, joints, wiring, strength and flex remain unmodeled.','Fixed-CAD booleans use transformed compiled original mesh vertices/faces, not convex hull approximations. Duplicate visual/collision copies are removed. Overlaps <=0.001 mm³ are treated as exported-coordinate noise.','A 0.001 mm expansion identifies intended near/surface contact; it does not enlarge actual geometry or establish manufacturable attachment.','Screen/eyes/camera overlay shapes are cosmetic zero-mass graphics. Tab5 physical-body coverage refers to its rounded outer-body mesh. Compiled float32 mesh-coordinate deviations are quantified; a 0.00001 mm analytical tolerance check does not alter the collider.']}

class DetailedAuditor(Auditor):
    def minimum_for_panel(self,p,states,transforms):
        w,eq,blo,bhi=self.stat[p];items=[]
        for i,meshes in enumerate(transforms):
            for g in self.legs:
                v,A,b,lo,hi=meshes[g];lower=float(np.linalg.norm(np.maximum(np.maximum(blo-hi,lo-bhi),0)));items.append((lower,i,g))
        best=np.inf;lower=np.inf;winner=None;failures=[];solved=0;seen={};maxgap=0.
        for bound,i,g in sorted(items):
            if bound>best+1e-7:lower=min(lower,bound);break
            v,A,b,lo,hi=transforms[i][g]
            # Exact-byte duplicate reuse only; no rounding or tolerance caching.
            key=(g,v.tobytes())
            if key in seen:r=seen[key]
            else:r=self.solve(p,g,transforms[i][g]);seen[key]=r;solved+=1
            lower=min(lower,r['lower_bound_mm']);maxgap=max(maxgap,r['duality_gap_mm'])
            if not r['optimizer_success'] or r['constraint_violation_mm']>1e-5:failures.append({'sample':i,'leg_geom':self.m.geom(g).name,**r})
            if r['feasible_upper_bound_mm']<best:best=r['feasible_upper_bound_mm'];winner={'sample_index':i,'leg_geom':self.m.geom(g).name,'leg_body':self.m.body(self.m.geom_bodyid[g]).name,'qpos':states[i].tolist(),**r}
        return {'geom':self.m.geom(p).name,'minimum_gap_lower_bound_mm':lower,'minimum_gap_upper_bound_mm':best,'winner':winner,'optimized_unique_pairs':solved,'max_pair_duality_gap_mm':maxgap,'optimizer_failures':failures}
    def detailed(self,label,states):
        transforms,hits=self.collect(states);rows=[self.minimum_for_panel(p,states,transforms) for p in self.panels]
        def group(rs):
            return {'minimum_gap_lower_bound_mm':min(r['minimum_gap_lower_bound_mm']for r in rs),'minimum_gap_upper_bound_mm':min(r['minimum_gap_upper_bound_mm']for r in rs),'winner_geom':min(rs,key=lambda r:r['minimum_gap_upper_bound_mm'])['geom']}
        shell=[r for r in rows if r['geom']!='tab5_collision'];frame=[r for r in rows if any(r['geom'].startswith('cube_portrait_'+s)for s in ['bottom_','top_','post_'])]
        result={'label':label,'sample_count':len(states),'new_physical_to_leg_contact_count':len(hits),'contact_examples':hits[:30],'all_new_parts':group(rows),'all_except_Tab5':group(shell),'frame_only':group(frame),'Tab5':group([r for r in rows if r['geom']=='tab5_collision']),'per_geom':rows}
        print(label,{k:result[k]for k in ['sample_count','new_physical_to_leg_contact_count','all_new_parts','frame_only','Tab5']},flush=True)
        return result

def baseline_checks(m,d,o):
    old=mujoco.MjData(o);base=json.loads((ROOT/'diagnostics/crouch_pose_report.json').read_text());out={}
    for key in ['STAND','SIT','FOLD']:
        q=m.key(key).qpos.copy();d.qpos[:]=q;old.qpos[:]=q;mujoco.mj_forward(m,d);mujoco.mj_forward(o,old);ground=[];maxdiff=0.
        for g in range(m.ngeom):
            if not m.geom(g).name.startswith('shell_guard_legmesh_'):continue
            v=vertices(m,d,g);tr=m.body('trunk_base').id;wv=v@d.xmat[tr].reshape(3,3).T+d.xpos[tr]*1000
            og=o.geom(m.geom(g).name).id;ov=vertices(o,old,og);otr=o.body('trunk_base').id;owv=ov@old.xmat[otr].reshape(3,3).T+old.xpos[otr]*1000
            maxdiff=max(maxdiff,float(np.max(np.abs(wv-owv))))
            if wv[:,2].min() < -1e-4:ground.append({'geom':m.geom(g).name,'minimum_world_z_mm':float(wv[:,2].min())})
        fold=[]
        for prior in base['samples'][key]['original_nonadjacent_proxy_overlaps']:
            gs=[m.geom(prior[a]['geom']).id for a in ['a','b']];eqs=[ConvexHull(vertices(m,d,g)).equations for g in gs];eq=np.concatenate(eqs);rr=linprog([0,0,0,-1],A_ub=np.c_[eq[:,:3],np.ones(len(eq))],b_ub=-eq[:,3],bounds=[(None,None)]*4,method='highs');assert rr.success
            fold.append({'a':prior['a']['geom'],'b':prior['b']['geom'],'shared_convex_interior_radius_mm':float(rr.x[3]),'prior_radius_mm':prior['shared_interior_radius_mm'],'unchanged_within_1e_minus_7_mm':bool(abs(rr.x[3]-prior['shared_interior_radius_mm'])<1e-7)})
        out[key]={'original_leg_vertices_max_difference_vs_rounded_mm':maxdiff,'ground_penetrations':ground,'reproduced_established_original_nonadjacent_convex_overlaps':fold}
    return {'checks':out,'source':'../diagnostics/crouch_pose_report.json','source_sha256':sha(ROOT/'diagnostics/crouch_pose_report.json'),'note':'SIT sole/foot penetration and FOLD nonadjacent original-leg convex-hull overlaps predate this frame and are reproduced without new geometry. Convex-hull overlaps are conservative flags, not proof of concave CAD solid collision. Saved kinematic SIT/FOLD paths are not certified physically achievable motions.'}

def main():
    m=mujoco.MjModel.from_xml_path(str(HERE/'scene_tab5_portrait.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d);o=mujoco.MjModel.from_xml_path(str(ROOT/'rounded/scene_tab5_lowcube_rounded.xml'))
    boxes=[g for g in range(m.ngeom)if m.geom(g).name.startswith('cube_portrait_')];physical=boxes+[m.geom('tab5_collision').id];legs=[g for g in range(m.ngeom)if m.geom(g).name.startswith('shell_guard_legmesh_')]
    pairs={tuple(sorted((int(m.pair_geom1[i]),int(m.pair_geom2[i]))))for i in range(m.npair)};missing=[(m.geom(p).name,m.geom(g).name)for p in physical for g in legs if tuple(sorted((p,g)))not in pairs]
    report={'scene':'scene_tab5_portrait.xml','model_sha256':sha(HERE/'tab5_portrait.xml'),'geometry_report_sha256':sha(HERE/'geometry_report.json'),'mujoco_version':mujoco.__version__,'scipy_version':scipy.__version__,'invariants':original_invariants(o,m),'new_physical_geom_count':len(physical),'leg_hull_count':len(legs),'missing_explicit_new_part_leg_pairs':missing,'geometry':geometry_check(m,d,physical),'baseline_pose_problems':baseline_checks(m,d,o),'reports':[],'scope':'Saved sampled states only. Compiled convex hulls of all 34 unchanged moving leg CAD meshes versus every active new box (frame, brackets, tray, compute, standoffs and portrait Tab5). Independent separating-plane support lower bounds plus feasible closest-point upper bounds; AABB pruning. No continuous collision, strength, wiring, manufacture, stability, controller or hardware guarantee.'}
    output=HERE/'validation_report.json';output.write_text(json.dumps(report,indent=2))
    print('Immutable original mechanism:',report['invariants']['all_exactly_equal'],'fixed CAD intersections:',report['geometry']['fixed_CAD_solid_intersections_over_0p001_mm3'],'connections:',len(report['geometry']['added_box_connection_components']),flush=True)
    if '--geometry-only' not in sys.argv:
        mujoco.mj_resetData(m,d);mujoco.mj_forward(m,d);auditor=DetailedAuditor(m,d,boxes);p=np.load(ROOT/'diagnostics/posture_sweeps.npz');states={k:p[k]for k in p.files};states['feet_flat_crouch_181']=np.load(ROOT/'diagnostics/feet_flat_crouch_paths.npz')['qpos']
        for name in ['walk_10s','idle_10s']:
            f=HERE/(name+'_states.npz')
            if f.exists():states[name]=np.load(f)['qpos'];report.setdefault('state_files',{})[f.name]={'sha256':sha(f),'sample_count':len(states[name])}
        for label,q in states.items():report['reports'].append(auditor.detailed(label,q));output.write_text(json.dumps(report,indent=2))
    report['total_sample_count']=sum(r['sample_count']for r in report['reports']);output.write_text(json.dumps(report,indent=2));print('Saved',output,flush=True)
if __name__=='__main__':main()
