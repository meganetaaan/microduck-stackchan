"""Independent exported-solid, unchanged-mechanism and sampled clearance audit.
No policy inference, networking, hardware or original-file mutations.
"""
from pathlib import Path
import sys, json, itertools, hashlib, time
sys.dont_write_bytecode=True
import numpy as np, trimesh, manifold3d as mf, mujoco, scipy
HERE=Path(__file__).resolve().parent;ROOT=HERE.parent
sys.path.insert(0,str(ROOT/'rounded'))
from validate_rounded import Auditor,vertices,invariants,sha
sys.path.insert(0,str(ROOT/'portrait'))
from validate_portrait import original_invariants,mesh_solid,baseline_checks,box

def load_solid(path,scale=1.):
    tm=trimesh.load(path,process=True);tm.vertices*=scale
    s=mf.Manifold(mf.Mesh64(np.ascontiguousarray(tm.vertices,dtype=np.float64),np.ascontiguousarray(tm.faces,dtype=np.uint64)))
    assert tm.is_watertight and tm.is_winding_consistent and s.status()==mf.Error.NoError,(path,tm.is_watertight,s.status())
    return tm,s

def geometry(m,d):
    built=json.loads((HERE/'geometry_report.json').read_text());p=built['parameters'];solids={};meshes={};parts=[]
    for part in built['parts']:
        name=part['name'];tm,s=load_solid(HERE/'assets'/f'{name}.obj',1000);solids[name]=s;meshes[name]=tm
        stl=None
        if part['printable']:
            sm,ss=load_solid(HERE/'stl'/f'{name}_mm.stl');stl={'units':'millimetres by 1:1 coordinate dimensions; STL has no embedded units','bounds_xyz_mm':sm.bounds.tolist(),'watertight':bool(sm.is_watertight),'winding_consistent':bool(sm.is_winding_consistent),'components':len(ss.decompose()),'volume_mm3':float(ss.volume()),'max_vertex_bounds_difference_from_metre_OBJ_mm':float(np.max(abs(sm.bounds-tm.bounds))),'volume_difference_from_metre_OBJ_mm3':abs(float(ss.volume()-s.volume()))}
        tm_m=tm.copy();tm_m.vertices*=.001;tm_m.density=p['plastic_density_kg_m3']if part['printable']else p['steel_density_kg_m3'];pr=tm_m.mass_properties;b=m.body('print_'+name).id
        R=np.empty(9);mujoco.mju_quat2Mat(R,m.body_iquat[b]);R=R.reshape(3,3);I=R@np.diag(m.body_inertia[b])@R.T
        parts.append({'part':name,'OBJ_watertight':bool(tm.is_watertight),'components':len(s.decompose()),'STL':stl,'density_kg_m3':tm_m.density,'independent_mass_kg':float(pr.mass),'compiled_mass_kg':float(m.body_mass[b]),'mass_abs_error_kg':abs(float(pr.mass-m.body_mass[b])),'COM_max_abs_error_m':float(np.max(abs(pr.center_mass-m.body_ipos[b]))),'full_inertia_max_abs_error_kg_m2':float(np.max(abs(pr.inertia-I))),'positive_inertia':bool(np.min(m.body_inertia[b])>0),'visual_file_sha256':sha(HERE/'assets'/f'{name}.obj')})
    tm,tab=mesh_solid(m,d,m.geom('tab5_visual').id);solids['Tab5']=tab;meshes['Tab5']=tm
    b=np.array(p['compute_bounds_xyz_mm']);solids['compute']=box(b);meshes['compute']=None
    fixed=[];seen=set();clashes=[];touches=[]
    for g in range(m.ngeom):
        if m.geom_bodyid[g]!=m.body('trunk_base').id:continue
        key=(int(m.geom_dataid[g]),d.geom_xpos[g].tobytes(),d.geom_xmat[g].tobytes())
        if key in seen:continue
        seen.add(key);tm,s=mesh_solid(m,d,g);name=m.mesh(m.geom_dataid[g]).name
        fixed.append({'mesh':name,'geom_id':g,'volume_mm3':float(s.volume()),'bounds_xyz_mm':tm.bounds.tolist(),'watertight':bool(tm.is_watertight)})
        for n,x in solids.items():
            inter=max(0,float((x^s).volume()))
            if inter>.001:clashes.append({'new_part':n,'fixed_mesh':name,'fixed_geom':g,'intersection_mm3':inter})
    print('FIXED CAD CLASHES',json.dumps(clashes),flush=True)
    pair_overlaps=[];contacts=[];tiny=mf.Manifold.cube([.002]*3,center=True)
    for a,b in itertools.combinations(solids,2):
        va=np.array(solids[a].bounding_box()).reshape(2,3);vb=np.array(solids[b].bounding_box()).reshape(2,3)
        if np.any(va[0]>vb[1]+.002)or np.any(vb[0]>va[1]+.002):continue
        inter=max(0,float((solids[a]^solids[b]).volume()))
        if inter>.001:pair_overlaps.append({'a':a,'b':b,'intersection_mm3':inter})
        else:
            # <=1 micron of analytical inflation finds face contact only.
            touch=max(0,float((solids[a].minkowski_sum(tiny)^solids[b]).volume()))
            if touch>.00001:contacts.append({'a':a,'b':b,'expanded_1micron_intersection_mm3':touch})
    print('ASSEMBLY SOLID OVERLAPS',json.dumps(pair_overlaps),flush=True)
    result={'parts':parts,'fixed_original_CAD':fixed,'fixed_CAD_solid_intersections_over_0p001_mm3':clashes,'new_assembly_solid_intersections_over_0p001_mm3':pair_overlaps,'nominal_face_contact_candidates':contacts,'total_model_mass_kg':float(m.body_mass.sum()),'printed_mass_kg':sum(x['independent_mass_kg']for x,y in zip(parts,built['parts'])if y['printable']),'methods':'Independent reread of exported OBJ/STL and exact triangle-surface Manifold booleans. Physical printed-solid inertia uses stated uniform material density; compiled principal inertia is rotated back into body coordinates. STL files have no native unit metadata, coordinates checked against metre OBJ and dimension parameters. Face contact candidates use analytical 1 micron expansion and are not attachment/strength validation.'}
    expected_stls={x['name']+'_mm.stl'for x in built['parts']if x['printable']};actual_stls={x.name for x in (HERE/'stl').glob('*.stl')};result['STL_inventory']={'expected':sorted(expected_stls),'found':sorted(actual_stls),'missing':sorted(expected_stls-actual_stls),'unexpected':sorted(actual_stls-expected_stls)}
    result['approximate_non_solid_mass_allowance']={'mass_kg':p['mount_fastener_allowance_kg'],'body':'mount_fastener_allowance','status':'Approximate unspecified mounting fastener mass and box inertia; no physical shape or collider is certified for this allowance.'}
    result['all_printed_parts_are_single_watertight_solids']=all(x['STL']['watertight']and x['STL']['winding_consistent']and x['STL']['components']==1 for x in parts if x['STL'])
    result['all_density_mass_and_inertias_agree_with_export_precision']=all(x['mass_abs_error_kg']<1e-8 and x['full_inertia_max_abs_error_kg_m2']<1e-10 and x['COM_max_abs_error_m']<1e-7 for x in parts)
    (HERE/'geometry_validation_report.json').write_text(json.dumps(result,indent=2))
    return result,solids

def coverage(solids):
    built=json.loads((HERE/'geometry_report.json').read_text());out=[];tol=.00001;tiny=mf.Manifold.cube([2*tol]*3,center=True)
    for part in built['parts']:
        name=part['name'];hulls=[];bad=[]
        for row in built['collision_parts']:
            if row['part']!=name:continue
            tm,s=load_solid(HERE/row['file'],1000);hulls.append(s)
            if abs(s.hull().volume()-s.volume())>1e-3:bad.append(row['name'])
        union=mf.Manifold.batch_boolean(hulls,mf.OpType.Add);unc=max(0,float((solids[name]-union).volume()));extra=max(0,float((union-solids[name]).volume()))
        expanded=mf.Manifold.batch_boolean([x.minkowski_sum(tiny)for x in hulls],mf.OpType.Add);unc_tol=max(0,float((solids[name]-expanded).volume()))
        row={'part':name,'convex_piece_count':len(hulls),'nonconvex_exported_pieces':bad,'solid_volume_mm3':float(solids[name].volume()),'collider_union_volume_mm3':float(union.volume()),'uncovered_volume_mm3':unc,'uncovered_after_0p00001mm_linf_tolerance_mm3':unc_tol,'conservative_extra_volume_mm3':extra,'extra_volume_ratio':extra/solids[name].volume()};out.append(row);print('COVERAGE',name,row,flush=True)
    return {'parts':out,'coordinate_tolerance_mm':tol,'method':'Independent reread and union of exported hull OBJ files, subtraction from exported physical solids. Local hulls are conservative; tolerance diagnoses independently rounded export-coordinate seams and does not alter the scene.'}

class GroupedAuditor(Auditor):
    def audit_groups(self,label,states):
        transforms,hits=self.collect(states);m=self.m;keys=['mounting','body','Tab5'];blo=np.array([self.stat[p][2]for p in self.panels]);bhi=np.array([self.stat[p][3]for p in self.panels]);lookup={p:('Tab5'if m.geom(p).name=='tab5_collision'else'mounting'if m.body(m.geom_bodyid[p]).name in ['print_central_mount','print_under_saddle','print_rear_hardware']else'body')for p in self.panels};indices={k:np.array([j for j,p in enumerate(self.panels)if lookup[p]==k])for k in keys}
        def bounds(meshes):
            lo=np.array([meshes[g][3]for g in self.legs]);hi=np.array([meshes[g][4]for g in self.legs]);return np.linalg.norm(np.maximum(np.maximum(blo[:,None,:]-hi[None,:,:],lo[None,:,:]-bhi[:,None,:]),0),axis=2)
        # Obtain an actual feasible upper bound for each group first. This permits
        # global AABB pruning without an arbitrary search-distance cutoff.
        seeds={k:(np.inf,None,None,None)for k in keys}
        for index,meshes in enumerate(transforms):
            lower=bounds(meshes)
            for k,ix in indices.items():
                a,b=np.unravel_index(np.argmin(lower[ix]),lower[ix].shape);panel=self.panels[ix[a]];value=lower[ix[a],b]
                if value<seeds[k][0]:seeds[k]=(float(value),index,panel,self.legs[b])
        initial={};candidates={k:[]for k in keys};outside={k:np.inf for k in keys}
        for k,(_,index,panel,leg)in seeds.items():initial[k]=self.solve(panel,leg,transforms[index][leg])
        for index,meshes in enumerate(transforms):
            lower=bounds(meshes)
            for k,ix in indices.items():
                cutoff=initial[k]['feasible_upper_bound_mm']+1e-7;sub=lower[ix];ps,ls=np.where(sub<=cutoff);far=sub[sub>cutoff]
                if len(far):outside[k]=min(outside[k],float(far.min()))
                for pp,ll in zip(ps,ls):candidates[k].append((float(sub[pp,ll]),index,self.panels[ix[pp]],self.legs[ll]))
        report={'label':label,'sample_count':len(states),'new_parts_to_moving_leg_contact_count':len(hits),'contact_examples':hits[:30],'groups':{}}
        for group,items in candidates.items():
            best=np.inf;lower=outside[group];solved=0;reused=0;failures=[];winner=None;maxgap=0.;seen={};nextbound=outside[group]
            for bound,index,panel,leg in sorted(items):
                if bound>=best-1e-6:nextbound=bound;break
                key=(panel,leg,transforms[index][leg][0].tobytes())
                if key in seen:r=seen[key];reused+=1
                else:r=self.solve(panel,leg,transforms[index][leg]);seen[key]=r;solved+=1
                lower=min(lower,r['lower_bound_mm']);maxgap=max(maxgap,r['duality_gap_mm'])
                if not r['optimizer_success']or r['constraint_violation_mm']>1e-5:failures.append({'sample':int(index),'panel':m.geom(panel).name,'leg':m.geom(leg).name,**r})
                if r['feasible_upper_bound_mm']<best:best=r['feasible_upper_bound_mm'];winner={'sample_index':int(index),'panel':m.geom(panel).name,'part_body':m.body(m.geom_bodyid[panel]).name,'leg_geom':m.geom(leg).name,'leg_body':m.body(m.geom_bodyid[leg]).name,'qpos':states[index].tolist(),**r}
            lower=min(lower,nextbound);row={'minimum_gap_lower_bound_mm':lower,'minimum_gap_upper_bound_mm':best,'winner':winner,'AABB_candidate_pairs':len(items),'optimized_unique_pairs':solved,'exact_byte_cached_pairs':reused,'max_pair_duality_gap_mm':maxgap,'optimizer_failures':failures};report['groups'][group]=row
            print('CLEARANCE',label,group,lower,best,'optimized',solved,'failures',len(failures),flush=True)
        return report

def main():
    start=time.monotonic();m=mujoco.MjModel.from_xml_path(str(HERE/'scene_tab5_portrait_print.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d);o=mujoco.MjModel.from_xml_path(str(ROOT/'portrait/scene_tab5_portrait.xml'))
    panels=[g for g in range(m.ngeom)if m.geom(g).name.startswith('rounded_shell_collision_')or m.geom(g).name=='cube_portrait_compute_collision'];legs=[g for g in range(m.ngeom)if m.geom(g).name.startswith('shell_guard_legmesh_')]
    pairs={tuple(sorted((int(m.pair_geom1[i]),int(m.pair_geom2[i]))))for i in range(m.npair)};allp=panels+[m.geom('tab5_collision').id];missing=[(m.geom(p).name,m.geom(g).name)for p in allp for g in legs if tuple(sorted((p,g)))not in pairs]
    inv=original_invariants(o,m);full=invariants(o,m);inv['Tab5_original_exactly_equal']=all(all(r['checks'].values())for r in full['records']if r.get('name','').startswith('tab5'));inv['intentional_exclusions']='Only former portrait-frame parts replaced; original mechanism and Tab5 are unchanged.'
    report={'scene':'scene_tab5_portrait_print.xml','model_sha256':sha(HERE/'tab5_portrait_print.xml'),'geometry_report_sha256':sha(HERE/'geometry_report.json'),'mujoco_version':mujoco.__version__,'scipy_version':scipy.__version__,'invariants':inv,'physical_collider_count_including_Tab5':len(allp),'leg_hull_count':len(legs),'missing_explicit_physical_leg_pairs':missing,'reports':[],'scope':'Sampled-state numerical clearance to all 34 unchanged moving leg CAD convex hulls, every printed-solid conservative cell collider, provisional hardware, compute and unchanged Tab5. AABB pruning plus independent support lower bounds and feasible-distance upper bounds. No continuous-path, balance/controller, strength, wiring, printer tolerances or hardware safety certification.'}
    path=HERE/'validation_report.json'
    if '--samples-only' in sys.argv:
        report=json.loads(path.read_text());assert report['model_sha256']==sha(HERE/'tab5_portrait_print.xml'),'Geometry report is stale';assert report['geometry_report_sha256']==sha(HERE/'geometry_report.json');
        if '--resume' not in sys.argv:report['reports']=[];report.pop('state_files',None)
    else:
        geo,solids=geometry(m,d);report['geometry']=geo;path.write_text(json.dumps(report,indent=2));print('INVARIANTS',inv['all_exactly_equal'],inv['Tab5_original_exactly_equal'],'missing pairs',len(missing),flush=True)
        if '--quick' in sys.argv:return
        report['independent_exported_collision_coverage']=coverage(solids);path.write_text(json.dumps(report,indent=2))
        report['baseline_pose_problems']=baseline_checks(m,d,o);path.write_text(json.dumps(report,indent=2))
    if '--geometry-only' not in sys.argv:
        mujoco.mj_resetData(m,d);mujoco.mj_forward(m,d);auditor=GroupedAuditor(m,d,panels);states={}
        if '--traces-only' not in sys.argv:
            p=np.load(ROOT/'diagnostics/posture_sweeps.npz');states.update({k:p[k]for k in p.files});states['feet_flat_crouch_181']=np.load(ROOT/'diagnostics/feet_flat_crouch_paths.npz')['qpos']
        for name in ['walk_10s','idle_10s']:
            f=HERE/(name+'_states.npz')
            if f.exists():states[name]=np.load(f)['qpos'];report.setdefault('state_files',{})[f.name]={'sha256':sha(f),'sample_count':len(states[name])}
        for label,q in states.items():
            if '--resume' in sys.argv and any(x['label']==label for x in report['reports']):continue
            report['reports'].append(auditor.audit_groups(label,q));path.write_text(json.dumps(report,indent=2))
    report['total_sample_count']=sum(x['sample_count']for x in report['reports']);report['elapsed_seconds']=time.monotonic()-start;path.write_text(json.dumps(report,indent=2));print('SAVED',path,'seconds',report['elapsed_seconds'],flush=True)
if __name__=='__main__':main()
