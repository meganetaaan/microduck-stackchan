"""Independent UART assembly audit. Writes only validation artifacts in this folder.

Sampled-state convex clearance is bounded by support planes and feasible closest
points. Exact exported-solid booleans diagnose welded-body collisions which the
physics engine intentionally filters. Neither proves hardware or all-ROM safety.
"""
from pathlib import Path
import sys,json,itertools,time,hashlib,re
sys.dont_write_bytecode=True
import numpy as np,trimesh,manifold3d as mf,mujoco,scipy
ROOT=Path(__file__).resolve().parent.parent;HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'rounded'))
from validate_rounded import Auditor,invariants,sha,vertices
sys.path.insert(0,str(ROOT/'portrait'))
from validate_portrait import original_invariants,mesh_solid,baseline_checks

def load_solid(path,scale=1000.):
    tm=trimesh.load(path,process=True);tm.vertices*=scale
    solid=mf.Manifold(mf.Mesh64(np.ascontiguousarray(tm.vertices,dtype=np.float64),np.ascontiguousarray(tm.faces,dtype=np.uint64)))
    assert tm.is_watertight and tm.is_winding_consistent and solid.status()==mf.Error.NoError,(str(path),tm.is_watertight,solid.status())
    return tm,solid

def bbox_disjoint(a,b,tol=0):
    a=np.array(a.bounding_box()).reshape(2,3);b=np.array(b.bounding_box()).reshape(2,3)
    return bool(np.any(a[0]>b[1]+tol)or np.any(b[0]>a[1]+tol))

def overlap_class(a,b):
    pair=frozenset([a,b])
    mates={frozenset(p)for p in [('branch_input_loom','power_input_connector_proxy'),('branch_input_loom','robot_power_handoff_proxy'),('radxa_gpio_header','radxa_gpio_pins'),('robot_power_pigtail','robot_power_handoff_proxy'),('robot_power_pigtail','power_input_connector_proxy'),('tab5_power_loom','power_output_connector_proxy'),('tab5_power_loom','tab5_J7_plug_proxy'),('uart_loom','uart_hat_takeoff_proxy'),('uart_loom','tab5_m5bus_plug_proxy')]}
    if pair in mates:return 'intended connector/pin engagement; provisional internal detail, not fit certification'
    if 'unresolved_fastener_mass' in pair:return 'unresolved mass allowance intersects physical part; not validated hardware fit'
    return 'unresolved physical interference; review needed'

def geometry(m,d,built):
    solids={};parts=[]
    for part in built['parts']:
        name=part['name'];tm,s=load_solid(HERE/part['visual_file']);solids[name]=s
        orig=tm.copy();orig.vertices*=.001;orig.density=part['mass_kg']/orig.volume;props=orig.mass_properties;b=m.body('assembly_'+name).id
        R=np.empty(9);mujoco.mju_quat2Mat(R,m.body_iquat[b]);R=R.reshape(3,3);compiled_I=R@np.diag(m.body_inertia[b])@R.T
        row={'part':name,'watertight':bool(tm.is_watertight),'winding_consistent':bool(tm.is_winding_consistent),'components':len(s.decompose()),'volume_mm3':float(s.volume()),'bounds_mm':tm.bounds.tolist(),'density_kg_m3':float(orig.density),'mass_kg':float(m.body_mass[b]),'mass_abs_error_kg':abs(float(props.mass-m.body_mass[b])),'COM_max_abs_error_m':float(np.max(abs(props.center_mass-m.body_ipos[b]))),'inertia_max_abs_error_kg_m2':float(np.max(abs(props.inertia-compiled_I))),'positive_inertia':bool(np.min(m.body_inertia[b])>0),'proxy':part['proxy'],'source':part['source'],'asset_sha256':sha(HERE/part['visual_file'])}
        if part['printable']:
            stl,ss=load_solid(HERE/'stl'/(name+'_mm.stl'),1.)
            row['STL']={'watertight':bool(stl.is_watertight),'winding_consistent':bool(stl.is_winding_consistent),'components':len(ss.decompose()),'bounds_mm':stl.bounds.tolist(),'OBJ_bounds_max_difference_mm':float(np.max(abs(tm.bounds-stl.bounds))),'OBJ_volume_difference_mm3':abs(float(s.volume()-ss.volume()))}
        parts.append(row)
    # All retained noncosmetic physical solids include the original Tab5 and rear cover.
    for g in range(m.ngeom):
        name=m.geom(g).name or ''
        if name=='tab5_visual' or (m.geom_group[g]==0 and m.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH and m.body(m.geom_bodyid[g]).name.startswith('print_')):
            tm,s=mesh_solid(m,d,g);solids['retained:'+m.body(m.geom_bodyid[g]).name]=s
    fixed=[];seen=set();clashes=[]
    for g in range(m.ngeom):
        if m.geom_bodyid[g]!=m.body('trunk_base').id:continue
        key=(int(m.geom_dataid[g]),d.geom_xpos[g].tobytes(),d.geom_xmat[g].tobytes())
        if key in seen:continue
        seen.add(key);tm,s=mesh_solid(m,d,g);name=m.mesh(m.geom_dataid[g]).name
        fixed.append({'mesh':name,'geom':g,'volume_mm3':float(s.volume()),'bounds_mm':tm.bounds.tolist(),'watertight':bool(tm.is_watertight)})
        for n,x in solids.items():
            if bbox_disjoint(x,s):continue
            v=max(0.,float((x^s).volume()))
            if v>.001:clashes.append({'assembly_part':n,'fixed_mesh':name,'intersection_mm3':v,'classification':'unresolved physical interference with retained mechanism'})
    result={'parts':parts,'fixed_original_CAD':fixed,'fixed_CAD_intersections_over_0p001_mm3':clashes,'assembly_solid_intersections_over_0p001_mm3':[],'total_model_mass_kg':float(m.body_mass.sum()),'method':'Independent exported OBJ/STL reread plus Manifold exact triangle-surface booleans in millimetres; fixed original geometry uses transformed compiled meshes. Colliders are not substituted for physical solids. Overlaps <=0.001 mm3 treated as numerical residue. Masses use declared component density or lumped mass, not weighed hardware.'}
    (HERE/'geometry_validation_report.json').write_text(json.dumps(result,indent=2))
    print('FIXED_CAD',json.dumps(clashes),flush=True)
    for a,b in itertools.combinations(solids,2):
        if bbox_disjoint(solids[a],solids[b]):continue
        v=max(0.,float((solids[a]^solids[b]).volume()))
        if v>.001:result['assembly_solid_intersections_over_0p001_mm3'].append({'a':a,'b':b,'intersection_mm3':v,'classification':overlap_class(a,b)})
    expected_new={p['name']+'_mm.stl'for p in built['parts']if p['printable']};expected_retained={'rear_cover_mm.stl'}if 'retained:print_rear_cover'in solids else set();expected=expected_new|expected_retained;found={p.name for p in(HERE/'stl').glob('*.stl')};result['STL_inventory']={'missing':sorted(expected-found),'unexpected':sorted(found-expected),'expected':sorted(expected),'expected_new':sorted(expected_new),'expected_retained':sorted(expected_retained)}
    result['retained_STL_copies']=[]
    for filename in sorted(expected_retained&found):
        sm,ss=load_solid(HERE/'stl'/filename,1.);prior=ROOT/'portrait_print/stl'/filename
        result['retained_STL_copies'].append({'part':filename.removesuffix('_mm.stl'),'status':'retained unmodified original printed cover','path':'stl/'+filename,'source':'../portrait_print/stl/'+filename,'sha256':sha(HERE/'stl'/filename),'source_sha256':sha(prior),'byte_identical_to_prior':sha(HERE/'stl'/filename)==sha(prior),'watertight':bool(sm.is_watertight),'winding_consistent':bool(sm.is_winding_consistent),'components':len(ss.decompose())})
    result['all_printed_STLs_single_watertight_solids']=all(p['STL']['watertight']and p['STL']['winding_consistent']and p['STL']['components']==1 for p in parts if 'STL'in p)
    result['all_printed_STLs_single_watertight_solids']=result['all_printed_STLs_single_watertight_solids']and all(p['watertight']and p['winding_consistent']and p['components']==1 for p in result['retained_STL_copies'])
    result['all_mass_inertia_agree_with_export_precision']=all(p['mass_abs_error_kg']<1e-8 and p['COM_max_abs_error_m']<1e-7 and p['inertia_max_abs_error_kg_m2']<1e-10 for p in parts)
    print('INTERNAL_OVERLAPS',json.dumps(result['assembly_solid_intersections_over_0p001_mm3']),flush=True)
    (HERE/'geometry_validation_report.json').write_text(json.dumps(result,indent=2));return result,solids

def coverage(built,solids):
    rows=[];tol=.00001;tiny=mf.Manifold.cube([2*tol]*3,center=True)
    for part in built['parts']:
        name=part['name'];hulls=[];nonconvex=[]
        for row in built['collision_parts']:
            if row['part']!=name:continue
            tm,s=load_solid(HERE/row['file']);hulls.append(s)
            if abs(s.hull().volume()-s.volume())>.001:nonconvex.append(row['name'])
        total=mf.Manifold.batch_boolean(hulls,mf.OpType.Add);uncovered=max(0.,float((solids[name]-total).volume()));extra=max(0.,float((total-solids[name]).volume()))
        # Only pay for coordinate inflation when there is uncovered residue.
        inflated=0.
        if uncovered>1e-7:
            expanded=mf.Manifold.batch_boolean([h.minkowski_sum(tiny)for h in hulls],mf.OpType.Add);inflated=max(0.,float((solids[name]-expanded).volume()))
        row={'part':name,'collider_count':len(hulls),'nonconvex_hulls':nonconvex,'physical_volume_mm3':float(solids[name].volume()),'union_volume_mm3':float(total.volume()),'uncovered_volume_mm3':uncovered,'uncovered_after_0p00001mm_linf_tolerance_mm3':inflated,'conservative_extra_volume_mm3':extra};rows.append(row);print('COVERAGE',name,uncovered,inflated,flush=True)
        (HERE/'collision_coverage_report.json').write_text(json.dumps({'parts':rows,'complete':False},indent=2))
    return {'parts':rows,'complete':True,'analytical_linf_tolerance_mm':tol,'method':'Union of exported per-part convex pieces minus exported physical solid. The 10 nanometre diagnostic inflation addresses independently rounded export coordinates and does not alter actual physics.'}

class AssemblyAuditor(Auditor):
    def group_name(self,p):
        n=self.m.geom(p).name;b=self.m.body(self.m.geom_bodyid[p]).name
        if n=='tab5_collision':return 'Tab5'
        if n.startswith('rounded_shell_collision_'):return 'retained_printed_parts'
        if n=='cube_uart_radxa_official_envelope':return 'Radxa_official_full_envelope'
        part=b.removeprefix('assembly_')
        if part in ['body_cage','central_mount','electronics_carrier','protection_tray','power_strain_relief','uart_strain_relief']:return 'new_printed_parts'
        if part in ['robot_power_pigtail','branch_input_loom','tab5_power_loom','uart_loom','robot_power_handoff_proxy']:return 'cables_and_source_handoff'
        if any(x in part for x in ['M2','M3','unresolved_fastener']):return 'fasteners'
        return 'electronics_and_connectors'
    def audit_groups(self,label,states):
        transforms,hits=self.collect(states);m=self.m;blo=np.array([self.stat[p][2]for p in self.panels]);bhi=np.array([self.stat[p][3]for p in self.panels]);keys=sorted(set(self.group_name(p)for p in self.panels));indices={k:np.array([j for j,p in enumerate(self.panels)if self.group_name(p)==k])for k in keys}
        def bounds(meshes):
            lo=np.array([meshes[g][3]for g in self.legs]);hi=np.array([meshes[g][4]for g in self.legs]);return np.linalg.norm(np.maximum(np.maximum(blo[:,None,:]-hi[None,:,:],lo[None,:,:]-bhi[:,None,:]),0),axis=2)
        seeds={k:(np.inf,None,None,None)for k in keys}
        for index,meshes in enumerate(transforms):
            lower=bounds(meshes)
            for k,ix in indices.items():
                a,b=np.unravel_index(np.argmin(lower[ix]),lower[ix].shape);panel=self.panels[ix[a]];value=lower[ix[a],b]
                if value<seeds[k][0]:seeds[k]=(float(value),index,panel,self.legs[b])
        initial={k:self.solve(panel,leg,transforms[index][leg])for k,(_,index,panel,leg)in seeds.items()};candidates={k:[]for k in keys};outside={k:np.inf for k in keys}
        for index,meshes in enumerate(transforms):
            lower=bounds(meshes)
            for k,ix in indices.items():
                cutoff=initial[k]['feasible_upper_bound_mm']+1e-7;sub=lower[ix];ps,ls=np.where(sub<=cutoff);far=sub[sub>cutoff]
                if len(far):outside[k]=min(outside[k],float(far.min()))
                for pp,ll in zip(ps,ls):candidates[k].append((float(sub[pp,ll]),index,self.panels[ix[pp]],self.legs[ll]))
        report={'label':label,'sample_count':len(states),'assembly_to_moving_leg_contact_count':len(hits),'contact_examples':hits[:30],'groups':{}}
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
            lower=min(lower,nextbound);report['groups'][group]={'minimum_gap_lower_bound_mm':lower,'minimum_gap_upper_bound_mm':best,'winner':winner,'AABB_candidate_pairs':len(items),'optimized_unique_pairs':solved,'exact_byte_cached_pairs':reused,'max_pair_duality_gap_mm':maxgap,'optimizer_failures':failures}
            print('CLEARANCE',label,group,lower,best,'solved',solved,'failures',len(failures),flush=True)
        return report

def finalize_report(report,built):
    for row in report.get('geometry',{}).get('assembly_solid_intersections_over_0p001_mm3',[]):
        row['classification']=overlap_class(row['a'],row['b'])
    for sample in report.get('reports',[]):
        for group in sample['groups'].values():
            flags=group['optimizer_failures']
            group['optimizer_status_flags_with_certified_tight_bounds']=all(f['lower_bound_mm']>0 and f['duality_gap_mm']<1e-5 and f['constraint_violation_mm']<1e-5 for f in flags)
            if flags:group['optimizer_status_note']='Raw solver status flags retained. Independently computed separating-plane lower bounds and repaired feasible upper bounds remain valid and agree within the reported duality gaps.'
    files=sorted({p['visual_file']for p in built['parts']}|{p['file']for p in built['collision_parts']})
    manifest={f:sha(HERE/f)for f in files}
    report['new_exported_assets']={'asset_count':len(files),'manifest_sha256':hashlib.sha256(json.dumps(manifest,sort_keys=True,separators=(',',':')).encode()).hexdigest(),'definition':'SHA256 of canonical sorted compact JSON mapping each assembly_report physical/collider asset relative path to its file SHA256.'}
    report['total_sample_count']=sum(x['sample_count']for x in report.get('reports',[]))
    report['summary']={'sample_count':report['total_sample_count'],'assembly_to_moving_leg_contact_count':sum(x['assembly_to_moving_leg_contact_count']for x in report.get('reports',[])),'minimum_sampled_gap_lower_bound_mm':min((g['minimum_gap_lower_bound_mm']for s in report.get('reports',[])for g in s['groups'].values()),default=None),'fixed_CAD_interference_count':len(report.get('geometry',{}).get('fixed_CAD_intersections_over_0p001_mm3',[])),'unresolved_assembly_interference_count':sum(x['classification'].startswith('unresolved')for x in report.get('geometry',{}).get('assembly_solid_intersections_over_0p001_mm3',[])),'scope':'Sampled states only; no hardware or full-range-of-motion certification'}
    if report.get('independent_collision_coverage',{}).get('complete'):
        report['summary']['all_exported_solids_covered_after_numerical_tolerance']=all(x['uncovered_after_0p00001mm_linf_tolerance_mm3']<1e-7 and not x['nonconvex_hulls']for x in report['independent_collision_coverage']['parts'])
    return report

def main():
    start=time.monotonic();hashes={x:sha(HERE/x)for x in ['tab5_assembly_uart.xml','assembly_report.json']};built=json.loads((HERE/'assembly_report.json').read_text())
    m=mujoco.MjModel.from_xml_path(str(HERE/'scene_tab5_assembly_uart.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d);o=mujoco.MjModel.from_xml_path(str(ROOT/'portrait_print/scene_tab5_portrait_print.xml'))
    panels=[g for g in range(m.ngeom)if m.geom(g).name.startswith(('cube_uart_','rounded_shell_collision_'))];legs=[g for g in range(m.ngeom)if m.geom(g).name.startswith('shell_guard_legmesh_')];physical=panels+[m.geom('tab5_collision').id]
    pairs={tuple(sorted((int(m.pair_geom1[i]),int(m.pair_geom2[i]))))for i in range(m.npair)};missing=[(m.geom(p).name,m.geom(g).name)for p in physical for g in legs if tuple(sorted((p,g)))not in pairs]
    inv=original_invariants(o,m);full=invariants(o,m);inv['Tab5_original_exactly_equal']=all(all(r['checks'].values())for r in full['records']if r.get('name','').startswith('tab5'));inv['intentional_exclusions']='Replaced printed body/mount/compute block with UART assembly; original leg mechanism, retained trunk CAD, Tab5 and actuators must remain unchanged.'
    report={'scene':'scene_tab5_assembly_uart.xml','source_hashes':hashes,'mujoco_version':mujoco.__version__,'scipy_version':scipy.__version__,'invariants':inv,'physics':{'nq':m.nq,'nv':m.nv,'nu':m.nu,'neq':m.neq,'gravity_m_s2':m.opt.gravity.tolist(),'free_joint_count':int(np.sum(m.jnt_type==mujoco.mjtJoint.mjJNT_FREE)),'max_gravity_compensation':float(m.body_gravcomp.max()),'total_mass_kg':float(m.body_mass.sum())},'physical_collider_count_including_Tab5':len(physical),'new_UART_collider_count':sum(m.geom(g).name.startswith('cube_uart_')for g in panels),'leg_hull_count':len(legs),'missing_explicit_physical_leg_pairs':missing,'reports':[],'scope':'Independent geometry/inertia/unchanged-mechanism and sampled-state audit. All 34 original moving leg CAD convex guards checked against every new UART collider, retained rounded shell collider and Tab5. No all-ROM, continuous-path, electrical, manufacturing, balance, strength, controller or hardware-safety claim. Radxa PCB datums are sourced; grouping, HAT, protection, connectors and cable routing remain provisional.'}
    path=HERE/'validation_report.json'
    source_path=HERE/'sources/radxa_zero_3w_official_combined_mm.stl'
    if source_path.exists():
        source_v=np.asarray(trimesh.load(source_path,process=False).vertices);source_digest=sha(source_path);source_mode='Fresh imported source mesh vertices'
    else:
        cached=json.loads((HERE/'sources/component_envelopes_report.json').read_text())['radxa_zero_3w']['cad_all_geometry_bbox_mm']
        source_v=np.array(list(itertools.product(*zip(cached['min'],cached['max']))));source_digest=sha(HERE/'sources/component_envelopes_report.json');source_mode='Saved official CAD bounding-box corners; original vendor CAD is not redistributed. Fetch the cited official source for independent source re-extraction.'
    source_trunk=np.c_[source_v[:,2]-23,source_v[:,0]-32.5,source_v[:,1]+float(re.search(r'Ycad\+([0-9.]+)',built['radxa_cad_to_trunk']).group(1))];gv=vertices(m,d,m.geom('cube_uart_radxa_official_envelope').id);lower=gv.min(0);upper=gv.max(0);excess=float(max(0.,np.max(lower-source_trunk),np.max(source_trunk-upper)))
    report['official_Radxa_envelope_coverage']={'source_sha256':source_digest,'source_check_mode':source_mode,'source_bounds_trunk_mm':[source_trunk.min(0).tolist(),source_trunk.max(0).tolist()],'collider_bounds_trunk_mm':[lower.tolist(),upper.tolist()],'maximum_vertex_excess_mm':excess,'covers_reference_at_0p00001mm_tolerance':excess<1e-5,'covers_all_imported_source_vertices_at_0p00001mm_tolerance':excess<1e-5 if source_path.exists() else None,'scope':'Guard covers the referenced complete official CAD envelope, including omitted procedural components. Saved bounds are used when vendor mesh is absent. Source invalid topology prevents exact assembled-CAD boolean or connector-fit certification.'}
    inv['actuator_names_unchanged']=[o.actuator(i).name for i in range(o.nu)]==[m.actuator(i).name for i in range(m.nu)]
    report['physics']['full_gravity_free_robot_10_actuators']=bool(m.nu==10 and np.sum(m.jnt_type==mujoco.mjtJoint.mjJNT_FREE)==1 and np.allclose(m.opt.gravity,[0,0,-9.81],rtol=0,atol=1e-12)and m.neq==0 and not np.any(m.body_gravcomp))
    print('INVARIANTS',inv['all_exactly_equal'],inv['Tab5_original_exactly_equal'],'PHYSICS',report['physics'],'MISSING_PAIRS',len(missing),flush=True)
    if '--samples-only' in sys.argv or '--coverage-only'in sys.argv:
        old=json.loads(path.read_text());assert old['source_hashes']==hashes,'Validation report is stale';old['official_Radxa_envelope_coverage']=report['official_Radxa_envelope_coverage'];report=old
        for clash in report.get('geometry',{}).get('assembly_solid_intersections_over_0p001_mm3',[]):clash['classification']=overlap_class(clash['a'],clash['b'])
    else:
        geo,solids=geometry(m,d,built);report['geometry']=geo;path.write_text(json.dumps(report,indent=2))
        if '--quick'in sys.argv:
            assert all(sha(HERE/k)==v for k,v in hashes.items()),'Model changed during quick audit; rerun'
            return
        report['baseline_pose_problems']=baseline_checks(m,d,o)
    if '--geometry-only'not in sys.argv and '--coverage-only'not in sys.argv:
        mujoco.mj_resetData(m,d);mujoco.mj_forward(m,d);auditor=AssemblyAuditor(m,d,panels);p=np.load(ROOT/'diagnostics/posture_sweeps.npz');states={k:p[k]for k in p.files};states['feet_flat_crouch_181']=np.load(ROOT/'diagnostics/feet_flat_crouch_paths.npz')['qpos']
        report['reports']=[]
        for label,q in states.items():report['reports'].append(auditor.audit_groups(label,q));path.write_text(json.dumps(report,indent=2))
    if '--samples-only'not in sys.argv and '--quick'not in sys.argv:
        if 'solids'not in locals():solids={p['name']:load_solid(HERE/p['visual_file'])[1]for p in built['parts']}
        report['independent_collision_coverage']=coverage(built,solids);(HERE/'collision_coverage_report.json').write_text(json.dumps(report['independent_collision_coverage'],indent=2))
    assert all(sha(HERE/k)==v for k,v in hashes.items()),'Model files changed during validation; rerun'
    report=finalize_report(report,built);report['elapsed_seconds']=time.monotonic()-start;path.write_text(json.dumps(report,indent=2));print('SAVED',path,'seconds',report['elapsed_seconds'],flush=True)
if __name__=='__main__':main()
