#!/usr/bin/env python3
"""Generate browser-port reference data without modifying frozen project files.

Run with the project's existing Python environment and -B to avoid bytecode writes.
"""
import os
os.environ['ORT_DISABLE_TELEMETRY'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
os.environ['OPENBLAS_NUM_THREADS'] = '1'
import sys, json, hashlib, shutil, platform
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import onnxruntime as ort

ROOT = Path(os.environ.get('MICRODUCK_PROJECT_ROOT', Path(__file__).resolve().parents[2])).resolve()
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'training/tab5_gaits'))
from env import Env, GAITS

BASE = ROOT/'prototype/assembly_uart/scene_tab5_assembly_uart.xml'
SCREEN = ROOT/'prototype/assembly_uart/screen_visual/scene_tab5_assembly_uart_screen.xml'
POLICY = ROOT/'training/tab5_gaits/delivery/results/policy.onnx'

def sha(b): return hashlib.sha256(b).hexdigest()
def dump(name, v):
    (OUT/name).write_text(json.dumps(v, indent=2, allow_nan=False)+'\n')
def arr(a): return np.asarray(a).tolist()

# Full dynamics arrays plus the arrays needed to render poses. The only excluded
# arrays are mesh/texture/skin/flex asset buffers and decorative presentation data.
PHYSICS_PREFIXES = ('qpos', 'body_', 'jnt_', 'dof_', 'geom_', 'pair_', 'exclude_',
                    'actuator_', 'sensor_', 'site_', 'key_', 'eq_', 'tendon_',
                    'wrap_', 'numeric_', 'tuple_')
VISUAL_GEOM_FIELDS = {'geom_rgba', 'geom_matid', 'geom_dataid', 'geom_type',
                      'geom_size', 'geom_aabb', 'geom_rbound', 'geom_pos', 'geom_quat'}

def model_arrays(m):
    return {k: np.asarray(getattr(m,k)).copy() for k in dir(m)
            if isinstance(getattr(m,k), np.ndarray)}

def describe_model(e, label):
    m=e.m
    arrays=model_arrays(m)
    countnames=('nq nv nu na nbody njnt ngeom nsite ncam nlight nmesh nmeshvert '
                'nmeshface npair nexclude neq nsensor nsensordata nkey nmocap '
                'ntendon nwrap ntex nmat nconmax njmax').split()
    counts={k:int(getattr(m,k)) for k in countnames if hasattr(m,k)}
    options={}
    for k in dir(m.opt):
        if k.startswith('_'): continue
        v=getattr(m.opt,k)
        if isinstance(v,np.ndarray): options[k]=v.tolist()
        elif isinstance(v,(int,float)): options[k]=v
        elif hasattr(v,'value'): options[k]=int(v)
    fields={k:{'dtype':v.dtype.str,'shape':list(v.shape),'elements':v.size,
               'sha256':sha(v.tobytes(order='C'))} for k,v in arrays.items()}
    result={'label':label,'mujoco_version':mujoco.__version__,
      'counts':counts,'total_mass_kg':float(m.body_mass.sum()), 'options':options,
      'compiled_model_sha256':e.base_model_sha256,'arrays':fields,
      'joint_names':[m.joint(i).name for i in range(m.njnt)],
      'actuator_names':list(e.names),'body_names':[m.body(i).name for i in range(m.nbody)],
      'geom_names':[m.geom(i).name for i in range(m.ngeom)],
      'qpos_indices':arr(e.qidx),'dof_indices':arr(e.vidx),
      'joint_indices':arr(e.ctrl.joint_indexes),'actuator_indices':arr(e.ctrl.act_indexes),
      'trunk_body_id':e.trunk,'gyro_sensor_adr':int(e.gyro_start),
      'default_joint_pos_float32':arr(e.default),'floor_geom_id':e.floor,
      'foot_geom_ids':e.feet,'shell_geom_ids':sorted(e.shell_ids),
      'guard_geom_ids':sorted(e.guard_ids),
      'contact_pairs': [{'index':i,'geom1':m.geom(int(m.pair_geom1[i])).name,
                        'geom2':m.geom(int(m.pair_geom2[i])).name,
                        'dim':int(m.pair_dim[i]),'friction':arr(m.pair_friction[i]),
                        'solref':arr(m.pair_solref[i]),'solimp':arr(m.pair_solimp[i]),
                        'margin':float(m.pair_margin[i]),'gap':float(m.pair_gap[i])}
                       for i in range(m.npair)]}
    dump(f'{label}_compiled_metadata.json',result)
    physics={k:arr(v) for k,v in arrays.items() if k.startswith(PHYSICS_PREFIXES)}
    dump(f'{label}_physics_arrays.json',physics)
    return arrays,result

def export_xml(e):
    path=OUT/'screen_after_bam.xml'
    # load_mujoco_with_bam uses MjSpec.compile(), which does not populate the
    # process-global last XML for mj_saveLastXML. Recreate exactly its mutations.
    spec=mujoco.MjSpec.from_file(str(SCREEN))
    bm=e.ctrl.model
    limit=bm.actuator.vin*bm.kt.value/bm.R.value
    for act in spec.actuators:
        tgt=act.target
        name=tgt.name if hasattr(tgt,'name') else str(tgt)
        if name.startswith('passive_'): continue
        act.set_to_motor();act.forcelimited=True;act.forcerange=(-limit,limit)
        act.ctrllimited=False;act.gear=[1.,0,0,0,0,0]
        for joint in spec.joints:
            if joint.name==name:
                joint.damping=np.zeros((3,1));joint.frictionloss=0
                joint.solref_friction=(-5e4,-2e2)
                joint.solimp_friction=(.99,.9999,.001,.5,2)
                joint.armature=bm.actuator.get_extra_inertia()
                break
    spec.option.timestep=.005
    path.write_text(spec.to_xml())
    root=ET.parse(path)
    assets=[]
    (OUT/'assets').mkdir(exist_ok=True)
    used={}
    for element in root.getroot().find('asset'):
        name=element.get('file')
        if not name: continue
        source=(SCREEN.parent/name).resolve()
        if not source.exists():
            # mj_saveLastXML may retain only a filename, whereas MjSpec compiled
            # it using the original XML-relative path. Resolve from source XML.
            original=ET.parse(SCREEN.parent/'tab5_assembly_uart_screen.xml')
            matches=[(SCREEN.parent/x.get('file')).resolve()
                     for x in original.getroot().find('asset')
                     if x.get('file') and Path(x.get('file')).name==Path(name).name]
            assert len(matches)==1,(name,matches)
            source=matches[0]
        destination='assets/'+source.name
        if destination in used and used[destination]!=str(source):
            destination='assets/'+sha(str(source).encode())[:10]+'_'+source.name
        used[destination]=str(source)
        shutil.copy2(source,OUT/destination)
        element.set('file',destination)
        assets.append({'kind':element.tag,'name':element.get('name'),
                       'original_path':str(source.relative_to(ROOT)),'path':destination,
                       'bytes':source.stat().st_size,'sha256':sha(source.read_bytes())})
    root.write(path,encoding='utf-8',xml_declaration=True)
    dump('assets_manifest.json',assets)
    offset=0;packed=[]
    with (OUT/'assets.bin').open('wb') as blob:
        for item in assets:
            raw=(OUT/item['path']).read_bytes();blob.write(raw)
            packed.append(dict(item,offset=offset,length=len(raw)))
            offset+=len(raw)
    dump('assets_index.json',packed)
    mujoco.mj_saveModel(e.m,str(OUT/'screen_after_bam.mjb'))
    return path

def export_renderer(e):
    m=e.m
    geoms=[];meshids=set()
    for i in range(m.ngeom):
        # Native viewer defaults show groups 0,1,2 and hide collision group 3.
        if int(m.geom_group[i])>2 or m.geom_rgba[i,3]==0: continue
        typ=int(m.geom_type[i]);mid=int(m.geom_dataid[i])
        if typ==int(mujoco.mjtGeom.mjGEOM_MESH):meshids.add(mid)
        geoms.append({'id':i,'name':m.geom(i).name,'type':typ,
                      'group':int(m.geom_group[i]),'bodyid':int(m.geom_bodyid[i]),
                      'rgba':arr(m.geom_rgba[i]),'size':arr(m.geom_size[i]),
                      'matid':int(m.geom_matid[i]),'dataid':mid})
    # Keep float32/index typed arrays binary, with 4-byte aligned offsets.
    buffers=[];offset=0;meshes=[]
    def pack(v):
        nonlocal offset
        v=np.ascontiguousarray(v)
        info={'offset':offset,'bytes':v.nbytes,'dtype':v.dtype.str,'shape':list(v.shape)}
        buffers.append(v.tobytes());offset+=v.nbytes
        return info
    for i in sorted(meshids):
        va,vn=int(m.mesh_vertadr[i]),int(m.mesh_vertnum[i])
        fa,fn=int(m.mesh_faceadr[i]),int(m.mesh_facenum[i])
        na,nn=int(m.mesh_normaladr[i]),int(m.mesh_normalnum[i])
        ta,tn=int(m.mesh_texcoordadr[i]),int(m.mesh_texcoordnum[i])
        meshes.append({'id':i,'name':m.mesh(i).name,
                       'vertices':pack(m.mesh_vert[va:va+vn]),
                       'faces':pack(m.mesh_face[fa:fa+fn]),
                       'normals':pack(m.mesh_normal[na:na+nn]),
                       'face_normals':pack(m.mesh_facenormal[fa:fa+fn]),
                       'texcoords':pack(m.mesh_texcoord[ta:ta+tn]) if tn else None,
                       'face_texcoords':pack(m.mesh_facetexcoord[fa:fa+fn]) if tn else None})
    (OUT/'renderer_meshes.bin').write_bytes(b''.join(buffers))
    mats=[]
    for i in range(m.nmat):
        mats.append({'id':i,'name':m.material(i).name,'rgba':arr(m.mat_rgba[i]),
                     'texture_ids_by_role':arr(m.mat_texid[i]),
                     'texuniform':bool(m.mat_texuniform[i]),'texrepeat':arr(m.mat_texrepeat[i]),
                     'emission':float(m.mat_emission[i]),'specular':float(m.mat_specular[i]),
                     'shininess':float(m.mat_shininess[i]),'reflectance':float(m.mat_reflectance[i])})
    tex=[{'id':i,'name':m.texture(i).name,'width':int(m.tex_width[i]),
          'height':int(m.tex_height[i]),'type':int(m.tex_type[i])} for i in range(m.ntex)]
    dump('renderer_metadata.json',{'geoms':geoms,'meshes':meshes,'materials':mats,
      'textures':tex,'mesh_binary':'renderer_meshes.bin',
      'transform':'Use data.geom_xpos/geom_xmat for each geom; compiled mesh vertices already include MuJoCo mesh preprocessing.',
      'visible_group_rule':'geom_group<=2 and geom_rgba alpha>0; default viewer groups 0,1,2',
      'texture_role_enum':{k:int(v) for k,v in mujoco.mjtTextureRole.__members__.items()}})

def export_runtime_config(meta,constants):
    keys=['counts','options','qpos_indices','dof_indices','joint_indices',
          'actuator_indices','actuator_names','trunk_body_id','gyro_sensor_adr',
          'default_joint_pos_float32','floor_geom_id','foot_geom_ids',
          'shell_geom_ids','guard_geom_ids']
    c={k:meta[k] for k in keys}
    q=c['default_joint_pos_float32']
    c.update(bam=constants,control_dt=.02,sim_dt=.005,substeps=4,
      action_clip=[-1.2,1.2],command_ramp_seconds=.5,
      reset={'keyframe_name':'STAND','keyframe_id':1,
             'qpos':[0,0,.125,1,0,0,0]+q,'qvel':[0]*16,'last_action':[0]*10,
             'ctrl':[0,-.0872664626,-.457924,-.00494,.452984,0,
                     .0872664626,.457924,.00494,-.452984],
             'clear_model_frictionloss_and_damping':True},
      files={'mjb':'screen_after_bam.mjb','xml':'screen_after_bam_exact.xml',
             'asset_blob':'assets.bin','asset_index':'assets_index.json',
             'render_metadata':'renderer_metadata.json',
             'render_blob':'renderer_meshes.bin',
             'trajectory':'trajectory_reference.json','policy':str(POLICY)},
      enums={'friction_dof':1},gaits=GAITS)
    dump('runtime_config.json',c)

def reset(e):
    # Independent evaluation explicitly restores these model-mutating fields.
    e.m.dof_frictionloss[e.vidx]=0
    e.m.dof_damping[e.vidx]=0
    return e.reset(GAITS['forward'],noise=0)

def state(e):
    d=e.d;m=e.m
    return {'time':float(d.time),'qpos':arr(d.qpos),'qvel':arr(d.qvel),
            'ctrl':arr(d.ctrl),'qfrc_actuator':arr(d.qfrc_actuator),
            'qfrc_bias':arr(d.qfrc_bias),'qfrc_constraint':arr(d.qfrc_constraint),
            'dof_frictionloss':arr(m.dof_frictionloss),
            'dof_damping':arr(m.dof_damping),'qacc_warmstart':arr(d.qacc_warmstart),
            'ncon':int(d.ncon),'nefc':int(d.nefc),
            'efc_type':arr(d.efc_type),'efc_id':arr(d.efc_id),'efc_force':arr(d.efc_force),
            'gyro':arr(d.sensordata[e.gyro_start:e.gyro_start+3]),
            'trunk_xmat':arr(d.xmat[e.trunk]),'obs':arr(e.obs())}

def trajectory(e, session, steps=50):
    reset(e)
    initial=state(e)
    frames=[]
    original_update=e.ctrl.update
    active=[]
    def record_update():
        before=state(e)
        q=e.d.qpos[e.qidx].copy();dq=e.d.qvel[e.vidx].copy()
        volts=e.ctrl.model.actuator.compute_control(e.ctrl.q_target,q,dq,e.d.time-e.ctrl.last_ts)
        original_update()
        active.append({'before':before,'volts':arr(volts),
                       'ctrl_after_update':arr(e.d.ctrl),
                       'frictionloss_after_update':arr(e.m.dof_frictionloss),
                       'damping_after_update':arr(e.m.dof_damping)})
    e.ctrl.update=record_update
    for i in range(steps):
        obs=e.obs();action=session.run(None,{'obs':obs[None]})[0][0]
        active=[]
        after_obs,reward,done,info=e.step(action)
        frames.append({'control_step':i,'obs_before':arr(obs),'action':arr(action),
                       'q_target':arr(e.ctrl.q_target),'substeps':active,
                       'after':state(e),'reward':reward,'done':done})
    e.ctrl.update=original_update
    return {'description':'Native policy-generated actions frozen for open-loop WASM replay',
            'command':GAITS['forward'],'noise':0,'randomize':False,
            'control_dt':.02,'simulation_dt':.005,'initial':initial,'frames':frames}

def compare_models(a,b):
    compared=[];different=[]
    for key in sorted(a.keys() & b.keys()):
        if not key.startswith(PHYSICS_PREFIXES) or key in VISUAL_GEOM_FIELDS: continue
        # Geom same local pose except the display box changed to mesh. Both are
        # noncolliding visuals attached to an explicitly inertial body.
        av,bv=a[key],b[key]
        compared.append(key)
        if not np.array_equal(av,bv):
            different.append({'field':key,'shape_a':list(av.shape),'shape_b':list(bv.shape),
                              'max_abs':float(np.max(np.abs(av.astype(float)-bv.astype(float))))
                              if av.shape==bv.shape and av.size else None})
    return {'fields_compared':compared,'different':different}

def main():
    print('Loading nominal baseline...',flush=True)
    base=Env(BASE,seed=0,randomize=False)
    base_arrays,base_meta=describe_model(base,'nominal')
    print('Loading visual screen variant...',flush=True)
    screen=Env(SCREEN,seed=0,randomize=False)
    screen_arrays,screen_meta=describe_model(screen,'screen')
    export_renderer(screen)
    comparison=compare_models(base_arrays,screen_arrays)
    dump('nominal_screen_comparison.json',comparison)
    path=export_xml(screen)
    print('Exported exact asset files; checking XML reload precision...',flush=True)
    reloaded=mujoco.MjModel.from_xml_path(str(path))
    reload_arrays=model_arrays(reloaded)
    dump('xml_reload_comparison.json',compare_models(screen_arrays,reload_arrays))
    opts=ort.SessionOptions();opts.intra_op_num_threads=1;opts.inter_op_num_threads=1
    session=ort.InferenceSession(str(POLICY),sess_options=opts,providers=['CPUExecutionProvider'])
    print('Recording policy trajectory...',flush=True)
    traj=trajectory(screen,session)
    dump('trajectory_reference.json',traj)
    # Check visual-only variant against baseline, using fixed reference actions.
    reset(base); parity=[]
    for frame in traj['frames']:
        base.step(np.asarray(frame['action'],dtype=np.float32))
        expected=frame['after']
        parity.append({'step':frame['control_step'],
                       'qpos_max_abs':float(np.max(np.abs(base.d.qpos-expected['qpos']))),
                       'qvel_max_abs':float(np.max(np.abs(base.d.qvel-expected['qvel']))),
                       'obs_max_abs':float(np.max(np.abs(base.obs()-expected['obs'])))})
    dump('nominal_screen_trajectory_parity.json',parity)
    bm=screen.ctrl.model;act=bm.actuator
    constants={k:v.value for k,v in bm.get_parameters().items()}
    constants.update(kp=act.kp,vin=act.vin,error_gain=act.error_gain,max_pwm=act.max_pwm,
                     max_current=act.max_current,vin_drop_gain=screen.ctrl.vin_drop_gain,
                     vin_min=screen.ctrl.vin_min,force_limit=7.4*bm.kt.value/bm.R.value,
                     friction_solref=[-5e4,-2e2],friction_solimp=[.99,.9999,.001,.5,2],
                     model='m6',actuator='xl330',current_limiting=False)
    dump('bam_constants.json',constants)
    export_runtime_config(screen_meta,constants)
    sources=[POLICY,BASE,BASE.parent/'tab5_assembly_uart.xml',SCREEN,
             SCREEN.parent/'tab5_assembly_uart_screen.xml',ROOT/'training/tab5_gaits/env.py',
             ROOT/'microduck_rl/scripts/infer_policy.py',ROOT/'bam/bam/mujoco.py',
             ROOT/'bam/bam/actuator.py',ROOT/'bam/bam/model.py',
             ROOT/'bam/bam/dynamixel/actuator.py',ROOT/'bam/bam/params/xl330/m6.json']
    dump('provenance.json',{'python':platform.python_version(),'numpy':np.__version__,
                           'mujoco':mujoco.__version__,'onnxruntime':ort.__version__,
                           'source_files':{str(p.relative_to(ROOT)):sha(p.read_bytes()) for p in sources},
                           'policy_input':[x.name for x in session.get_inputs()],
                           'policy_output':[x.name for x in session.get_outputs()]})
    print(json.dumps({'screen_counts':screen_meta['counts'],
                      'different_physics_arrays':comparison['different'],
                      'max_nominal_screen_trajectory_error':max(x['qpos_max_abs'] for x in parity),
                      'frames':len(traj['frames']), 'outputs':str(OUT)},indent=2),flush=True)

if __name__=='__main__':main()
