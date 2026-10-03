#!/usr/bin/env python3
"""Long native closed-loop verification, explicit zero-noise/random stream contract."""
import os
os.environ['ORT_DISABLE_TELEMETRY']='1'
os.environ['OMP_NUM_THREADS']='1'
os.environ['OPENBLAS_NUM_THREADS']='1'
import sys,json,time,hashlib,concurrent.futures,multiprocessing
from pathlib import Path
import numpy as np,mujoco,onnxruntime as ort

H=Path(__file__).resolve().parent
ROOT=Path(os.environ.get('MICRODUCK_PROJECT_ROOT', Path(__file__).resolve().parents[2])).resolve()
sys.path.insert(0,str(ROOT/'training/tab5_gaits'))
import env
from bam.mujoco import MujocoController

_CACHE=None
def fast_load(xml_path,bam_model,timestep,vin_drop_gain,vin_min):
    m=mujoco.MjModel.from_binary_path(str(H/'screen_after_bam.mjb'))
    assert m.opt.timestep==timestep
    d=mujoco.MjData(m)
    names=[m.actuator(i).name for i in range(m.nu)]
    ctrl=MujocoController(bam_model,names,m,d,vin_drop_gain,vin_min)
    return m,d,ctrl,names

def run_one(task):
    global _CACHE
    if _CACHE is None:
        env.up.load_mujoco_with_bam=fast_load
        e=env.Env(ROOT/'prototype/assembly_uart/screen_visual/scene_tab5_assembly_uart_screen.xml',seed=0,horizon=20,randomize=False)
        opt=ort.SessionOptions();opt.intra_op_num_threads=1;opt.inter_op_num_threads=1
        s=ort.InferenceSession(str(ROOT/'training/tab5_gaits/delivery/results/policy.onnx'),sess_options=opt,providers=['CPUExecutionProvider'])
        _CACHE=e,s
    e,s=_CACHE
    gait,seed=task
    started=time.monotonic()
    noise=np.random.default_rng(seed).normal(0,.006,10)
    e.rng=np.random.default_rng(seed);e.seed=seed
    e.m.dof_frictionloss[e.vidx]=0;e.m.dof_damping[e.vidx]=0
    obs=e.reset(env.GAITS[gait],noise=.006)
    initial_qpos=e.d.qpos.copy()
    assert np.array_equal(initial_qpos[e.qidx],e.default+noise)
    frames=[];done=False;info={}
    while not done:
        before=obs
        action=s.run(None,{'obs':before[None]})[0][0]
        obs,reward,done,info=e.step(action)
        if e.steps<=50:
            frames.append({'step':e.steps,'time':float(e.d.time),
                           'obs_before':before.tolist(),'action':action.tolist(),
                           'qpos_after':e.d.qpos.tolist(),'qvel_after':e.d.qvel.tolist(),
                           'obs_after':obs.tolist()})
    steady=e.rows[50:]
    mean=[float(np.mean([r[k] for r in steady])) for k in ['vx','vy','wz']] if steady else None
    return {'gait':gait,'seed':seed,'command':list(env.GAITS[gait]),
            'initial_joint_noise_rad':noise.tolist(),'initial_qpos':initial_qpos.tolist(),
            'control_steps':e.steps,'sim_seconds':float(e.d.time),
            'early_termination':e.steps<1000,'fall':info['fall'],
            'unsafe_self_contact':bool(e.unsafe_self_contact),
            'self_contact_samples':e.self_contact_samples,
            'minimum_self_contact_distance_m':e.minimum_self_contact_distance,
            'max_tilt_deg':max(r['tilt_deg'] for r in e.rows),
            'max_torque_nm':max(r['torque'] for r in e.rows),
            'any_nonfoot':any(r['nonfoot'] for r in e.rows),
            'mean_body_velocity_after_1s':mean,
            'velocity_mean_frames':len(steady),'velocity_mean_start_step':51,
            'terminal_qpos':e.d.qpos.tolist(),'terminal_qvel':e.d.qvel.tolist(),
            'metrics_source':info,'first_second':frames,
            'elapsed_wall_seconds':time.monotonic()-started}

def main():
    tasks=[(g,s) for g in env.GAITS for s in [12340,12341,12342]]
    results=[];start=time.monotonic()
    with (H/'long_reference_progress.jsonl').open('w') as progress:
        with concurrent.futures.ProcessPoolExecutor(max_workers=3,
                mp_context=multiprocessing.get_context('spawn')) as pool:
            futures={pool.submit(run_one,task):task for task in tasks}
            for f in concurrent.futures.as_completed(futures):
                value=f.result();results.append(value)
                slim={k:v for k,v in value.items() if k not in ['first_second','metrics_source']}
                progress.write(json.dumps(slim)+'\n');progress.flush()
                print(json.dumps({k:slim[k] for k in ['gait','seed','sim_seconds','fall','unsafe_self_contact','max_tilt_deg','mean_body_velocity_after_1s']}),flush=True)
    results.sort(key=lambda x:(list(env.GAITS).index(x['gait']),x['seed']))
    provenance=json.load(open(H/'provenance.json'))
    report={'description':'Exact native closed-loop trained-policy reference, 8 gaits x 3 independent fresh seeds, 20s requested per case',
            'mujoco':mujoco.__version__,'onnxruntime':ort.__version__,'numpy':np.__version__,
            'mjb_sha256':hashlib.sha256((H/'screen_after_bam.mjb').read_bytes()).hexdigest(),
            'policy_sha256':provenance['source_files']['training/tab5_gaits/delivery/results/policy.onnx'],
            'reset_noise_recipe':'np.random.default_rng(seed).normal(0,0.006,10) from a FRESH generator for every case; exported exact vector added in float64 to float32 defaults',
            'reset_contract':'Independent reset clears actuated dof_frictionloss/damping, mj_resetDataKeyframe STAND, z=.125, default+noise, controller.reset, q_target=default, last_ts=0, mj_forward; no domain randomization',
            'mean_velocity_contract':'Arithmetic mean of body-local vx,vy and trunk gyro wz recorded after mj_forward on control steps 51 through terminal; first 50 control steps excluded',
            'termination_contract':'Frozen Env.step: nonfinite qpos, tilt>55deg, root z<.075, penetrating non-foot floor contact, shell/guard selfcontact dist<-.0005, or 1000 control steps',
            'elapsed_wall_seconds':time.monotonic()-start,'cases':results}
    (H/'long_reference.json').write_text(json.dumps(report,indent=2)+'\n')
    compact=dict(report,cases=[{k:v for k,v in c.items() if k!='first_second'} for c in results])
    (H/'long_reference_metrics.json').write_text(json.dumps(compact,indent=2)+'\n')
    print('COMPLETE '+str(len(results))+' cases in '+str(time.monotonic()-start),flush=True)

if __name__=='__main__':main()
