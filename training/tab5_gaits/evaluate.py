"""Independent seeded nominal/randomized and transition tests, actual MuJoCo dynamics."""
import os
os.environ['ORT_DISABLE_TELEMETRY']='1'
os.environ['OPENBLAS_NUM_THREADS']='1'
import json,argparse,time,hashlib,concurrent.futures,multiprocessing as mp
from pathlib import Path
import numpy as np,onnxruntime as ort
from env import Env,ROOT,GAITS

_CACHE={}
def evaluate_one(task):
 scene,policy,gait,command,seed,seconds,randomize,record,out=task
 key=(scene,policy,seconds,randomize)
 if key not in _CACHE:
  e=Env(scene,seed,seconds,randomize);opt=ort.SessionOptions();opt.intra_op_num_threads=1;opt.inter_op_num_threads=1;s=ort.InferenceSession(policy,sess_options=opt,providers=['CPUExecutionProvider']);_CACHE[key]=(e,s)
 e,s=_CACHE[key];e.rng=np.random.default_rng(seed);e.seed=seed
 # Restore BAM's compile-time friction fields before each independent reset.
 e.m.dof_frictionloss[e.vidx]=0.;e.m.dof_damping[e.vidx]=0.
 obs=e.reset(command,noise=.006 if seed else 0.);done=False
 while not done:
  action=s.run(None,{s.get_inputs()[0].name:obs[None]})[0][0];obs,r,done,info=e.step(action,record)
 info.update(seed=seed,gait=gait,randomize=randomize,total_mass_kg=float(e.m.body_mass.sum()),voltage_v=float(e.ctrl.model.actuator.vin),floor_friction=float(e.m.geom_friction[e.floor,0]),actuator_force_limit_nm=float(e.m.actuator_forcerange[0,1]),mass_randomization_kind=e.mass_randomization_kind,body_mass_sha256=hashlib.sha256(e.m.body_mass.tobytes()).hexdigest(),compiled_model_sha256=e.base_model_sha256,scene_sha256=hashlib.sha256(Path(scene).read_bytes()).hexdigest(),policy_sha256=hashlib.sha256(Path(policy).read_bytes()).hexdigest())
 if record:
  np.savez_compressed(Path(out)/f'{gait}_seed{seed}_trace.npz',qpos=e.states,rows=np.array([[r[k]for k in ['t','x','y','z','vx','vy','wz','yaw','tilt_deg','torque']]for r in e.rows]),dt=.02)
 return info

def main():
 p=argparse.ArgumentParser();p.add_argument('--scene',required=True);p.add_argument('--policy',required=True);p.add_argument('--out',required=True);p.add_argument('--seconds',type=float,default=20);p.add_argument('--seeds',type=int,default=5);p.add_argument('--seed-base',type=int,default=3000);p.add_argument('--randomize',action='store_true');p.add_argument('--record',action='store_true');p.add_argument('--workers',type=int,default=8);a=p.parse_args();out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
 tasks=[(str(Path(a.scene).resolve()),str(Path(a.policy).resolve()),g,cmd,a.seed_base+i,a.seconds,a.randomize,a.record and i==0,str(out))for g,cmd in GAITS.items()for i in range(a.seeds)]
 t=time.monotonic();results=[]
 with concurrent.futures.ProcessPoolExecutor(a.workers,mp_context=mp.get_context('spawn'))as ex:
  for info in ex.map(evaluate_one,tasks):results.append(info);print(json.dumps(info),flush=True)
 summary={}
 for g in GAITS:
  rr=[r for r in results if r['gait']==g];summary[g]={'successes':sum(r['success']for r in rr),'falls':sum(r['fall']for r in rr),'trials':len(rr),'mean_velocity':np.mean([r['mean_body_velocity']for r in rr],axis=0).tolist(),'max_tilt':max(r['max_tilt_deg']for r in rr),'mean_tracking_error':float(np.mean([r['mean_linear_error']for r in rr])),'mean_yaw_error':float(np.mean([r['mean_yaw_error']for r in rr])),'mean_cross_track_drift_m':float(np.mean([r['cross_track_drift_m']for r in rr])),'max_cross_track_drift_m':max(r['cross_track_drift_m']for r in rr),'mean_abs_heading_drift_rad':float(np.mean([abs(r['heading_drift_rad'])for r in rr]))}
 report={'summary':summary,'trials':results,'elapsed_wall_s':time.monotonic()-t,'criteria':'No fall, tilt<20deg, mean linear tracking error<0.025m/s, mean yaw error<0.15rad/s; motion must reach >=50% commanded projection. For zero translation, mean drift speed<0.015m/s.','test_seeds':f'{a.seed_base}+index; distinct from training seed stream and initial 1000-series development validation','physics':'Full assembly/BAM M6/XL330; free root; no external forces, no kinematic target overwrite','evaluation_code_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'environment_code_sha256':hashlib.sha256(Path(__file__).with_name('env.py').read_bytes()).hexdigest()}
 (out/'evaluation.json').write_text(json.dumps(report,indent=2));print(json.dumps(summary,indent=2))
if __name__=='__main__':main()
