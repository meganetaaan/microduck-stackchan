"""Continuous command transitions; same state and policy throughout (no resets)."""
import os
os.environ['ORT_DISABLE_TELEMETRY']='1';os.environ['OPENBLAS_NUM_THREADS']='1'
import argparse,json,hashlib
from pathlib import Path
import numpy as np,onnxruntime as ort
from env import Env,GAITS
p=argparse.ArgumentParser();p.add_argument('--scene',required=True);p.add_argument('--policy',required=True);p.add_argument('--out',required=True);p.add_argument('--seeds',type=int,default=5);a=p.parse_args();out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
sequence=['stand','forward_slow','forward','stand','backward','stand','left','right','stand','turn_left','turn_right','stand'];segment=5.;opt=ort.SessionOptions();opt.intra_op_num_threads=1;opt.inter_op_num_threads=1;s=ort.InferenceSession(a.policy,sess_options=opt,providers=['CPUExecutionProvider']);results=[]
for seed in range(2000,2000+a.seeds):
 e=Env(a.scene,seed,len(sequence)*segment);ob=e.reset(GAITS[sequence[0]]);parts=[];fallen=False
 for i,name in enumerate(sequence):
  previous=e.command.copy();target=np.array(GAITS[name]);start=e.d.qpos[:3].copy();start_yaw=e.yaw_unwrapped
  for j in range(round(segment/.02)):
   e.command=previous+(target-previous)*min(1.,(j+1)*.02/.5);ob=e.obs();action=s.run(None,{s.get_inputs()[0].name:ob[None]})[0][0];ob,r,done,info=e.step(action,record=seed==2000)
   if done and info.get('fall'):fallen=True;break
  rr=e.rows[-min(round(segment/.02),len(e.rows)):];settled=rr[min(50,len(rr)//3):];parts.append({'gait':name,'fall':fallen,'displacement':(e.d.qpos[:3]-start).tolist(),'yaw_change':e.yaw_unwrapped-start_yaw,'mean_velocity':[float(np.mean([r[k]for r in settled]))for k in ['vx','vy','wz']],'max_tilt':max(r['tilt_deg']for r in rr)})
  if fallen:break
 result={'seed':seed,'fall':fallen,'completed_segments':len(parts),'segments':parts,'simulated_seconds':e.d.time};results.append(result);print(json.dumps(result),flush=True)
 if seed==2000:np.savez_compressed(out/'transitions_seed2000_trace.npz',qpos=e.states,dt=.02)
(out/'transitions.json').write_text(json.dumps({'results':results,'sequence':sequence,'segment_seconds':segment,'command_ramp_seconds':.5,'policy_sha256':hashlib.sha256(Path(a.policy).read_bytes()).hexdigest(),'scene_sha256':hashlib.sha256(Path(a.scene).read_bytes()).hexdigest(),'notes':'One free-running rollout per seed; no pose resets at command switches; synthetic command schedule, physical motion learned by policy'},indent=2))
