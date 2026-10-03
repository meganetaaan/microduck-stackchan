"""Learn a small command calibration in real MuJoCo rollouts after PPO.
Nelder-Mead minimizes measured velocity and heading errors; no scripted motion.
The new coefficients are inside the exported 39-input/10-action ONNX network.
"""
import os
os.environ['ORT_DISABLE_TELEMETRY']='1'
for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[k]='1'
import argparse,json,hashlib,time,concurrent.futures,multiprocessing as mp
from pathlib import Path
import numpy as np,torch,torch.nn as nn
from scipy.optimize import minimize
from env import Env,GAITS
from policy import Actor,export_actor
class CalibratedActor(nn.Module):
 def __init__(self,core):
  super().__init__();self.core=core;self.register_buffer('calibration',torch.zeros(7,3))
 def forward(self,obs):
  cmd=obs[...,36:39];pos=torch.relu(cmd);neg=-torch.relu(-cmd)
  warped=pos*self.core.command_scale[0]+neg*self.core.command_scale[1]+torch.tanh(pos*100)*self.core.command_bias[0]+torch.tanh(neg*100)*self.core.command_bias[1]
  x,y,w=cmd[...,0],cmd[...,1],cmd[...,2]
  features=torch.stack([torch.clamp(x/.075,0,1),torch.clamp((x-.075)/.045,0,2),torch.clamp(-x/.15,0,1.5),torch.clamp(y/.07,0,1.5),torch.clamp(-y/.055,0,1.5),torch.clamp(w/.7,0,1.5),torch.clamp(-w/.7,0,1.5)],-1)
  warped=warped+features@self.calibration
  return self.core.net((torch.cat([obs[...,:36],warped],-1)-self.core.mean)/self.core.denom)

def fit_one(task):
 scene,checkpoint,out,gait,row,axes,maxfev,right_expanded=task;torch.set_num_threads(1);ck=torch.load(checkpoint,weights_only=False);core=Actor();core.load_state_dict(ck['actor']);actor=CalibratedActor(core).eval();e=Env(scene,seed=131,horizon=8);cmd=np.array(GAITS[gait]);saved=Path(out)/f'{gait}_search.json';log=json.loads(saved.read_text())['records']if saved.exists()else[];steps=log[-1]['cumulative_control_steps']if log else 0;previous_wall=log[-1]['wall_s']if log else 0;start=time.monotonic();seeds=[131,132,133];cache={tuple(np.round(r['parameters'],7)):r['objective']for r in log};linear_std=.012 if np.linalg.norm(cmd[:2])<.001 else .03
 def objective(theta):
  nonlocal steps
  theta=np.clip(np.asarray(theta),[-.18,-.7],[.18,.7]);key=tuple(np.round(theta,7))
  if key in cache:return cache[key]
  with torch.no_grad():
   actor.calibration[row].zero_();actor.calibration[row,axes[0]]=theta[0];actor.calibration[row,axes[1]]=theta[1]
  trials=[];costs=[]
  for seed in seeds:
   e.rng=np.random.default_rng(seed);e.m.dof_frictionloss[e.vidx]=0.;e.m.dof_damping[e.vidx]=0.;obs=e.reset(cmd,noise=.006);done=False
   while not done:
    with torch.no_grad():action=actor(torch.from_numpy(obs)).numpy()
    obs,reward,done,info=e.step(action);steps+=1
   vel=np.array(info['mean_body_velocity']);rate=e.yaw_unwrapped/max(e.d.time-.25,.1)
   cost=np.sum(((vel[:2]-cmd[:2])/linear_std)**2)+((rate-cmd[2])/.07)**2
   if info['fall']or info['unsafe_self_contact']:cost+=100+50*(1-e.d.time/8)
   cost+=.001*np.sum(theta**2);costs.append(float(cost));trials.append(info)
  value=float(np.mean(costs)+.25*np.std(costs));record={'evaluation':len(log)+1,'parameters':theta.tolist(),'objective':value,'trial_costs':costs,'trials':trials,'cumulative_control_steps':steps,'wall_s':previous_wall+time.monotonic()-start};log.append(record);cache[key]=value
  (Path(out)/f'{gait}_search.json').write_text(json.dumps({'gait':gait,'row':row,'axes':axes,'seeds':seeds,'algorithm':'bounded Nelder-Mead on measured MuJoCo rollout cost','model_compiled_sha256':e.base_model_sha256,'records':log},indent=2));print(gait,len(log),value,theta.tolist(),flush=True);return value
 simplex=np.array([[0,0],[-.05,0],[-.05,-.2]])if gait=='turn_right'and right_expanded else np.array([[0,0],[.025,0],[0,.15]])
 result=minimize(objective,np.zeros(2),method='Nelder-Mead',options={'initial_simplex':simplex,'maxfev':maxfev,'xatol':.002,'fatol':.01,'adaptive':True})
 best=min(log,key=lambda r:r['objective']);return {'gait':gait,'row':row,'axes':axes,'best':best,'initial':log[0],'evaluations':len(log),'control_steps':steps,'optimizer_message':str(result.message),'wall_s':previous_wall+time.monotonic()-start}

def main():
 p=argparse.ArgumentParser();p.add_argument('--scene',required=True);p.add_argument('--checkpoint',required=True);p.add_argument('--out',required=True);p.add_argument('--maxfev',type=int,default=32);p.add_argument('--workers',type=int,default=3);p.add_argument('--gaits',nargs='*');p.add_argument('--right-expanded',action='store_true');a=p.parse_args();out=Path(a.out);out.mkdir(parents=True,exist_ok=True);torch.set_num_threads(1)
 # The slow-forward operating point is already accurate; high-forward has a separate basis.
 cases=[('forward',1,[1,2]),('backward',2,[1,2]),('left',3,[0,2]),('right',4,[0,2]),('turn_left',5,[0,2]),('turn_right',6,[0,2])];prior=json.loads((out/'calibration_report.json').read_text())if(out/'calibration_report.json').exists()else None
 if a.gaits:cases=[c for c in cases if c[0]in a.gaits]
 tasks=[(str(Path(a.scene).resolve()),str(Path(a.checkpoint).resolve()),str(out.resolve()),g,row,axes,a.maxfev,a.right_expanded)for g,row,axes in cases];results=[]
 with concurrent.futures.ProcessPoolExecutor(a.workers,mp_context=mp.get_context('spawn'))as ex:
  for result in ex.map(fit_one,tasks):results.append(result)
 ck=torch.load(a.checkpoint,weights_only=False);core=Actor();core.load_state_dict(ck['actor']);actor=CalibratedActor(core)
 if prior:
  actor.calibration.copy_(torch.tensor(prior['calibration']))
  merged={r['gait']:r for r in prior['results']};merged.update({r['gait']:r for r in results});results=list(merged.values())
 for r in results:
  with torch.no_grad():actor.calibration[r['row'],r['axes'][0]]=r['best']['parameters'][0];actor.calibration[r['row'],r['axes'][1]]=r['best']['parameters'][1]
 export_actor(actor,out/'policy.onnx');ck['command_calibration']=actor.calibration;ck['calibration_control_steps']=sum(r['control_steps']for r in results);torch.save(ck,out/'policy.pt')
 report={'algorithm':'PPO + real-MuJoCo Nelder-Mead command calibration','source_ppo_checkpoint':str(Path(a.checkpoint).resolve()),'source_ppo_checkpoint_sha256':hashlib.sha256(Path(a.checkpoint).read_bytes()).hexdigest(),'ppo_control_steps':ck['steps'],'calibration_control_steps':ck['calibration_control_steps'],'training_seeds':[131,132,133],'calibration':actor.calibration.tolist(),'results':results,'policy_sha256':hashlib.sha256((out/'policy.onnx').read_bytes()).hexdigest(),'notes':'Zero-shot core and PPO are not replaced by animations; only measured command-to-velocity bias is fitted, then baked into the native 10-action network.'};(out/'calibration_report.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items()if k!='results'},indent=2))
if __name__=='__main__':main()
