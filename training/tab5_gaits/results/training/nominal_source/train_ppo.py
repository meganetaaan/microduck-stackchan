"""CPU PPO warm-start training, all ten physical action outputs trainable.
Checkpoints include optimizer, complete provenance, seed and RNG state.
"""
import os
for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[k]='1'
os.environ['ORT_DISABLE_TELEMETRY']='1'
import argparse,json,time,hashlib,copy,shutil,multiprocessing as mp
from pathlib import Path
import numpy as np,torch
from policy import Actor,Critic,export_actor
from env import Env,ROOT,GAITS

def worker(pipe,scene,seed,horizon,randomize):
 e=Env(scene,seed,horizon,randomize)
 pipe.send((e.reset(),e.base_model_sha256))
 while True:
  a=pipe.recv()
  if a is None:break
  obs,r,done,info=e.step(a)
  if done:obs=e.reset()
  pipe.send((obs,r,done,info))

def main():
 p=argparse.ArgumentParser();p.add_argument('--scene',required=True);p.add_argument('--out',required=True);p.add_argument('--seed',type=int,default=7);p.add_argument('--envs',type=int,default=8);p.add_argument('--steps',type=int,default=64);p.add_argument('--iterations',type=int,default=2000);p.add_argument('--lr',type=float,default=1e-5);p.add_argument('--std',type=float,default=.025);p.add_argument('--scales',nargs=3,type=float,default=[5,7,4]);p.add_argument('--calibrated-init',action='store_true');p.add_argument('--resume');p.add_argument('--randomize',action='store_true');a=p.parse_args()
 torch.set_num_threads(1);torch.manual_seed(a.seed);np.random.seed(a.seed)
 out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
 actor=Actor.from_teacher(a.scales);
 if a.calibrated_init:actor.calibrated_init()
 critic=Critic();anchor=copy.deepcopy(actor).eval()
 opt=torch.optim.Adam([{'params':actor.net.parameters(),'lr':a.lr},{'params':[actor.command_scale,actor.command_bias],'lr':1e-4},{'params':critic.parameters(),'lr':3e-4}]);start=0
 if a.resume:
  ck=torch.load(a.resume,weights_only=False);actor.load_state_dict(ck['actor']);critic.load_state_dict(ck['critic']);opt.load_state_dict(ck['optimizer']);start=ck['iteration'];torch.set_rng_state(ck['torch_rng'])
 scene=Path(a.scene).resolve();scenehash=hashlib.sha256(scene.read_bytes()).hexdigest();xml=scene.parent/('tab5_assembly_uart.xml'if 'assembly_uart'in str(scene) else 'tab5_portrait_print.xml');modelhash=hashlib.sha256(xml.read_bytes()).hexdigest()
 config=vars(a)|{'scene_sha256':scenehash,'model_sha256':modelhash,'torch':torch.__version__,'device':'cpu','obs_dim':39,'actions':10,'algorithm':'PPO-clip','gamma':.99,'gae_lambda':.95,'clip':.15,'epochs':4,'gait_commands':GAITS,'anchor_kl_weight':.02,'normalizer':'frozen teacher normalizer baked in export','horizon_s':8,'physics_dt':.005,'control_hz':50,'teacher_initialization_only':True,'physics':'MuJoCo full assembly + BAM M6 XL330 at 200 Hz'}
 (out/'source').mkdir(exist_ok=True)
 config['source_sha256']={}
 for fname in ['env.py','policy.py','train_ppo.py','evaluate.py','transitions.py']:
  source=Path(__file__).with_name(fname);shutil.copy2(source,out/'source'/fname);config['source_sha256'][fname]=hashlib.sha256(source.read_bytes()).hexdigest()
 (out/'config.json').write_text(json.dumps(config,indent=2));torch.save(actor.state_dict(),out/'initial_actor.pt');export_actor(actor,out/'initial.onnx')
 pipes=[];processes=[];ctx=mp.get_context('spawn')
 for i in range(a.envs):
  pa,pb=ctx.Pipe();pr=ctx.Process(target=worker,args=(pb,str(scene),a.seed*100+i,8,a.randomize));pr.start();pipes.append(pa);processes.append(pr)
 initial=[p.recv()for p in pipes];obs=torch.from_numpy(np.array([v[0]for v in initial]));config['compiled_model_sha256']=initial[0][1];(out/'config.json').write_text(json.dumps(config,indent=2));done=torch.zeros(a.envs);t0=time.monotonic();total=start*a.envs*a.steps;recent=[]
 log=(out/'learning.jsonl').open('a',buffering=1)
 try:
  for iteration in range(start,a.iterations):
   ob=[];ac=[];lp=[];va=[];re=[];dn=[];newinfos=[]
   for t in range(a.steps):
    with torch.no_grad():
     mu=actor(obs);dist=torch.distributions.Normal(mu,a.std);act=dist.sample();lprob=dist.log_prob(act).sum(-1);val=critic((obs-actor.mean)/actor.denom)
    ob.append(obs);ac.append(act);lp.append(lprob);va.append(val);dn.append(done)
    for p,action in zip(pipes,act.numpy()):p.send(action)
    results=[p.recv()for p in pipes];obs=torch.from_numpy(np.array([r[0]for r in results]));re.append(torch.tensor([r[1]for r in results]));done=torch.tensor([r[2]for r in results],dtype=torch.float32)
    newinfos.extend(r[3]for r in results if r[2]);total+=a.envs
   ob=torch.stack(ob);ac=torch.stack(ac);lp=torch.stack(lp);va=torch.stack(va);re=torch.stack(re);dn=torch.stack(dn)
   with torch.no_grad():
    nxt=critic((obs-actor.mean)/actor.denom);adv=torch.zeros_like(re);last=0
    for t in reversed(range(a.steps)):
     alive=1-(done if t==a.steps-1 else dn[t+1]);nextv=nxt if t==a.steps-1 else va[t+1]
     delta=re[t]+.99*nextv*alive-va[t];last=delta+.99*.95*alive*last;adv[t]=last
    returns=adv+va
   B=a.steps*a.envs;bo=ob.reshape(B,39);ba=ac.reshape(B,10);blp=lp.flatten();br=returns.flatten();bv=adv.flatten();bv=(bv-bv.mean())/(bv.std()+1e-8)
   with torch.no_grad():anchor_mu=anchor(bo)
   losses=[];kls=[]
   for epoch in range(4):
    for idx in torch.randperm(B).split(min(256,B)):
     mu=actor(bo[idx]);dist=torch.distributions.Normal(mu,a.std);newlp=dist.log_prob(ba[idx]).sum(-1);ratio=(newlp-blp[idx]).exp();ploss=-torch.min(ratio*bv[idx],ratio.clamp(.85,1.15)*bv[idx]).mean();value=critic((bo[idx]-actor.mean)/actor.denom);vloss=.5*(value-br[idx]).square().mean();anchorloss=.02*((mu-anchor_mu[idx])/a.std).square().sum(-1).mean();loss=ploss+vloss+anchorloss
     opt.zero_grad();loss.backward();torch.nn.utils.clip_grad_norm_(list(actor.parameters())+list(critic.parameters()),1.);opt.step();losses.append(float(loss.detach()));kls.append(float((blp[idx]-newlp).mean().detach()))
    if np.mean(kls[-max(1,B//256):])>.03:break
   recent=(recent+newinfos)[-100:];elapsed=time.monotonic()-t0
   row={'iteration':iteration+1,'steps':total,'elapsed_wall_s':elapsed,'control_steps_per_wall_s':(total-start*a.envs*a.steps)/elapsed,'rollout_mean_reward':float(re.mean()),'loss':float(np.mean(losses)),'approx_kl':float(np.mean(kls)),'episodes_this_iteration':len(newinfos),'recent_episodes':len(recent),'recent_return':float(np.mean([r['return']for r in recent]))if recent else None,'recent_fall_rate':float(np.mean([r['fall']for r in recent]))if recent else None,'recent_success_rate':float(np.mean([r['success']for r in recent]))if recent else None}
   row['per_gait_recent']={}
   for gait,command in GAITS.items():
    rr=[r for r in recent if np.allclose(r['command'],command)]
    if rr:row['per_gait_recent'][gait]={'n':len(rr),'success':float(np.mean([r['success']for r in rr])),'return':float(np.mean([r['return']for r in rr])),'velocity':np.mean([r['mean_body_velocity']for r in rr],axis=0).tolist()}
   print(json.dumps(row),flush=True);log.write(json.dumps(row)+'\n')
   if (iteration+1)%10==0 or iteration+1==a.iterations or (out/'STOP').exists():
    ck={'actor':actor.state_dict(),'critic':critic.state_dict(),'optimizer':opt.state_dict(),'torch_rng':torch.get_rng_state(),'iteration':iteration+1,'steps':total,'config':config};torch.save(ck,out/f'checkpoint_{iteration+1:05d}.pt');torch.save(ck,out/'latest.pt');export_actor(actor,out/'latest.onnx')
   if (out/'STOP').exists():break
 finally:
  for p in pipes:
   try:p.send(None)
   except (BrokenPipeError,EOFError):pass
  for pr in processes:pr.join(10)
  log.close()
if __name__=='__main__':main()
