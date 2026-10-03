"""Native 39-to-10 actor initialized exactly from upstream ONNX with absent head fixed.
Unused observation contributions are folded into the first-layer bias; no 14-action
policy is called during training/deployment. Frozen normalizer is part of export.
"""
import os
os.environ['ORT_DISABLE_TELEMETRY']='1'
import numpy as np,torch,torch.nn as nn,onnx
from pathlib import Path
from onnx import numpy_helper
from env import ROOT,LEG,OBS_KEEP,teacher_session,teacher_action
class Actor(nn.Module):
 def __init__(self):
  super().__init__();self.command_scale=nn.Parameter(torch.ones(2,3));self.command_bias=nn.Parameter(torch.zeros(2,3));self.register_buffer('mean',torch.zeros(39));self.register_buffer('denom',torch.ones(39));self.net=nn.Sequential(nn.Linear(39,512),nn.ELU(),nn.Linear(512,256),nn.ELU(),nn.Linear(256,128),nn.ELU(),nn.Linear(128,10))
 def forward(self,obs):
  cmd=obs[...,36:39];pos=torch.relu(cmd);neg=-torch.relu(-cmd)
  warped=pos*self.command_scale[0]+neg*self.command_scale[1]+torch.tanh(pos*100)*self.command_bias[0]+torch.tanh(neg*100)*self.command_bias[1]
  x=torch.cat([obs[...,:36],warped],dim=-1)
  return self.net((x-self.mean)/self.denom)
 def calibrated_init(self):
  with torch.no_grad():
   self.command_scale.copy_(torch.tensor([[1.55,3.,1.5],[1.,3.,1.5]]));self.command_bias.copy_(torch.tensor([[.115,.24,.4],[.21,.15,.4]]))
  return self
 @classmethod
 def from_teacher(cls,scales=(1,1,1)):
  g=onnx.load(ROOT/'policies/alpha_walking.onnx');w={i.name:numpy_helper.to_array(i).copy()for i in g.graph.initializer};a=cls()
  mean=w['obs_normalizer._mean'][0];den=w['onnx::Div_24'][0];drop=np.setdiff1d(np.arange(61),OBS_KEEP)
  with torch.no_grad():
   a.mean.copy_(torch.from_numpy(mean[OBS_KEEP]));a.denom.copy_(torch.from_numpy(den[OBS_KEEP]));
   for i in [0,2,4,6]:
    W=w[f'mlp.{i}.weight'];b=w[f'mlp.{i}.bias']
    if i==0:
     b=b+W[:,drop]@(-mean[drop]/den[drop]);W=W[:,OBS_KEEP]
    if i==6:W=W[LEG];b=b[LEG]
    a.net[i].weight.copy_(torch.from_numpy(W));a.net[i].bias.copy_(torch.from_numpy(b))
   # Scaling actual physical commands into the teacher's command operating range.
   for j,scale in enumerate(scales):
    idx=36+j
    # (scale*x - mean)/den = scale*(x-mean)/den + (scale-1)*mean/den
    old=a.net[0].weight[:,idx].clone();a.net[0].weight[:,idx]*=scale;a.net[0].bias+=old*(scale-1)*a.mean[idx]/a.denom[idx]
  return a
class Critic(nn.Module):
 def __init__(self):super().__init__();self.net=nn.Sequential(nn.Linear(39,128),nn.ELU(),nn.Linear(128,128),nn.ELU(),nn.Linear(128,1))
 def forward(self,x):return self.net(x).squeeze(-1)
def export_actor(a,path):
 a.eval();torch.onnx.export(a,torch.zeros(1,39),str(path),input_names=['obs'],output_names=['actions'],opset_version=17,dynamo=False,dynamic_axes={'obs':{0:'batch'},'actions':{0:'batch'}})
 m=onnx.load(path);props={'observation_layout':'gyro3,projected_gravity3,joint_delta10,joint_velocity10,previous_action10,velocity_command3','action_layout':'left_yaw,left_roll,left_pitch,left_knee,left_ankle,right_yaw,right_roll,right_pitch,right_knee,right_ankle','normalizer':'baked_into_graph','robot':'portrait Tab5 / head removed / 10 actuators','teacher_source_sha256':'e36332d383997d51401897734cd3e79cf5038406feddb18b4d57ecfb141daa6c'}
 onnx.helper.set_model_props(m,props);onnx.checker.check_model(m);onnx.save(m,path)
 import onnxruntime as ort
 opts=ort.SessionOptions();opts.intra_op_num_threads=1;opts.inter_op_num_threads=1
 session=ort.InferenceSession(str(path),sess_options=opts,providers=['CPUExecutionProvider'])
 sample=np.random.default_rng(420).normal(0,.1,(16,39)).astype(np.float32);sample[:,5]=-1.
 with torch.no_grad():expected=a(torch.from_numpy(sample)).numpy()
 actual=session.run(None,{'obs':sample})[0]
 assert actual.shape==(16,10) and np.all(np.isfinite(actual))
 np.testing.assert_allclose(actual,expected,atol=2e-6,rtol=1e-5)

if __name__=='__main__':
 torch.set_num_threads(1);a=Actor.from_teacher((3,2,1));s=teacher_session();rng=np.random.default_rng(0);err=0
 for _ in range(100):
  obs=rng.normal(0,.1,39).astype(np.float32);obs[5]=-1.
  err=max(err,float(np.max(np.abs(a(torch.from_numpy(obs)).detach().numpy()-teacher_action(s,obs,(3,2,1))))))
 print('exact native10 initialization max abs error',err);assert err<1e-5
 export_actor(a,Path(__file__).with_name('native10_initial.onnx'))
