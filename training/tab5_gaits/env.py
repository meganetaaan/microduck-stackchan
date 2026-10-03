"""Full-contact CPU MuJoCo/BAM training environment, native 39D obs/10D action.
No externally applied body wrench, no pose rewriting outside reset.
"""
import os
os.environ['ORT_DISABLE_TELEMETRY']='1'
os.environ.setdefault('OMP_NUM_THREADS','1')
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
import sys, importlib.util, contextlib, io, math, hashlib, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np, mujoco
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'bam'))
spec=importlib.util.spec_from_file_location('upstream_infer',ROOT/'microduck_rl/scripts/infer_policy.py')
up=importlib.util.module_from_spec(spec);spec.loader.exec_module(up)
LEG=np.array([0,1,2,3,4,9,10,11,12,13]); OBS_KEEP=np.r_[np.arange(6),6+LEG,20+LEG,34+LEG,np.arange(48,51)]
GAITS={'stand':[0,0,0], 'forward_slow':[.075,0,0], 'forward':[.12,0,0], 'backward':[-.15,0,0], 'left':[0,.07,0], 'right':[0,-.055,0], 'turn_left':[0,0,.7], 'turn_right':[0,0,-.7]}
class Env:
 def __init__(self,scene,seed=0,horizon=8,randomize=False,yaw_variance=.25):
  self.rng=np.random.default_rng(seed); self.seed=seed; self.horizon=round(horizon/.02); self.randomize=randomize;self.yaw_variance=yaw_variance
  with contextlib.redirect_stdout(io.StringIO()):
   bm=up.load_bam_model(200.,7.4,None); self.m,self.d,self.ctrl,self.names=up.load_mujoco_with_bam(str(scene),bm,.005,0.,6.)
  m=self.m
  assert m.nu==10 and m.jnt_type[0]==mujoco.mjtJoint.mjJNT_FREE
  assert m.opt.gravity[2]==-9.81
  self.qidx=m.jnt_qposadr[m.actuator_trnid[:,0]];self.vidx=m.jnt_dofadr[m.actuator_trnid[:,0]]
  self.default=up.DEFAULT_POSE[LEG]; self.trunk=m.body('trunk_base').id;self.floor=m.geom('floor').id
  self.feet=[m.geom('left_foot_collision').id,m.geom('right_foot_collision').id]
  self.gyro_start=m.sensor_adr[m.sensor('imu_ang_vel').id]
  self.shell_ids={i for i in range(m.ngeom)if m.geom(i).name.startswith(('cube_','rounded_shell_collision_')) or m.geom(i).name=='tab5_collision'}
  self.guard_ids={i for i in range(m.ngeom)if m.geom(i).name.startswith('shell_guard_legmesh_')}
  self.mass0=m.body_mass.copy();self.inertia0=m.body_inertia.copy();self.friction0=m.geom_friction.copy()
  self.mass_low=self.mass0*.97;self.mass_high=self.mass0*1.03;self.mass_randomization_kind='global_mass_scale_0.97_1.03'
  low=Path(scene).parent/'tab5_assembly_uart_light.xml';high=Path(scene).parent/'tab5_assembly_uart_heavy.xml'
  if low.exists() and high.exists():
   self.mass_low=self.mass0.copy();self.mass_high=self.mass0.copy()
   for path,masses in [(low,self.mass_low),(high,self.mass_high)]:
    for body in ET.parse(path).getroot().iter('body'):
     inertial=body.find('inertial');name=body.get('name')
     if inertial is not None and name:
      try:masses[m.body(name).id]=float(inertial.get('mass'))
      except KeyError:pass
   self.mass_randomization_kind='interpolate_explicit_component_light_heavy_variants_preserve_original_legs_Tab5'
  self.base_model_sha256=self.model_hash();self.reset()
 def model_hash(self):
  buffer=np.empty(mujoco.mj_sizeModel(self.m),dtype=np.uint8);mujoco.mj_saveModel(self.m,buffer=buffer);return hashlib.sha256(buffer.tobytes()).hexdigest()
 def reset(self,command=None,noise=None):
  m,d=self.m,self.d
  mujoco.mj_resetDataKeyframe(m,d,m.key('STAND').id)
  if self.randomize:
   blend=self.rng.uniform(0,1);m.body_mass[:]=self.mass_low+blend*(self.mass_high-self.mass_low)
   ratio=np.divide(m.body_mass,self.mass0,out=np.ones_like(self.mass0),where=self.mass0>0);m.body_inertia[:]=self.inertia0*ratio[:,None]
   m.geom_friction[:]=self.friction0;m.geom_friction[:,0]*=self.rng.uniform(.85,1.15)
   self.ctrl.model.actuator.vin=self.rng.uniform(6.8,8.0);self.ctrl.vin_drop_gain=self.rng.uniform(0,.1)
   limit=self.ctrl.model.actuator.vin*self.ctrl.model.kt.value/self.ctrl.model.R.value
   m.actuator_forcerange[:,0]=-limit;m.actuator_forcerange[:,1]=limit
   mujoco.mj_setConst(m,d)
  d.qpos[2]=.125
  d.qpos[self.qidx]=self.default+self.rng.normal(0,.006 if noise is None else noise,10)
  self.ctrl.reset(d.qpos);self.ctrl.last_ts=0.;self.ctrl.q_target[:]=self.default
  mujoco.mj_forward(m,d)
  self.command=np.array(command if command is not None else list(GAITS.values())[self.rng.integers(len(GAITS))],dtype=np.float32)
  self.last=np.zeros(10,np.float32);self.steps=0;self.start=d.qpos[:3].copy();self.prev_contacts=np.ones(2,bool);self.liftoffs=np.zeros(2,int)
  self.self_contact_samples=0;self.minimum_self_contact_distance=None;self.unsafe_self_contact=False;self.filtered_velocity=np.zeros(3);self.episode_return=0.;self.rows=[];self.states=[d.qpos.copy()];self.yaw_unwrapped=0.;self.prev_yaw=0.
  return self.obs()
 def obs(self):
  d=self.d;R=d.xmat[self.trunk].reshape(3,3)
  cmd=self.command*min(1.,self.steps*.02/.5)
  return np.r_[d.sensordata[self.gyro_start:self.gyro_start+3],R.T@np.array([0,0,-1.]),d.qpos[self.qidx]-self.default,d.qvel[self.vidx],self.last,cmd].astype(np.float32)
 def step(self,action,record=False):
  m,d=self.m,self.d; action=np.asarray(action); old=self.last.copy()
  self.ctrl.q_target[:]=self.default+np.clip(action,-1.2,1.2)
  max_tau=0.;nonfoot=False
  for _ in range(4):
   self.ctrl.update();mujoco.mj_step(m,d);max_tau=max(max_tau,float(np.max(np.abs(d.actuator_force))))
   for c in d.contact[:d.ncon]:
    pair={int(c.geom1),int(c.geom2)}
    if pair&self.shell_ids and pair&self.guard_ids:
     self.self_contact_samples+=1;self.minimum_self_contact_distance=float(c.dist)if self.minimum_self_contact_distance is None else min(self.minimum_self_contact_distance,float(c.dist))
     self.unsafe_self_contact|=c.dist<-.0005
    if c.dist<0 and (int(c.geom1)==self.floor or int(c.geom2)==self.floor):
     other=int(c.geom2) if int(c.geom1)==self.floor else int(c.geom1)
     if other not in self.feet:nonfoot=True
  mujoco.mj_forward(m,d)
  self.last=np.clip(action,-1.2,1.2).astype(np.float32);self.steps+=1
  R=d.xmat[self.trunk].reshape(3,3);tilt=math.acos(np.clip(R[2,2],-1,1));yaw=math.atan2(R[1,0],R[0,0]);self.yaw_unwrapped+=(yaw-self.prev_yaw+math.pi)%(2*math.pi)-math.pi;self.prev_yaw=yaw
  vel=R.T@d.qvel[:3];wz=d.sensordata[self.gyro_start+2]
  contacts=np.zeros(2,bool)
  for c in d.contact[:d.ncon]:
   if c.dist<.001:
    pair=(int(c.geom1),int(c.geom2))
    if self.floor in pair:
     for i,f in enumerate(self.feet):contacts[i]|=f in pair
  self.liftoffs+=(self.prev_contacts&~contacts);self.prev_contacts=contacts
  cmd=self.command*min(1.,self.steps*.02/.5)
  self.filtered_velocity=.96*self.filtered_velocity+.04*np.r_[vel[:2],wz]
  linerr=float(np.sum((self.filtered_velocity[:2]-cmd[:2])**2));angerr=float((self.filtered_velocity[2]-cmd[2])**2)
  # All positive reward gated by upright/clearance; falling never a reward basin.
  fallen=not np.all(np.isfinite(d.qpos)) or tilt>math.radians(55) or d.qpos[2]<.075 or nonfoot
  reward=(1.5*math.exp(-linerr/.004)+.5*math.exp(-angerr/self.yaw_variance)+.4*math.exp(-tilt**2/.04) -.001*np.sum((action-old)**2)-.0001*np.sum(d.actuator_force**2)-.005*abs(vel[2]))*.02
  if fallen or self.unsafe_self_contact:reward=-2.
  self.episode_return+=reward
  done=fallen or self.unsafe_self_contact or self.steps>=self.horizon
  row={'t':float(d.time),'x':float(d.qpos[0]),'y':float(d.qpos[1]),'z':float(d.qpos[2]),'vx':float(vel[0]),'vy':float(vel[1]),'wz':float(wz),'yaw':self.yaw_unwrapped,'tilt_deg':math.degrees(tilt),'torque':max_tau,'nonfoot':nonfoot,'left_contact':bool(contacts[0]),'right_contact':bool(contacts[1]),'linear_error_sq':linerr,'yaw_error_sq':angerr}
  self.rows.append(row)
  if record:self.states.append(d.qpos.copy())
  info={}
  if done: info=self.metrics(fallen)
  return self.obs(),float(reward),done,info
 def metrics(self,fallen=False):
  rows=self.rows;warm=rows[min(50,len(rows)//3):] or rows;dt=self.steps*.02
  out={'fall':bool(fallen),'seconds':dt,'return':self.episode_return,'displacement':(self.d.qpos[:3]-self.start).tolist(),'mean_body_velocity':[float(np.mean([r[k] for r in warm]))for k in ['vx','vy','wz']],'linear_rmse':float(np.sqrt(np.mean([r['linear_error_sq']for r in warm]))),'yaw_rmse':float(np.sqrt(np.mean([r['yaw_error_sq']for r in warm]))),'max_tilt_deg':max(r['tilt_deg']for r in rows),'max_torque_nm':max(r['torque']for r in rows),'yaw_change_rad':self.yaw_unwrapped,'liftoffs':self.liftoffs.tolist(),'command':self.command.tolist()}
  measured=np.array(out['mean_body_velocity']);cmd=self.command
  moving=np.linalg.norm(cmd[:2])>.001
  directed=(float(measured[:2]@cmd[:2])>=.5*float(cmd[:2]@cmd[:2])) if moving else np.linalg.norm(measured[:2])<.015
  out['mean_linear_error']=float(np.linalg.norm(measured[:2]-cmd[:2]));out['mean_yaw_error']=float(abs(measured[2]-cmd[2]))
  displacement=np.array(out['displacement'][:2]);direction=cmd[:2]/max(np.linalg.norm(cmd[:2]),1e-9)
  out['cross_track_drift_m']=float(abs(displacement@np.array([-direction[1],direction[0]]))) if moving else float(np.linalg.norm(displacement))
  out['heading_drift_rad']=float(self.yaw_unwrapped-cmd[2]*dt)
  out['command_frame']='body-local velocity; cross-track measured relative to initial heading; yaw may drift'
  out['self_contact_samples']=self.self_contact_samples;out['minimum_self_contact_distance_m']=self.minimum_self_contact_distance;out['unsafe_self_contact']=bool(self.unsafe_self_contact)
  out['success']=bool(not fallen and not self.unsafe_self_contact and directed and out['mean_linear_error']<.025 and out['mean_yaw_error']<.15 and out['max_tilt_deg']<20)
  return out

def teacher_session():
 import onnxruntime as ort
 opt=ort.SessionOptions();opt.intra_op_num_threads=1;opt.inter_op_num_threads=1
 return ort.InferenceSession(str(ROOT/'policies/alpha_walking.onnx'),sess_options=opt,providers=['CPUExecutionProvider'])
def teacher_action(session,obs,scales=(1,1,1)):
 old=np.zeros((1,61),np.float32);old[0,OBS_KEEP]=obs;old[0,48:51]*=scales
 return session.run(None,{session.get_inputs()[0].name:old})[0][0,LEG]
