// BAM M6 / XL330 controller port of bam/{actuator,model,mujoco}.py.
// Copyright 2025 Marc Duclusaud & Grégoire Passault. Apache-2.0.
// Adaptation: scalar JavaScript implementation, preserving update ordering.
import {clamp} from './control.js';
export function bamFriction(p,motor,external,dq){
 const stribeck=Math.exp(-Math.pow(Math.abs(dq/p.dtheta_stribeck),p.alpha));
 const gearbox=Math.abs(external*p.load_friction_external-motor*p.load_friction_motor);
 const gearboxS=Math.abs(external*p.load_friction_external_stribeck-motor*p.load_friction_motor_stribeck);
 let loss=p.friction_base+gearbox+stribeck*p.friction_stribeck+gearboxS*stribeck;
 if(Math.sign(external)!==Math.sign(motor)){
  if(Math.abs(external)<Math.abs(motor))loss+=stribeck*p.load_friction_external_quad*Math.abs(external)**2;
  else if(Math.abs(external)>Math.abs(motor))loss+=stribeck*p.load_friction_motor_quad*Math.abs(motor)**2;
 }
 return [loss,p.friction_viscous];
}
export class Physics{
 constructor(mj,model,config){this.mj=mj;this.model=model;this.data=new mj.MjData(model);this.config=config;this.p=config.bam.parameters;this.a=config.bam.actuator;this.last=new Float32Array(10);this.target=new Float64Array(10);this.shell=new Set(config.shell_geom_ids);this.guard=new Set(config.guard_geom_ids);this.feet=new Set(config.foot_geom_ids);this.reset();}
 reset(jointNoise=null){const {model:m,data:d,mj,config:c}=this;for(const k of c.dof_indices){m.dof_frictionloss[k]=0;m.dof_damping[k]=0;}mj.mj_resetDataKeyframe(m,d,c.stand_key_id);d.qpos[2]=.125;for(let i=0;i<10;i++){d.qpos[c.qpos_indices[i]]=c.default_joint_pos_float32[i]+(jointNoise?.[i]??0);this.target[i]=c.default_joint_pos_float32[i];}this.last.fill(0);this.steps=0;this.maxTorque=0;this.fallen=false;this.unsafeContact=false;this.nonfoot=false;this.lastCommand=[0,0,0];mj.mj_forward(m,d);return this.observation();}
 observation(command=this.lastCommand){const d=this.data,c=this.config,obs=new Float32Array(39),r=d.xmat,b=9*c.trunk_body_id;for(let i=0;i<3;i++)obs[i]=d.sensordata[c.gyro_sensor_adr+i];obs[3]=-r[b+6];obs[4]=-r[b+7];obs[5]=-r[b+8];for(let i=0;i<10;i++){obs[6+i]=d.qpos[c.qpos_indices[i]]-c.default_joint_pos_float32[i];obs[16+i]=d.qvel[c.dof_indices[i]];obs[26+i]=this.last[i];}const ramp=Math.min(1,this.steps*.02/.5);for(let i=0;i<3;i++)obs[36+i]=Math.fround(Math.fround(command[i])*Math.fround(ramp));return obs;}
 bamUpdate(){const d=this.data,m=this.model,c=this.config,p=this.p,a=this.a;for(let i=0;i<10;i++){const qi=c.qpos_indices[i],vi=c.dof_indices[i],ji=c.joint_indices[i],ai=c.actuator_indices[i],q=d.qpos[qi],dq=d.qvel[vi];let duty=(this.target[i]-q)*a.kp*a.error_gain;if(a.max_current!==null&&a.max_current!==undefined){const center=p.kt*dq/a.vin,span=p.R*a.max_current/a.vin;duty=clamp(duty,center-span,center+span);}const volts=a.vin*clamp(duty,-a.max_pwm,a.max_pwm);d.ctrl[ai]=p.kt*volts/p.R-p.kt**2*dq/p.R;let friction=0;for(let k=0;k<d.nefc;k++)if(d.efc_id[k]===ji&&d.efc_type[k]===1)friction+=d.efc_force[k];const ext=-d.qfrc_bias[vi]+d.qfrc_constraint[vi]-friction;const [loss,damping]=bamFriction(p,d.qfrc_actuator[vi],ext,dq);m.dof_frictionloss[vi]=loss;m.dof_damping[vi]=damping;}}
 applyAction(action){if(action.length!==10||!Array.from(action).every(Number.isFinite))throw new Error('Policy produced invalid actions');for(let i=0;i<10;i++){const v=Math.fround(clamp(action[i],-1.2,1.2));this.target[i]=Math.fround(this.config.default_joint_pos_float32[i]+v);} }
 substep(){this.bamUpdate();this.mj.mj_step(this.model,this.data);const d=this.data;for(let i=0;i<10;i++)this.maxTorque=Math.max(this.maxTorque,Math.abs(d.actuator_force[i]));for(let k=0;k<d.ncon;k++){const contact=d.contact.get(k),a=contact.geom1,b=contact.geom2;if(contact.dist<-.0005&&((this.shell.has(a)&&this.guard.has(b))||(this.shell.has(b)&&this.guard.has(a))))this.unsafeContact=true;if(contact.dist<0){if(a===this.config.floor_geom_id&&!this.feet.has(b))this.nonfoot=true;if(b===this.config.floor_geom_id&&!this.feet.has(a))this.nonfoot=true;}}}
 finishStep(action){for(let i=0;i<10;i++)this.last[i]=clamp(action[i],-1.2,1.2);this.steps++;this.mj.mj_forward(this.model,this.data);const d=this.data,r=d.xmat,c=this.config,tilt=Math.acos(clamp(r[c.trunk_body_id*9+8],-1,1));this.fallen=!Array.from(d.qpos).every(Number.isFinite)||tilt>55*Math.PI/180||d.qpos[2]<.075||this.nonfoot;}
 step(action,command=this.lastCommand,onSubstep=null){this.lastCommand=Array.from(command);this.maxTorque=0;this.nonfoot=false;this.applyAction(action);for(let k=0;k<4;k++){this.substep();if(onSubstep)onSubstep(k,this);}this.finishStep(action);return this.telemetry();}
 telemetry(){const d=this.data,c=this.config,r=d.xmat,o=9*c.trunk_body_id;return {time:d.time,steps:this.steps,x:d.qpos[0],y:d.qpos[1],z:d.qpos[2],vx:r[o]*d.qvel[0]+r[o+3]*d.qvel[1]+r[o+6]*d.qvel[2],vy:r[o+1]*d.qvel[0]+r[o+4]*d.qvel[1]+r[o+7]*d.qvel[2],wz:d.sensordata[c.gyro_sensor_adr+2],tilt:Math.acos(clamp(r[o+8],-1,1))*180/Math.PI,torque:this.maxTorque,fallen:this.fallen,unsafeContact:this.unsafeContact};}
 transforms(ids){const d=this.data,out=new Float32Array(ids.length*12);for(let i=0;i<ids.length;i++){const g=ids[i];out.set(d.geom_xpos.subarray(3*g,3*g+3),i*12);out.set(d.geom_xmat.subarray(9*g,9*g+9),i*12+3);}return out;}
 dispose(){this.data.delete();this.model.delete();}
}
