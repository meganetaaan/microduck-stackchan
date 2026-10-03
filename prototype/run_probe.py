"""Headless physics tests; pretrained transfer is explicitly NOT retraining.

The modified model has only ten real actuators. A diagnostic adapter zero-pads
the missing head joint observations at the old reference pose and drops the four
head actions. No head joints, hidden actuators, anchors, or applied body forces
are introduced. This is an intentionally naive zero-shot transfer test.
"""
import argparse
import csv
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("MESA_SHADER_CACHE_DIR", "/tmp/microduck-mesa-cache")
import imageio.v2 as imageio
import mujoco
import numpy as np
import onnxruntime as ort
from PIL import Image, ImageDraw

ROOT=Path(__file__).resolve().parent
UP=ROOT.parent/"microduck_rl"
spec=importlib.util.spec_from_file_location("upstream_infer",UP/"scripts/infer_policy.py")
up=importlib.util.module_from_spec(spec);spec.loader.exec_module(up)
LEG=np.array([0,1,2,3,4,9,10,11,12,13])

def run(scene, policy, out, seconds=10, seed=0, speed=.10, noise=0.0, render=False, voltage=7.4, current_limit=None, save_states=False):
    out=Path(out);out.parent.mkdir(parents=True,exist_ok=True)
    bm=up.load_bam_model(200.,voltage,current_limit)
    m,d,ctrl,names=up.load_mujoco_with_bam(str(scene),bm,.005,None,6.0)
    assert m.nu in (10,14)
    assert m.opt.gravity[2] == -9.81
    assert m.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE
    assert np.all(m.actuator_forcelimited)
    mujoco.mj_resetDataKeyframe(m,d,m.key("STAND").id)
    # Match the upstream CPU inference entry state, rather than the viewer keyframe.
    d.qpos[2]=.125
    indices=LEG if m.nu==10 else np.arange(14)
    qidx=np.array([m.jnt_qposadr[m.actuator_trnid[i,0]] for i in range(m.nu)])
    vidx=np.array([m.jnt_dofadr[m.actuator_trnid[i,0]] for i in range(m.nu)])
    rng=np.random.default_rng(seed)
    d.qpos[qidx]+=rng.normal(0,noise,m.nu)
    default=up.DEFAULT_POSE[indices]
    d.qpos[qidx]=default+rng.normal(0,noise,m.nu)
    ctrl.reset(d.qpos)
    ctrl.q_target[:]=default
    mujoco.mj_forward(m,d)
    initial_com=d.subtree_com[m.body("trunk_base").id].copy()
    initial_pos=d.qpos[:3].copy();last=np.zeros(14,dtype=np.float32)
    session=None
    if policy:
        opt=ort.SessionOptions();opt.intra_op_num_threads=1;opt.inter_op_num_threads=1
        session=ort.InferenceSession(str(policy),sess_options=opt,providers=["CPUExecutionProvider"])
        assert session.get_inputs()[0].shape==[1,61]
        assert session.get_outputs()[0].shape==[1,14]
        assert "projected_gravity" in session.get_modelmeta().custom_metadata_map["observation_names"]
    sensors={n:(int(m.sensor_adr[m.sensor(n).id]),int(m.sensor_dim[m.sensor(n).id])) for n in ["imu_ang_vel","imu_accel"]}
    feet={m.geom("left_foot_collision").id,m.geom("right_foot_collision").id};floor=m.geom("floor").id
    rows=[];fall=None;max_torque=0.;nonfoot=False;cycles={f:0 for f in feet};previous={f:True for f in feet}
    states=[d.qpos.copy()];shell_leg_contacts=0;minimum_shell_contact_distance=None
    shell_ids={g for g in range(m.ngeom) if m.geom(g).name.startswith(("cube_", "rounded_shell_collision_")) or m.geom(g).name=="tab5_collision"}
    guard_ids={g for g in range(m.ngeom) if m.geom(g).name.startswith("shell_guard_legmesh_")}
    renderer=None;writer=None
    if render:
        renderer=mujoco.Renderer(m,height=360,width=640);cam=mujoco.MjvCamera();cam.azimuth=125;cam.elevation=-16;cam.distance=.63
        writer=imageio.get_writer(str(out)+".mp4",fps=25,codec="libx264",quality=7)
    for step in range(round(seconds/.02)):
        if session:
            gstart,gnum=sensors["imu_ang_vel"];astart,anum=sensors["imu_accel"]
            gyro=d.sensordata[gstart:gstart+gnum].copy()
            # ONNX metadata specifies projected gravity, NOT raw accelerometer.
            accel=d.xmat[m.body("trunk_base").id].reshape(3,3).T@np.array([0,0,-1.])
            q=np.zeros(14);v=np.zeros(14)
            q[indices]=d.qpos[qidx]-default;v[indices]=d.qvel[vidx]
            command=np.zeros(13);command[0]=speed*min(1.,step*.02/.5)
            obs=np.concatenate([gyro,accel,q,v,last,command]).astype(np.float32)[None]
            action=session.run(None,{session.get_inputs()[0].name:obs})[0][0]
            assert np.all(np.isfinite(action))
            last[:]=0;last[indices]=action[indices]
            ctrl.q_target[:]=default+action[indices]
        for sub in range(4):
            ctrl.update();mujoco.mj_step(m,d)
            max_torque=max(max_torque,float(np.max(np.abs(d.actuator_force))))
            for c in d.contact[:d.ncon]:
                pair={int(c.geom1),int(c.geom2)}
                if pair & shell_ids and pair & guard_ids:
                    shell_leg_contacts+=1
                    minimum_shell_contact_distance=float(c.dist) if minimum_shell_contact_distance is None else min(minimum_shell_contact_distance,float(c.dist))
        mujoco.mj_forward(m,d)
        tilt=math.degrees(math.acos(np.clip(d.xmat[m.body("trunk_base").id].reshape(3,3)[2,2],-1,1)))
        contacts=set()
        for c in d.contact[:d.ncon]:
            pair={int(c.geom1),int(c.geom2)}
            if floor in pair:
                other=next(iter(pair-{floor}))
                if other in feet:contacts.add(other)
                else:nonfoot=True
        for f in feet:
            if previous[f] and f not in contacts:cycles[f]+=1
            previous[f]=f in contacts
        row={"time_s":float(d.time),"x_m":float(d.qpos[0]),"y_m":float(d.qpos[1]),"z_m":float(d.qpos[2]),"tilt_deg":tilt,"left_contact":int(m.geom("left_foot_collision").id in contacts),"right_contact":int(m.geom("right_foot_collision").id in contacts),"max_abs_torque_nm":float(np.max(np.abs(d.actuator_force)))};rows.append(row)
        if not np.all(np.isfinite(d.qpos)) or not np.all(np.isfinite(d.qvel)):fall="nonfinite"
        elif tilt>60:fall="tilt_over_60_deg"
        elif d.qpos[2]<.055:fall="trunk_below_55_mm"
        if renderer and step%2==0:
            cam.lookat[:]=[d.qpos[0],d.qpos[1],.14]
            renderer.update_scene(d,camera=cam);im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im)
            draw.rectangle((0,0,800,46),fill=(12,18,24))
            mode="original policy" if m.nu==14 else "10-DOF / zero-shot adapter / NOT retrained"
            draw.text((12,8),f"{Path(scene).stem} | {mode if session else 'fixed pose hold, no gait'}",fill="white")
            draw.text((12,26),f"t={d.time:.2f}s  tilt={tilt:.1f} deg  {('FALL: '+fall) if fall else ''}",fill="white")
            writer.append_data(np.array(im))
            if step==0:im.save(str(out)+"_initial.png")
        if fall:break
        if save_states:states.append(d.qpos.copy())
    if writer:writer.close();renderer.close()
    if save_states:np.savez_compressed(str(out)+"_states.npz",qpos=np.array(states),dt=.02)
    with open(str(out)+".csv","w") as f:
        w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
    elapsed=float(d.time);dx=float(d.qpos[0]-initial_pos[0]);dy=float(d.qpos[1]-initial_pos[1])
    result={"scene":str(scene),"controller":"fixed_reference_pose" if policy is None else ("official_61D_14D_policy" if m.nu==14 else "naive_zero_padded_61D_14D_policy_adapter_to_10_actuators"),"policy":str(policy) if policy else None,"policy_sha256":hashlib.sha256(Path(policy).read_bytes()).hexdigest() if policy else None,"seed":seed,"joint_noise_std_rad":noise,"command_vx_m_s":speed if policy else 0,"requested_horizon_s":seconds,"simulated_s":elapsed,"fall_reason":fall,"fall_time_s":elapsed if fall else None,"forward_displacement_m":dx,"lateral_displacement_m":dy,"mean_forward_speed_m_s":dx/elapsed,"max_tilt_deg":max(r['tilt_deg'] for r in rows),"final_tilt_deg":rows[-1]['tilt_deg'],"min_trunk_height_m":min(r['z_m'] for r in rows),"nonfoot_floor_contact":nonfoot,"left_liftoffs":cycles[m.geom("left_foot_collision").id],"right_liftoffs":cycles[m.geom("right_foot_collision").id],"max_abs_actuator_torque_nm":max_torque,"actuator_force_limit_nm":float(m.actuator_forcerange[0,1]),"actuators":m.nu,"total_mass_kg":float(sum(m.body_mass)),"initial_com_m":initial_com.tolist(),"mujoco_version":mujoco.__version__,"ort_version":ort.__version__,"dt_s":.005,"control_hz":50,"gravity_m_s2":m.opt.gravity.tolist(),"bam_model":"xl330/m6 pinned upstream lock","voltage_v":voltage,"current_limit_a":current_limit,"notes":"Free floating body, real contact/friction, finite BAM torque limits, no external wrench, no pose overwrite during simulation. Deterministic nominal dynamics, not hardware validation."}
    result.update(shell_leg_contact_samples=shell_leg_contacts,minimum_shell_contact_distance_m=minimum_shell_contact_distance,shell_contact_check_hz=200)
    Path(str(out)+".json").write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));return result

if __name__=="__main__":
    ap=argparse.ArgumentParser();ap.add_argument("--scene",type=Path,required=True);ap.add_argument("--policy",type=Path);ap.add_argument("--out",type=Path,required=True);ap.add_argument("--seconds",type=float,default=10);ap.add_argument("--seed",type=int,default=0);ap.add_argument("--speed",type=float,default=.1);ap.add_argument("--noise",type=float,default=0);ap.add_argument("--render",action="store_true");ap.add_argument("--voltage",type=float,default=7.4);ap.add_argument("--current-limit",type=float);ap.add_argument("--save-states",action="store_true");args=ap.parse_args();run(args.scene,args.policy,args.out,args.seconds,args.seed,args.speed,args.noise,args.render,args.voltage,args.current_limit,args.save_states)
