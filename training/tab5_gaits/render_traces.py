"""Render recorded dynamics only. Simulation outcome comes from evaluate.py.
Replay sets poses solely for visualization of already-completed physics traces.
"""
import os
os.environ['MUJOCO_GL']='egl';os.environ['MESA_SHADER_CACHE_DIR']='/tmp/tab5-training-mesa';os.environ['ORT_DISABLE_TELEMETRY']='1'
import argparse,json,hashlib
from pathlib import Path
import mujoco,numpy as np,imageio.v2 as imageio
from PIL import Image,ImageDraw
from env import GAITS
p=argparse.ArgumentParser();p.add_argument('--scene',required=True);p.add_argument('--traces',required=True);p.add_argument('--out',required=True);p.add_argument('--clip-seconds',type=float,default=6);a=p.parse_args();root=Path(a.traces);out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
m=mujoco.MjModel.from_xml_path(a.scene);d=mujoco.MjData(m);r=mujoco.Renderer(m,width=640,height=368);cam=mujoco.MjvCamera();cam.azimuth=125;cam.elevation=-16;cam.distance=.68;opt=mujoco.MjvOption();opt.geomgroup[3]=0;manifest=[]
with imageio.get_writer(out/'trained_gaits.mp4',fps=25,codec='libx264',quality=7)as writer:
 for name in GAITS:
  matches=sorted(root.glob(f'{name}_seed*_trace.npz'))
  if not matches:continue
  trace=matches[0]
  states=np.load(trace)['qpos'];count=min(len(states),round(a.clip_seconds/.02));manifest.append({'gait':name,'trace':str(trace),'sha256':hashlib.sha256(trace.read_bytes()).hexdigest(),'source_frames':count})
  for i in range(0,count,2):
   d.qpos[:]=states[i];mujoco.mj_forward(m,d);cam.lookat[:]=[d.qpos[0],d.qpos[1],.14];r.update_scene(d,camera=cam,scene_option=opt);r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=0;im=Image.fromarray(r.render());draw=ImageDraw.Draw(im);draw.rectangle((0,0,640,46),fill=(12,18,24));draw.text((10,7),f'Tab5 assembly | trained 10-DOF MuJoCo policy | {name}',fill='white');draw.text((10,25),f'Actual dynamics trace replay | t={i*.02:.2f}s | simulation, not hardware proof',fill='white');writer.append_data(np.array(im))
   if i==min(150,count-2):im.save(out/f'{name}.png')
r.close();(out/'video_provenance.json').write_text(json.dumps({'source_scene':a.scene,'scene_sha256':hashlib.sha256(Path(a.scene).read_bytes()).hexdigest(),'clips':manifest,'note':'All frames replay qpos recorded by real MuJoCo+BAM simulation; no invented motion'},indent=2))
