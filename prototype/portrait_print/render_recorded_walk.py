"""Faithful video replay of an already-completed full-physics MuJoCo rollout.

No controller/physics outcome is generated here. The original full-model run,
source trace hash and run metadata are retained. Static render-only mesh copy
omits invisible collider meshes solely to reduce renderer load.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl');os.environ.setdefault('MESA_SHADER_CACHE_DIR','/tmp/microduck-mesa-cache')
from pathlib import Path
import json,hashlib,csv
import mujoco,numpy as np,imageio.v2 as imageio
from PIL import Image,ImageDraw
HERE=Path(__file__).resolve().parent
m=mujoco.MjModel.from_xml_path(str(HERE/'_render_only_scene.xml'));d=mujoco.MjData(m);states=np.load(HERE/'walk_10s_states.npz')['qpos'];source=json.loads((HERE/'walk_10s.json').read_text());assert source['fall_reason']is None;assert abs(source['total_mass_kg']-sum(m.body_mass))<1e-10
r=mujoco.Renderer(m,width=640,height=368);cam=mujoco.MjvCamera();cam.azimuth=125;cam.elevation=-16;cam.distance=.63
with imageio.get_writer(HERE/'walking_demo.mp4',fps=25,codec='libx264',quality=7)as w:
 for i in range(150):
  index=2*i;d.qpos[:]=states[index];mujoco.mj_forward(m,d);cam.lookat[:]=[d.qpos[0],d.qpos[1],.14];r.update_scene(d,camera=cam);r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=0;im=Image.fromarray(r.render());draw=ImageDraw.Draw(im);draw.rectangle((0,0,640,46),fill=(12,18,24));draw.text((10,8),'Recorded MuJoCo physics | portrait print concept | not retrained',fill='white');draw.text((10,26),f't={index*.02:.2f}s | replay of completed full-model policy rollout',fill='white');w.append_data(np.array(im))
  if i==75:im.save(HERE/'walking_frame3s.png')
r.close()
rows=list(csv.DictReader((HERE/'walk_10s.csv').open()))[:300]
with (HERE/'walking_demo.csv').open('w')as f:
 w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
np.savez_compressed(HERE/'walking_demo_states.npz',qpos=states[:301],dt=.02)
result={'video_kind':'faithful_replay_of_completed_full_physics_rollout','source_trace':'walk_10s_states.npz','source_trace_sha256':hashlib.sha256((HERE/'walk_10s_states.npz').read_bytes()).hexdigest(),'source_run':'walk_10s.json','full_physics_model_sha256':hashlib.sha256((HERE/'tab5_portrait_print.xml').read_bytes()).hexdigest(),'duration_s':6,'frame_count':150,'fps':25,'source_state_hz':50,'rendered_indices':'0,2,...,298','forward_displacement_m_6s':float(rows[-1]['x_m'])-float(states[0,0]),'lateral_displacement_m_6s':float(rows[-1]['y_m'])-float(states[0,1]),'max_tilt_deg_6s':max(float(q['tilt_deg'])for q in rows),'fall_reason':None,'total_mass_kg':source['total_mass_kg'],'source_runtime_privacy':source['runtime_privacy'],'note':'Only visualization uses the render-only model; source policy rollout used full collisions, gravity and finite BAM torques. No invented trajectory or pose is used.'}
(HERE/'walking_demo.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
