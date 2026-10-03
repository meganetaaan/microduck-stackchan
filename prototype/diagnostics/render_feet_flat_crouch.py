from pathlib import Path
import os
os.environ.setdefault('MUJOCO_GL','egl');os.environ.setdefault('MESA_SHADER_CACHE_DIR','/tmp/microduck-mesa-cache')
import mujoco,numpy as np
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parents[1];poses=np.load(ROOT/'diagnostics/feet_flat_crouch_paths.npz')
canvas=Image.new('RGB',(1920,540),(14,20,27));draw=ImageDraw.Draw(canvas)
items=[(ROOT/'models/scene_tab5_lowcube.xml','CURRENT / 90 deg knee: side-shell contact'),(ROOT/'diagnostics/scene_lowcube_cutout_candidate.xml','CUTOUT OPTION / 90 deg knee: shell clear'),(ROOT/'diagnostics/scene_lowcube_cutout_candidate.xml','CUTOUT OPTION / 80 deg knee: margin to limit')]
for col,(scene,title) in enumerate(items):
 m=mujoco.MjModel.from_xml_path(str(scene));d=mujoco.MjData(m);d.qpos[:]=poses['crouch_80deg_qpos' if col==2 else 'crouch_90deg_qpos'];mujoco.mj_forward(m,d)
 if col==0:
  for g in range(m.ngeom):
   if m.geom(g).name in ['cube_side_left_collision','cube_side_right_collision']:m.geom_rgba[g]=[1,.3,.2,.6]
 r=mujoco.Renderer(m,height=480,width=640);cam=mujoco.MjvCamera();cam.azimuth=125;cam.elevation=-10;cam.distance=.46;cam.lookat[:]=[0,0,.075];r.update_scene(d,camera=cam);im=Image.fromarray(r.render());canvas.paste(im,(col*640,60));r.close()
 draw.text((col*640+12,8),title,fill='white');draw.text((col*640+12,27),'Feet flat, same legs + Tab5; kinematic clearance only',fill=(161,221,230));draw.text((col*640+12,43),'No seated-balance or dynamic transition claim',fill=(220,220,220))
canvas.save(ROOT/'diagnostics/feet_flat_crouch_comparison.png')
