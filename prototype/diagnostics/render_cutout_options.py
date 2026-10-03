from pathlib import Path
import os
os.environ.setdefault('MUJOCO_GL','egl');os.environ.setdefault('MESA_SHADER_CACHE_DIR','/tmp/microduck-mesa-cache')
import mujoco,numpy as np
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parents[1]
canvas=Image.new('RGB',(1920,1060),(14,20,27));draw=ImageDraw.Draw(canvas)
for row,(fn,title) in enumerate([(ROOT/'models/scene_tab5_lowcube.xml','CURRENT: continuous chin + low side window'),(ROOT/'diagnostics/scene_lowcube_cutout_candidate.xml','OPTION A: fixed cutouts, unchanged Tab5 + legs')]):
 m=mujoco.MjModel.from_xml_path(str(fn));d=mujoco.MjData(m);renderer=mujoco.Renderer(m,height=480,width=640)
 for col,(pose,az,desc) in enumerate([('STAND',125,'Standing silhouette'),('SIT',165,'Old SIT envelope; not validated sitting'),('FOLD',105,'Folding envelope; not balance-tested')]):
  mujoco.mj_resetDataKeyframe(m,d,m.key(pose).id);mujoco.mj_forward(m,d)
  # Whole-body translation for a clear ground-referenced inspection, not a dynamic result.
  zmin=1e6
  for n in ['left_foot_collision','right_foot_collision']:
   g=m.geom(n).id;mi=m.geom_dataid[g];st=m.mesh_vertadr[mi];num=m.mesh_vertnum[mi];v=m.mesh_vert[st:st+num]@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g];zmin=min(zmin,v[:,2].min())
  d.qpos[2]+=.003-zmin;mujoco.mj_forward(m,d)
  rgba=m.geom_rgba.copy()
  if row==0:
   for g in range(m.ngeom):
    n=m.geom(g).name
    if pose=='SIT' and n=='cube_chin_collision':m.geom_rgba[g]=[1,.3,.2,.65]
    if pose=='FOLD' and n in ['cube_side_left_collision','cube_side_right_collision']:m.geom_rgba[g]=[1,.3,.2,.65]
  cam=mujoco.MjvCamera();cam.azimuth=az;cam.elevation=-12;cam.distance=.50;cam.lookat[:]=[0,0,.10]
  renderer.update_scene(d,camera=cam);im=Image.fromarray(renderer.render());canvas.paste(im,(640*col,530*row+50));m.geom_rgba[:]=rgba
  draw.text((640*col+12,530*row+9),title,fill='white');draw.text((640*col+12,530*row+28),desc,fill=(161,221,230))
 renderer.close()
canvas.save(ROOT/'diagnostics/cutout_candidate_comparison.png')
