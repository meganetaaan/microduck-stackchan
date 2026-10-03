"""Actual-mesh matched interface views; display separation is labeled explicitly."""
import os
os.environ.setdefault('MUJOCO_GL','egl');os.environ.setdefault('MESA_SHADER_CACHE_DIR','/tmp/microduck-mesa-cache')
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np,mujoco
from PIL import Image,ImageDraw,ImageFont
HERE=Path(__file__).resolve().parent
font=lambda s:ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',s)
def scene(folder):
 t=ET.parse(folder/'tab5_portrait_print.xml');root=t.getroot();root.remove(root.find('contact'))
 for b in root.iter('body'):
  for g in list(b.findall('geom')):
   if g.get('name','').startswith('rounded_shell_collision_'):b.remove(g)
 for a in list(root.find('asset').findall('mesh')):
  if a.get('name','').startswith('rounded_shell_collision_'):root.find('asset').remove(a)
 t.write(folder/'_render_only_robot.xml');s=ET.parse(folder/'scene_tab5_portrait_print.xml');s.getroot().find('include').set('file','_render_only_robot.xml');s.write(folder/'_render_only_scene.xml');return folder/'_render_only_scene.xml'
def render(path,az=90,separate=False):
 m=mujoco.MjModel.from_xml_path(str(path));d=mujoco.MjData(m);mujoco.mj_resetDataKeyframe(m,d,m.key('STAND').id)
 if separate:m.body_pos[m.body('tab5').id,0]+=.040
 mujoco.mj_forward(m,d);m.vis.global_.offwidth=1000;m.vis.global_.offheight=760;r=mujoco.Renderer(m,width=1000,height=760);cam=mujoco.MjvCamera();cam.azimuth=az;cam.elevation=-12;cam.distance=.285 if separate else .245;cam.lookat[:]=[-.005,0,.205];r.update_scene(d,camera=cam);r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=0;im=Image.fromarray(r.render());r.close();return im
c=Image.new('RGB',(2000,1660),(14,20,27));d=ImageDraw.Draw(c)
for col,(folder,label) in enumerate([(HERE.parent/'portrait_print_v1','BEFORE | front depth roundover R6'),(HERE,'AFTER | square, planar mating rim')]):
 s=scene(folder)
 for row in range(2):c.paste(render(s,90 if row==0 else 145,bool(row)),(col*1000,70+row*820))
 d.text((col*1000+22,18),label,font=font(28),fill='white')
 d.rectangle((col*1000,830,col*1000+1000,890),fill=(14,20,27));d.text((col*1000+22,846),'Separated view: Tab5 moved 40 mm for illustration only',font=font(21),fill=(158,220,230))
d.rectangle((0,1610,2000,1660),fill=(14,20,27));d.text((22,1625),'Tab5 and body placement unchanged in assembly | contact plane X41 mm | side/rear curves and open center retained',font=font(22),fill='white');c.save(HERE/'flush_interface_comparison.png')
