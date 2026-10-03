"""Actual assembly views; exploded positions are labeled display transforms only."""
import os
os.environ.setdefault('MUJOCO_GL','egl');os.environ.setdefault('MESA_SHADER_CACHE_DIR','/tmp/microduck-mesa-cache')
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco,numpy as np
from PIL import Image,ImageDraw,ImageFont
HERE=Path(__file__).resolve().parent
FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
def text(d,pos,msg,size=22):d.text(pos,msg,font=ImageFont.truetype(FONT,size),fill='white')
def setup():
 t=ET.parse(HERE/'tab5_assembly_uart.xml');r=t.getroot();r.remove(r.find('contact'))
 for b in r.iter('body'):
  for g in list(b.findall('geom')):
   if g.get('group')=='3':b.remove(g)
 used={g.get('mesh')for g in r.iter('geom')if g.get('mesh')}
 for a in list(r.find('asset')):
  if a.tag=='mesh'and (a.get('name')or Path(a.get('file')).stem)not in used:r.find('asset').remove(a)
 t.write(HERE/'_render_only_robot.xml');s=ET.parse(HERE/'scene_tab5_assembly_uart.xml');s.getroot().find('include').set('file','_render_only_robot.xml');s.write(HERE/'_render_only_scene.xml')
def view(az,w=900,h=850,dist=.47,z=.14,service=False):
 m=mujoco.MjModel.from_xml_path(str(HERE/'_render_only_scene.xml'));d=mujoco.MjData(m);mujoco.mj_resetDataKeyframe(m,d,m.key('STAND').id)
 if service:
  m.body_pos[m.body('tab5').id,0]+=.065;m.body_pos[m.body('print_rear_cover').id,0]-=.045
  m.geom_rgba[m.geom('assembly_body_cage_visual').id,3]=.14
 mujoco.mj_forward(m,d);m.vis.global_.offwidth=w;m.vis.global_.offheight=h;r=mujoco.Renderer(m,width=w,height=h);cam=mujoco.MjvCamera();cam.azimuth=az;cam.elevation=-18;cam.distance=dist;cam.lookat[:]=[0,0,z];r.update_scene(d,camera=cam);r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=0;im=Image.fromarray(r.render());r.close();return im
if __name__=='__main__':
 setup();c=Image.new('RGB',(2700,950),(14,20,27));d=ImageDraw.Draw(c)
 for i,(az,label)in enumerate([(180,'Front'),(90,'Side'),(125,'Three quarter')]):
  im=view(az);c.paste(im,(900*i,100));text(d,(900*i+20,20),label+' | UART component assembly',25);text(d,(900*i+20,58),'Portrait Tab5 / original legs / provisional internals',19)
 c.save(HERE/'assembly_views.png')
 c=Image.new('RGB',(2000,1200),(14,20,27));d=ImageDraw.Draw(c)
 for i,(az,label)in enumerate([(65,'Rear / board carrier'),(125,'Front / UART and power routes')]):
  c.paste(view(az,1000,1040,.43,.19,True),(1000*i,95));text(d,(1000*i+20,16),label,25);text(d,(1000*i+20,55),'Service view: screen/cover separated, shell translucent',18)
 text(d,(20,1145),'Red: power | blue: UART | blue/amber controller stack is provisional | no hardware-fit certification',23);c.save(HERE/'assembly_service_view.png')
