"""Matched MuJoCo engineering views; shadows disabled to avoid tiny-shell acne."""
import os
os.environ.setdefault('MUJOCO_GL','egl');os.environ.setdefault('MESA_SHADER_CACHE_DIR','/tmp/microduck-mesa-cache')
from pathlib import Path
import mujoco
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parent

def render(scene,azimuth,width=640,height=480,distance=.54,lookz=.125):
 m=mujoco.MjModel.from_xml_path(str(scene));d=mujoco.MjData(m);mujoco.mj_resetDataKeyframe(m,d,m.key('STAND').id);mujoco.mj_forward(m,d)
 m.vis.global_.offwidth=width;m.vis.global_.offheight=height
 rr=mujoco.Renderer(m,height=height,width=width);cam=mujoco.MjvCamera();cam.azimuth=azimuth;cam.elevation=-12;cam.distance=distance;cam.lookat[:]=[0,0,lookz]
 rr.update_scene(d,camera=cam);rr.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=0
 im=Image.fromarray(rr.render());rr.close();return im

def main():
 inputs=[(ROOT/'diagnostics/scene_lowcube_cutout_candidate.xml','BEFORE: angular cutout design'),(ROOT/'rounded/scene_tab5_lowcube_rounded.xml','AFTER: actual rounded shell')]
 close=Image.new('RGB',(1920,1040),(14,20,27));cd=ImageDraw.Draw(close)
 for i,(scene,label) in enumerate(inputs):
  im=render(scene,125,960,960,.30,.145);close.paste(im,(i*960,80));cd.text((i*960+24,22),label,fill='white');cd.text((i*960+24,46),'Same Tab5 and legs | enlarged curved openings | same camera',fill=(158,220,230))
 close.save(ROOT/'rounded/rounded_closeup.png')
 c=Image.new('RGB',(1920,1080),(14,20,27));d=ImageDraw.Draw(c)
 for row,(scene,label) in enumerate(inputs):
  for col,(az,name) in enumerate([(180,'Front'),(90,'Side'),(125,'Three-quarter')]):
   im=render(scene,az);im.save(ROOT/f'rounded/{row}_{name.lower()}.png');c.paste(im,(640*col,540*row+60));d.text((col*640+12,row*540+12),label+' | '+name,fill='white');d.text((col*640+12,row*540+33),'Outer R6 / foot arches R5 / side openings R8 mm' if row else 'Same camera and standing pose',fill=(159,218,231))
 c.save(ROOT/'rounded/rounded_before_after.png')
if __name__=='__main__':main()
