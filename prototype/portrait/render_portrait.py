"""Matched real MuJoCo previews for the separate portrait/open-frame concept."""
import os
os.environ.setdefault('MUJOCO_GL','egl');os.environ.setdefault('MESA_SHADER_CACHE_DIR','/tmp/microduck-mesa-cache')
from pathlib import Path
import sys,mujoco
from PIL import Image,ImageDraw,ImageFont
HERE=Path(__file__).resolve().parent;ROOT=HERE.parent
sys.path.insert(0,str(ROOT));from render_rounded_comparison import render
FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
def font(n):
 try:return ImageFont.truetype(FONT,n)
 except:return ImageFont.load_default()
def main():
 scene=HERE/'scene_tab5_portrait.xml';old=ROOT/'rounded/scene_tab5_lowcube_rounded.xml'
 views=[]
 for az,name in [(180,'Front'),(90,'Side'),(125,'Three-quarter')]:
  im=render(scene,az,800,800,.48,.135);im.save(HERE/(name.lower()+'.png'));views.append(im)
 canvas=Image.new('RGB',(2400,900),(14,20,27));d=ImageDraw.Draw(canvas)
 for i,(im,name)in enumerate(zip(views,['Front: camera at top','Side: open square frame','Three-quarter: cover removed'])):
  canvas.paste(im,(800*i,100));d.text((800*i+20,16),name,font=font(25),fill='white');d.text((800*i+20,52),'Body80W x80D | portrait Tab5 80W x128H',font=font(19),fill=(150,222,228))
 canvas.save(HERE/'portrait_views.png')
 c=Image.new('RGB',(1920,1050),(14,20,27));d=ImageDraw.Draw(c)
 for i,(s,name)in enumerate([(old,'Saved rounded design'),(scene,'Separate portrait open-frame design')]):
  c.paste(render(s,125,960,960,.47,.135),(960*i,90));d.text((960*i+20,18),name,font=font(26),fill='white');d.text((960*i+20,53),'Same leg mechanism and camera scale',font=font(20),fill=(160,222,230))
 c.save(HERE/'portrait_comparison.png')
 im=render(scene,180,960,960,.31,.185);im.save(HERE/'portrait_face_closeup.png')
if __name__=='__main__':main()
