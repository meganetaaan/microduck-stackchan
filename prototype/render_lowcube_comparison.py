"""Direct MuJoCo renders at identical camera positions and scale."""
import os
os.environ.setdefault("MUJOCO_GL","egl")
os.environ.setdefault("MESA_SHADER_CACHE_DIR","/tmp/microduck-mesa-cache")
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image,ImageDraw

ROOT=Path(__file__).resolve().parent
def render(scene,azimuth,out):
    m=mujoco.MjModel.from_xml_path(str(scene));d=mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m,d,m.key("STAND").id);mujoco.mj_forward(m,d)
    renderer=mujoco.Renderer(m,height=480,width=640);cam=mujoco.MjvCamera()
    cam.azimuth=azimuth;cam.elevation=-8;cam.distance=.54;cam.lookat[:]=[0,0,.125]
    renderer.update_scene(d,camera=cam);im=Image.fromarray(renderer.render());renderer.close()
    im.save(out);return im
def main():
    canvas=Image.new("RGB",(1920,1100),(14,20,27));dr=ImageDraw.Draw(canvas)
    for row,(variant,label) in enumerate([("bare","BEFORE: first Tab5 concept"),("lowcube","AFTER: lower, deeper body / side hip cutouts")]):
        for col,(az,title) in enumerate([(180,"Front"),(90,"Side"),(125,"Three-quarter")]):
            out=ROOT/f"results/lowcube_{variant}_{title.lower().replace('-','')}.png"
            im=render(ROOT/f"models/scene_tab5_{variant}.xml",az,out);canvas.paste(im,(col*640,row*550+65))
            dr.text((col*640+15,row*550+9),label+" | "+title,fill="white")
            dims="Case D70 x W132 x H84 mm | top Z240 mm" if variant=="bare" else "Case D120 x W132 x H118 mm | top Z193 mm"
            detail="Tab5 landscape 128 x 80 mm, center Z198 mm" if variant=="bare" else "Tab5 lowered 47 mm | chin 36 mm | same legs"
            dr.text((col*640+15,row*550+27),dims,fill=(155,216,226))
            dr.text((col*640+15,row*550+45),detail,fill=(225,225,225))
    canvas.save(ROOT/"results/lowcube_before_after.png")
if __name__=="__main__":main()
