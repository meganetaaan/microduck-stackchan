"""Static kinematic posture illustration; never an animation/balance claim."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import mujoco,numpy as np
from PIL import Image,ImageDraw,ImageFont
HERE=Path(__file__).resolve().parent;ROOT=HERE.parent
m=mujoco.MjModel.from_xml_path(str(HERE/'scene_tab5_portrait.xml'));d=mujoco.MjData(m)
m.vis.global_.offwidth=800;m.vis.global_.offheight=800
r=mujoco.Renderer(m,width=800,height=800);cam=mujoco.MjvCamera();cam.azimuth=125;cam.elevation=-12;cam.distance=.48;cam.lookat[:]=[0,0,.125]
mujoco.mj_resetDataKeyframe(m,d,m.key('STAND').id);stand=d.qpos.copy()
crouch=np.load(ROOT/'diagnostics/feet_flat_crouch_paths.npz')['qpos'];q=crouch[np.argmin(np.abs(crouch[:,10]-np.deg2rad(85)))]
mujoco.mj_resetDataKeyframe(m,d,m.key('FOLD').id);fold=d.qpos.copy()
c=Image.new('RGB',(2400,930),(14,20,27));draw=ImageDraw.Draw(c)
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',24);small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',19)
for i,(pose,title)in enumerate([(stand,'Standing reference'),(q,'Feet-flat crouch: knee85deg'),(fold,'Original FOLD clearance pose')]):
 d.qpos[:]=pose;mujoco.mj_forward(m,d);r.update_scene(d,camera=cam);r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=0;c.paste(Image.fromarray(r.render()),(i*800,100));draw.text((i*800+18,17),title,font=font,fill='white');draw.text((i*800+18,55),'Kinematic geometry only; seated/fold balance unverified',font=small,fill=(172,217,227))
draw.text((18,906),'Same legs and camera. FOLD retains original same-leg convex-proxy overlaps and hovering feet; this is not a validated floor-resting pose.',font=small,fill=(229,205,168))
c.save(HERE/'portrait_clearance_poses.png');r.close()
