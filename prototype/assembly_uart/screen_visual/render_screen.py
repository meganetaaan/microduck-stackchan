"""Static MuJoCo previews of the screen visual variant; no physics rollouts."""
import os
os.environ.setdefault('MUJOCO_GL','egl');os.environ.setdefault('MESA_SHADER_CACHE_DIR','/tmp/microduck-mesa-cache')
from pathlib import Path
import xml.etree.ElementTree as E,json
import mujoco,numpy as np,trimesh
from PIL import Image,ImageDraw,ImageFont
H=Path(__file__).resolve().parent
FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
def label(d,xy,t,s=25):d.text(xy,t,font=ImageFont.truetype(FONT,s),fill='white')
def setup():
 t=E.parse(H/'tab5_assembly_uart_screen.xml');r=t.getroot();r.remove(r.find('contact'))
 for b in r.iter('body'):
  for g in list(b.findall('geom')):
   if g.get('group')=='3':b.remove(g)
 used={g.get('mesh')for g in r.iter('geom')if g.get('mesh')}
 for a in list(r.find('asset')):
  if a.tag=='mesh'and(a.get('name')or Path(a.get('file')).stem)not in used:r.find('asset').remove(a)
 t.write(H/'_render_only_robot.xml');s=E.parse(H/'scene_tab5_assembly_uart_screen.xml');s.getroot().find('include').set('file','_render_only_robot.xml');s.write(H/'_render_only_scene.xml')
def view(az,w=900,h=1000,dist=.43,z=.145,el=-12):
 m=mujoco.MjModel.from_xml_path(str(H/'_render_only_scene.xml'));d=mujoco.MjData(m);mujoco.mj_resetDataKeyframe(m,d,m.key('STAND').id);mujoco.mj_forward(m,d);m.vis.global_.offwidth=w;m.vis.global_.offheight=h
 r=mujoco.Renderer(m,width=w,height=h);cam=mujoco.MjvCamera();cam.azimuth=az;cam.elevation=el;cam.distance=dist;cam.lookat[:]=[0,0,z];r.update_scene(d,camera=cam);r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=0;im=Image.fromarray(r.render());r.close();return im

def glb():
 root=E.parse(H/'tab5_assembly_uart_screen.xml').getroot();assets={a.get('name')or Path(a.get('file')).stem:(H/a.get('file')).resolve()for a in root.find('asset').findall('mesh')};trunk=root.find(".//body[@name='trunk_base']");scene=trimesh.Scene();names=[]
 for b in trunk.findall('body'):
  name=b.get('name','')
  if not(name.startswith('assembly_')or name in ['print_rear_cover','print_under_saddle','print_rear_hardware','tab5']):continue
  bp=np.fromstring(b.get('pos','0 0 0'),sep=' ')
  for g in b.findall('geom'):
   if g.get('group')=='3':continue
   color=np.fromstring(g.get('rgba','.8 .8 .8 1'),sep=' ')
   if color[3]<=0:continue
   typ=g.get('type','sphere');size=np.fromstring(g.get('size',''),sep=' ')
   if g.get('mesh'):m=trimesh.load(assets[g.get('mesh')],process=False)
   elif typ=='box':m=trimesh.creation.box(extents=2*size)
   elif typ=='ellipsoid':m=trimesh.creation.icosphere(subdivisions=2);m.apply_scale(size)
   elif typ=='cylinder':m=trimesh.creation.cylinder(radius=size[0],height=2*size[1],sections=32)
   else:continue
   q=np.fromstring(g.get('quat','1 0 0 0'),sep=' ');T=trimesh.transformations.quaternion_matrix(q);T[:3,3]=bp+np.fromstring(g.get('pos','0 0 0'),sep=' ');m.apply_transform(T)
   if g.get('name')=='screen':
    tex=Image.open(H/'assets/stackchan_microduck_screen_720x1280.png').convert('RGB');mat=trimesh.visual.material.PBRMaterial(name='StackchanScreen',baseColorTexture=tex,baseColorFactor=[255,255,255,255],emissiveTexture=tex,emissiveFactor=[1,1,1],metallicFactor=0,roughnessFactor=1,doubleSided=False);m.visual=trimesh.visual.TextureVisuals(uv=m.visual.uv,material=mat)
   else:m.visual.vertex_colors=np.tile(np.rint(color*255).astype(np.uint8),(len(m.vertices),1))
   n=g.get('name',name);scene.add_geometry(m,node_name=n,geom_name=n);names.append(n)
 scene.export(H/'upper_assembly_screen.glb');(H/'upper_assembly_screen_export.json').write_text(json.dumps({'units':'metres','geometry_count':len(names),'nodes':names,'scope':'Same physical upper assembly, updated UV-mapped screen graphics. Original MicroDuck leg CAD omitted; view whole robot with MJCF.'},indent=2)+'\n')
if __name__=='__main__':
 setup();c=Image.new('RGB',(2700,1140),(14,20,27));d=ImageDraw.Draw(c)
 for i,(az,t)in enumerate([(180,'Front'),(90,'Side'),(125,'Three quarter')]):c.paste(view(az),(900*i,100));label(d,(900*i+20,20),t+' | Stack-chan display',27);label(d,(900*i+20,60),'Portrait camera up / screen artwork only',20)
 label(d,(25,1100),'Reference face above / decorative robot panel below / original assembly and trained physics preserved',24);c.save(H/'screen_robot_views.png')
 c=Image.new('RGB',(1900,1500),(14,20,27));d=ImageDraw.Draw(c);c.paste(view(180,950,1350,.245,.182,0),(0,95));c.paste(view(143,950,1350,.245,.185,-6),(950,95));label(d,(25,22),'Screen close-up | white dot eyes + neutral mouth',29);label(d,(975,22),'MicroDuck colors | warm white, gray, orange',28);label(d,(25,1452),'720 x 1280 display artwork; lower panel is decorative, not additional physical hardware',25);c.save(H/'screen_closeup.png')
 glb();print('Rendered views and close-up; exported textured upper assembly GLB')
