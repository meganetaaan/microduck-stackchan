"""Actual MuJoCo views, static exploded assembly, and print-orientation previews."""
import os
os.environ.setdefault('MUJOCO_GL','egl');os.environ.setdefault('MESA_SHADER_CACHE_DIR','/tmp/microduck-mesa-cache')
from pathlib import Path
import sys,json,xml.etree.ElementTree as ET
import mujoco,numpy as np,trimesh
from PIL import Image,ImageDraw,ImageFont
HERE=Path(__file__).resolve().parent;ROOT=HERE.parent
sys.path.insert(0,str(ROOT));from render_rounded_comparison import render
FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
def font(s):return ImageFont.truetype(FONT,s)
def title(d,xy,s,size=22,color='white'):d.text(xy,s,font=font(size),fill=color)
def main():
 # Render-only copy omits invisible collision hulls; all physics/audits use the original scene.
 tree=ET.parse(HERE/'tab5_portrait_print.xml');rr=tree.getroot();contact=rr.find('contact');rr.remove(contact)
 for b in rr.iter('body'):
  for g in list(b.findall('geom')):
   if g.get('name','').startswith('rounded_shell_collision_'):b.remove(g)
 for a in list(rr.find('asset').findall('mesh')):
  if a.get('name','').startswith('rounded_shell_collision_'):rr.find('asset').remove(a)
 tree.write(HERE/'_render_only_robot.xml');sc=ET.parse(HERE/'scene_tab5_portrait_print.xml');sc.getroot().find('include').set('file','_render_only_robot.xml');sc.write(HERE/'_render_only_scene.xml')
 scene=HERE/'_render_only_scene.xml';old=ROOT/'portrait/scene_tab5_portrait.xml'
 c=Image.new('RGB',(2400,900),(14,20,27));d=ImageDraw.Draw(c)
 for i,(az,n)in enumerate([(180,'Front'),(90,'Side'),(125,'Three-quarter')]):
  im=render(scene,az,800,800,.48,.135);c.paste(im,(i*800,100));title(d,(i*800+18,15),n+' | selected portrait / print concept');title(d,(i*800+18,52),'Body80x80x80 mm | R6 outside / R10 side opening',18,(158,220,230));im.save(HERE/(n.lower()+'.png'))
 c.save(HERE/'print_views.png')
 c=Image.new('RGB',(1920,1050),(14,20,27));d=ImageDraw.Draw(c)
 for i,(s,n)in enumerate([(old,'Before: open bar frame'),(scene,'After: rounded printable body concept')]):
  c.paste(render(s,125,960,960,.44,.14),(i*960,90));title(d,(i*960+18,17),n,25);title(d,(i*960+18,54),'Same Tab5, face position, leg geometry and camera scale',19,(158,220,230))
 c.save(HERE/'print_comparison.png')
 # Static exploded view only: display transforms are never used by simulation.
 m=mujoco.MjModel.from_xml_path(str(scene));data=mujoco.MjData(m);mujoco.mj_resetDataKeyframe(m,data,m.key('STAND').id)
 for n,delta in [('print_body_cage',[0,0,.055]),('print_rear_cover',[-.045,0,.055]),('print_central_mount',[0,0,.018]),('print_under_saddle',[0,0,-.016]),('print_compute_pedestal',[0,0,.055]),('cube_portrait_compute',[0,0,.055]),('tab5',[.065,0,.055])]:m.body_pos[m.body(n).id]+=delta
 for n,color in [('print_central_mount_visual',[.3,.65,.85,1]),('print_under_saddle_visual',[.27,.32,.37,1]),('print_compute_pedestal_visual',[.8,.63,.30,1])]:m.geom_rgba[m.geom(n).id]=color
 m.geom_rgba[m.geom('print_rear_hardware_visual').id,3]=0
 mujoco.mj_forward(m,data);m.vis.global_.offwidth=1300;m.vis.global_.offheight=1100;r=mujoco.Renderer(m,width=1300,height=1100);cam=mujoco.MjvCamera();cam.azimuth=120;cam.elevation=-17;cam.distance=.61;cam.lookat[:]=[.006,0,.17];r.update_scene(data,camera=cam);r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=0
 im=Image.fromarray(r.render());r.close();d=ImageDraw.Draw(im);d.rectangle((0,0,1300,92),fill=(14,20,27));title(d,(20,15),'Static exploded assembly | positions separated for illustration',27);title(d,(20,53),'White: cage + rear cover | blue: central mount | dark: metal washer | gold: compute support',18,(158,220,230));d.rectangle((0,1020,1300,1100),fill=(14,20,27));title(d,(20,1034),'M2 rear-cover fittings are provisional; Tab5 M3 engagement and base-clamp strength need verification',18);title(d,(20,1062),'Four printable parts plus metal washer/fasteners. Original robot parts and Tab5 are not supplied as printable replacements.',18);im.save(HERE/'exploded_assembly.png')
 # Export every printable solid in a proposed build orientation with its bed at Z0.
 out=HERE/'stl_print_oriented';out.mkdir(exist_ok=True);items=[]
 for n in ['body_cage','rear_cover','central_mount','compute_pedestal']:
  tm=trimesh.load(HERE/'stl'/f'{n}_mm.stl',process=True);axis=[0,1,0]if n in ['body_cage','rear_cover']else[1,0,0];angle=np.pi/2 if n in ['body_cage','rear_cover','central_mount']else 0
  R=trimesh.transformations.rotation_matrix(angle,axis);tm.apply_transform(R);shift=np.r_[-tm.bounds.mean(0)[:2],-tm.bounds[0,2]];tm.apply_translation(shift);tm.export(out/(n+'_print_mm.stl'));obj=out/(n+'_print_m.obj');cp=tm.copy();cp.apply_scale(.001);cp.export(obj)
  items.append({'name':n,'file':str((out/(n+'_print_mm.stl')).relative_to(HERE)),'rotation_matrix':R.tolist(),'translation_mm':shift.tolist(),'print_bounds_mm':tm.bounds.tolist(),'watertight':bool(tm.is_watertight),'connected_components':len(tm.split()),'supports_note':'Local supports expected under opening roofs; test slice required'if n=='body_cage'else'Check slicer overhangs and fit; no support-free claim'})
 (HERE/'print_orientation.json').write_text(json.dumps({'units':'mm','parts':items,'note':'Suggested orientations, not slicer-verified G-code or printer-specific profiles.'},indent=2))
 canvas=Image.new('RGB',(2400,1200),(230,233,235));draw=ImageDraw.Draw(canvas)
 for i,item in enumerate(items):
  n=item['name'];xml=f'<mujoco><asset><mesh name="part" file="{str(out/(n+"_print_m.obj"))}"/></asset><visual><headlight ambient=".20 .20 .20" diffuse=".40 .40 .40"/><global offwidth="800" offheight="500"/></visual><worldbody><light pos="0 0 1" diffuse=".55 .55 .55"/><geom type="plane" size="1 1 .01" rgba=".22 .26 .30 1"/><geom type="mesh" mesh="part" rgba=".72 .75 .72 1"/></worldbody></mujoco>'
  mm=mujoco.MjModel.from_xml_string(xml);dd=mujoco.MjData(mm);mujoco.mj_forward(mm,dd);rr=mujoco.Renderer(mm,width=800,height=500);ca=mujoco.MjvCamera();ca.azimuth=130;ca.elevation=-28;dims=np.ptp(np.array(item['print_bounds_mm']),axis=0);ca.distance=max(.09,float(max(dims))*.0032);ca.lookat[:]=[0,0,dims[2]*.00035];rr.update_scene(dd,camera=ca);image=Image.fromarray(rr.render());rr.close();x=(i%3)*800;y=(i//3)*600;canvas.paste(image,(x,y+80));title(draw,(x+18,y+15),n.replace('_',' '),24,(20,30,38));title(draw,(x+18,y+48),'Build bounds '+ ' x '.join(f'{v:.1f}'for v in dims)+' mm',18,(42,67,78))
 title(draw,(1620,638),'STL units: millimetres',25,(20,30,38));title(draw,(1620,685),'Main cage: front toward build plate',19,(42,67,78));title(draw,(1620,723),'Rear cover: flat. Mount: on its side.',19,(42,67,78));title(draw,(1620,775),'Print concept: local supports, fit coupon,',19,(42,67,78));title(draw,(1620,808),'thread engagement and strength need checks.',19,(42,67,78));canvas.save(HERE/'print_parts_orientation.png')
if __name__=='__main__':main()
