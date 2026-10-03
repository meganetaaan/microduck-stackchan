"""Independent open-frame portrait concept; no edits to prior model variants."""
from pathlib import Path
import json,sys,xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull
HERE=Path(__file__).resolve().parent;ROOT=HERE.parent
sys.path.insert(0,str(ROOT))
from build_models import add_box,vec

def rounded_prism(path,depth,width,height,radius):
    points=[]
    for cy,cz,start in [(width/2-radius,height/2-radius,0),(-width/2+radius,height/2-radius,90),(-width/2+radius,-height/2+radius,180),(width/2-radius,-height/2+radius,270)]:
        for angle in np.linspace(start,start+90,13):
            a=np.deg2rad(angle);points.append([cy+radius*np.cos(a),cz+radius*np.sin(a)])
    v=np.array([[x,y,z]for x in [-depth/2,depth/2]for y,z in points])
    hull=ConvexHull(v)
    lines=['# rounded rectangular prism, metres']+['v '+vec(p)for p in v]
    for face,equation in zip(hull.simplices,hull.equations):
        a,b,c=v[face]
        if np.dot(np.cross(b-a,c-a),equation[:3])<0:face=face[::-1]
        lines.append('f '+' '.join(str(i+1)for i in face))
    path.write_text('\n'.join(lines)+'\n')

def build():
    p=json.loads((HERE/'parameters.json').read_text());tree=ET.parse(ROOT/'models/tab5_lowcube.xml');r=tree.getroot();r.set('model',p['variant']);trunk=r.find(".//body[@name='trunk_base']")
    for b in list(trunk.findall('body')):
        if b.find('joint') is None:trunk.remove(b)
    contact=r.find('contact')
    for pair in list(contact):
        if pair.get('geom1','').startswith('cube_')or pair.get('geom1')=='tab5_collision':contact.remove(pair)
    bounds=np.array(p['body_bounds_xyz_mm'],float);t=p['frame_bar_mm'];(x0,x1),(y0,y1),(z0,z1)=bounds;parts=[]
    def part(name,bounds,mass=None,color='.91 .91 .86 1'):
        b=np.array(bounds,float)/1000;s=b[:,1]-b[:,0]
        if mass is None:mass=float(np.prod(s)*p['frame_density_kg_m3'])
        body=add_box(trunk,'cube_portrait_'+name,mass,s,b.mean(1),color)
        parts.append({'name':'cube_portrait_'+name,'bounds_xyz_mm':(b*1000).tolist(),'mass_kg':mass})
        return body
    # Non-overlapping 12-piece cuboid skeleton; all sides/top/bottom remain open.
    for label,z in [('bottom',[z0,z0+t]),('top',[z1-t,z1])]:
        for side,y in [('left',[y0,y0+t]),('right',[y1-t,y1])]:part(label+'_'+side,[[x0,x1],y,z])
        for side,x in [('back',[x0,x0+t]),('front',[x1-t,x1])]:part(label+'_'+side,[x,[y0+t,y1-t],z])
    for xn,x in [('back',[x0,x0+t]),('front',[x1-t,x1])]:
        for yn,y in [('left',[y0,y0+t]),('right',[y1-t,y1])]:part('post_'+xn+'_'+yn,[x,y,[z0+t,z1-t]])
    frame_mass=sum(a['mass_kg']for a in parts)
    # Explicit central base bracket plus tray bridging to side rails.
    
    for i,bounds in enumerate(p['mount_posts_bounds_xyz_mm']):part('mount_'+str(i),bounds,p['mount_mass_kg']/2,'.22 .25 .28 1')
    part('support_tray',[[-20,20],[-36,36],[z0,z0+2]],color='.3 .33 .35 1')
    part('compute',p['compute_bounds_xyz_mm'],p['compute_mass_kg'],'.05 .35 .2 1')
    part('compute_standoff',[[-24,-16],[-8,8],[z0+2,48]],mass=.002,color='.22 .25 .28 1')
    tab=add_box(trunk,'tab5',p['tab5_mass_kg'],np.array(p['tab5_size_xyz_mm'])/1000,np.array(p['tab5_center_xyz_mm'])/1000,'.94 .94 .91 0')
    # Collision bounds cover the real rounded body conservatively, no fake holes.
    tab.find('geom').set('group','3')
    asset=r.find('asset');rounded_prism(HERE/'assets/tab5_rounded.obj',.012,.080,.128,.004)
    ET.SubElement(asset,'mesh',name='portrait_tab5_visual',file='../../../../../../prototype/portrait/assets/tab5_rounded.obj')
    ET.SubElement(tab,'geom',name='tab5_visual',type='mesh',mesh='portrait_tab5_visual',rgba='.94 .94 .91 1',contype='0',conaffinity='0',mass='0',group='0')
    ET.SubElement(tab,'geom',name='screen',type='box',size='.00015 .0311 .0552',pos='.00616 0 -.0024',rgba='.008 .022 .029 1',contype='0',conaffinity='0',mass='0',group='0')
    for y in [-.014,.014]:
        ET.SubElement(tab,'geom',name='eye_'+('left'if y<0 else'right'),type='ellipsoid',pos=vec([.0064,y,.0252]),size='.0001 .004 .009',rgba='.10 .95 .90 1',contype='0',conaffinity='0',mass='0',group='0')
    for name,rad,x,color in [('camera_rim',.0025,.00614,'.13 .15 .16 1'),('camera_lens',.0018,.00634,'.015 .04 .06 1'),('camera_glint',.0005,.00655,'.26 .45 .65 1')]:
        ET.SubElement(tab,'geom',name=name,type='cylinder',size=f'{rad} .00012',pos=f'{x} 0 .05955',quat='.7071067812 0 .7071067812 0',rgba=color,contype='0',conaffinity='0',mass='0',group='0')
    # Every added physical box is checked against every original moving CAD hull.
    moving=[g.get('name')for g in r.iter('geom')if g.get('name','').startswith('shell_guard_legmesh_')]
    physical=[a['name']+'_collision'for a in parts]+['tab5_collision']
    for box in physical:
        for leg in moving:ET.SubElement(contact,'pair',geom1=box,geom2=leg,condim='3',friction='1 1 .005 .0001 .0001')
    ET.indent(tree);tree.write(HERE/'tab5_portrait.xml',encoding='utf-8',xml_declaration=True)
    scene=ET.parse(ROOT/'models/scene_tab5_lowcube.xml');scene.getroot().find('include').set('file','tab5_portrait.xml');scene.write(HERE/'scene_tab5_portrait.xml',encoding='utf-8',xml_declaration=True)
    report={'parameters':p,'frame_only_mass_kg':frame_mass,'added_frame_mount_compute_mass_kg':sum(a['mass_kg']for a in parts),'parts':parts,'collision_pairs':len(physical)*len(moving),'tab5_visual_collision_note':'R4 visible body, full12x80x128mm conservative bounding-box collision. Screen/lens/eyes are zero-mass graphics; device mass includes components.','model_mass_kg_expected':.45731578+.1184+sum(a['mass_kg']for a in parts)}
    (HERE/'geometry_report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':build()
