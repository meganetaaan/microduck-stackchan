"""Parametric printable portrait body concept; mm CAD, m MuJoCo, mm STL."""
from pathlib import Path
import sys,json,itertools,xml.etree.ElementTree as ET
import numpy as np,manifold3d as mf,trimesh
HERE=Path(__file__).resolve().parent;ROOT=HERE.parent
sys.path.insert(0,str(ROOT));from build_models import vec,add_box

def box(b):
 b=np.array(b,float);return mf.Manifold.cube(b[:,1]-b[:,0]).translate(b[:,0])
def roundedbox(b,r):
 b=np.array(b,float);return mf.Manifold.cube(b[:,1]-b[:,0]-2*r,center=True).minkowski_sum(mf.Manifold.sphere(r,32)).translate(b.mean(1))
def rr(a,b,r):return mf.CrossSection.square([a[1]-a[0]-2*r,b[1]-b[0]-2*r],center=True).offset(r,circular_segments=48).translate([(a[0]+a[1])/2,(b[0]+b[1])/2])
def side(section,y0,y1):return section.extrude(y1-y0).transform([[1,0,0,0],[0,0,1,y0],[0,1,0,0]])
def front(section,x0,x1):return section.extrude(x1-x0).transform([[0,0,1,x0],[1,0,0,0],[0,1,0,0]])
def cylx(r,x0,x1,y,z,n=32):return mf.Manifold.cylinder(x1-x0,r,circular_segments=n).rotate([0,90,0]).translate([x0,y,z])
def hex_x(af,x0,x1,y,z):return cylx(af/np.sqrt(3),x0,x1,y,z,6)
def mesh(s,scale=1):
 o=s.to_mesh64();return trimesh.Trimesh(np.asarray(o.vert_properties)[:,:3]*scale,np.asarray(o.tri_verts),process=False)
def write(s,name):
 m=mesh(s,.001);m.export(HERE/'assets'/f'{name}.obj');mm=mesh(s);mm.export(HERE/'stl'/f'{name}_mm.stl');return m

def main():
 p=json.loads((HERE/'parameters.json').read_text());B=np.array(p['body_bounds_xyz_mm'],float);t=p['wall_mm'];r=p['outer_corner_radius_mm']
 outer=roundedbox(B,r);inner=roundedbox(B+np.array([[t,-t]]*3),r-t);shell=outer-inner
 cuts=[]
 for y0,y1 in [(-45,-34),(34,45)]:cuts.append(side(rr(*p['side_window_xz_mm'],p['side_window_radius_mm']),y0,y1))
 cuts.append(front(rr(*p['front_aperture_yz_mm'],p['front_aperture_radius_mm']),34,45))
 cuts.append(front(rr(*p['rear_aperture_yz_mm'],p['rear_aperture_radius_mm']),-45,-30))
 shell=mf.Manifold.batch_boolean([shell,*cuts],mf.OpType.Subtract)
 # Rear access bosses, cut after adding. Interior captive-nut access is open
 # while Tab5/rear cover are removed; no arbitrary MicroDuck holes are invented.
 bosses=[cylx(5,-36.6,-31,y,z)for y,z in p['rear_screw_yz_mm']]
 shell=mf.Manifold.batch_boolean([shell,*bosses],mf.OpType.Add)
 holes=[]
 for y,z in p['rear_screw_yz_mm']:
  holes.extend([cylx(p['rear_clearance_hole_diameter_mm']/2,-40,-30,y,z),hex_x(p['rear_nut_pocket_across_flats_mm'],-32.9,-30.9,y,z)])
 shell=mf.Manifold.batch_boolean([shell,*holes],mf.OpType.Subtract)
 # Four rear Tab5 lands use verified upper/middle M3 hole centers.
 tabs=[cylx(4,32,41,y,z)for y,z in p['tab5_rear_mount_yz_mm']]
 shell=mf.Manifold.batch_boolean([shell,*tabs],mf.OpType.Add)
 for y,z in p['tab5_rear_mount_yz_mm']:shell=shell-cylx(p['tab5_rear_clearance_hole_mm']/2,31,42,y,z)
 gap=p['rear_cover_gap_mm'];(ya,yb),(za,zb)=p['rear_aperture_yz_mm'];cover=front(rr([ya+gap,yb-gap],[za+gap,zb-gap],p['rear_aperture_radius_mm']-gap),-39,-36.6)
 covercuts=[]
 for y,z in p['rear_screw_yz_mm']:covercuts.extend([cylx(1.2,-40,-35,y,z),cylx(2.1,-39.1,-38,y,z)])
 cover=mf.Manifold.batch_boolean([cover,*covercuts],mf.OpType.Subtract)
 # Posts filleted inside the old envelopes; lower feet remain planar contact.
 mount=mf.Manifold.batch_boolean([roundedbox(b,p['mount_edge_radius_mm'])for b in p['mount_post_bounds_xyz_mm']]+[roundedbox(p['mount_crosshead_bounds_xyz_mm'],1.0)],mf.OpType.Add)
 # A saddle clamp spans the existing central slot, without invented trunk holes.
 mount=mount+roundedbox([[-10,10],[-6,6],[0,3]],1.0)
 center_hole=mf.Manifold.cylinder(12,1.2,circular_segments=32).translate([0,0,-6]);mount=mount-center_hole
 under_saddle=mf.Manifold.cylinder(p['under_saddle_thickness_mm'],p['under_saddle_outer_diameter_mm']/2,circular_segments=64).translate([0,0,-1-p['under_saddle_thickness_mm']])-mf.Manifold.cylinder(3,p['under_saddle_inner_diameter_mm']/2,circular_segments=48).translate([0,0,-3])
 # Top interface reaches the flat shell floor at z46, with matched pilot holes.
 for x in [-8,8]:
  hole=mf.Manifold.cylinder(9,1.1,circular_segments=32).translate([x,0,40]);mount=mount-hole;shell=shell-hole
 # Relocated compute allowance supported by a printable pedestal off the floor.
 pedestal=roundedbox([[-24,-16],[-8,8],[48.4,54]],1.0)
 solids={'body_cage':shell,'rear_cover':cover,'central_mount':mount,'compute_pedestal':pedestal}
 # Hardware is shown and integrated separately; STL files below are printed parts only.
 screws=[]
 # M2 central clamp plus nominal M2.5 wide steel washer; no plastic underside protrusion.
 shank=mf.Manifold.cylinder(6.8,.98,circular_segments=32).translate([0,0,-3.8])
 head=mf.Manifold.cylinder(1.5,1.9,circular_segments=32).translate([0,0,3])
 nut=mf.Manifold.cylinder(1.6,4.0/np.sqrt(3),circular_segments=6).translate([0,0,-3.4])-mf.Manifold.cylinder(3,1.1,circular_segments=32).translate([0,0,-4])
 screws.extend([shank+head,nut])
 for y,z in p['rear_screw_yz_mm']:
  screw=cylx(.98,-38.0,-30.0,y,z)+cylx(1.95,-38.7,-38.0,y,z)
  nut=hex_x(4.0,-32.9,-31.3,y,z)-cylx(1.05,-33,-30,y,z);screws.extend([screw,nut])
 hardware=mf.Manifold.batch_boolean(screws,mf.OpType.Add)
 tree=ET.parse(ROOT/'portrait/tab5_portrait.xml');root=tree.getroot();root.set('model','tab5_portrait_print_concept');root.find('compiler').attrib.pop('meshdir',None)
 asset=root.find('asset')
 for a in asset.findall('mesh'):
  name=a.get('name')or''
  a.set('file','../portrait/assets/tab5_rounded.obj'if name=='portrait_tab5_visual'else'../../microduck_rl/src/mjlab_microduck/robot/microduck/assets/'+Path(a.get('file')).name)
 trunk=root.find(".//body[@name='trunk_base']")
 for b in list(trunk.findall('body')):
  if b.get('name','').startswith('cube_portrait_'):trunk.remove(b)
 contact=root.find('contact')
 for c in list(contact):
  if c.get('geom1','').startswith('cube_portrait_'):contact.remove(c)
 legs=[g.get('name')for g in root.iter('geom')if g.get('name','').startswith('shell_guard_legmesh_')]
 reports=[];collision=[]
 axes=[[-39,-36.6,-33,-30,-27,-24,-12,-10,-6,0,6,10,12,24,29,32,35,38.6,41],[-40,-37.6,-34,-30,-27,-6,0,6,27,30,34,37.6,40],[-6,-3.4,-1,0,2,3,6,26,40,43.6,46,48.4,52,54,58,62,86,110,114,118,120,123.6,126]]
 def add_solid(name,solid,density,printable=True):
  assert solid.status()==mf.Error.NoError and solid.volume()>0
  m=write(solid,name)if printable else mesh(solid,.001)
  if not printable:m.export(HERE/'assets'/f'{name}.obj')
  assert m.is_watertight,(name,'not watertight');m.density=density;pr=m.mass_properties;I=pr.inertia
  body=ET.SubElement(trunk,'body',name='print_'+name,pos='0 0 0');ET.SubElement(body,'inertial',pos=vec(pr.center_mass),mass=str(pr.mass),fullinertia=vec([I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]))
  ET.SubElement(asset,'mesh',name='print_'+name+'_visual',file=f'assets/{name}.obj');ET.SubElement(body,'geom',name='print_'+name+'_visual',type='mesh',mesh='print_'+name+'_visual',rgba='.91 .92 .89 1'if printable else'.25 .27 .28 1',contype='0',conaffinity='0',group='0',mass='0')
  bounds=np.array(solid.bounding_box()).reshape(2,3);vb=0;hb=0;uncovered=0;count=0
  # Partition only intersecting cells, preventing a single hull from filling windows.
  ranges=[]
  for i,a in enumerate(axes):ranges.append([(lo,hi)for lo,hi in zip(a[:-1],a[1:])if hi>bounds[0,i]-1e-8 and lo<bounds[1,i]+1e-8])
  for bb in itertools.product(*ranges):
   s=solid^box(bb)
   if s.is_empty()or s.volume()<1e-7:continue
   for part in s.decompose():
    if part.volume()<1e-7:continue
    hull=part.hull();n=len(collision);geom=f'rounded_shell_collision_{n:04d}';fn=f'collision_{n:04d}.obj';mesh(hull,.001).export(HERE/'assets'/fn);res=max(0,(part-hull).volume());uncovered+=res;vb+=part.volume();hb+=hull.volume();count+=1
    ET.SubElement(asset,'mesh',name=geom,file='assets/'+fn);ET.SubElement(body,'geom',name=geom,type='mesh',mesh=geom,contype='1',conaffinity='3',group='3',rgba='.3 .7 1 .1',mass='0')
    for leg in legs:ET.SubElement(contact,'pair',geom1=geom,geom2=leg,condim='3',friction='1 1 .005 .0001 .0001')
    collision.append({'name':geom,'part':name,'file':'assets/'+fn,'bounds_mm':bb,'solid_volume_mm3':part.volume(),'hull_volume_mm3':hull.volume(),'uncovered_volume_mm3':res})
  assert abs(vb-solid.volume())<max(1e-5,solid.volume()*1e-7),(name,vb,solid.volume())
  reports.append({'name':name,'printable':printable,'volume_mm3':solid.volume(),'mass_kg':pr.mass,'com_m':pr.center_mass.tolist(),'inertia_kg_m2':I.tolist(),'components':len(solid.decompose()),'watertight':bool(m.is_watertight),'bounds_mm':bounds.tolist(),'collision_piece_count':count,'hull_extra_volume_ratio':hb/solid.volume()-1,'uncovered_volume_mm3':uncovered})
 for name,s in solids.items():add_solid(name,s,p['plastic_density_kg_m3'])
 add_solid('under_saddle',under_saddle,p['steel_density_kg_m3'],False)
 add_solid('rear_hardware',hardware,p['steel_density_kg_m3'],False)
 compute=add_box(trunk,'cube_portrait_compute',p['compute_mass_kg'],np.diff(np.array(p['compute_bounds_xyz_mm']),axis=1)[:,0]*.001,np.mean(p['compute_bounds_xyz_mm'],axis=1)*.001,'.05 .35 .2 1')
 for leg in legs:ET.SubElement(contact,'pair',geom1='cube_portrait_compute_collision',geom2=leg,condim='3',friction='1 1 .005 .0001 .0001')
 # Small explicit mass allowance, colocated inside solid mount (not duplicate collision).
 a=add_box(trunk,'mount_fastener_allowance',p['mount_fastener_allowance_kg'],[.016,.008,.010],[0,0,.040],'.2 .2 .2 0',collision=False);a.find('geom').set('group','3')
 ET.indent(tree);tree.write(HERE/'tab5_portrait_print.xml',encoding='utf-8',xml_declaration=True)
 s=ET.parse(ROOT/'portrait/scene_tab5_portrait.xml');s.getroot().find('include').set('file','tab5_portrait_print.xml');s.write(HERE/'scene_tab5_portrait_print.xml',encoding='utf-8',xml_declaration=True)
 report={'parameters':p,'parts':reports,'collision_parts':collision,'printed_mass_kg':sum(x['mass_kg']for x in reports if x['printable']),'total_replacement_mass_kg':sum(x['mass_kg']for x in reports)+p['compute_mass_kg']+p['mount_fastener_allowance_kg']+.1184,'collision_note':'Each connected cell-solid is covered by its convex hull. Small concavities may be filled locally; visual/collider coverage and static CAD interactions require independent audit. No window-spanning single convex hull.','stl_units':'millimetres','cad_status':'Print concept. Watertight solids and nominal assembly geometry, not validated hardware screw interfaces or strength.'}
 (HERE/'geometry_report.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items()if k not in ['collision_parts','parameters']},indent=2))
if __name__=='__main__':main()
