"""Export the designed upper assembly, without redistributing original MicroDuck leg meshes."""
from pathlib import Path
import xml.etree.ElementTree as E,json
import numpy as np,trimesh
H=Path(__file__).resolve().parent
root=E.parse(H/'tab5_assembly_uart.xml').getroot();assets={a.get('name')or Path(a.get('file')).stem:(H/a.get('file')).resolve()for a in root.find('asset').findall('mesh')};trunk=root.find(".//body[@name='trunk_base']");scene=trimesh.Scene();names=[]
for b in trunk.findall('body'):
 name=b.get('name','')
 if not(name.startswith('assembly_')or name in ['print_rear_cover','print_under_saddle','print_rear_hardware','tab5']):continue
 bp=np.fromstring(b.get('pos','0 0 0'),sep=' ')
 for g in b.findall('geom'):
  if g.get('group')=='3':continue
  color=np.fromstring(g.get('rgba','.8 .8 .8 1'),sep=' ')
  if color[3]<=0:continue
  typ=g.get('type','sphere');size=np.fromstring(g.get('size',''),sep=' ')
  if g.get('mesh'):m=trimesh.load(assets[g.get('mesh')],process=True)
  elif typ=='box':m=trimesh.creation.box(extents=2*size)
  elif typ=='ellipsoid':m=trimesh.creation.icosphere(subdivisions=2);m.apply_scale(size)
  elif typ=='cylinder':m=trimesh.creation.cylinder(radius=size[0],height=2*size[1],sections=32)
  else:continue
  q=np.fromstring(g.get('quat','1 0 0 0'),sep=' ');T=trimesh.transformations.quaternion_matrix(q);T[:3,3]=bp+np.fromstring(g.get('pos','0 0 0'),sep=' ');m.apply_transform(T);m.visual.vertex_colors=np.tile(np.rint(color*255).astype(np.uint8),(len(m.vertices),1));n=g.get('name',name);scene.add_geometry(m,node_name=n,geom_name=n);names.append(n)
scene.export(H/'upper_assembly.glb');(H/'upper_assembly_export.json').write_text(json.dumps({'file':'upper_assembly.glb','units':'metres','node_count':len(names),'nodes':names,'scope':'Designed upper assembly in trunk coordinates, including approximate Tab5. Original MicroDuck legs/bodyCAD intentionally omitted; full articulated model remainsMJCF withpinned upstreambootstrap. Inertial-only allowance hidden.'},indent=2))
# Suggested build orientation only; no slicer or printer compatibility claim.
out=H/'stl_print_oriented';out.mkdir(exist_ok=True);rows=[]
for p in sorted((H/'stl').glob('*.stl')):
 m=trimesh.load(p,process=True);name=p.stem[:-3]if p.stem.endswith('_mm')else p.stem
 if name in ['body_cage','rear_cover']:T=trimesh.transformations.rotation_matrix(np.pi/2,[0,1,0])
 elif name=='central_mount':T=trimesh.transformations.rotation_matrix(np.pi/2,[1,0,0])
 else:T=np.eye(4)
 m.apply_transform(T);shift=np.r_[-m.bounds.mean(0)[:2],-m.bounds[0,2]];m.apply_translation(shift);dest=out/(name+'_print_mm.stl');m.export(dest);rows.append({'name':name,'file':str(dest.relative_to(H)),'rotation':T.tolist(),'translation_mm':shift.tolist(),'bounds_mm':m.bounds.tolist(),'watertight':bool(m.is_watertight),'components':len(m.split()),'note':'Suggested only; slice/support/fit check required. Carrier base and protection tray toward bed.'})
(H/'print_orientation.json').write_text(json.dumps({'units':'mm','parts':rows,'note':'Five printableparts; rigid rotations/translations only. NotG-code or printer-certified orientations.'},indent=2));print('GLB nodes',len(names),'printparts',len(rows))
