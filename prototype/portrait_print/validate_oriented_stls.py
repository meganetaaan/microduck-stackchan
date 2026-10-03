from pathlib import Path
import sys,json
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
from validate_print_design import *
from scipy.spatial import cKDTree
src=json.loads((HERE/'print_orientation.json').read_text());build=json.loads((HERE/'geometry_report.json').read_text());expected={x['name']for x in build['parts']if x['printable']};rows=[]
for part in src['parts']:
 name=part['name'];before,bs=load_solid(HERE/'stl'/f'{name}_mm.stl');after,ss=load_solid(HERE/part['file']);T=np.array(part['rotation_matrix']);before.apply_transform(T);before.apply_translation(part['translation_mm']);delta=max(cKDTree(before.vertices).query(after.vertices)[0].max(),cKDTree(after.vertices).query(before.vertices)[0].max());rows.append({'name':name,'sha256':sha(HERE/part['file']),'watertight':bool(after.is_watertight),'winding_consistent':bool(after.is_winding_consistent),'connected_components':len(ss.decompose()),'bounds_mm':after.bounds.tolist(),'minimum_Z_mm':float(after.bounds[0,2]),'rigid_transform_determinant':float(np.linalg.det(T[:3,:3])),'bidirectional_nearest_vertex_error_mm':float(delta),'volume_change_mm3':float(abs(ss.volume()-bs.volume())),'pass':bool(after.is_watertight and after.is_winding_consistent and len(ss.decompose())==1 and abs(after.bounds[0,2])<1e-5 and delta<1e-4)})
found={x['name']for x in src['parts']};r={'expected_parts':sorted(expected),'found_parts':sorted(found),'exactly_expected_inventory':expected==found,'parts':rows,'all_pass':expected==found and all(x['pass']for x in rows),'scope':'Rigid transform, unit scale, triangle-surface integrity and placement on Z=0 only. These tests do not prove support strategy or printer/slicer compatibility.'};(HERE/'oriented_stl_validation_report.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
