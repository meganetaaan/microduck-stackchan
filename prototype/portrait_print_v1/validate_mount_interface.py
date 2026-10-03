from pathlib import Path
import sys,json
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
from validate_print_design import *
p=HERE;m=mujoco.MjModel.from_xml_path(str(p/'scene_tab5_portrait_print.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
fixed=[g for g in range(m.ngeom)if m.geom_bodyid[g]==m.body('trunk_base').id and m.mesh(m.geom_dataid[g]).name=='trunk_base'];plate=mesh_solid(m,d,fixed[0])[1];tiny=mf.Manifold.cube([.002]*3,center=True);rows=[]
for n in ['central_mount','under_saddle','rear_hardware']:
    tm,s=load_solid(p/'assets'/f'{n}.obj',1000);overlap=max(0.,float((s^plate).volume()));touch=max(0.,float((s.minkowski_sum(tiny)^plate).volume()));rows.append({'part':n,'exact_solid_overlap_mm3':overlap,'expanded_1micron_intersection_mm3':touch,'bearing_contact_candidate':touch>1e-5})
r={'checks':rows,'scope':'Geometric bearing contact with original stock trunk plate only; bolt preload, frictional anti-rotation, fatigue, layer adhesion and strength are unverified. Existing slot is also a potential cable route.'}
(p/'mount_interface_validation.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
