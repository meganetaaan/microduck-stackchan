from pathlib import Path
import sys,json,xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from build_models import add_box,vec
out=ROOT/'diagnostics';tree=ET.parse(ROOT/'models/tab5_lowcube.xml');r=tree.getroot();r.set('model','lowcube_cutout_candidate');trunk=r.find(".//body[@name='trunk_base']")
trunk.remove(trunk.find("body[@name='cube_chin']"));rho=1240
panels=[('cube_chin_bridge',[[.0395,.041],[-.066,.066],[-.015,-.009]]),('cube_chin_center',[[.0395,.041],[-.025,.025],[-.045,-.015]])]
for side,y in [('left',[-.066,-.0645]),('right',[.0645,.066])]:
 panels.extend([('cube_side_rear_lip_'+side,[[-.0775,-.050],y,[.010,.035]]),('cube_side_front_lip_'+side,[[.025,.0395],y,[.010,.035]])])
for name,b in panels:
 a=np.array(b);s=a[:,1]-a[:,0];p=a.mean(1);add_box(trunk,name,float(np.prod(s)*rho),s,p,'.92 .91 .85 1')
for side in ['left','right']:
 b=trunk.find(f"body[@name='cube_side_{side}']");g=b.find('geom');s=np.fromstring(g.get('size'),sep=' ')*2;s[2]=.0715-.035;pos=np.fromstring(b.get('pos'),sep=' ');pos[2]=(.0715+.035)/2;b.set('pos',vec(pos));g.set('size',vec(s/2));i=b.find('inertial');mass=float(np.prod(s)*rho);i.set('mass',str(mass));i.set('diaginertia',vec(mass/12*np.array([s[1]**2+s[2]**2,s[0]**2+s[2]**2,s[0]**2+s[1]**2])))
contact=r.find('contact');removed=[]
for p in list(contact):
 if p.get('geom1')=='cube_chin_collision':removed.append(p.get('geom2'));contact.remove(p)
for name,_ in panels:
 for g in removed:ET.SubElement(contact,'pair',geom1=name+'_collision',geom2=g,condim='3',friction='1 1 .005 .0001 .0001')
ET.indent(tree);tree.write(out/'lowcube_cutout_candidate.xml',encoding='utf-8',xml_declaration=True)
s=ET.parse(ROOT/'models/scene_tab5_lowcube.xml');s.getroot().find('include').set('file','lowcube_cutout_candidate.xml');s.write(out/'scene_lowcube_cutout_candidate.xml',encoding='utf-8',xml_declaration=True)
(out/'cutout_candidate_parameters.json').write_text(json.dumps({'status':'Diagnostic only; not selected or replacing final model','unchanged':'Tab5 size and location, case exterior bounds, all leg mechanics','front_arches_trunk_mm':{'left_y':[25,66],'right_y':[-66,-25],'z':[-45,-15],'depth_x':[39.5,41]},'center_chin_width_mm':50,'remaining_chin_top_band_height_mm':6,'side_window_top_z_mm':35,'side_upper_notch_x_mm':[-50,25],'previous_side_window_top_z_mm':10,'clearance_target_mm':3},indent=2))
