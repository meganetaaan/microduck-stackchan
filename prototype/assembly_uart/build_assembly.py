"""UART assembly concept. Millimetre solids, explicit masses, real collision routes.
The robot/Tab5 geometry is inherited; new components are sourced or labeled proxies.
"""
from pathlib import Path
import sys,json,itertools,hashlib,shutil,xml.etree.ElementTree as ET
import numpy as np,trimesh,manifold3d as mf
HERE=Path(__file__).resolve().parent;ROOT=HERE.parent
sys.path.insert(0,str(ROOT/'portrait_print'));from build_print_design import box,roundedbox,rr,front,cylx,mesh
sys.path.insert(0,str(ROOT));from build_models import vec

def loadsolid(path):
 m=trimesh.load(path,process=True);return mf.Manifold(mf.Mesh64(np.asarray(m.vertices,dtype=np.float64),np.asarray(m.faces,dtype=np.uint64)))
def cylz(r,z0,z1,x,y,n=32):return mf.Manifold.cylinder(z1-z0,r,circular_segments=n).translate([x,y,z0])
def hexz(af,z0,z1,x,y):return cylz(af/np.sqrt(3),z0,z1,x,y,6)
def capsule(a,b,r):
 a=np.array(a,float);b=np.array(b,float);return mf.Manifold.batch_hull([mf.Manifold.sphere(r,12).translate(a),mf.Manifold.sphere(r,12).translate(b)])
def union(parts):return mf.Manifold.batch_boolean(parts,mf.OpType.Add)
def subtract(a,parts):return mf.Manifold.batch_boolean([a,*parts],mf.OpType.Subtract)
def main():
 p=json.loads((HERE/'parameters.json').read_text());src=ROOT/'portrait_print';r=ET.parse(src/'tab5_portrait_print.xml');root=r.getroot();root.set('model','tab5_uart_assembly_v1');asset=root.find('asset');trunk=root.find(".//body[@name='trunk_base']");contact=root.find('contact')
 for a in asset.findall('mesh'):
  f=a.get('file');a.set('file','../portrait_print/'+f if f.startswith('assets/')else f)
 removed={'print_body_cage','print_central_mount','print_compute_pedestal','cube_portrait_compute','mount_fastener_allowance'};removed_geoms=set();removed_mass=0.
 for b in list(trunk.findall('body')):
  if b.get('name')in removed:
   removed_geoms.update(g.get('name')for g in b.iter('geom'));removed_mass+=float(b.find('inertial').get('mass'));trunk.remove(b)
 for c in list(contact):
  if c.get('geom1')in removed_geoms or c.get('geom2')in removed_geoms:contact.remove(c)
 used={g.get('mesh')for g in root.iter('geom')if g.get('mesh')}
 for a in list(asset.findall('mesh')):
  if (a.get('name')or Path(a.get('file')).stem)not in used:asset.remove(a)
 legs=[g.get('name')for g in root.iter('geom')if g.get('name','').startswith('shell_guard_legmesh_')];parts=[];colliders=[];allowed=[]
 axes=[[-39,-36.6,-33,-30,-27,-24,-20,-12,-10,-6,0,6,10,12,24,29,32,35,38.6,41],[-40,-37.6,-34,-30,-27,-6,0,6,27,30,34,37.6,40],[-6,-3.4,-1,0,2,3,6,26,40,43.6,46,48.4,52,54,58,62,86,110,114,118,120,123.6,126]]

 def add(name,solid,color,mass_g=None,density=None,printable=False,partition=False,proxy=False,source='assumed component envelope',uncertainty=None):
  assert solid.status()==mf.Error.NoError and solid.volume()>0,(name,solid.status())
  tm=mesh(solid,.001);tm.density=density or ((mass_g/1000)/(solid.volume()*1e-9));props=tm.mass_properties;I=props.inertia
  assert tm.is_watertight,(name,'not watertight')
  fn='assets/'+name+'.obj';tm.export(HERE/fn)
  if printable:mesh(solid).export(HERE/'stl'/(name+'_mm.stl'))
  b=ET.SubElement(trunk,'body',name='assembly_'+name,pos='0 0 0');ET.SubElement(b,'inertial',mass=str(props.mass),pos=vec(props.center_mass),fullinertia=vec([I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]));mn='assembly_'+name+'_visual';ET.SubElement(asset,'mesh',name=mn,file=fn);ET.SubElement(b,'geom',name=mn,type='mesh',mesh=mn,rgba=color,contype='0',conaffinity='0',group='0',mass='0')
  pieces=[]
  if partition:
   bounds=np.array(solid.bounding_box()).reshape(2,3);partition_axes=axes
   if name=='electronics_carrier':partition_axes=[[-34,-31.6,-29.6,-24.6,-20,-12,-10,-6,0,6,10,12,20,22.4,24,28,31],[-35,-34,-28,-21,-18,-10,-6,0,6,24,34,35],[48.4,50.8,56,59.6,62,63,66,69,75,81,83,86,90,93,98,102]]
   elif name=='power_feedthrough_grommet':partition_axes=[[-25,-20,-15],[-5,0,5],[45.5,46,50.8,51.3]]
   ranges=[[(lo,hi)for lo,hi in zip(a[:-1],a[1:])if hi>bounds[0,i]-1e-7 and lo<bounds[1,i]+1e-7]for i,a in enumerate(partition_axes)]
   for cell in itertools.product(*ranges):
    cut=solid^box(cell)
    if cut.is_empty()or cut.volume()<1e-7:continue
    pieces.extend(x.hull()for x in cut.decompose()if x.volume()>1e-5 and np.min(np.ptp(np.array(x.bounding_box()).reshape(2,3),axis=0))>0.001)
  else:pieces=[solid.hull()]
  vol=0
  for i,h in enumerate(pieces):
   n=f'cube_uart_{name}_{i:04d}';fn='assets/'+n+'.obj';mesh(h,.001).export(HERE/fn);ET.SubElement(asset,'mesh',name=n,file=fn);ET.SubElement(b,'geom',name=n,type='mesh',mesh=n,contype='1',conaffinity='3',group='3',mass='0',rgba='.4 .7 1 .1')
   for leg in legs:ET.SubElement(contact,'pair',geom1=n,geom2=leg,condim='3',friction='1 1 .005 .0001 .0001')
   colliders.append({'name':n,'part':name,'file':fn});vol+=h.volume()
  parts.append({'name':name,'visual_file':'assets/'+name+'.obj','mass_kg':props.mass,'center_of_mass_m':props.center_mass.tolist(),'inertia_kg_m2':I.tolist(),'bounds_mm':np.array(solid.bounding_box()).reshape(2,3).tolist(),'volume_mm3':solid.volume(),'printable':printable,'proxy':proxy,'source':source,'uncertainty_mass_g':uncertainty,'collision_piece_count':len(pieces),'convex_sum_extra_volume_ratio':vol/solid.volume()-1,'watertight':bool(tm.is_watertight)})
  return b
 # Keep the selected visible shell. Make actual holes for the lower M5-Bus plug and power loom.
 cage=loadsolid(src/'stl/body_cage_mm.stl');mount=loadsolid(src/'stl/central_mount_mm.stl')
 feed=cylz(3.5,44,54,-20,0)
 relief=front(rr([-22,22],[49,62],2),30,43)
 cage=subtract(cage,[feed,relief,*[cylx(3.0,28.5,32,y,z)for y,z in [[-36,75],[36,75],[-36,122],[36,122]]]])
 # Two body/carrier screws use captive M2 nuts in the printed top crosshead.
 for x in [-8,8]:
  bore=cylz(1.2,40.5,54,x,0);cage=cage-bore;mount=mount-bore
  pocket=hexz(4.3,41.5,43.3,x,0)+box([[x-2.15,x+2.15],[0,7],[41.5,43.3]])
  mount=mount-pocket
 add('body_cage',cage,'.91 .92 .89 1',density=1240,printable=True,partition=True,source='selected flush-front shell; actual M5-Bus relief and power feedthrough added')
 add('central_mount',mount,'.91 .92 .89 1',density=1240,printable=True,partition=True,source='original printed mount with two M2 captive-nut slots and clearance bores')
 # Official Radxa PCB datum: X 0..65, Y0..30, Z-1.6..0 -> trunk Y=X-32.5,Z=Y+56,X=Z-23.
 holes=np.array([[3.549904,3.599942],[3.599942,26.450036],[61.399928,26.500074],[61.399928,3.599942]])
 H=np.c_[holes[:,0]-32.5,holes[:,1]+68]
 pcb=front(rr([-32.5,32.5],[68,98],3),-24.6,-23)
 pcb=subtract(pcb,[cylx(1.4,-25,-22,y,z)for y,z in H])
 add('radxa_pcb',pcb,'.08 .42 .23 1',mass_g=6.2,source='Radxa Zero3W official CAD v1.11:65x30x1.6mm; four exact CAD-derived Ø2.8 holes',uncertainty=[5,8])
 # Separate conservative component envelopes keep the sourced overall bounds; details are schematic.
 under=box([[-27.4841,-24.6],[-24,24],[73,90]])
 add('radxa_under_components',under,'.22 .24 .25 1',mass_g=1.8,proxy=True,source='official CAD overall underside depth; component grouping is a proxy',uncertainty=[1,3])
 chips=box([[-23,-19.5],[-18,21],[74,89]])
 add('radxa_top_components',chips,'.12 .13 .15 1',mass_g=2.0,proxy=True,source='provisional grouped top-component shape within official CAD bound',uncertainty=[1,3])
 # 40-pin grid from official STEP; visual header protrudes8.5mm from PCB top.
 hdr=box([[-23,-20.5],[8.32-32.5-1.27,56.58-32.5+1.27],[92.1351,97.2151]])
 pins=[]
 for i in range(20):
  for z in [93.4051,95.9451]:pins.append(box([[-20.5,-14.5],[8.32+2.54*i-32.5-.32,8.32+2.54*i-32.5+.32],[z-.32,z+.32]]))
 add('radxa_gpio_header',hdr,'.08 .09 .1 1',mass_g=1.1,source='official CAD40pin grid2.54mm; simplified insulating body',uncertainty=[.7,1.5])
 add('radxa_gpio_pins',union(pins),'.71 .64 .31 1',mass_g=.9,source='official CAD40pin grid and8.5mmtopextent; squarepins simplified',uncertainty=[.6,1.2])
 # Conservative guard covers the entire official CAD envelope, including ports omitted by procedural details.
 guard_body=root.find(".//body[@name='assembly_radxa_pcb']")
 limits=np.array([[-27.4841,-14.5],[-32.51347,32.50165],[66.5,98.00553]])*.001
 ET.SubElement(guard_body,'geom',name='cube_uart_radxa_official_envelope',type='box',pos=vec(limits.mean(1)),size=vec(np.diff(limits,axis=1)[:,0]/2),contype='1',conaffinity='3',group='3',mass='0')
 for leg in legs:ET.SubElement(contact,'pair',geom1='cube_uart_radxa_official_envelope',geom2=leg,condim='3')
 # Include original controller/HAT functions as an explicit uncertain stack, not an omitted mass.
 hat=box([[-14.5,-12.9],[-33,33],[67,101]])
 add('robot_hat_pcb_proxy',hat,'.18 .35 .48 1',mass_g=9,proxy=True,source='MicroDuck Robot HAT retained; exact mechanical files unavailable,66x34mm provisional',uncertainty=[6,15])
 add('robot_hat_components_proxy',box([[-12.9,-3],[-30,30],[69,97]]),'.52 .37 .18 .75',mass_g=10,proxy=True,source='provisional controller/component keepout; not actual CAD',uncertainty=[6,18])
 # Printable removable carrier uses the four verified Radxa holes; its floor shares two M2 mounts.
 base=box([[-31.6,31],[-32,32],[48.4,50.8]])
 upright=box([[-31.6,-29.6],[-34,34],[66,102]])+box([[-31.6,-29.6],[-18,-14],[50.8,66]])+box([[-31.6,-29.6],[14,18],[50.8,66]])
 upright=upright-front(rr([-24,24],[75,93],3),-33,-28)
 standoffs=[cylx(2.7,-29.6,-24.6,y,z)for y,z in H]
 clip1=box([[-13,-5],[-18,-10],[50.8,56]])-cylx(1.4,-14,-4,-14,53.6)
 clip2=(box([[20,28],[-28,-21],[62,69]])-cylz(1.4,61,70,24,-24.5))+box([[20,22.4],[-28,-21],[50.8,62]])
 carrier=union([base,upright,*standoffs,clip1,clip2]);carrier=subtract(carrier,[feed,*[cylx(1.4,-35,-23,y,z)for y,z in H],*[cylz(1.2,47,53,x,0)for x in [-8,8]]])
 add('electronics_carrier',carrier,'.73 .78 .82 1',density=1240,printable=True,partition=True,source='new parametric carrier, nominal hardware fit pending print coupon')
 for i,(y,z)in enumerate(H):
  shaft=cylx(1.24,-35,-23,y,z);head=cylx(2.25,-23,-20.5,y,z);nut=cylx(5/np.sqrt(3),-33.6,-31.6,y,z,6)-cylx(1.3,-34,-31,y,z)
  add(f'radxa_M2p5_{i}',shaft+head+nut,'.32 .35 .38 1',density=7850,source='nominal M2.5x12 screw and2mmnut; verify actual head/engagement')
 for i,x in enumerate([-8,8]):
  bolt=cylz(.98,40.8,50.8,x,0)+cylz(1.9,50.8,52.4,x,0);nut=hexz(4,41.6,43.2,x,0)-cylz(1.05,41,44,x,0)
  add(f'carrier_M2_{i}',bolt+nut,'.32 .35 .38 1',density=7850,source='nominalM2x10body/carrierbolt andcaptivenut; printfit unverified')
 # Tab5 fastener visible portions: insertion length into device is deliberately not invented.
 tab_positions=[[-36,75],[36,75],[-36,122],[36,122]]
 for i,(y,z)in enumerate(tab_positions):
  add(f'tab5_M3_visible_{i}',cylx(1.49,32,41,y,z)+cylx(2.75,29,32,y,z),'.35 .37 .4 1',density=7850,source='M3head and9mmexternalshank only; internal engagementunknown, notcomplete screw length')
 # Power branch protective module is a reserved PCB/component assembly; no fuse rating or connector family claim.
 pad=box([[8,30],[-17,17],[50.8,51.3]])-cylz(2.2,50,54,8,0)
 add('protection_mount_pad',pad,'.22 .24 .25 1',mass_g=.45,proxy=True,source='provisional insulated adhesive mountingpad; retention/temperatureunverified',uncertainty=[.2,.8])
 tray=box([[8,30],[-17,17],[51.3,53.7]])
 for x in [11,27]:
  for y in [-13,13]:tray=tray+cylz(2,53.7,56,x,y)
 tray=tray-cylz(2.2,50,54,8,0)
 add('protection_tray',tray,'.74 .78 .82 1',density=1240,printable=True,source='provisional modular protection-board support; no fixedconnectorcutout')
 add('protection_pcb_proxy',box([[9,29],[-15,15],[56,57.6]]),'.13 .45 .22 1',mass_g=1.8,proxy=True,source='20x30mm board allocation; source-side fuse/rating/circuit TBD',uncertainty=[1,3])
 add('branch_protection_proxy',box([[16,23],[-4,6],[57.6,64.6]]),'.22 .23 .25 1',mass_g=2.2,proxy=True,source='protectioncomponentallocation; no electricalratingsselected',uncertainty=[1,5])
 add('power_input_connector_proxy',box([[9,15],[-13,-5],[57.6,64.6]]),'.2 .43 .27 1',mass_g=.8,proxy=True,source='modular2poleadapterenvelope; family/pitchnotselected',uncertainty=[.4,1.5])
 add('power_output_connector_proxy',box([[23,29],[7,15],[57.6,64.6]]),'.2 .43 .27 1',mass_g=.8,proxy=True,source='modular2poleadapterenvelope; family/pitchnotselected',uncertainty=[.4,1.5])
 # Grommet bore is real; power pair passes through a modeled empty opening.
 grommet=cylz(4.5,45.5,46,-20,0)+cylz(3.4,46,50.8,-20,0)+cylz(4.5,50.8,51.3,-20,0);grommet=grommet-cylz(2,44,52,-20,0)
 add('power_feedthrough_grommet',grommet,'.08 .09 .1 1',mass_g=.35,proxy=True,partition=True,source='nominalØ7feedthrough withØ4bore, flexiblegrommet rigidproxy; fitTBD',uncertainty=[.2,.6])
 # Approximate drawing-based Tab5 back connector centers; these are beyond its rear plane, never inside the screen body.
 add('tab5_m5bus_plug_proxy',box([[32,41],[-19.5,19.5],[50,56]]),'.09 .1 .11 1',mass_g=2,proxy=True,source='rear drawing-scaledM5Bus2x15plug allocation; housing/engagementTBD',uncertainty=[1,3])
 add('tab5_J7_plug_proxy',box([[35,41],[-5.5,5.5],[58.3,64.3]]),'.88 .87 .81 1',mass_g=.7,proxy=True,source='J7 approximate drawingposition, supplied1.25mm6Pplug; exacthousingTBD',uncertainty=[.3,1])
 add('uart_hat_takeoff_proxy',box([[-3,3],[-13,1],[92,99]]),'.09 .1 .11 1',mass_g=.8,proxy=True,source='GPIO16/18/GNDtakeoff orinterposer allocation; HATaccessnotconfirmed',uncertainty=[.3,2])
 # Fixed looms are conservative rigid envelopes, no flexible-wire physics claim.
 def cable(name,path,radius,mass_g,color,source):
  solids=[capsule(a,b,radius)for a,b in zip(path[:-1],path[1:])];whole=union(solids)
  # Individual convex segment colliders avoid filling a loop's interior.
  body=add(name,whole,color,mass_g=mass_g,proxy=True,source=source,uncertainty=[mass_g*.65,mass_g*1.5])
  for g in list(body.findall('geom')):
   if g.get('name','').startswith('cube_uart_'):
    oldname=g.get('name');body.remove(g)
    for c in list(contact):
     if c.get('geom1')==oldname:contact.remove(c)
    for a in list(asset):
     if a.get('name')==oldname:asset.remove(a)
    colliders[:]=[c for c in colliders if c['name']!=oldname]
  for i,s in enumerate(solids):
   n=f'cube_uart_{name}_seg_{i:03d}';fn='assets/'+n+'.obj';mesh(s,.001).export(HERE/fn);ET.SubElement(asset,'mesh',name=n,file=fn);ET.SubElement(body,'geom',name=n,type='mesh',mesh=n,contype='1',conaffinity='3',group='3',mass='0')
   for leg in legs:ET.SubElement(contact,'pair',geom1=n,geom2=leg,condim='3',friction='1 1 .005 .0001 .0001')
   colliders.append({'name':n,'part':name,'file':fn})
  parts[-1]['path_mm']=path;parts[-1]['routing_radius_mm']=radius;parts[-1]['collision_piece_count']=len(solids);parts[-1]['convex_sum_extra_volume_ratio']=None
 cable('robot_power_pigtail',[[-20,0,43.3],[-20,0,48],[-20,0,51.5],[-19,0,53.6],[-16,-7,53.6],[-15,-14,53.6],[-13,-14,53.6],[-3,-14,53.6],[-1,-21,54],[0,-24,56]],1.1,1.5,'.65 .1 .1 1','2wirepowerloom envelope; robot-end matinglocationTBD, no actual batteryterminal claimed')
 cable('branch_input_loom',[[8,-24,56],[12,-20,60],[12,-12,60]],1.1,.55,'.65 .1 .1 1','short protectedbranchinput frominternalreplaceableadapter toprotectionmodule')
 cable('tab5_power_loom',[[26,12,60],[30,18,64],[33,17,66],[34,10,65],[35,4,61.3]],.9,.9,'.78 .16 .12 1','J7powerpair route; supplied6Punused4wireendsindividuallyinsulated, ratingTBD')
 cable('uart_loom',[[0,-7,96],[5,-20,96],[13,-25,89],[22,-24.5,72],[24,-24.5,69],[24,-24.5,62],[27,-24,57],[30,-22,53],[32,-17,53]],1.0,1.4,'.13 .37 .79 1','3wire3.3VUART+GND loom; keepawayfrommotorleadroute, no RS232voltage')
 # Strain-relief guide saddles are integrated into the printable carrier above.
 add('robot_power_handoff_proxy',box([[0,8],[-28,-20],[53,59]]),'.11 .12 .13 1',mass_g=.7,proxy=True,source='removable source-side pigtail endpoint allocation, actual branchconnectorlocationTBD',uncertainty=[.3,1.5])
 # Provisional hardware total is represented physically except unknown internal M3 engagement.
 m3allow=box([[34,38],[-3,3],[99,105]])
 add('unresolved_fastener_mass',m3allow,'.3 .32 .34 0',mass_g=1.0,proxy=True,source='1g allowanceforunknowninternalM3engagement andsmallclips, nofitclaim',uncertainty=[0,2])
 ET.indent(r);r.write(HERE/'tab5_assembly_uart.xml',encoding='utf-8',xml_declaration=True)
 sc=ET.parse(src/'scene_tab5_portrait_print.xml');sc.getroot().find('include').set('file','tab5_assembly_uart.xml');sc.write(HERE/'scene_tab5_assembly_uart.xml',encoding='utf-8',xml_declaration=True)
 report={'parameters':p,'parts':parts,'collision_parts':colliders,'removed_old_mass_kg':removed_mass,'added_assembly_mass_kg':sum(x['mass_kg']for x in parts),'body_source_sha256':hashlib.sha256((src/'tab5_portrait_print.xml').read_bytes()).hexdigest(),'radxa_official_cad_hole_centers_xy_mm':holes.tolist(),'radxa_cad_to_trunk':'[X,Y,Z]trunk=[Zcad-23,Xcad-32.5,Ycad+68] mm','source_scope':'OfficialRadxaCAD dimensions andholecoordinates; componentgrouping/HAT/protection/connector/wiring areprovisional. No physicalfit/strength/electricalratingcertification.','assembly_contact_note':'Fasteners occupy intended clearances; connectors meet cableterminals. Internalcontacts are welded. Independentactualsolid checks required; preserved leg explicitpairsremain.'}
 shutil.copyfile(src/'stl/rear_cover_mm.stl',HERE/'stl/rear_cover_mm.stl')
 (HERE/'assembly_report.json').write_text(json.dumps(report,indent=2));print('Built',len(parts),'parts',len(colliders),'newcolliders, added mass',report['added_assembly_mass_kg'],'removed',removed_mass)
if __name__=='__main__':main()
