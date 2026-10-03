"""Real rounded hollow shell plus a conservative hole-preserving convex cover.

Constructed in millimetres via Manifold booleans, exported in metres. The prior
angular model remains untouched. Collision cells include every point of the
watertight physical shell; no single convex hull is taken across an opening.
"""
from pathlib import Path
import sys,json,hashlib,itertools,xml.etree.ElementTree as ET
import numpy as np
import manifold3d as mf
import trimesh
from build_models import ROOT,vec

OUT=ROOT/'rounded';ASSETS=OUT/'assets';ASSETS.mkdir(parents=True,exist_ok=True)
R=6.0;T=1.5;RHO=1240.0
B=np.array([[-79.,41.],[-66.,66.],[-45.,73.]])

def box(bounds):
    a=np.array(bounds,dtype=float);return mf.Manifold.cube(a[:,1]-a[:,0]).translate(a[:,0])
def rect(x,z):return mf.CrossSection.square([x[1]-x[0],z[1]-z[0]]).translate([x[0],z[0]])
def roundrect(x,z,r):
    return mf.CrossSection.square([x[1]-x[0]-2*r,z[1]-z[0]-2*r],center=True).offset(r,circular_segments=48).translate([(x[0]+x[1])/2,(z[0]+z[1])/2])
def extrudefront(section):return section.extrude(40).transform([[0,0,1,25],[1,0,0,0],[0,1,0,0]])
def extrudeside(section,y0,y1):return section.extrude(y1-y0).transform([[1,0,0,0],[0,0,1,y0],[0,1,0,0]])
def roundedbox(bounds,r):
    a=np.array(bounds);c=a.mean(1);s=a[:,1]-a[:,0]
    return mf.Manifold.cube(s-2*r,center=True).minkowski_sum(mf.Manifold.sphere(r,48)).translate(c)
def mesh_of(solid):
    o=solid.to_mesh64();return trimesh.Trimesh(np.asarray(o.vert_properties)[:,:3]*.001,np.asarray(o.tri_verts),process=False)
def export(solid,path):
    m=mesh_of(solid);m.export(path);return m

def main():
    outer=roundedbox(B,R);inner=roundedbox(B+np.array([[T,-T]]*3),R-T)
    shell=outer-inner
    # Open underside, front screen aperture, and explicitly preserved prior voids.
    bottom=box([[-77.5,39.5],[-64.5,64.5],[-65,-35]])
    screen=extrudefront(rect([-64,64],[-9,71]))
    # A 5 mm radius grows each foot opening inward/downward. Its top stays -15,
    # retaining the full 6 mm bridge beneath Tab5 instead of eating that band.
    leftarch=roundrect([20,90],[-80,-15],5)
    rightarch=roundrect([-90,-20],[-80,-15],5)
    frontvoids=[extrudefront(leftarch),extrudefront(rightarch)]
    # Rounded supersets of the lower and upper angular side windows. The union
    # is inflated by 1 mm to soften the transition while expanding clearance.
    side=(roundrect([-70,49],[-90,10],8)+roundrect([-58,33],[-90,35],8)).offset(1,circular_segments=32)
    sidevoids=[extrudeside(side,-85,-58),extrudeside(side,58,85)]
    shell=mf.Manifold.batch_boolean([shell,bottom,screen,*frontvoids,*sidevoids],mf.OpType.Subtract)
    # Hidden front-frame ribs join the chin to the roof behind the unchanged Tab5.
    # 4x3mm sections stay outside the preserved foot/hip opening volumes.
    ribs=[box([[35.5,39.5],yy,[-15,73]])^outer for yy in [[50,53],[-53,-50]]]
    shell=mf.Manifold.batch_boolean([shell,*ribs],mf.OpType.Add)
    assert not shell.is_empty() and shell.status()==mf.Error.NoError
    assert len(shell.decompose())==1, 'Shell must be one joined solid'
    visual=export(shell,ASSETS/'rounded_shell.obj')
    assert visual.is_watertight and visual.volume>0
    visual.density=RHO;props=visual.mass_properties
    oldvoids=[box([[39.5,42],[25,67],[-46,-15]]),box([[39.5,42],[-67,-25],[-46,-15]])]
    for yy in [[-67,-64.5],[64.5,67]]:
        oldvoids += [box([[-62,42],yy,[-46,10]]),box([[-50,25],yy,[10,35]])]
    removed_voxel_overlap=sum((shell^v).volume() for v in oldvoids)
    assert removed_voxel_overlap<1e-7,removed_voxel_overlap
    # Cell boundaries include every plane of the established angular voids.
    # Hence no convex hull is allowed to span through a previously empty slot.
    axes=[[-79,-77.5,-73,-71,-62,-58,-50,-25,0,20,25,33,35,35.5,39.5,41],
          [-66,-64.5,-60,-58,-53,-50,-25,-20,0,20,25,50,53,58,60,64.5,66],
          [-45,-43.5,-39,-21,-15,-9,2,10,27,35,37,65.5,67,71.5,73]]
    parts=[];vsum=0.;hvol=0.;containment=0.;void_overlap=0.
    for ix,iy,iz in itertools.product(*[range(len(a)-1) for a in axes]):
        bounds=[[axes[j][i],axes[j][i+1]] for j,i in enumerate([ix,iy,iz])]
        part=shell^box(bounds)
        if part.is_empty() or part.volume()<1e-7:continue
        # Separate disconnected chunks before the hull, for tighter proxies.
        for component in part.decompose():
            if component.volume()<1e-7:continue
            hull=component.hull();n=len(parts);path=ASSETS/f'collision_{n:03d}.obj'
            export(hull,path)
            residual=(component-hull).volume();containment+=max(0,residual)
            vo=sum((hull^v).volume() for v in oldvoids);void_overlap+=max(0,vo)
            assert residual<1e-6 and vo<1e-6,(residual,vo)
            parts.append({'name':f'rounded_shell_collision_{n:03d}','file':str(path.relative_to(OUT)), 'cell_bounds_mm':bounds,'solid_volume_mm3':component.volume(),'hull_volume_mm3':hull.volume(),'uncovered_volume_mm3':residual,'prior_void_overlap_mm3':vo})
            vsum+=component.volume();hvol+=hull.volume()
    assert abs(vsum-shell.volume())<1e-4*shell.volume(),(vsum,shell.volume())
    tree=ET.parse(ROOT/'diagnostics/lowcube_cutout_candidate.xml');r=tree.getroot();r.set('model','tab5_lowcube_rounded')
    # Assets exported from this directory; upstream meshes retain relative paths.
    r.find('compiler').attrib.pop('meshdir',None)
    asset=r.find('asset')
    for a in asset.findall('mesh'):
        a.set('file','../../microduck_rl/src/mjlab_microduck/robot/microduck/assets/'+Path(a.get('file')).name)
    trunk=r.find(".//body[@name='trunk_base']")
    for b in list(trunk.findall('body')):
        if b.get('name','').startswith('cube_'):trunk.remove(b)
    contact=r.find('contact')
    for p in list(contact):
        if p.get('geom1','').startswith('cube_'):contact.remove(p)
    body=ET.SubElement(trunk,'body',name='rounded_shell',pos='0 0 0')
    I=props.inertia
    ET.SubElement(body,'inertial',pos=vec(props.center_mass),mass=str(props.mass),fullinertia=vec([I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]))
    ET.SubElement(asset,'mesh',name='rounded_shell_visual',file='assets/rounded_shell.obj')
    ET.SubElement(body,'geom',name='rounded_shell_visual',type='mesh',mesh='rounded_shell_visual',contype='0',conaffinity='0',group='0',rgba='.90 .91 .88 1',mass='0')
    guards=[g.get('name') for g in r.iter('geom') if g.get('name','').startswith('shell_guard_legmesh_')]
    for p in parts:
        ET.SubElement(asset,'mesh',name=p['name'],file=p['file'])
        ET.SubElement(body,'geom',name=p['name'],type='mesh',mesh=p['name'],contype='1',conaffinity='3',group='3',rgba='.3 .7 1 .15',mass='0')
        for g in guards:ET.SubElement(contact,'pair',geom1=p['name'],geom2=g,condim='3',friction='1 1 .005 .0001 .0001')
    ET.indent(tree);tree.write(OUT/'tab5_lowcube_rounded.xml',encoding='utf-8',xml_declaration=True)
    scene=ET.parse(ROOT/'models/scene_tab5_lowcube.xml');scene.getroot().find('include').set('file','tab5_lowcube_rounded.xml');scene.write(OUT/'scene_tab5_lowcube_rounded.xml',encoding='utf-8',xml_declaration=True)
    report={'outer_design_bounds_mm':B.tolist(),'outer_round_radius_mm':R,'nominal_wall_thickness_mm':T,'foot_arch_radius_mm':5,'side_window_corner_radius_mm':8,'side_window_expansion_mm':1,'front_frame_ribs_mm':{'x':[35.5,39.5],'y':[[-53,-50],[50,53]],'z':[-15,73]},'density_kg_m3':RHO,'actual_mesh_bounds_m':visual.bounds.tolist(),'shell_mass_kg':props.mass,'shell_com_m':props.center_mass.tolist(),'shell_inertia_kg_m2':I.tolist(),'watertight':bool(visual.is_watertight),'connected_components':len(shell.decompose()),'shell_volume_mm3':shell.volume(),'collision_piece_count':len(parts),'cover_solid_volume_sum_mm3':vsum,'collision_hull_total_volume_mm3':hvol,'conservative_extra_volume_ratio':hvol/shell.volume()-1,'uncovered_volume_sum_mm3':containment,'prior_angular_void_overlap_mm3':void_overlap,'containment_method':'Boolean-exact cell intersections partition the complete solid; each connected piece is enclosed by its convex hull. Verified residual component minus hull volume and preserved angular void intersections. Numerical Manifold meshes, not analytic CAD certification.','collision_caveat':'Convex hulls conservatively cover each cell piece; small concavities within each cell may collide earlier than the visible mesh. Existing rectangular leg voids are not bridged. No single shell-wide convex hull is used.','parts':parts}
    (OUT/'geometry_report.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='parts'},indent=2))
if __name__=='__main__':main()
