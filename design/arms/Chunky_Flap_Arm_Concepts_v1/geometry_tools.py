"""Four static visual arm studies. No collision/dynamics/servo design claims.
Run this self-contained source package with Python + requirements.txt.
"""
from pathlib import Path
import os, copy, json, hashlib, shutil, xml.etree.ElementTree as E
os.environ.setdefault('MUJOCO_GL','egl')
os.environ.setdefault('MESA_SHADER_CACHE_DIR','/tmp/microduck-mesa-cache')
import numpy as np
import trimesh, mujoco
from PIL import Image, ImageDraw, ImageFont
H=Path(__file__).resolve().parent
SOURCE=H.parents[2]/'prototype/assembly_uart/screen_visual'
A=H/'assets'; A.mkdir(exist_ok=True)
OUT=H/'renders'; OUT.mkdir(exist_ok=True)
GLB=H/'glb';GLB.mkdir(exist_ok=True)
WHITE=(.84,.86,.83,1); GRAY=(.28,.32,.32,1); ORANGE=(1,.34,.025,1); PALE=(.68,.72,.70,1)
NAMES={1:'Short paddle',2:'Folding elbow',3:'Mitten hands',4:'Side flaps'}
DESCRIPTIONS={1:'Compact / fore-aft shoulder',2:'Two links / shoulder + elbow',3:'Short links / two-axis shoulders',4:'Rear vertical hinge / fold-flat panels'}
FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def s(v):return ' '.join(f'{x:.9g}' for x in v)
def rb(ext,r):
    mesh=trimesh.creation.icosphere(subdivisions=2,radius=1)
    v=mesh.vertices*r+np.sign(mesh.vertices)*(np.asarray(ext)/2-r)
    return trimesh.Trimesh(vertices=v,faces=mesh.faces,process=True).convex_hull

def capsule_between(p,q,width,depth,r):
    p=np.array(p);q=np.array(q);d=q-p
    m=rb((width,depth,np.linalg.norm(d)),r)
    T=trimesh.geometry.align_vectors([0,0,1],d/np.linalg.norm(d));T[:3,3]=(p+q)/2;m.apply_transform(T);return m

def prepare_base():
    dest=H/'base_visual_robot.xml'
    if dest.exists():return
    root=E.parse(SOURCE/'tab5_assembly_uart_screen.xml').getroot()
    for node in [root.find('contact')]:
        if node is not None:root.remove(node)
    for b in root.iter('body'):
        for g in list(b.findall('geom')):
            if g.get('group')=='3':b.remove(g)
    used={g.get('mesh') for g in root.iter('geom') if g.get('mesh')}
    assets=root.find('asset')
    for i,a in enumerate(list(assets)):
        if a.tag=='mesh' and (a.get('name') or Path(a.get('file')).stem) not in used:
            assets.remove(a);continue
        if a.get('file'):
            if a.tag=='mesh' and not a.get('name'):a.set('name',Path(a.get('file')).stem)
            src=(SOURCE/a.get('file')).resolve();dst=A/(f'base_{i:03d}_'+src.name)
            shutil.copy2(src,dst);a.set('file','assets/'+dst.name)
    E.indent(root);E.ElementTree(root).write(dest,encoding='utf-8',xml_declaration=True)
    # Exact original STAND keyframe; source base is only a visual surrogate.
    scene=E.parse(SOURCE/'scene_tab5_assembly_uart_screen.xml').getroot()
    for child in list(scene):
        if child.tag not in ['keyframe']:scene.remove(child)
    E.ElementTree(scene).write(H/'base_keyframes.xml',encoding='utf-8',xml_declaration=True)
    (H/'source_baseline.json').write_text(json.dumps({'original_screen_model_sha256':sha(SOURCE/'tab5_assembly_uart_screen.xml'),'original_physical_model_sha256':sha(SOURCE.parent/'tab5_assembly_uart.xml'),'original_screen_texture_sha256':sha(SOURCE/'assets/stackchan_microduck_screen_720x1280.png'),'original_physical_mass_g':736.537,'source_scope':'Separate static visual study; adopted assembly and training models are not modified.'},indent=2))

class ArmModel:
    def __init__(self,num,state):
        self.num=num;self.state=state;self.id=f'{num:02d}_{["paddle","elbow","mitten","flap"][num-1]}_{state}'
        self.root=E.parse(H/'base_visual_robot.xml').getroot();self.assets=self.root.find('asset');self.trunk=self.root.find(".//body[@name='trunk_base']")
        self.body=E.SubElement(self.trunk,'body',name='arm_concept_visual_only');self.items=[]
    def add(self,name,m,color):
        n=self.id+'_'+name;f=A/(n+'.obj');m.export(f)
        E.SubElement(self.assets,'mesh',name=n,file='assets/'+f.name)
        E.SubElement(self.body,'geom',name=n,type='mesh',mesh=n,rgba=s(color),group='2',contype='0',conaffinity='0',mass='0')
        self.items.append({'name':name,'color':color,'bounds_m':m.bounds.tolist()})
    def box(self,n,center,ext,r,col,rotation=None):
        m=rb(ext,r)
        if rotation is not None:m.apply_transform(rotation)
        m.apply_translation(center);self.add(n,m,col)
    def ell(self,n,p,ext,col):
        m=trimesh.creation.icosphere(subdivisions=3);m.apply_scale(np.array(ext)/2);m.apply_translation(p);self.add(n,m,col)
    def cylinder(self,n,p,r,h,col,axis=(0,1,0)):
        m=trimesh.creation.cylinder(radius=r,height=h,sections=40)
        T=trimesh.geometry.align_vectors([0,0,1],axis);T[:3,3]=p;m.apply_transform(T);self.add(n,m,col)
    def link(self,n,p,q,w,d,r,col):self.add(n,capsule_between(p,q,w,d,r),col)
    def shoulder(self,sign,z=.092):
        # Mount plate is a silhouette placeholder, not a verified fastener interface.
        tag='L' if sign>0 else 'R'
        self.box(tag+'_mount',(.032,sign*.0425,z),(.020,.006,.030),.0025,PALE)
        self.cylinder(tag+'_shoulder',(.026,sign*.048,z),.010,.011,GRAY)
        self.cylinder(tag+'_orange_cap',(.026,sign*.055,z),.0063,.002,ORANGE)
        self.cylinder(tag+'_cap_center',(.026,sign*.0563,z),.0025,.0015,PALE)
        return tag,np.array([.026,sign*.055,z])
    def build(self):
        for sign in [-1,1]:
            if self.num==1:
                tag,p=self.shoulder(sign)
                if self.state=='stowed':tip=p+[0,sign*.001,-.042]
                else:tip=p+[.015,sign*.001,-.039]
                self.link(tag+'_paddle',p+[0,sign*.002,-.003],tip,.021,.011,.0045,WHITE)
                cap=tip+(p-tip)/np.linalg.norm(p-tip)*.004
                self.box(tag+'_tip_accent',cap,(.015,.012,.005),.002,ORANGE)
            elif self.num==2:
                tag,p=self.shoulder(sign)
                elbow=p+np.array([-.003,sign*.003,-.035])
                hand=elbow+(np.array([.007,sign*.009,.033]) if self.state=='stowed' else np.array([.034,sign*.003,.011]))
                self.link(tag+'_upper',p+[0,0,-.005],elbow,.015,.012,.004,WHITE)
                self.cylinder(tag+'_elbow',elbow,.0075,.016,GRAY)
                self.cylinder(tag+'_elbow_cap',elbow+[0,sign*.009,0],.0045,.002,ORANGE)
                self.link(tag+'_forearm',elbow,hand,.013,.012,.004,WHITE)
                self.ell(tag+'_hand',hand,(.016,.016,.017),ORANGE)
            elif self.num==3:
                tag,p=self.shoulder(sign,.091)
                self.ell(tag+'_ball_joint',p+[.001,sign*.006,0],(.019,.019,.019),GRAY)
                if self.state=='stowed':hand=p+[.006,sign*.008,-.037]
                elif sign>0:hand=p+[.006,sign*.039,.020]
                else:hand=p+[.006,sign*.035,-.023]
                cuff=hand+(p-hand)/np.linalg.norm(p-hand)*.012
                self.link(tag+'_short_link',p,cuff,.011,.011,.0035,GRAY)
                self.ell(tag+'_orange_cuff',cuff,(.018,.018,.015),ORANGE)
                # Broad mitten with one small thumb; no articulated fingers.
                self.box(tag+'_mitten',hand,(.022,.025,.030),.008,WHITE)
                self.ell(tag+'_thumb',hand+[.005,-sign*.012,-.003],(.016,.015,.021),WHITE)
                self.box(tag+'_hand_stripe',hand+[.011,0,-.005],(.002,.017,.006),.0008,ORANGE)
            else:
                tag='L' if sign>0 else 'R';hinge=np.array([-.028,sign*.043,.086])
                self.box(tag+'_hinge_mount',(-.030,sign*.041,.086),(.012,.005,.057),.002,PALE)
                self.cylinder(tag+'_vertical_hinge',hinge,.0037,.057,GRAY,axis=(0,0,1))
                for dz in [-.021,.021]:self.cylinder(tag+f'_hinge_band_{dz}',hinge+[0,0,dz],.004,.006,ORANGE,axis=(0,0,1))
                angle=0 if self.state=='stowed' else sign*np.deg2rad(60)
                R=trimesh.transformations.rotation_matrix(angle,[0,0,1]);u=R[:3,:3]@np.array([1,0,0])
                center=hinge+u*.029
                self.box(tag+'_side_flap',center,(.058,.005,.054),.0024,WHITE,R)
                self.box(tag+'_orange_edge',hinge+u*.055,(.004,.006,.038),.0018,ORANGE,R)
                self.box(tag+'_panel_inset',hinge+u*.030+R[:3,:3]@np.array([0,sign*.0033,0]),(.037,.0012,.032),.0005,PALE,R)
        path=H/(self.id+'.xml');E.indent(self.root);E.ElementTree(self.root).write(path,encoding='utf-8',xml_declaration=True)
        scene=E.Element('mujoco',model='static_arm_silhouette_study')
        E.SubElement(scene,'include',file=path.name)
        visual=E.SubElement(scene,'visual');E.SubElement(visual,'headlight',diffuse='.75 .75 .75',ambient='.42 .42 .42',specular='.08 .08 .08');E.SubElement(visual,'rgba',haze='.11 .15 .19 1');E.SubElement(visual,'quality',shadowsize='2048')
        ass=E.SubElement(scene,'asset');E.SubElement(ass,'texture',type='skybox',builtin='gradient',rgb1='.12 .17 .22',rgb2='.035 .05 .07',width='512',height='3072')
        wb=E.SubElement(scene,'worldbody');E.SubElement(wb,'light',pos='0 -1 2',dir='0 .3 -1',diffuse='.6 .6 .6',directional='true');E.SubElement(wb,'geom',name='floor',size='0 0 .05',type='plane',rgba='.10 .13 .16 1',contype='0',conaffinity='0')
        scene.append(copy.deepcopy(E.parse(H/'base_keyframes.xml').getroot().find('keyframe')))
        sp=H/('scene_'+self.id+'.xml');E.indent(scene);E.ElementTree(scene).write(sp,encoding='utf-8',xml_declaration=True)
        return sp

def load(sp):
    m=mujoco.MjModel.from_xml_path(str(sp));d=mujoco.MjData(m);mujoco.mj_resetDataKeyframe(m,d,m.key('STAND').id);mujoco.mj_forward(m,d);return m,d

def render(m,d,az=140,w=850,h=1000):
    m.vis.global_.offwidth=w;m.vis.global_.offheight=h
    r=mujoco.Renderer(m,width=w,height=h);cam=mujoco.MjvCamera();cam.azimuth=az;cam.elevation=-13;cam.distance=.43;cam.lookat[:]=[.005,0,.125]
    r.update_scene(d,camera=cam);r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=1
    im=Image.fromarray(r.render());r.close();return im

def export_glb(m,d,path):
    scene=trimesh.Scene();texfile=next(A.glob('*stackchan_microduck_screen_720x1280.png'));tex=Image.open(texfile).convert('RGB')
    texmat=trimesh.visual.material.PBRMaterial(name='StackchanScreen',baseColorTexture=tex,baseColorFactor=[255]*4,emissiveTexture=tex,emissiveFactor=[1,1,1],metallicFactor=0,roughnessFactor=1)
    count=0
    for i in range(m.ngeom):
        name=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,i) or f'original_visual_{i:03d}'
        if name=='floor' or m.geom_group[i]==3 or m.geom_rgba[i,3]<=0:continue
        typ=m.geom_type[i];size=m.geom_size[i]
        if typ==mujoco.mjtGeom.mjGEOM_MESH:
            mid=m.geom_dataid[i];va=m.mesh_vertadr[mid];vn=m.mesh_vertnum[mid];fa=m.mesh_faceadr[mid];fn=m.mesh_facenum[mid]
            verts=m.mesh_vert[va:va+vn].copy();faces=m.mesh_face[fa:fa+fn].copy()
            if name=='screen':
                tx=m.mesh_texcoordadr[mid];tfaces=m.mesh_facetexcoord[fa:fa+fn];uv=m.mesh_texcoord[tx+tfaces.reshape(-1)]
                mesh=trimesh.Trimesh(vertices=verts[faces.reshape(-1)],faces=np.arange(fn*3).reshape(-1,3),process=False);mesh.visual=trimesh.visual.TextureVisuals(uv=uv,material=texmat)
            else:mesh=trimesh.Trimesh(vertices=verts,faces=faces,process=False)
        elif typ==mujoco.mjtGeom.mjGEOM_BOX:mesh=trimesh.creation.box(extents=2*size)
        elif typ==mujoco.mjtGeom.mjGEOM_ELLIPSOID:mesh=trimesh.creation.icosphere(subdivisions=2);mesh.apply_scale(size)
        elif typ==mujoco.mjtGeom.mjGEOM_CYLINDER:mesh=trimesh.creation.cylinder(radius=size[0],height=size[1]*2,sections=32)
        elif typ==mujoco.mjtGeom.mjGEOM_SPHERE:mesh=trimesh.creation.icosphere(subdivisions=2,radius=size[0])
        else:continue
        if name!='screen':mesh.visual.vertex_colors=np.tile(np.rint(m.geom_rgba[i]*255).astype(np.uint8),(len(mesh.vertices),1))
        T=np.eye(4);T[:3,:3]=d.geom_xmat[i].reshape(3,3);T[:3,3]=d.geom_xpos[i];mesh.apply_transform(T);scene.add_geometry(mesh,node_name=name,geom_name=name);count+=1
    scene.apply_transform(trimesh.transformations.rotation_matrix(-np.pi/2,[1,0,0]))
    scene.metadata={'units':'metres','up_axis':'Y (glTF standard); source MJCF uses Z up','pose':'STATIC STAND with concept arm display pose','scope':'Full robot visual model, not printable CAD or a validated dynamic model'}
    scene.export(path);reload=trimesh.load(path,force='scene');assert len(reload.geometry)==count
    assert len(reload.geometry['screen'].visual.uv)>0
    return {'geometry_count':count,'bounds_m':scene.bounds.tolist(),'embedded_screen_texture':True,'file_sha256':sha(path)}

def title(draw,xy,text,size=29,color='#eef1ef'):
    draw.text(xy,text,font=ImageFont.truetype(FONT,size),fill=color)

