"""Original thin flap and two round armored flap silhouettes. Static visuals only."""
from pathlib import Path
import json, hashlib, xml.etree.ElementTree as E
import numpy as np
from PIL import Image,ImageDraw
import geometry_tools as g
H=Path(__file__).resolve().parent
LABELS=['A  Original slim flap','B  Rounded armor','C  Chunky armor']
IDS=['A_slim_flap','B_rounded_armor','C_chunky_armor']

def build(index,state):
    a=g.ArmModel(4,state);a.id=IDS[index]+'_'+state
    scene=a.build() # Exact earlier slim-flap scene structure and baseline.
    if index==0:return a,scene
    # Replace only arm visuals in this new, isolated model.
    for node in list(a.body):a.body.remove(node)
    for asset in list(a.assets):
        if (asset.get('name') or '').startswith(a.id+'_'):a.assets.remove(asset)
    a.items=[]
    for sign in [-1,1]:
        tag='L' if sign>0 else 'R';hinge=np.array([-.028,sign*.043,.086])
        R=g.trimesh.transformations.rotation_matrix(0 if state=='stowed' else sign*np.deg2rad(60),[0,0,1])
        u=R[:3,:3]@np.array([1,0,0]);v=R[:3,:3]@np.array([0,1,0])
        heavy=index==2
        a.box(tag+'_side_mount',(-.030,sign*.042,.086),(.014,.006,.059),.0025,g.PALE)
        a.cylinder(tag+'_vertical_joint',hinge,.008 if heavy else .0058,.058 if heavy else .055,g.GRAY,axis=(0,0,1))
        for dz in [-.022,.022]:a.cylinder(tag+f'_joint_band_{dz}',hinge+[0,0,dz],.0084 if heavy else .0063,.006,g.ORANGE,axis=(0,0,1))
        if not heavy:
            a.box(tag+'_round_armor',hinge+u*.030,(.058,.014,.054),.0065,g.WHITE,R)
            a.box(tag+'_rounded_tip',hinge+u*.055,(.013,.016,.045),.006,g.WHITE,R)
            a.box(tag+'_orange_tip',hinge+u*.060,(.003,.0165,.032),.0014,g.ORANGE,R)
            for face in [-1,1]:
                a.box(tag+f'_recess_{face}',hinge+u*.029+v*(face*.0073),(.031,.0015,.028),.0006,g.PALE,R)
                a.box(tag+f'_upper_rib_{face}',hinge+u*.028+v*(face*.008)+np.array([0,0,.019]),(.032,.002,.005),.0009,g.WHITE,R)
                a.box(tag+f'_lower_rib_{face}',hinge+u*.028+v*(face*.008)+np.array([0,0,-.019]),(.032,.002,.005),.0009,g.WHITE,R)
        else:
            # Broad, softer armored forearm and a compact rounded end block.
            a.box(tag+'_main_armor',hinge+u*.030,(.057,.024,.060),.0105,g.WHITE,R)
            a.box(tag+'_dark_seam',hinge+u*.053,(.006,.021,.043),.0025,g.GRAY,R)
            a.box(tag+'_chunky_end',hinge+u*.063,(.024,.028,.051),.011,g.WHITE,R)
            a.box(tag+'_orange_end_cap',hinge+u*.073,(.0045,.024,.035),.002,g.ORANGE,R)
            for face in [-1,1]:
                a.box(tag+f'_side_recess_{face}',hinge+u*.027+v*(face*.0123),(.028,.0016,.031),.0007,g.PALE,R)
                a.box(tag+f'_upper_armor_rail_{face}',hinge+u*.027+v*(face*.011)+np.array([0,0,.024]),(.032,.008,.009),.0035,g.WHITE,R)
                a.box(tag+f'_lower_armor_rail_{face}',hinge+u*.027+v*(face*.011)+np.array([0,0,-.024]),(.032,.008,.009),.0035,g.WHITE,R)
                a.box(tag+f'_orange_side_detail_{face}',hinge+u*.064+v*(face*.013),(.009,.002,.008),.0009,g.ORANGE,R)
    E.indent(a.root);E.ElementTree(a.root).write(H/(a.id+'.xml'),encoding='utf-8',xml_declaration=True)
    return a,scene

def main():
    images={};records=[]
    baseline=hashlib.sha256((H/'base_visual_robot.xml').read_bytes()).hexdigest()
    for i in range(3):
        for state in ['display','stowed']:
            a,scene=build(i,state);m,d=g.load(scene)
            for az,view in [(140,'oblique'),(180,'front'),(90,'side')]:
                im=g.render(m,d,az);im.save(g.OUT/(a.id+'_'+view+'.png'));images[(i,state,view)]=im
            glb=g.export_glb(m,d,g.GLB/(a.id+'.glb'))
            records.append({'id':a.id,'scene':scene.name,'glb':glb,'arm_parts':len(a.items),'nq':m.nq,'nu':m.nu})
            print('BUILT',a.id,flush=True)
    for view in ['oblique','front','side']:
        im=Image.new('RGB',(2550,1140),'#111c26');dr=ImageDraw.Draw(im)
        for i in range(3):
            x=i*850;g.title(dr,(x+26,20),LABELS[i],31);im.paste(images[(i,'display',view)],(x,83))
        g.title(dr,(28,1100),'STATIC SILHOUETTES  /  Same robot, scale, camera and opening angle  /  Mechanics and walking unvalidated',22,'#aabbc4')
        im.save(H/('chunky_flaps_comparison.png' if view=='oblique' else f'chunky_flaps_{view}.png'))
    im=Image.new('RGB',(2550,2240),'#111c26');dr=ImageDraw.Draw(im)
    for row,state in enumerate(['stowed','display']):
        for i in range(3):
            x=i*850;y=row*1090;g.title(dr,(x+25,y+20),LABELS[i]+' / '+state,26);im.paste(images[(i,state,'oblique')],(x,y+77))
    g.title(dr,(28,2200),'Illustrative static poses only. No verified masses, movement clearance or manufacturing details.',21,'#aabbc4');im.save(H/'chunky_flaps_stowed_vs_display.png')
    assert hashlib.sha256((H/'base_visual_robot.xml').read_bytes()).hexdigest()==baseline
    report={'scope':'VISUAL CONCEPTS ONLY','baseline_visual_hash':baseline,'base_visual_unchanged':True,'existing_adopted_assembly_and_site':'Not accessed or modified; these assets are recovered copies from the prior concept deliverable.','slim_original':'Same original #4 arm geometry, placement, colors and 60-degree display opening.','new_variants':['B: thicker rounded panel and reinforced-looking hinge','C: bulkier rounded armor, layered edges and a chunky end block'],'physics_clearance_printability_validated':False,'models':records,'render_pixel_inspection':'pending'}
    (H/'validation_report.json').write_text(json.dumps(report,indent=2)+'\n')
if __name__=='__main__':main()
