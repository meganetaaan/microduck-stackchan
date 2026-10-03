"""Build a low-slung, genuinely hollow collision shell as a separate variant."""
from pathlib import Path
import argparse,json,xml.etree.ElementTree as ET
import numpy as np
from build_models import ROOT,vec,add_box

def build(params=ROOT/"parameters_lowcube.json"):
    p=json.loads(Path(params).read_text());name=p["variant"];out=ROOT/"models"
    tree=ET.parse(out/"tab5_bare.xml");r=tree.getroot();r.set("model",name)
    trunk=r.find(".//body[@name='trunk_base']")
    for b in list(trunk.findall("body")):
        if b.get("name") in ["box_shell","mount","compute_allowance","tab5"]:trunk.remove(b)
    (xmin,xmax),(ymin,ymax),(zmin,zmax)=p["outer_bounds_xyz_m"]
    t=p["wall_thickness_m"];rho=p["shell_material_density_kg_m3"]
    tx,ty,tz=p["tab5_center_m"];td,tw,th=p["tab5_size_xyz_m"]
    nb,ne=p["side_notch_x_range_m"];nt=p["side_notch_top_z_m"]
    panels=[]
    def panel(n,bounds):
        a=np.array(bounds);s=a[:,1]-a[:,0]
        if np.any(s<=1e-8):return
        mass=float(np.prod(s)*rho);pos=a.mean(1)
        add_box(trunk,"cube_"+n,mass,s,pos,"0.92 0.91 0.85 1")
        panels.append({"name":"cube_"+n,"bounds_xyz_m":a.tolist(),"mass_kg":mass})
    # Non-overlapping wall slabs; open bottom. Screen aperture follows true Tab5 size.
    panel("top",[[xmin,xmax],[ymin,ymax],[zmax-t,zmax]])
    panel("rear",[[xmin,xmin+t],[ymin,ymax],[zmin,zmax-t]])
    panel("chin",[[xmax-t,xmax],[ymin,ymax],[zmin,tz-th/2]])
    panel("brow",[[xmax-t,xmax],[ymin,ymax],[tz+th/2,zmax-t]])
    panel("face_left",[[xmax-t,xmax],[ymin,-tw/2],[tz-th/2,tz+th/2]])
    panel("face_right",[[xmax-t,xmax],[tw/2,ymax],[tz-th/2,tz+th/2]])
    for sign,yr in [("left",[ymin,ymin+t]),("right",[ymax-t,ymax])]:
        panel("side_"+sign,[[xmin+t,xmax-t],yr,[nt,zmax-t]])
        panel("skirt_rear_"+sign,[[xmin+t,nb],yr,[zmin,nt]])
        panel("skirt_front_"+sign,[[ne,xmax-t],yr,[zmin,nt]])
    add_box(trunk,"mount",p["mount_mass_kg"],p["mount_size_xyz_m"],p["mount_center_m"],"0.2 0.2 0.22 1",collision=False)
    add_box(trunk,"compute_allowance",p["compute_mass_kg"],p["compute_size_xyz_m"],p["compute_center_m"],"0.1 0.3 0.1 0",collision=False)
    tab=add_box(trunk,"tab5",p["tab5_mass_kg"],p["tab5_size_xyz_m"],p["tab5_center_m"],"0.15 0.16 0.17 1")
    ET.SubElement(tab,"geom",name="screen",type="box",size="0.0002 0.058 0.032",pos="0.00621 0 0",rgba="0.01 0.025 0.03 1",contype="0",conaffinity="0",mass="0",group="0")
    for y in [-.025,.025]:ET.SubElement(tab,"geom",type="box",size="0.0002 0.006 0.013",pos=f"0.0065 {y} .003",rgba=".1 .95 .9 1",contype="0",conaffinity="0",mass="0",group="0")
    # Explicit pairs make new shell/Tab5 contact every moving leg mesh, including
    # CAD visual housings absent from the curated upstream floor collision set.
    # They also bypass parent-child filtering at the hip. Original leg-to-leg and
    # leg-to-floor behavior is unchanged; no original inertial/joint is modified.
    contact=r.find("contact")
    if contact is None:contact=ET.SubElement(r,"contact")
    moving_geoms=[]
    for b in trunk.findall("body"):
        if b.find("joint") is None:continue
        for g in b.iter("geom"):
            if not g.get("mesh"):continue
            if g.get("class")!="visual":continue
            if not g.get("name"):g.set("name",f"shell_guard_legmesh_{len(moving_geoms):03d}")
            moving_geoms.append(g.get("name"))
    for panel_name in [x["name"]+"_collision" for x in panels]+["tab5_collision"]:
        for g in moving_geoms:ET.SubElement(contact,"pair",geom1=panel_name,geom2=g,condim="3",friction="1 1 .005 .0001 .0001")
    ET.indent(tree);tree.write(out/f"{name}.xml",encoding="utf-8",xml_declaration=True)
    scene=ET.parse(out/"scene_tab5_bare.xml");scene.getroot().find("include").set("file",f"{name}.xml")
    scene.write(out/f"scene_{name}.xml",encoding="utf-8",xml_declaration=True)
    report={"parameters":p,"panels":panels,"shell_mass_kg":sum(x["mass_kg"] for x in panels),"leg_mesh_contact_pairs":len(moving_geoms)*(len(panels)+1)}
    (ROOT/"results/lowcube_geometry.json").write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))

if __name__=="__main__":
    ap=argparse.ArgumentParser();ap.add_argument("--params",type=Path,default=ROOT/"parameters_lowcube.json");a=ap.parse_args();build(a.params)
