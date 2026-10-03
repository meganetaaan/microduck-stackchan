"""Build 10-DOF concepts without altering upstream repositories or leg physics."""
from pathlib import Path
import argparse
import json
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parent
UPSTREAM = ROOT.parent / "microduck_rl/src/mjlab_microduck/robot/microduck"
LEG_INDICES = [0, 1, 2, 3, 4, 9, 10, 11, 12, 13]
HEAD_NAMES = {"neck_pitch", "head_pitch", "head_yaw", "head_roll"}

def vec(x):
    return " ".join(f"{v:.10g}" for v in x)

def add_box(parent, name, mass, size, pos, color, collision=True, inertia=None):
    b = ET.SubElement(parent, "body", name=name, pos=vec(pos))
    x, y, z = size
    if inertia is None:
        inertia = mass / 12 * np.array([y*y+z*z, x*x+z*z, x*x+y*y])
    ET.SubElement(b, "inertial", pos="0 0 0", mass=str(mass), diaginertia=vec(inertia))
    ET.SubElement(b, "geom", name=name+"_collision", type="box", size=vec(np.array(size)/2),
                  rgba=color, contype="1" if collision else "0", conaffinity="3" if collision else "0",
                  group="0", mass="0")
    return b

def shell_inertia(mass, size, thickness):
    # Uniform-density hollow closed rectangular shell, subtract inner box inertia.
    size=np.array(size); inner=np.maximum(size-2*thickness, 1e-6)
    rho=mass/(np.prod(size)-np.prod(inner))
    def inertia(s): return rho*np.prod(s)/12*np.array([s[1]**2+s[2]**2,s[0]**2+s[2]**2,s[0]**2+s[1]**2])
    return inertia(size)-inertia(inner)

def build(params_path=ROOT/"parameters.json", outdir=ROOT/"models"):
    p=json.loads(Path(params_path).read_text());outdir=Path(outdir);outdir.mkdir(parents=True,exist_ok=True)
    for battery in [False,True]:
        name="tab5_kit" if battery else "tab5_bare"
        tree=ET.parse(UPSTREAM/"robot_groundcontact.xml");r=tree.getroot();r.set("model",name)
        # Relative asset path preserves portable checkout-directory layout.
        r.find("compiler").set("meshdir", "../../microduck_rl/src/mjlab_microduck/robot/microduck/assets")
        trunk=r.find(".//body[@name='trunk_base']")
        neck=trunk.find("body[@name='neck']")
        removed_mass=sum(float(i.get("mass")) for i in neck.iter("inertial"));trunk.remove(neck)
        for a in list(r.find("actuator")):
            if a.get("joint") in HEAD_NAMES:r.find("actuator").remove(a)
        shell=add_box(trunk,"box_shell",p["shell_mass_kg"],p["body_size_xyz_m"],p["body_center_m"],"0.92 0.91 0.85 1",inertia=shell_inertia(p["shell_mass_kg"],p["body_size_xyz_m"],p["shell_wall_thickness_m"]))
        add_box(trunk,"mount",p["mount_mass_kg"],p["mount_size_xyz_m"],p["mount_center_m"],"0.2 0.2 0.22 1")
        add_box(trunk,"compute_allowance",p["compute_mass_kg"],p["compute_size_xyz_m"],p["compute_center_m"],"0.1 0.3 0.1 0",collision=False)
        tab=add_box(trunk,"tab5",p["tab5_mass_kg"],p["tab5_size_xyz_m"],p["tab5_center_m"],"0.15 0.16 0.17 1")
        # Decorative screen/eyes have zero mass/contact; they do not affect mechanics.
        ET.SubElement(tab,"geom",name="screen",type="box",size="0.0002 0.058 0.032",pos="0.00621 0 0",rgba="0.01 0.025 0.03 1",contype="0",conaffinity="0",mass="0",group="0")
        for y in [-0.025,0.025]:
            ET.SubElement(tab,"geom",type="box",size="0.0002 0.006 0.013",pos=f"0.0065 {y} 0.003",rgba="0.1 0.95 0.9 1",contype="0",conaffinity="0",mass="0",group="0")
        if battery:add_box(trunk,"tab5_battery",p["tab5_battery_mass_kg"],p["tab5_battery_size_xyz_m"],p["tab5_battery_center_m"],"0.2 0.2 0.2 0",collision=False)
        # Remaining ground-contact geometry is unchanged. Remove unused head assets only.
        used_meshes={g.get("mesh") for g in r.iter("geom") if g.get("mesh")}
        asset=r.find("asset")
        for a in list(asset):
            if a.tag=="mesh" and (a.get("name") or Path(a.get("file")).stem) not in used_meshes:asset.remove(a)
        ET.indent(tree);tree.write(outdir/f"{name}.xml",encoding="utf-8",xml_declaration=True)
        scene=ET.parse(UPSTREAM/"scene.xml");sr=scene.getroot();sr.find("include").set("file",f"{name}.xml")
        for key in sr.findall("keyframe/key"):
            q=np.fromstring(key.get("qpos"),sep=" ");u=np.fromstring(key.get("ctrl"),sep=" ")
            key.set("qpos",vec(np.r_[q[:7],q[7:][LEG_INDICES]]));key.set("ctrl",vec(u[LEG_INDICES]))
        ET.indent(scene);scene.write(outdir/f"scene_{name}.xml",encoding="utf-8",xml_declaration=True)
        print(f"Built {name}; removed {removed_mass:.7f} kg, four joints/actuators")

if __name__=="__main__":
    ap=argparse.ArgumentParser();ap.add_argument("--params",type=Path,default=ROOT/"parameters.json");ap.add_argument("--outdir",type=Path,default=ROOT/"models");args=ap.parse_args();build(args.params,args.outdir)
