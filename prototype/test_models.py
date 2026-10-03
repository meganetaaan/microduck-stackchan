import unittest
from pathlib import Path
import numpy as np
import mujoco

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT.parent/"microduck_rl/src/mjlab_microduck/robot/microduck/scene.xml"

class ModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original=mujoco.MjModel.from_xml_path(str(SOURCE))
    def test_models_and_real_dynamics(self):
        for variant,mass in [("bare",.68071578),("kit",.77961578)]:
            m=mujoco.MjModel.from_xml_path(str(ROOT/f"models/scene_tab5_{variant}.xml"))
            self.assertEqual(m.nu,10);self.assertEqual(m.nq,17);self.assertEqual(m.nv,16)
            self.assertEqual(m.jnt_type[0],mujoco.mjtJoint.mjJNT_FREE)
            np.testing.assert_allclose(m.opt.gravity,[0,0,-9.81])
            self.assertAlmostEqual(sum(m.body_mass),mass)
            self.assertEqual(m.neq,0)
            for name in ["neck_pitch","head_pitch","head_yaw","head_roll"]:
                self.assertEqual(mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,name),-1)
            for i in range(m.nu):
                name=m.actuator(i).name;j=m.actuator_trnid[i,0]
                old=self.original.joint(name).id
                np.testing.assert_allclose(m.jnt_range[j],self.original.jnt_range[old])
                np.testing.assert_allclose(m.jnt_axis[j],self.original.jnt_axis[old])
                np.testing.assert_allclose(m.jnt_pos[j],self.original.jnt_pos[old])
            for name in ["trunk_base","yaw2roll","hip_l","upper_leg_left","leg","ankle_left"]:
                b=m.body(name).id;o=self.original.body(name).id
                np.testing.assert_allclose(m.body_mass[b],self.original.body_mass[o])
                np.testing.assert_allclose(m.body_inertia[b],self.original.body_inertia[o])
            for name in ["left_foot_collision","right_foot_collision"]:
                g=m.geom(name).id;o=self.original.geom(name).id
                self.assertEqual(m.geom_contype[g],self.original.geom_contype[o])
                self.assertEqual(m.geom_conaffinity[g],self.original.geom_conaffinity[o])
                np.testing.assert_allclose(m.geom_friction[g],self.original.geom_friction[o])
            self.assertGreater(m.geom("box_shell_collision").contype[0],0)
            self.assertGreater(m.geom("tab5_collision").contype[0],0)

    def test_lowcube_preserves_legs_and_has_real_shell_contacts(self):
        m=mujoco.MjModel.from_xml_path(str(ROOT/"models/scene_tab5_lowcube.xml"))
        d=mujoco.MjData(m)
        self.assertEqual((m.nu,m.nq,m.nv),(10,17,16));self.assertEqual(m.neq,0)
        self.assertEqual(m.jnt_type[0],mujoco.mjtJoint.mjJNT_FREE)
        np.testing.assert_allclose(m.opt.gravity,[0,0,-9.81])
        self.assertAlmostEqual(sum(m.body_mass),.7282765)
        self.assertGreater(m.npair,100)
        for i in range(m.nu):
            name=m.actuator(i).name;j=m.actuator_trnid[i,0];old=self.original.joint(name).id
            np.testing.assert_allclose(m.jnt_range[j],self.original.jnt_range[old])
            np.testing.assert_allclose(m.jnt_axis[j],self.original.jnt_axis[old])
            np.testing.assert_allclose(m.jnt_pos[j],self.original.jnt_pos[old])
        for b in range(1,m.nbody):
            old=mujoco.mj_name2id(self.original,mujoco.mjtObj.mjOBJ_BODY,m.body(b).name)
            if old>=0:
                for attr in ["body_mass","body_inertia","body_ipos","body_iquat","body_pos","body_quat"]:
                    np.testing.assert_allclose(getattr(m,attr)[b],getattr(self.original,attr)[old])
        shell={g for g in range(m.ngeom) if m.geom(g).name.startswith("cube_") or m.geom(g).name=="tab5_collision"}
        legs={g for g in range(m.ngeom) if m.geom(g).name.startswith("shell_guard_legmesh_")}
        def shell_hits(pose):
            mujoco.mj_resetDataKeyframe(m,d,m.key(pose).id);mujoco.mj_forward(m,d)
            return [c for c in d.contact[:d.ncon] if {int(c.geom1),int(c.geom2)}&shell and {int(c.geom1),int(c.geom2)}&legs]
        self.assertEqual(len(shell_hits("STAND")),0)
        # These are deliberate negative tests: do not use old SIT/FOLD poses with this hood.
        self.assertGreater(len(shell_hits("SIT")),0)
        self.assertGreater(len(shell_hits("FOLD")),0)

if __name__=="__main__":unittest.main()
