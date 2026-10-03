import os
os.environ['ORT_DISABLE_TELEMETRY']='1'
import unittest,torch,numpy as np
from env import *
from policy import Actor
class PipelineTests(unittest.TestCase):
 def test_exact_native_initialization(self):
  torch.set_num_threads(1);a=Actor.from_teacher();s=teacher_session();rng=np.random.default_rng(1)
  for _ in range(100):
   ob=rng.normal(0,.1,39).astype(np.float32);ob[5]=-1
   np.testing.assert_allclose(a(torch.from_numpy(ob)).detach(),teacher_action(s,ob),atol=2e-6,rtol=1e-5)
 def test_native_dimensions_and_normalizer(self):
  a=Actor();self.assertEqual(tuple(a(torch.zeros(4,39)).shape),(4,10));self.assertEqual(tuple(a.mean.shape),(39,));self.assertTrue(all(torch.isfinite(a(torch.zeros(4,39))).flatten()))
 def test_real_physics(self):
  scene=Path(os.environ.get('TAB5_SCENE',str(ROOT/'prototype/portrait_print/scene_tab5_portrait_print.xml')));e=Env(scene);s=teacher_session();ob=e.reset([0,0,0],noise=0)
  self.assertEqual(e.m.nu,10);self.assertEqual(e.m.nq,17);self.assertEqual(e.m.nv,16);self.assertEqual(e.m.jnt_type[0],mujoco.mjtJoint.mjJNT_FREE)
  initial=e.d.qpos.copy()
  for _ in range(100):ob,r,done,info=e.step(teacher_action(s,ob));self.assertFalse(done);self.assertTrue(np.isfinite(r));self.assertEqual(float(np.max(np.abs(e.d.xfrc_applied))),0.)
  self.assertGreater(np.max(np.abs(initial-e.d.qpos)),1e-4);self.assertAlmostEqual(e.d.time,2.);self.assertFalse(e.metrics()['fall'])
if __name__=='__main__':unittest.main()
