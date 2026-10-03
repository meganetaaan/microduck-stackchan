"""Fast immutable geometry, orientation and saved-audit consistency checks."""
from pathlib import Path
import hashlib,json,unittest
import mujoco,numpy as np
HERE=Path(__file__).resolve().parent
class PortraitTests(unittest.TestCase):
 def setUp(self):
  self.m=mujoco.MjModel.from_xml_path(str(HERE/'scene_tab5_portrait.xml'))
  self.p=json.loads((HERE/'parameters.json').read_text())
 def test_square_frame_and_device(self):
  b=np.array(self.p['body_bounds_xyz_mm']);self.assertEqual(b[0,1]-b[0,0],b[1,1]-b[1,0]);self.assertEqual(self.m.nu,10)
  np.testing.assert_allclose(self.m.geom_size[self.m.geom('tab5_collision').id]*2000,[12,80,128])
  self.assertGreater(self.m.geom_pos[self.m.geom('camera_lens').id,2],.055)
  screen=self.m.geom_pos[self.m.geom('screen').id,2]
  for n in ['eye_left','eye_right']:
   i=self.m.geom(n).id;self.assertGreater(self.m.geom_pos[i,2]-self.m.geom_size[i,2],screen)
 def test_physics_and_pairs(self):
  m=self.m;self.assertEqual(m.jnt_type[0],mujoco.mjtJoint.mjJNT_FREE);np.testing.assert_allclose(m.opt.gravity,[0,0,-9.81]);self.assertTrue(np.all(m.actuator_forcelimited))
  boxes=[i for i in range(m.ngeom)if m.geom(i).name.startswith('cube_portrait_')or m.geom(i).name=='tab5_collision'];legs=[i for i in range(m.ngeom)if m.geom(i).name.startswith('shell_guard_legmesh_')];pairs={tuple(sorted((int(a),int(b))))for a,b in zip(m.pair_geom1,m.pair_geom2)}
  self.assertEqual(len(boxes),18);self.assertEqual(len(legs),34)
  self.assertTrue(all(tuple(sorted((a,b)))in pairs for a in boxes for b in legs))
 def test_saved_audit_is_current(self):
  r=json.loads((HERE/'validation_report.json').read_text());self.assertEqual(r['model_sha256'],hashlib.sha256((HERE/'tab5_portrait.xml').read_bytes()).hexdigest());self.assertTrue(r['invariants']['all_exactly_equal']);self.assertFalse(r['geometry']['fixed_CAD_solid_intersections_over_0p001_mm3']);self.assertEqual(len(r['geometry']['added_box_connection_components']),1);self.assertEqual(r['total_sample_count'],1486)
  for row in r['reports']:self.assertEqual(row['new_physical_to_leg_contact_count'],0);self.assertGreater(row['all_new_parts']['minimum_gap_lower_bound_mm'],0)
  for name,meta in r['state_files'].items():self.assertEqual(meta['sha256'],hashlib.sha256((HERE/name).read_bytes()).hexdigest())
 def test_final_rollouts(self):
  for name in ['walk_10s','idle_10s','walk_seed1','walking_demo']:
   r=json.loads((HERE/(name+'.json')).read_text());self.assertIsNone(r['fall_reason']);self.assertEqual(r['shell_leg_contact_samples'],0);self.assertEqual(r['runtime_privacy']['ORT_DISABLE_TELEMETRY'],'1');self.assertAlmostEqual(r['total_mass_kg'],sum(self.m.body_mass),places=12)
if __name__=='__main__':unittest.main()
