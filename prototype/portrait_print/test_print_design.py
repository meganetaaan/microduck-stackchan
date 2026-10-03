"""Fast final-artifact integrity and physics-result checks."""
from pathlib import Path
import hashlib,json,unittest
import numpy as np,mujoco,trimesh
HERE=Path(__file__).resolve().parent
class PrintDesignTests(unittest.TestCase):
 def test_solids_and_units(self):
  r=json.loads((HERE/'geometry_report.json').read_text());parts=[p for p in r['parts']if p['printable']];self.assertEqual(len(parts),4)
  self.assertEqual({f.stem for f in (HERE/'stl').glob('*.stl')},{p['name']+'_mm'for p in parts})
  for p in parts:
   mesh=trimesh.load(HERE/'stl'/(p['name']+'_mm.stl'),process=True);self.assertTrue(mesh.is_watertight);self.assertEqual(len(mesh.split()),1);self.assertGreater(mesh.volume,0);np.testing.assert_allclose(mesh.bounds,p['bounds_mm'],atol=1e-5)
  o=json.loads((HERE/'print_orientation.json').read_text());self.assertEqual(len(o['parts']),4)
  for p in o['parts']:
   mesh=trimesh.load(HERE/p['file'],process=True);self.assertTrue(mesh.is_watertight);self.assertAlmostEqual(mesh.bounds[0,2],0,places=5)
 def test_audit_matches_final_files(self):
  r=json.loads((HERE/'validation_report.json').read_text());self.assertEqual(r['model_sha256'],hashlib.sha256((HERE/'tab5_portrait_print.xml').read_bytes()).hexdigest());self.assertTrue(r['invariants']['all_exactly_equal']);self.assertEqual(r['total_sample_count'],1486);self.assertFalse(r['missing_explicit_physical_leg_pairs']);self.assertFalse(r['geometry']['fixed_CAD_solid_intersections_over_0p001_mm3']);self.assertFalse(r['geometry']['new_assembly_solid_intersections_over_0p001_mm3'])
  for row in r['reports']:
   self.assertEqual(row['new_parts_to_moving_leg_contact_count'],0)
   for group in row['groups'].values():self.assertGreater(group['minimum_gap_lower_bound_mm'],3)
  for name,meta in r['state_files'].items():self.assertEqual(meta['sha256'],hashlib.sha256((HERE/name).read_bytes()).hexdigest())
 def test_real_rollouts(self):
  m=mujoco.MjModel.from_xml_path(str(HERE/'scene_tab5_portrait_print.xml'));self.assertEqual(m.nu,10);self.assertEqual(m.jnt_type[0],mujoco.mjtJoint.mjJNT_FREE);np.testing.assert_allclose(m.opt.gravity,[0,0,-9.81])
  for name in ['walk_10s','idle_10s']:
   r=json.loads((HERE/(name+'.json')).read_text());self.assertIsNone(r['fall_reason']);self.assertAlmostEqual(r['simulated_s'],10);self.assertEqual(r['shell_leg_contact_samples'],0);self.assertEqual(r['runtime_privacy']['ORT_DISABLE_TELEMETRY'],'1');self.assertAlmostEqual(r['total_mass_kg'],sum(m.body_mass),places=12)
 def test_video_provenance(self):
  r=json.loads((HERE/'walking_demo.json').read_text());self.assertEqual(r['source_trace_sha256'],hashlib.sha256((HERE/'walk_10s_states.npz').read_bytes()).hexdigest());self.assertEqual(r['full_physics_model_sha256'],hashlib.sha256((HERE/'tab5_portrait_print.xml').read_bytes()).hexdigest());self.assertEqual(r['frame_count'],150);self.assertEqual(r['duration_s'],6)
if __name__=='__main__':unittest.main()
