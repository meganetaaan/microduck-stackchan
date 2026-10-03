"""Reproducible read-only low-cube moving-leg clearance audit.

Run with the repository .venv Python. Only the requested JSON output is written.
Distances are independently optimized between each compiled leg-mesh convex hull
and each shell/Tab5 box. They are NOT raw mj_geomDistance positive distances:
MuJoCo 3.10.0 sometimes reports a spurious zero for separated mesh/box pairs.
The bounds here are geometric, sampled, and do not establish hardware safety.
"""
from pathlib import Path
import argparse
import hashlib
import itertools
import json
import mujoco
import numpy as np
import scipy
from scipy.optimize import LinearConstraint, minimize
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parent


class Auditor:
    def __init__(self, scene):
        self.model = m = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(m)
        self.trunk = m.body('trunk_base').id
        self.legs = [i for i in range(m.ngeom)
                     if m.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH
                     and m.geom_group[i] == 2
                     and m.geom_bodyid[i] != self.trunk]
        self.panels = [i for i in range(m.ngeom)
                       if m.geom(i).name.startswith('cube_')
                       or m.geom(i).name == 'tab5_collision']
        self.hulls, self.vertices = {}, {}
        for i in self.legs:
            mesh = m.geom_dataid[i]
            if mesh in self.hulls:
                continue
            start, count = m.mesh_vertadr[mesh], m.mesh_vertnum[mesh]
            verts = m.mesh_vert[start:start + count].astype(float) * 1000
            hull = ConvexHull(verts)
            self.vertices[mesh] = verts[hull.vertices]
            self.hulls[mesh] = hull.equations

    def forward(self, qpos):
        m, d = self.model, self.data
        d.qpos[:] = qpos
        mujoco.mj_forward(m, d)
        self.tr = d.xmat[self.trunk].reshape(3, 3)
        self.tp = d.xpos[self.trunk].copy()

    def mesh_transform(self, geom):
        m, d = self.model, self.data
        rotation = self.tr.T @ d.geom_xmat[geom].reshape(3, 3)
        position = (d.geom_xpos[geom] - self.tp) @ self.tr * 1000
        verts = self.vertices[m.geom_dataid[geom]] @ rotation.T + position
        return verts.min(0), verts.max(0), rotation, position

    def box_bounds(self, geom):
        m, d = self.model, self.data
        # All tested case/device boxes are axis aligned with the trunk frame.
        rotation = self.tr.T @ d.geom_xmat[geom].reshape(3, 3)
        assert np.allclose(rotation, np.eye(3), atol=1e-10)
        center = (d.geom_xpos[geom] - self.tp) @ self.tr * 1000
        size = m.geom_size[geom] * 1000
        return center - size, center + size

    def optimize_pair(self, panel, leg):
        m = self.model
        lo, hi, rotation, position = self.mesh_transform(leg)
        box_lo, box_hi = self.box_bounds(panel)
        equations = self.hulls[m.geom_dataid[leg]]
        matrix = equations[:, :3] @ rotation.T
        bound = matrix @ position - equations[:, 3]

        def objective(point):
            delta = point - np.clip(point, box_lo, box_hi)
            return np.dot(delta, delta)

        def gradient(point):
            return 2 * (point - np.clip(point, box_lo, box_hi))

        result = minimize(
            objective, (lo + hi) / 2, jac=gradient,
            constraints=[LinearConstraint(matrix, -np.inf, bound)],
            method='SLSQP', options={'ftol': 1e-9, 'maxiter': 150})
        return {
            'gap_mm': float(np.sqrt(max(0, result.fun))),
            'optimizer_success': bool(result.success),
            'optimizer_status': int(result.status),
            'constraint_violation_mm': float(max(0, np.max(matrix @ result.x - bound))),
            'mesh_closest_point_trunk_mm': result.x.tolist(),
            'box_closest_point_trunk_mm': np.clip(result.x, box_lo, box_hi).tolist(),
        }

    def audit_states(self, label, states, sample_dt=None):
        m = self.model
        # AABB separation is a rigorous lower bound on convex-mesh/box distance.
        # Keep candidates below 20 mm; every reported winner is below this bound.
        candidates = {'shell': [], 'tab5': []}
        for index, qpos in enumerate(states):
            self.forward(qpos)
            meshes = {i: self.mesh_transform(i)[:2] for i in self.legs}
            for panel in self.panels:
                blo, bhi = self.box_bounds(panel)
                group = 'tab5' if m.geom(panel).name == 'tab5_collision' else 'shell'
                for leg, (lo, hi) in meshes.items():
                    lower = float(np.linalg.norm(np.maximum(np.maximum(blo - hi, lo - bhi), 0)))
                    if lower < 20:
                        candidates[group].append((lower, index, panel, leg))
        report = {'label': label, 'sample_count': len(states), 'groups': {}}
        for group, items in candidates.items():
            best, winner, solved, failures = 20., None, 0, []
            for lower, index, panel, leg in sorted(items):
                if lower > best + 1e-6:
                    break
                self.forward(states[index])
                result = self.optimize_pair(panel, leg)
                solved += 1
                if not result['optimizer_success'] or result['constraint_violation_mm'] > 1e-5:
                    failures.append({'sample': index, 'panel': m.geom(panel).name,
                                     'leg_geom': m.geom(leg).name, **result})
                if result['gap_mm'] < best:
                    best = result['gap_mm']
                    winner = {
                        'sample_index': index,
                        'time_s': None if sample_dt is None else index * sample_dt,
                        'panel': m.geom(panel).name,
                        'leg_body': m.body(m.geom_bodyid[leg]).name,
                        'leg_mesh': m.mesh(m.geom_dataid[leg]).name,
                        'leg_geom': m.geom(leg).name,
                        'qpos': states[index].tolist(),
                        'joint_angles_deg': np.rad2deg(states[index][7:]).tolist(),
                        **result,
                    }
                    # Negative values corroborate overlap in the simulator's
                    # convex collision proxies. This is not a certified physical
                    # penetration measurement of the original concave CAD mesh.
                    if best < 1e-5:
                        winner['mujoco_signed_distance_mm'] = float(
                            mujoco.mj_geomDistance(m, self.data, panel, leg, .003, None) * 1000)
                if best < 1e-7:
                    break  # Unsigned distance cannot be less than zero.
            report['groups'][group] = {
                'minimum_gap_mm': None if winner is None else best,
                'all_pairs_at_least_20mm': winner is None,
                'winner': winner, 'candidate_pairs': len(items),
                'optimized_pairs': solved, 'optimizer_failures': failures,
            }
        return report


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene', type=Path, default=ROOT / 'models/scene_tab5_lowcube.xml')
    parser.add_argument('--trace', type=Path, default=ROOT / 'results/lowcube_walk_030_states.npz')
    parser.add_argument('--out', type=Path, default=ROOT / 'results/lowcube_clearance_audit.json')
    args = parser.parse_args()
    auditor = Auditor(args.scene)
    m, d = auditor.model, auditor.data
    reports = []
    for key in ['STAND', 'SIT', 'FOLD']:
        mujoco.mj_resetDataKeyframe(m, d, m.key(key).id)
        reports.append(auditor.audit_states(key, np.array([d.qpos.copy()])))
    mujoco.mj_resetDataKeyframe(m, d, m.key('STAND').id)
    stand = d.qpos.copy()
    bounded = []
    offsets = list(itertools.product([-10, 0, 10], [-5, 0, 5],
                                    [-10, 0, 10], [-10, 0, 10], [-10, 0, 10]))
    for offset in offsets:
        q = stand.copy()
        q[7:12] += np.deg2rad(offset)
        q[12:17] -= np.deg2rad(offset)
        bounded.append(q)
    reports.append(auditor.audit_states('bounded_per_leg_grid', np.array(bounded)))
    trace = np.load(args.trace)
    reports.append(auditor.audit_states('nominal_walking_trace', trace['qpos'], float(trace['dt'])))
    output = {
        'scene': str(args.scene), 'scene_sha256': sha(args.scene),
        'model_sha256': sha(args.scene.parent / 'tab5_lowcube.xml'),
        'trace': str(args.trace), 'trace_sha256': sha(args.trace),
        'mujoco_version': mujoco.__version__, 'scipy_version': scipy.__version__,
        'moving_visual_mesh_count': len(auditor.legs),
        'shell_and_tab5_box_count': len(auditor.panels),
        'method': 'Compiled MuJoCo mesh_vert geometry in its geom frame, Qhull convex hull via scipy.spatial.ConvexHull, transformed into trunk millimetres. SLSQP minimizes squared distance of a convex-hull point to the axis-aligned shell/Tab5 box, under every hull half-space inequality. AABB lower bounds prune pairs. This is independent of MuJoCo contact filtering.',
        'optimizer': {'ftol_squared_mm': 1e-9, 'maxiter': 150,
                      'gradient': 'analytic', 'constraint_tolerance_mm': 1e-5},
        'bounded_grid': {
            'center_qpos': stand.tolist(), 'center_joint_angles_deg': np.rad2deg(stand[7:]).tolist(),
            'per_leg_joint_order': ['hip_yaw', 'hip_roll', 'hip_pitch', 'knee', 'ankle'],
            'offset_choices_degrees': [[-10, 0, 10], [-5, 0, 5], [-10, 0, 10], [-10, 0, 10], [-10, 0, 10]],
            'count': 243,
            'scope': 'Every 3^5 independent-joint sample for each leg is represented. Right offsets are mirrored in the same states. This covers marginal shell-versus-each-leg samples; it does not audit leg-versus-leg interactions or prove the continuous interior of the joint range is clear.',
        },
        'caveats': [
            'All distances are to convex hulls of the visual meshes, conservative for concave details. Positive hull separation guarantees separation of enclosed mesh geometry at that sampled pose. Hull overlap alone need not imply that the original concave surfaces intersect.',
            'This is sampled geometry, not continuous collision detection or certified manufacturing clearance. No numerical optimizer result is a formal proof. Check solver success and residuals.',
            'The 50 Hz walking trace can miss a smaller gap between samples. Cable routing, wire flex, connectors, tolerances, structural deflection, print strength, mounting and assembly access are not represented.',
            'SIT and FOLD are deliberately retained as obstruction tests; a successful nominal gait does not imply unrestricted original articulation.',
            'Raw positive mj_geomDistance readings are not used because this local MuJoCo build returned spurious zeros for some clearly separated pairs. Signed overlap is supplied only as a simulator-proxy corroboration when independently optimized distance is zero.',
        ],
        'reports': reports,
    }
    args.out.write_text(json.dumps(output, indent=2))
    for report in reports:
        print(report['label'], {g: r['minimum_gap_mm'] for g, r in report['groups'].items()})
    print('Wrote', args.out)


if __name__ == '__main__':
    main()
