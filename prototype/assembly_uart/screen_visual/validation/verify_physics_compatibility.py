"""Independently verify the screen-only variant against the frozen UART model.

No training, inference runtime, or artifact mutation occurs. Only this directory
receives a report. XML equality covers every model setting; compiled checks
independently cover physics, contacts, and policy-facing index contracts.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True
import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
ASSEMBLY = HERE.parents[1]
PROJECT = ASSEMBLY.parents[1]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def canonical(node, source, include_alias=None):
    attrs = dict(node.attrib)
    if node.tag == 'mujoco':
        attrs.pop('model', None)
    if 'file' in attrs:
        resolved = str((source.parent / attrs['file']).resolve())
        attrs['file'] = (include_alias or {}).get(resolved, resolved)
    return [node.tag, sorted(attrs.items()), (node.text or '').strip(),
            [canonical(child, source, include_alias) for child in node]]


def xml_contract(baseline, variant):
    a = ET.parse(baseline).getroot()
    b = ET.parse(variant).getroot()
    old_screen = a.find('./worldbody//body[@name="tab5"]/geom[@name="screen"]')
    new_screen = b.find('./worldbody//body[@name="tab5"]/geom[@name="screen"]')
    assert old_screen is not None and new_screen is not None
    screen_attrs = dict(new_screen.attrib)
    assert old_screen.get('mass') == new_screen.get('mass') == '0'
    assert old_screen.get('contype') == new_screen.get('contype') == '0'
    assert old_screen.get('conaffinity') == new_screen.get('conaffinity') == '0'
    assert new_screen.get('type') == 'mesh'
    allowed_screen_attrs = {'type', 'size', 'mesh', 'material', 'rgba'}
    assert {k: v for k, v in old_screen.attrib.items() if k not in allowed_screen_attrs} == {
        k: v for k, v in new_screen.attrib.items() if k not in allowed_screen_attrs}
    new_screen.attrib.clear()
    new_screen.attrib.update(old_screen.attrib)
    visual_names = {'screen'}
    for pair in b.findall('./contact/pair'):
        assert pair.get('geom1') not in visual_names and pair.get('geom2') not in visual_names
    eye_changes = []
    for eye in ('eye_left', 'eye_right'):
        ae = a.find(f'.//geom[@name="{eye}"]')
        be = b.find(f'.//geom[@name="{eye}"]')
        assert ae is not None and be is not None
        av = np.fromstring(ae.get('rgba'), sep=' ')
        bv = np.fromstring(be.get('rgba'), sep=' ')
        assert np.array_equal(av[:3], bv[:3]) and bv[3] == 0, eye
        eye_changes.append({'geom': eye, 'old_rgba': av.tolist(), 'new_rgba': bv.tolist()})
        be.set('rgba', ae.get('rgba'))
    old_assets = {(x.tag, x.get('name') or Path(x.get('file', '')).stem)
                  for x in a.find('asset')}
    extras = []
    for item in list(b.find('asset')):
        key = item.tag, item.get('name') or Path(item.get('file', '')).stem
        if key not in old_assets:
            assert item.tag in ('mesh', 'material', 'texture'), item.attrib
            extras.append((item.tag, dict(item.attrib)))
            b.find('asset').remove(item)
    mesh_names = {attrs.get('name') for tag, attrs in extras if tag == 'mesh'}
    material_names = {attrs.get('name') for tag, attrs in extras if tag == 'material'}
    texture_names = {attrs.get('name') for tag, attrs in extras if tag == 'texture'}
    assert mesh_names == {screen_attrs.get('mesh')}
    assert material_names == {screen_attrs.get('material')}
    assert texture_names == {attrs.get('texture') for tag, attrs in extras if tag == 'material'}
    # A new visual asset must never be referenced by any retained model node.
    for node in b.iter():
        assert node.get('mesh') not in mesh_names
        assert node.get('material') not in material_names
        assert node.get('texture') not in texture_names
    mesh = next(attrs for tag, attrs in extras if tag == 'mesh')
    mesh_path = (variant.parent / mesh['file']).resolve()
    vertices = np.array([[float(v) for v in line.split()[1:]] for line in mesh_path.read_text().splitlines()
                         if line.startswith('v ')])
    extents = np.fromstring(old_screen.get('size'), sep=' ')
    exact(vertices.min(axis=0), -extents, 'screen_mesh_minimum_bounds')
    exact(vertices.max(axis=0), extents, 'screen_mesh_maximum_bounds')
    ac, bc = canonical(a, baseline), canonical(b, variant)
    assert ac == bc, 'Model XML changed outside the permitted screen visuals'
    encoded = json.dumps(ac, separators=(',', ':')).encode()
    return {'pass': True, 'normalized_model_sha256': hashlib.sha256(encoded).hexdigest(),
            'screen_replacement': {'old': dict(old_screen.attrib), 'new': screen_attrs,
                                   'mesh_bounds_m': [vertices.min(axis=0).tolist(), vertices.max(axis=0).tolist()],
                                   'bounds_identical_to_original_screen': True},
            'eye_alpha_changes': eye_changes,
            'new_visual_assets': [{'kind': t, **v} for t, v in extras]}, visual_names


def scene_contract(baseline, variant, baseline_model, variant_model):
    aliases = {str(baseline_model): '<verified_model>', str(variant_model): '<verified_model>'}
    a = canonical(ET.parse(baseline).getroot(), baseline, aliases)
    b = canonical(ET.parse(variant).getroot(), variant, aliases)
    assert a == b, 'Scene settings changed outside its verified model include'
    return {'pass': True, 'baseline_sha256': sha(baseline), 'variant_sha256': sha(variant)}


def exact(a, b, label):
    a, b = np.asarray(a), np.asarray(b)
    assert a.shape == b.shape and np.array_equal(a, b, equal_nan=True), (
        label, a.shape, b.shape,
        float(np.max(np.abs(a.astype(float) - b.astype(float)))) if a.shape == b.shape and a.size else None)


def compiled_contract(a, b, visual_names):
    counts = ('nbody', 'njnt', 'nq', 'nv', 'nu', 'na', 'nsensor', 'nsensordata',
              'nsite', 'npair', 'nexclude', 'neq', 'ntendon', 'nflex', 'nkey', 'nmocap')
    sizes = {k: [int(getattr(a, k)), int(getattr(b, k))] for k in counts}
    assert all(v[0] == v[1] for v in sizes.values()), sizes
    assert b.ngeom == a.ngeom
    retained = np.arange(b.ngeom)
    assert len(retained) == a.ngeom
    inverse = np.full(b.ngeom, -1, dtype=int)
    inverse[retained] = np.arange(a.ngeom)
    assert [a.geom(i).name for i in range(a.ngeom)] == [b.geom(i).name for i in retained]
    order = {}
    for kind, count in [('body', a.nbody), ('joint', a.njnt), ('actuator', a.nu), ('sensor', a.nsensor), ('site', a.nsite)]:
        aa = [getattr(a, kind)(i).name for i in range(count)]
        bb = [getattr(b, kind)(i).name for i in range(count)]
        assert aa == bb, kind
        order[kind] = aa
    fields = []
    # The existing screen slot is reused, so even body geometry and broad-phase
    # allocation must retain their original order and indices.
    storage_only = set()
    prefixes = ('body_', 'jnt_', 'dof_', 'actuator_', 'sensor_', 'site_', 'tendon_',
                'ten_', 'eq_', 'exclude_', 'flex', 'tree_', 'key_', 'qpos',
                'numeric_', 'tuple_', 'wrap_', 'plugin', 'B_', 'D_', 'M_', 'map')
    for key in dir(a):
        value = getattr(a, key)
        if isinstance(value, np.ndarray) and key.startswith(prefixes) and key not in storage_only:
            exact(value, getattr(b, key), key)
            fields.append(key)
    option_fields = []
    for key in dir(a.opt):
        if key.startswith('_'):
            continue
        value = getattr(a.opt, key)
        if isinstance(value, (int, float, np.ndarray)):
            exact(value, getattr(b.opt, key), 'opt.' + key)
            option_fields.append(key)
    geom_fields = []
    for key in dir(a):
        if not key.startswith('geom_') or not isinstance(getattr(a, key), np.ndarray):
            continue
        av, bv = getattr(a, key).copy(), getattr(b, key)[retained].copy()
        if key == 'geom_matid':
            # The new material precedes the scene's ground material. Compare
            # existing material identity, not its shifted asset-table address.
            remap = np.array([mujoco.mj_name2id(a, mujoco.mjtObj.mjOBJ_MATERIAL,
                mujoco.mj_id2name(b, mujoco.mjtObj.mjOBJ_MATERIAL, i)) for i in range(b.nmat)])
            mask = bv >= 0
            bv[mask] = remap[bv[mask]]
        if key == 'geom_rgba':
            for eye in ('eye_left', 'eye_right'):
                bv[a.geom(eye).id, 3] = av[a.geom(eye).id, 3]
        if key in ('geom_type', 'geom_dataid', 'geom_matid', 'geom_rgba', 'geom_pos',
                   'geom_quat', 'geom_size', 'geom_aabb', 'geom_rbound', 'geom_sameframe'):
            for name in visual_names:
                bv[a.geom(name).id] = av[a.geom(name).id]
        exact(av, bv, key)
        geom_fields.append(key)
    pair_fields = []
    for key in dir(a):
        if key.startswith('pair_') and isinstance(getattr(a, key), np.ndarray):
            bv = getattr(b, key)
            if key in ('pair_geom1', 'pair_geom2'):
                bv = inverse[bv]
                assert np.all(bv >= 0), 'Screen visual appears in explicit contact pairs'
            exact(getattr(a, key), bv, key)
            pair_fields.append(key)
    for name in visual_names:
        g = b.geom(name).id
        assert b.geom_contype[g] == b.geom_conaffinity[g] == 0
        assert b.geom_bodyid[g] == b.body('tab5').id
    # Original mesh assets must preserve compilation, vertices, triangles,
    # normals, and convex collision graph; only append-only new assets allowed.
    assert [a.mesh(i).name for i in range(a.nmesh)] == [b.mesh(i).name for i in range(a.nmesh)]
    mesh_fields = []
    for key in dir(a):
        if key.startswith('mesh_') and isinstance(getattr(a, key), np.ndarray):
            if key in ('mesh_pathadr', 'mesh_bvhadr'):
                continue
            av = getattr(a, key)
            bv = getattr(b, key)[:len(av)]
            exact(av, bv, key)
            mesh_fields.append(key)
    qidx = a.jnt_qposadr[a.actuator_trnid[:, 0]]
    vidx = a.jnt_dofadr[a.actuator_trnid[:, 0]]
    exact(qidx, b.jnt_qposadr[b.actuator_trnid[:, 0]], 'policy_qpos_indices')
    exact(vidx, b.jnt_dofadr[b.actuator_trnid[:, 0]], 'policy_qvel_indices')
    gyro = int(a.sensor_adr[a.sensor('imu_ang_vel').id])
    assert gyro == int(b.sensor_adr[b.sensor('imu_ang_vel').id])
    return {'pass': True, 'counts_baseline_and_variant': sizes,
            'geometry_counts': [a.ngeom, b.ngeom], 'mesh_counts': [a.nmesh, b.nmesh],
            'total_mass_kg': float(a.body_mass.sum()),
            'maximum_mass_or_inertia_difference': 0.0,
            'checked_physical_arrays': fields, 'checked_option_fields': option_fields,
            'checked_retained_geom_arrays': geom_fields,
            'checked_pair_arrays': pair_fields, 'checked_original_mesh_arrays': mesh_fields,
            'excluded_storage_index_arrays': sorted(storage_only),
            'ordered_names': order,
            'policy_observation_contract': {
                'dimensions': 39,
                'order': 'gyro3,projected_gravity3,joint_delta10,joint_velocity10,previous_action10,velocity_command3',
                'qpos_indices': qidx.tolist(), 'qvel_indices': vidx.tolist(),
                'gyro_sensor_data_offset': gyro, 'action_order': order['actuator'],
                'source': 'training/tab5_gaits/env.py:25-30,64-68; policy.py:46',
                'environment_code_sha256': sha(PROJECT/'training/tab5_gaits/env.py')},
            'unchanged_collision_geometry_count': int(np.count_nonzero((a.geom_contype != 0) | (a.geom_conaffinity != 0))),
            'unchanged_explicit_pairs': a.npair, 'unchanged_exclusions': a.nexclude,
            'geom_id_shift_count': int(np.count_nonzero(retained != np.arange(a.ngeom))),
            'material_comparison': 'Existing materials compared by name; the new screen material shifts the groundplane material asset index by one',
            'ignored_screen_only_geom_arrays': ['geom_type', 'geom_dataid', 'geom_matid', 'geom_rgba',
                'geom_pos', 'geom_quat', 'geom_size', 'geom_aabb', 'geom_rbound', 'geom_sameframe']}, inverse


def forward_contract(a, b, inverse):
    da, db = mujoco.MjData(a), mujoco.MjData(b)
    rows = []
    # All saved postures and a perturbed standing state test real contact and
    # mass-matrix computation. This is an equivalence check, not posture safety.
    states = [(a.key(i).name, a.key_qpos[i].copy(), a.key_ctrl[i].copy()) for i in range(a.nkey)]
    stand = a.key_qpos[a.key('STAND').id].copy()
    stand[0:3] += np.array([.015, -.01, .012])
    stand[7:] += np.linspace(-.025, .025, a.nu)
    states.append(('PERTURBED_STAND', stand, a.key_ctrl[a.key('STAND').id].copy()))
    data_fields = ('qM', 'qfrc_bias', 'qfrc_passive', 'qfrc_actuator', 'qfrc_constraint',
                   'qacc', 'sensordata', 'subtree_com', 'xipos', 'ximat', 'xpos', 'xquat')
    for name, qpos, ctrl in states:
        for model, data in ((a, da), (b, db)):
            mujoco.mj_resetData(model, data)
            data.qpos[:] = qpos
            data.qvel[:] = np.linspace(-.015, .015, model.nv)
            data.ctrl[:] = ctrl
            mujoco.mj_forward(model, data)
        for field in data_fields:
            exact(getattr(da, field), getattr(db, field), name + '.' + field)
        assert da.ncon == db.ncon, name
        for ca, cb in zip(da.contact[:da.ncon], db.contact[:db.ncon]):
            exact([ca.geom1, ca.geom2], inverse[[cb.geom1, cb.geom2]], name + '.contact_geoms')
            for field in ('dist', 'pos', 'frame', 'friction', 'solref', 'solimp', 'dim', 'includemargin'):
                exact(getattr(ca, field), getattr(cb, field), name + '.contact.' + field)
        rows.append({'state': name, 'pass': True, 'contact_count': da.ncon,
                     'maximum_compared_dynamics_difference': 0.0})
    return {'pass': True, 'checked_data_arrays': list(data_fields), 'states': rows,
            'scope': 'Exact forward-dynamics/contact equivalence; no gait replay, BAM rollout, ORT, or training invoked'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--variant', type=Path, default=HERE.parent/'tab5_assembly_uart_screen.xml')
    parser.add_argument('--scene', type=Path, default=HERE.parent/'scene_tab5_assembly_uart_screen.xml')
    parser.add_argument('--report', type=Path, default=HERE/'physics_compatibility_report.json')
    args = parser.parse_args()
    args.variant, args.scene = args.variant.resolve(), args.scene.resolve()
    assert args.report.resolve().is_relative_to(HERE), 'Report must remain in validation directory'
    baseline = ASSEMBLY/'tab5_assembly_uart.xml'
    baseline_scene = ASSEMBLY/'scene_tab5_assembly_uart.xml'
    started = time.monotonic()
    report = {'status': 'running', 'mujoco_version': mujoco.__version__,
              'baseline_model': str(baseline), 'variant_model': str(args.variant),
              'baseline_sha256': sha(baseline), 'variant_sha256': sha(args.variant),
              'verifier_sha256': sha(__file__),
              'usage_limit': 'Display and nominal visualization only. Keep training and randomized evaluation on the frozen parent-folder scenes. training/tab5_gaits/env.py:32-42 finds component-specific light/heavy mass variants beside the supplied scene; using the screen_visual scene with randomize=True would instead fall back to global 0.97-1.03 mass scaling. No inference or additional mass-variant rollout was performed.'}
    try:
        manifest = json.loads((ASSEMBLY/'frozen_model_manifest.json').read_text())
        entries = manifest.get('relative_files')
        mismatches = []
        for filename, expected in (entries or manifest['files']).items():
            path = PROJECT/filename if not Path(filename).is_absolute() else Path(filename)
            if not path.exists() or sha(path) != expected:
                mismatches.append(str(path))
        assert not mismatches, ('Frozen source mismatch', mismatches)
        report['frozen_files'] = {'pass': True, 'verified_count': len(entries or manifest['files']),
                                  'manifest_sha256': sha(ASSEMBLY/'frozen_model_manifest.json'),
                                  'nominal_model_sha256': manifest['nominal_model_sha256']}
        report['xml'], new_names = xml_contract(baseline, args.variant)
        report['scene_xml'] = scene_contract(baseline_scene, args.scene, baseline, args.variant)
        print('Frozen hashes and complete normalized XML: PASS', flush=True)
        a = mujoco.MjModel.from_xml_path(str(baseline_scene))
        print('Compiled baseline scene', flush=True)
        b = mujoco.MjModel.from_xml_path(str(args.scene))
        print('Compiled screen variant scene', flush=True)
        report['compiled'], inverse = compiled_contract(a, b, new_names)
        report['forward_dynamics'] = forward_contract(a, b, inverse)
        report['status'] = 'pass'
    except Exception as error:
        report['status'] = 'fail'
        report['error'] = repr(error)
        raise
    finally:
        report['elapsed_seconds'] = time.monotonic() - started
        args.report.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({'status': report['status'], 'report': str(args.report),
                          'elapsed_seconds': report['elapsed_seconds']}), flush=True)


if __name__ == '__main__':
    main()
