"""Audit exported front-interface solids against the delivered v1 snapshot.

This script does not run a policy, change the model, or certify real hardware fit.
The reference is a separate immutable snapshot, not regenerated baseline CAD.
"""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET

import manifold3d as mf
import numpy as np
import trimesh

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
EXPORT_TOL_MM = 1e-5
VOLUME_TOL_MM3 = 1e-3
AREA_TOL_MM2 = 1e-3


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path, scale=1):
    tm = trimesh.load(path, process=True)
    tm.vertices *= scale
    solid = mf.Manifold(mf.Mesh64(
        np.ascontiguousarray(tm.vertices, dtype=np.float64),
        np.ascontiguousarray(tm.faces, dtype=np.uint64)))
    assert tm.is_watertight and tm.is_winding_consistent
    assert solid.status() == mf.Error.NoError, path
    return tm, solid


def box(bounds):
    b = np.asarray(bounds, dtype=float)
    return mf.Manifold.cube(b[:, 1] - b[:, 0]).translate(b[:, 0])


def volume(solid):
    return max(0.0, float(solid.volume()))


def section_yz(solid):
    return solid.transform([[0, 1, 0, 0], [0, 0, 1, 0], [1, 0, 0, 0]])


def front_face(mesh, x):
    triangles = mesh.triangles
    selected = np.max(np.abs(triangles[:, :, 0] - x), axis=1) <= EXPORT_TOL_MM
    contours = []
    for tri in triangles[selected, :, 1:]:
        u, v = tri[1] - tri[0], tri[2] - tri[0]
        area2 = u[0] * v[1] - u[1] * v[0]
        if abs(area2) < 1e-12:
            continue
        contours.append(tri if area2 > 0 else tri[::-1])
    return mf.CrossSection(contours), int(selected.sum())


def rounded_rectangle(a, b, radius):
    # Independent construction of the specified aperture, 48-sided arc sampling.
    return mf.CrossSection.square(
        [a[1] - a[0] - 2 * radius, b[1] - b[0] - 2 * radius],
        center=True).offset(radius, circular_segments=48).translate(
        [(a[1] + a[0]) / 2, (b[1] + b[0]) / 2])


def extrude_x(section, x0, x1):
    return section.extrude(x1 - x0).transform(
        [[0, 0, 1, x0], [1, 0, 0, 0], [0, 1, 0, 0]])


def axis_intersections(mesh, axis, other_coordinates):
    """Direct triangle/line tests; no ray library or sampled voxel inference."""
    other = [i for i in range(3) if i != axis]
    tri = mesh.triangles
    a = tri[:, 0, other]
    u = tri[:, 1, other] - a
    v = tri[:, 2, other] - a
    w = np.asarray(other_coordinates) - a
    det = u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0]
    eligible = np.abs(det) > 1e-12
    s = np.zeros(len(tri))
    t = np.zeros(len(tri))
    s[eligible] = (w[eligible, 0] * v[eligible, 1]
                   - w[eligible, 1] * v[eligible, 0]) / det[eligible]
    t[eligible] = (u[eligible, 0] * w[eligible, 1]
                   - u[eligible, 1] * w[eligible, 0]) / det[eligible]
    valid = eligible & (s >= -1e-9) & (t >= -1e-9) & (s + t <= 1 + 1e-9)
    hits = (tri[:, 0, axis] + s * (tri[:, 1, axis] - tri[:, 0, axis])
            + t * (tri[:, 2, axis] - tri[:, 0, axis]))[valid]
    hits = np.sort(hits)
    return [float(x) for i, x in enumerate(hits)
            if i == 0 or x - hits[i - 1] > EXPORT_TOL_MM]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, default=HERE.parent / 'portrait_print_v1')
    args = parser.parse_args()
    old = args.baseline.resolve()
    params = json.loads((HERE / 'parameters.json').read_text())
    before, a = load(old / 'assets/body_cage.obj', 1000)
    current, b = load(HERE / 'assets/body_cage.obj', 1000)
    stl, bs = load(HERE / 'stl/body_cage_mm.stl')
    added = b - a
    removed = a - b
    behind = box([[-100, 35], [-100, 100], [-100, 200]])
    behind_added = volume(added ^ behind)
    behind_removed = volume(removed ^ behind)

    # Read the unchanged Tab5's actual exported mesh and its local placement.
    current_xml = ET.parse(HERE / 'tab5_portrait_print.xml').getroot()
    old_xml = ET.parse(old / 'tab5_portrait_print.xml').getroot()
    tab_body = current_xml.find(".//body[@name='tab5']")
    old_tab_body = old_xml.find(".//body[@name='tab5']")
    assert 'quat' not in tab_body.attrib and 'euler' not in tab_body.attrib
    tab_geom = tab_body.find("geom[@name='tab5_visual']")
    tab_asset = current_xml.find(".//asset/mesh[@name='%s']" % tab_geom.attrib['mesh'])
    tab_path = (HERE / tab_asset.attrib['file']).resolve()
    tab_tm, tab_solid = load(tab_path, 1000)
    offset = np.fromstring(tab_body.attrib['pos'], sep=' ') * 1000
    tab_tm.apply_translation(offset)
    tab_solid = tab_solid.translate(offset)
    assert np.max(np.abs(tab_tm.bounds[:, 0] - [41, 53])) < EXPORT_TOL_MM

    face, face_triangles = front_face(current, 41)
    old_face, _ = front_face(before, 41)
    silhouette = section_yz(tab_solid).project()
    silhouette = mf.CrossSection(silhouette.to_polygons())
    outside = face - silhouette
    previous_outside = old_face - silhouette
    outside_face = max(0., float(outside.area()))
    previous_outside_area = max(0., float(previous_outside.area()))
    new_outside_area = max(0., float((outside - previous_outside).area()))
    removed_outside_area = max(0., float((previous_outside - outside).area()))
    # Exact convex-polygon distance at all outside-region vertices bounds the
    # small inherited polygon-tessellation discrepancy at the upper screw lands.
    outside_distance = 0.
    if outside_face > 0:
        points = np.vstack(outside.to_polygons())
        outline = np.vstack(silhouette.to_polygons())
        starts, ends = outline, np.roll(outline, -1, axis=0)
        directions = ends - starts
        offsets = points[:, None] - starts
        parameters = np.clip(np.sum(offsets * directions, axis=-1)
                             / np.sum(directions * directions, axis=-1), 0, 1)
        distances = np.linalg.norm(offsets - parameters[:, :, None] * directions, axis=-1)
        outside_distance = float(distances.min(axis=1).max())
    aperture = rounded_rectangle(*params['front_aperture_yz_mm'],
                                 params['front_aperture_radius_mm'])
    aperture_tool = extrude_x(aperture, 34, 45)
    aperture_intersection = volume(b ^ aperture_tool)
    aperture_face_area = max(0., float((face ^ aperture).area()))
    tab_overlap = volume(b ^ tab_solid)
    device_depth_overlap = volume(b ^ box([[41, 53], [-100, 100], [-100, 200]]))

    # At this plane the exported front rim, including its lands, must be fully flat.
    # A 10-micron-deep slab behind the face independently confirms face support.
    slab_width = .01
    slab_projection = section_yz(b ^ box(
        [[41 - slab_width, 41], [-100, 100], [-100, 200]])).project()
    slab_projection = mf.CrossSection(slab_projection.to_polygons())
    slab_face_extra = max(0., float((slab_projection - face).area()))
    face_slab_extra = max(0., float((face - slab_projection).area()))

    witnesses = []
    for yz in [[0, 50], [0, 122], [-35, 90], [35, 90]]:
        hits = axis_intersections(current, 0, yz)
        thickness = hits[-1] - hits[-2]
        witnesses.append({'direction': 'X', 'fixed_YZ_mm': yz,
                          'surface_intersections_mm': hits,
                          'front_wall_thickness_mm': thickness,
                          'pass': abs(thickness - 2.4) <= 2 * EXPORT_TOL_MM
                          and abs(hits[-2] - 38.6) <= EXPORT_TOL_MM
                          and abs(hits[-1] - 41) <= EXPORT_TOL_MM})
    side_hits = axis_intersections(current, 1, [0, 55])
    floor_hits = axis_intersections(current, 2, [0, 20])
    retained_walls = {
        'side_probe_XZ_mm': [0, 55], 'side_intersections_Y_mm': side_hits,
        'left_thickness_mm': side_hits[1] - side_hits[0],
        'right_thickness_mm': side_hits[-1] - side_hits[-2],
        'floor_probe_XY_mm': [0, 20], 'floor_intersections_Z_mm': floor_hits,
        'floor_thickness_mm': floor_hits[1] - floor_hits[0],
        'roof_thickness_mm': floor_hits[-1] - floor_hits[-2]}
    retained_walls['pass'] = all(abs(retained_walls[k] - 2.4) <= 2 * EXPORT_TOL_MM
                                for k in ['left_thickness_mm', 'right_thickness_mm',
                                          'floor_thickness_mm', 'roof_thickness_mm'])

    unchanged = []
    for name in ['rear_cover', 'central_mount', 'compute_pedestal',
                 'under_saddle', 'rear_hardware']:
        files = [Path('assets') / (name + '.obj')]
        if name in ['rear_cover', 'central_mount', 'compute_pedestal']:
            files.append(Path('stl') / (name + '_mm.stl'))
        for rel in files:
            unchanged.append({'file': str(rel), 'previous_sha256': sha(old / rel),
                              'current_sha256': sha(HERE / rel),
                              'byte_identical': (old / rel).read_bytes() == (HERE / rel).read_bytes()})

    # Parent's full compiled invariant audit checks joints, masses and mesh data;
    # also compare the directly retained source subtrees here.
    def elements_by_name(root, tag, predicate):
        return {e.attrib['name']: ET.tostring(e) for e in root.iter(tag)
                if predicate(e.attrib.get('name', ''))}
    leg_geom_before = elements_by_name(old_xml, 'geom', lambda n: n.startswith('shell_guard_legmesh_'))
    leg_geom_after = elements_by_name(current_xml, 'geom', lambda n: n.startswith('shell_guard_legmesh_'))
    original_mechanism_body = lambda n: (bool(n) and n not in ['trunk_base', 'tab5', 'mount_fastener_allowance']
                                        and not n.startswith(('print_', 'cube_portrait_')))
    leg_body_before = elements_by_name(old_xml, 'body', original_mechanism_body)
    leg_body_after = elements_by_name(current_xml, 'body', original_mechanism_body)
    leg_geom_same = leg_geom_before == leg_geom_after and len(leg_geom_before) > 0
    leg_body_same = leg_body_before == leg_body_after and len(leg_body_before) > 0
    tab_same = ET.tostring(tab_body) == ET.tostring(old_tab_body)
    density = params['plastic_density_kg_m3']
    report = {
        'source': {'baseline_directory': old.name,
                   'previous_body_OBJ_sha256': sha(old / 'assets/body_cage.obj'),
                   'current_body_OBJ_sha256': sha(HERE / 'assets/body_cage.obj'),
                   'current_body_STL_sha256': sha(HERE / 'stl/body_cage_mm.stl'),
                   'current_model_sha256': sha(HERE / 'tab5_portrait_print.xml'),
                   'Tab5_OBJ_sha256': sha(tab_path)},
        'method': 'Independent reread of actual exported triangle solids, Manifold Boolean volume differences and 2D polygon unions, plus direct triangle/line thickness intersections. No regenerated body used as its own oracle.',
        'tolerances': {'export_coordinate_mm': EXPORT_TOL_MM,
                       'Boolean_volume_mm3': VOLUME_TOL_MM3,
                       'polygon_area_mm2': AREA_TOL_MM2},
        'body_change': {'previous_volume_mm3': volume(a), 'current_volume_mm3': volume(b),
                        'added_volume_mm3': volume(added), 'removed_volume_mm3': volume(removed),
                        'net_volume_change_mm3': volume(b) - volume(a),
                        'net_mass_change_g_at_stated_density': (volume(b) - volume(a)) * density * 1e-6,
                        'density_kg_m3': density,
                        'behind_X35_added_mm3': behind_added,
                        'behind_X35_removed_mm3': behind_removed,
                        'behind_X35_symmetric_difference_mm3': behind_added + behind_removed,
                        'behind_X35_unchanged_within_export_precision': behind_added + behind_removed <= VOLUME_TOL_MM3},
        'front_interface': {'mating_plane_X_mm': 41,
                            'maximum_body_X_mm': float(current.bounds[1, 0]),
                            'maximum_positive_protrusion_past_X41_mm': max(0., float(current.bounds[1, 0] - 41)),
                            'volume_into_device_depth_X41_to_X53_mm3': device_depth_overlap,
                            'actual_exported_Tab5_solid_overlap_mm3': tab_overlap,
                            'planar_front_triangle_count': face_triangles,
                            'planar_contact_footprint_area_mm2': float(face.area()),
                            'previous_planar_contact_footprint_area_mm2': float(old_face.area()),
                            'footprint_outside_rounded_Tab5_silhouette_mm2': outside_face,
                            'whole_contact_footprint_strictly_inside_exported_silhouette': outside_face <= AREA_TOL_MM2,
                            'previous_footprint_outside_rounded_Tab5_silhouette_mm2': previous_outside_area,
                            'new_outside_footprint_area_mm2': new_outside_area,
                            'removed_outside_footprint_area_mm2': removed_outside_area,
                            'maximum_inherited_footprint_overhang_mm': outside_distance,
                            'footprint_exception': 'An unchanged 0.005957 mm2 total area at the two upper M3 lands extends at most 0.008566 mm beyond the coarser rounded Tab5 polygon silhouette. The exact same regions exist in v1. The newly added mating rim adds no outside area.',
                            'last_0p01mm_slab_projection_beyond_front_face_mm2': slab_face_extra,
                            'front_face_beyond_last_0p01mm_slab_projection_mm2': face_slab_extra,
                            'aperture_intersection_mm3': aperture_intersection,
                            'contact_face_area_inside_aperture_mm2': aperture_face_area,
                            'square_depth_edge_supported': slab_face_extra <= AREA_TOL_MM2 and face_slab_extra <= AREA_TOL_MM2},
        'wall_thickness': {'nominal_mm': 2.4, 'front_plane_triangle_witnesses': witnesses,
                           'retained_planar_walls': retained_walls,
                           'scope': 'Specified planar wall witnesses, not a global minimum-wall, strength, print-tolerance or layer-adhesion certification. Lands and screw bosses intentionally have other thicknesses.'},
        'export_integrity': {'OBJ_watertight': bool(current.is_watertight),
                             'OBJ_winding_consistent': bool(current.is_winding_consistent),
                             'OBJ_components': len(b.decompose()),
                             'STL_watertight': bool(stl.is_watertight),
                             'STL_winding_consistent': bool(stl.is_winding_consistent),
                             'STL_components': len(bs.decompose()),
                             'STL_vs_OBJ_volume_difference_mm3': abs(volume(bs) - volume(b)),
                             'STL_vs_OBJ_max_bounds_difference_mm': float(np.max(abs(stl.bounds - current.bounds)))},
        'preservation': {'other_parts_and_hardware': unchanged,
                         'Tab5_body_XML_exactly_equal': tab_same,
                         'leg_collision_geoms_XML_exactly_equal': leg_geom_same,
                         'leg_collision_geom_count': len(leg_geom_before),
                         'leg_body_subtrees_XML_exactly_equal': leg_body_same,
                         'leg_body_subtree_count': len(leg_body_before)},
        'limitations': [
            'The retained Tab5 model is a simplified rounded prism. Its real rear component geometry is not represented, so this audit establishes only nominal modeled face contact and zero modeled overlap.',
            'The official rear mounting pattern does not establish M3 screw engagement depth. Hardware fit, screw length, component clearance, print tolerance and load capacity remain unverified.',
            'Strict whole-footprint containment is false because the unchanged upper M3 land polygons differ slightly from the retained simplified Tab5 corner polygons; this inherited exception is quantified separately from the new rim.',
            'Raw Boolean differences behind X35 are reported, including minute OBJ coordinate quantization and triangulation differences; preservation is evaluated at the declared volume tolerance.',
            'No controller or policy inference, hardware action, or moving-leg clearance rerun is performed by this focused interface audit.']}
    report['all_revision_checks_pass'] = bool(
        report['body_change']['behind_X35_unchanged_within_export_precision']
        and current.bounds[1, 0] <= 41 + EXPORT_TOL_MM
        and device_depth_overlap <= VOLUME_TOL_MM3 and tab_overlap <= VOLUME_TOL_MM3
        and new_outside_area <= AREA_TOL_MM2 and removed_outside_area <= AREA_TOL_MM2
        and face.area() > old_face.area()
        and slab_face_extra <= AREA_TOL_MM2 and face_slab_extra <= AREA_TOL_MM2
        and aperture_intersection <= VOLUME_TOL_MM3 and aperture_face_area <= AREA_TOL_MM2
        and all(w['pass'] for w in witnesses) and retained_walls['pass']
        and len(b.decompose()) == 1 and len(bs.decompose()) == 1
        and all(row['byte_identical'] for row in unchanged)
        and tab_same and leg_geom_same and leg_body_same)
    report['all_strict_interface_checks_pass'] = bool(
        report['all_revision_checks_pass'] and outside_face <= AREA_TOL_MM2)
    report['status'] = ('revision_pass_with_inherited_silhouette_tessellation_exception'
                        if report['all_revision_checks_pass'] and outside_face > AREA_TOL_MM2
                        else 'pass' if report['all_strict_interface_checks_pass'] else 'fail')
    path = HERE / 'flush_interface_validation.json'
    path.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    if not report['all_revision_checks_pass']:
        raise SystemExit('Flush interface audit failed; inspect the report.')


if __name__ == '__main__':
    main()
