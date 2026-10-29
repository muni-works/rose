"""3MFs of the node print parts for a P1S plate (256 x 256) -> ../fab/node/. Run after node_print.py.

  node-mvp.3mf       plain core-spec 3MF, meshes only: the slicer imports it as a model with your current settings.
  node-mvp-fast.3mf  Bambu project layout (the one vcad kit writes, with its plate numbered from 1 -- plater_id 0
                     segfaults Bambu Studio / Orca on load) carrying FAST per-object settings: 0.28 layers, 2 walls,
                     10% infill, tree supports at a 30 deg threshold. Pick the "0.28mm Extra Draft" process preset
                     for the speeds; the object settings override the rest.
  node-mvp-facedown.3mf  the same, with the upper laid on its face (less support inside the crown).
  node-mvp-fastest.3mf  face-down, 0.4 nozzle, everything turned toward speed (see FASTEST).
  node-mvp-06.3mf    face-down, for a 0.6 mm nozzle: 0.36 layers, 2 walls, no infill, supports at 35 deg.
  node-mvp-quality.3mf  for the look (PLA Matte): a variable layer-height profile from the surface slopes (0.08 on the
                     shallow dome, 0.24 on steep walls), slow outer walls, a scarf seam at the back, fewer travels
                     over the grilles. Pick a PLA Matte filament profile; dry it first.
  node-mvp-tilt.3mf  the quality settings with both parts tilted 30 deg back as assembled (crown toward the bed's back,
                     the face up): the crown and sides print steep, supports land on the occiput; uniform 0.12 layers (adaptive
                     goes fine everywhere once tilted). One filament for everything, so no prime tower.
"""
import math, os, zipfile
import trimesh

F = os.path.join(os.path.dirname(__file__) or '.', '..', 'fab', 'node')
PLACE = {'node-rear': (68, 128), 'node-face': (190, 128)}   # the face module lies face-down
FAST = {
    'layer_height': '0.28',
    'wall_loops': '2',
    'sparse_infill_density': '10%',
    'sparse_infill_pattern': 'grid',
    'top_shell_layers': '4',
    'bottom_shell_layers': '3',
    'enable_support': '1',
    'support_type': 'tree(auto)',
    'support_threshold_angle': '30',
    'brim_type': 'auto_brim',
}

# Fastest on a 0.4 nozzle: 0.5 mm lines so 2 loops per face fill the 2.0 shell exactly (no gap fill, no infill),
# draft speeds set on the objects so they hold under any process preset, supports only past 40 deg (face-down).
# Near-flat roofs (the back of the head, printed face-down) are ~7 layers thick through a 2.0 shell: 3 top + 3 bottom
# with 0% infill left the middle layer empty -- holes. 6 top + 4 bottom covers it, and 10% infill backs up anything else.
FASTEST = dict(FAST, sparse_infill_density='10%', top_shell_layers='6', bottom_shell_layers='4',
               line_width='0.5', outer_wall_line_width='0.5', inner_wall_line_width='0.5',
               outer_wall_speed='250', inner_wall_speed='350', sparse_infill_speed='350',
               internal_solid_infill_speed='300', top_surface_speed='200', gap_infill_speed='250',
               support_speed='250', support_threshold_angle='40')

# For a 0.6 mm nozzle (select the P1S 0.6 nozzle printer preset): 0.36 layers, 2 x 0.62 walls fill the 2.5 shell
# on their own, so no infill at all.
NOZZLE06 = dict(FAST, layer_height='0.36', sparse_infill_density='10%', top_shell_layers='5', bottom_shell_layers='3',
                support_threshold_angle='35')

# For the look (2026-10-28, after the fastest print showed layer lines on the dome and strings over the grilles):
# a variable layer-height profile does the real work (layer_profile below); the outer wall slows and prints between
# its inner walls, the seam goes to the back as a scarf joint, and travel avoids crossing walls so it doesn't string
# across the hundreds of grille holes.
QUALITY = dict(FAST, layer_height='0.2', wall_loops='3', top_shell_layers='5', bottom_shell_layers='4',
               outer_wall_speed='80', outer_wall_acceleration='3000', inner_wall_speed='200', top_surface_speed='100',
               wall_sequence='inner-outer-inner wall', seam_position='back', seam_slope_type='external',
               reduce_crossing_wall='1', support_threshold_angle='35')

ASA_LAYER = '0.2'                # ASA plate: uniform, so the prime tower stays legal

def layer_profile(m, h_min=0.08, h_max=0.24, cusp=0.045, step=0.5):
    """(z, h) pairs for the slicer's variable layer height: the stair a layer leaves on a surface grows with how
    flat that surface is, so each 0.5 mm band of the convex skin gets h = cusp / |n_z| of its flattest sloped point (dead-flat faces
    slice clean and are skipped), clamped, then each band takes the finest h within 1 mm of it."""
    import numpy as np
    sk = m.convex_hull                                  # the skin you see: inner posts, hole walls and dots don't count
    pts, fi = trimesh.sample.sample_surface_even(sk, 60000)
    nz = np.abs(sk.face_normals[fi, 2]); zc = pts[:, 2]
    top = m.bounds[1][2]; zs = np.arange(0.0, top + step, step)
    need = np.full(len(zs), h_max)
    sloped = (nz > 0.05) & (nz < 0.995)
    for i, z in enumerate(zs):
        sel = sloped & (np.abs(zc - z) <= step / 2)
        if sel.any():
            need[i] = np.clip(cusp / nz[sel].max(), h_min, h_max)
    k = int(round(1.0 / step))
    need = np.array([need[max(0, i - k):i + k + 1].min() for i in range(len(zs))])
    return [(float(z), float(round(h, 3))) for z, h in zip(zs, need)]

PLACE_TILT = {'node-rear': (128, 76), 'node-face': (128, 205)}   # tilted they're bigger: front to back, brims 4.5 clear

def meshes(face_down=False, tilt=0.0):
    for name, (x, y) in (PLACE_TILT if tilt else PLACE).items():
        m = trimesh.load(os.path.join(F, name + '.stl'))
        if tilt:                                         # lean back as assembled: top toward -x, the face toward the sky;
            m.apply_transform(trimesh.transformations.rotation_matrix(math.radians(-tilt), [0, 1, 0]))   # the face plate too
            if name == 'node-face':                      # its long side across the bed, so the two stack front to back
                m.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 0, 1]))
        if name == 'node-face' and not tilt:
            # the face module always prints face-down: its face plane on the bed (best surface, no supports)
            #
            m.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
            m.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 0, 1]))   # long side front-to-back
        c = m.bounds.mean(axis=0)
        m.apply_translation([x - c[0], y - c[1], -m.bounds[0][2]])
        yield name, m

def plain():
    scene = trimesh.Scene()
    for name, m in meshes():
        scene.add_geometry(m, node_name=name, geom_name=name)
    scene.export(os.path.join(F, 'node-mvp.3mf'))

NS = ('xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" '
      'xmlns:BambuStudio="http://schemas.bambulab.com/package/2021" '
      'xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06" requiredextensions="p"')

def uuid(n):
    return '%08x-0000-4000-8000-%012x' % (n, n)

def fast(face_down=False, out='node-mvp-fast.3mf', settings=None, adaptive=False, tilt=0.0):
    parts = list(meshes(face_down, tilt))
    prof = []
    files, objs, items, rels, cfg = {}, [], [], [], ['<?xml version="1.0" encoding="UTF-8"?>', '<config>']
    for k, (name, m) in enumerate(parts):
        mesh_id, obj_id = 2 * k + 1, 2 * k + 2
        c = m.bounds.mean(axis=0)
        c[2] = m.bounds[0][2]
        v = m.vertices - c                              # mesh local to its centre, placed by the build transform
        verts = ''.join('<vertex x="%.9g" y="%.9g" z="%.9g"/>' % tuple(p) for p in v)
        tris = ''.join('<triangle v1="%d" v2="%d" v3="%d"/>' % tuple(t) for t in m.faces)
        path = '3D/Objects/object_%d.model' % mesh_id
        files[path] = ('<?xml version="1.0" encoding="UTF-8"?>\n<model unit="millimeter" xml:lang="en-US" %s>\n'
                       ' <metadata name="BambuStudio:3mfVersion">1</metadata>\n <resources>\n'
                       '  <object id="%d" p:UUID="%s" type="model"><mesh><vertices>%s</vertices><triangles>%s</triangles></mesh></object>\n'
                       ' </resources>\n <build/>\n</model>\n') % (NS, mesh_id, uuid(mesh_id), verts, tris)
        objs.append('  <object id="%d" p:UUID="%s" type="model"><components><component p:path="/%s" objectid="%d" '
                    'p:UUID="%s" transform="1 0 0 0 1 0 0 0 1 0 0 0"/></components></object>'
                    % (obj_id, uuid(obj_id), path, mesh_id, uuid(100 + mesh_id)))
        items.append('  <item objectid="%d" p:UUID="%s" transform="1 0 0 0 1 0 0 0 1 %.4f %.4f %.4f" printable="1"/>'
                     % (obj_id, uuid(200 + obj_id), c[0], c[1], c[2]))
        rels.append(' <Relationship Target="/%s" Id="rel-%d" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>'
                    % (path, k + 1))
        if adaptive:                                    # Metadata/layer_heights_profile.txt: object_id=<n>|z;h;z;h;...
            prof.append('object_id=%d|%s' % (k + 1, ';'.join('%.3f;%.3f' % zh for zh in layer_profile(m))))
        cfg += ['  <object id="%d">' % obj_id, '    <metadata key="name" value="%s"/>' % name,
                '    <metadata key="extruder" value="1"/>']
        cfg += ['    <metadata key="%s" value="%s"/>' % kv for kv in (settings or FAST).items()]
        cfg += ['    <part id="1" subtype="normal_part">', '      <metadata key="name" value="%s"/>' % name,
                '      <metadata key="matrix" value="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"/>', '    </part>', '  </object>']
    cfg += ['  <plate>', '    <metadata key="plater_id" value="1"/>', '    <metadata key="plater_name" value="node MVP"/>',
            '    <metadata key="locked" value="false"/>']
    for k in range(len(parts)):
        cfg += ['    <model_instance>', '      <metadata key="object_id" value="%d"/>' % (2 * k + 2),
                '      <metadata key="instance_id" value="0"/>', '      <metadata key="identify_id" value="%d"/>' % (100 + k),
                '    </model_instance>']
    cfg += ['  </plate>', '</config>', '']
    files['3D/3dmodel.model'] = ('<?xml version="1.0" encoding="UTF-8"?>\n<model unit="millimeter" xml:lang="en-US" %s>\n'
                                 ' <metadata name="Application">node_3mf.py</metadata>\n'
                                 ' <metadata name="BambuStudio:3mfVersion">1</metadata>\n'
                                 ' <metadata name="Title">node MVP (fast)</metadata>\n <resources>\n%s\n </resources>\n'
                                 ' <build p:UUID="%s">\n%s\n </build>\n</model>\n') % (NS, '\n'.join(objs), uuid(999), '\n'.join(items))
    files['3D/_rels/3dmodel.model.rels'] = ('<?xml version="1.0" encoding="UTF-8"?>\n<Relationships '
                                            'xmlns="http://schemas.openxmlformats.org/package/2006/relationships">\n%s\n</Relationships>\n'
                                            % '\n'.join(rels))
    files['_rels/.rels'] = ('<?xml version="1.0" encoding="UTF-8"?>\n<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
                            '<Relationship Target="/3D/3dmodel.model" Id="rel-0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>\n')
    files['[Content_Types].xml'] = ('<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
                                    '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
                                    '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>'
                                    '<Default Extension="config" ContentType="text/xml"/></Types>')
    files['Metadata/model_settings.config'] = '\n'.join(cfg)
    if prof:
        files['Metadata/layer_heights_profile.txt'] = '\n'.join(prof) + '\n'
    with zipfile.ZipFile(os.path.join(F, out), 'w', zipfile.ZIP_DEFLATED) as z:
        for order in ('[Content_Types].xml', '_rels/.rels', '3D/3dmodel.model', '3D/_rels/3dmodel.model.rels'):
            z.writestr(order, files.pop(order))
        for path, data in files.items():
            z.writestr(path, data)

if __name__ == '__main__':
    plain()
    fast()
    fast(face_down=True, out='node-mvp-facedown.3mf')
    fast(face_down=True, out='node-mvp-06.3mf', settings=NOZZLE06)
    fast(face_down=True, out='node-mvp-fastest.3mf', settings=FASTEST)
    fast(face_down=True, out='node-mvp-quality.3mf', settings=QUALITY, adaptive=True)
    # tilted, every band of the egg has a shallow patch somewhere, so an adaptive profile is fine everywhere (~1960
    # layers); a uniform 0.12 is the better trade -- and the same height on both parts keeps a prime tower legal
    fast(face_down=True, out='node-mvp-tilt.3mf', tilt=30.0,
         settings=dict(QUALITY, layer_height='0.12', support_filament='0', support_interface_filament='0',
                       brim_type='outer_only', brim_width='5'))
    # ASA body (filament 1) with Support For PLA (filament 2) as the interface only: PLA doesn't bond to ASA, so the
    # support peels clean at 0 top gap; the bulk of the support stays ASA, so few swaps. Bambu refuses the pair
    # (PLA 190-240 vs ASA 240-280 nozzle ranges) until the support preset is set up for it: see print/PRINT.md (ASA).
    fast(face_down=True, out='node-mvp-asa.3mf', tilt=30.0,
         settings=dict(QUALITY, layer_height=ASA_LAYER, support_filament='1', support_interface_filament='2',
                       support_top_z_distance='0', support_bottom_z_distance='0', support_interface_top_layers='2',
                       support_interface_bottom_layers='2', support_interface_spacing='0.2',
                       flush_into_infill='1', flush_into_support='1',
                       brim_type='outer_only', brim_width='8'))
    print('node-mvp.3mf (plain), node-mvp-fast.3mf + node-mvp-facedown.3mf (Bambu projects, fast settings):', ', '.join(PLACE))
