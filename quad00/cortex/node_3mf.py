"""3MFs of the node print parts for a P1S plate (256 x 256) -> ../fab/node/. Run after node_print.py.

  node-mvp.3mf       plain core-spec 3MF, meshes only: the slicer imports it as a model with your current settings.
  node-mvp-fast.3mf  Bambu project layout (the one vcad kit writes, with its plate numbered from 1 -- plater_id 0
                     segfaults Bambu Studio / Orca on load) carrying FAST per-object settings: 0.28 layers, 2 walls,
                     10% infill, tree supports at a 30 deg threshold. Pick the "0.28mm Extra Draft" process preset
                     for the speeds; the object settings override the rest.
  node-mvp-facedown.3mf  the same, with the upper laid on its face (less support inside the crown).
  node-mvp-fastest.3mf  face-down, 0.4 nozzle, everything turned toward speed (see FASTEST).
  node-mvp-06.3mf    face-down, for a 0.6 mm nozzle: 0.36 layers, 2 walls, no infill, supports at 35 deg.
"""
import math, os, zipfile
import trimesh

F = os.path.join(os.path.dirname(__file__) or '.', '..', 'fab', 'node')
PLACE = {'node-rear': (68, 128), 'node-face': (190, 128), 'node-shutter': (68, 228)}   # the face module lies face-down
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

def meshes(face_down=False):
    for name, (x, y) in PLACE.items():
        m = trimesh.load(os.path.join(F, name + '.stl'))
        if name in ('node-face', 'node-shutter'):
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

def fast(face_down=False, out='node-mvp-fast.3mf', settings=None):
    parts = list(meshes(face_down))
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
    print('node-mvp.3mf (plain), node-mvp-fast.3mf + node-mvp-facedown.3mf (Bambu projects, fast settings):', ', '.join(PLACE))
