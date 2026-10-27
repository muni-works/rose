"""3MFs of the node print parts for a P1S plate (256 x 256) -> ../fab/node/. Run after node_print.py.

  node-mvp.3mf       plain core-spec 3MF, meshes only: the slicer imports it as a model with your current settings.
  node-mvp-fast.3mf  Bambu project layout (the one vcad kit writes, with its plate numbered from 1 -- plater_id 0
                     segfaults Bambu Studio / Orca on load) carrying FAST per-object settings: 0.28 layers, 2 walls,
                     10% infill, tree supports at a 30 deg threshold. Pick the "0.28mm Extra Draft" process preset
                     for the speeds; the object settings override the rest.
"""
import os, zipfile
import trimesh

F = os.path.join(os.path.dirname(__file__) or '.', '..', 'fab', 'node')
PLACE = {'node-upper': (64, 128), 'node-base': (191, 128)}
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

def meshes():
    for name, (x, y) in PLACE.items():
        m = trimesh.load(os.path.join(F, name + '.stl'))
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

def fast():
    parts = list(meshes())
    files, objs, items, rels, cfg = {}, [], [], [], ['<?xml version="1.0" encoding="UTF-8"?>', '<config>']
    for k, (name, m) in enumerate(parts):
        mesh_id, obj_id = 2 * k + 1, 2 * k + 2
        c = m.bounds.mean(axis=0)
        c[2] = m.bounds[0][2]
        v = m.vertices - c                              # mesh local to its centre, placed by the build transform
        verts = ''.join('<vertex x="%.7g" y="%.7g" z="%.7g"/>' % tuple(p) for p in v)
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
        cfg += ['    <metadata key="%s" value="%s"/>' % kv for kv in FAST.items()]
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
    with zipfile.ZipFile(os.path.join(F, 'node-mvp-fast.3mf'), 'w', zipfile.ZIP_DEFLATED) as z:
        for order in ('[Content_Types].xml', '_rels/.rels', '3D/3dmodel.model', '3D/_rels/3dmodel.model.rels'):
            z.writestr(order, files.pop(order))
        for path, data in files.items():
            z.writestr(path, data)

if __name__ == '__main__':
    plain()
    fast()
    print('node-mvp.3mf (plain), node-mvp-fast.3mf (Bambu project, fast settings):', ', '.join(PLACE))
