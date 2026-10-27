"""Plain core-spec 3MF of the node print parts for a P1S plate (256 x 256) -> ../fab/node/node-mvp.3mf.
Run after node_print.py. Plain on purpose: vcad kit's Bambu project metadata (plater_id 0) segfaults Bambu Studio /
Orca on load, so this carries meshes only and the slicer imports it as a model."""
import os
import trimesh

F = os.path.join(os.path.dirname(__file__) or '.', '..', 'fab', 'node')
PLACE = {'node-upper': (64, 128), 'node-base': (191, 128)}

scene = trimesh.Scene()
for name, (x, y) in PLACE.items():
    m = trimesh.load(os.path.join(F, name + '.stl'))
    c = m.bounds.mean(axis=0)
    m.apply_translation([x - c[0], y - c[1], -m.bounds[0][2]])
    scene.add_geometry(m, node_name=name, geom_name=name)
scene.export(os.path.join(F, 'node-mvp.3mf'))
print('node-mvp.3mf:', ', '.join(PLACE))
