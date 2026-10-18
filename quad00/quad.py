"""The whole quad00: four identical legs (left/right mirrored) on a torso, as one GLB.

All four legs face the same way, knees back. Each leg's ab/ad RS00 sits ahead of its hip and
back-mounts on a transverse bulkhead: the front pair on the front bulkhead, the rear pair on a
bulkhead inside the body. A spine box (battery + compute) runs between them.

Run leg.py --assembly first; this reads out/assembly/*.stl.
  python3 quad.py  ->  out/assembly/quad.glb
"""
import glob
import math
import os

import numpy as np
import trimesh
from manifold3d import CrossSection, FillRule

import leg as L

HERE = os.path.dirname(os.path.abspath(__file__))
A = os.path.join(HERE, 'out', 'assembly')

HIP_X = 210.0                       # front hips at +210, rear at -210 (420 apart: legs can't meet)
LEG_Y = 102.0                       # leg planes at +/-102: ab/ad axes at +/-60
SPINE = (-270.0, HIP_X + L.X_BULK[0], 30.0, (-30.0, 70.0))   # x0, x1, half-width, z range


def bulkhead():
    """YZ plate holding a left and a right ab/ad RS00 by their back faces."""
    ya = LEG_Y + L.ABAD_Y                                   # ab/ad axis, body y
    b = L.rounded(-(ya + 35), -40, ya + 35, 40, 8)
    for s in (1, -1):
        b = b.difference(L.holes_on(L.BACK_PCD, L.BACK_N, L.M3, phase=45, at=(s * ya, 0)))
        b = b.difference(L.circle(s * ya, 0, 16))
    for z in (-28, 28):
        b = b.difference(L.circle(0, z, 6.4))               # spine bolts
    m = CrossSection(L.rings(b), FillRule.EvenOdd).extrude(2 * L.T)
    return L.tri(L.to_world(m, 'yz'))


def main():
    scene = trimesh.Scene()
    parts = sorted(f for f in glob.glob(os.path.join(A, '*.stl'))
                   if not os.path.basename(f).startswith(('step', 'bulkhead')))
    geoms = {os.path.basename(f)[:-4]: trimesh.load(f) for f in parts}
    for n, g in geoms.items():
        g.merge_vertices()
        scene.add_geometry(g, geom_name=n, node_name=f'{n}.proto', transform=np.diag([0, 0, 0, 1.0]))
    for tag, x, s in (('fl', HIP_X, 1), ('fr', HIP_X, -1), ('rl', -HIP_X, 1), ('rr', -HIP_X, -1)):
        T = trimesh.transformations.translation_matrix([x, s * LEG_Y, 0]) @ np.diag([1, s, 1, 1.0])
        for n in geoms:
            scene.graph.update(frame_to=f'{n}.{tag}', frame_from=scene.graph.base_frame, matrix=T, geometry=n)
    for n in [k for k in scene.graph.nodes_geometry if k.endswith('.proto')]:
        scene.graph.transforms.remove_node(n) if hasattr(scene.graph.transforms, 'remove_node') else None
    bh = bulkhead()
    scene.add_geometry(bh, geom_name='chassis-bulkhead', node_name='chassis-bulkhead.front',
                       transform=trimesh.transformations.translation_matrix([HIP_X, 0, 0]))
    scene.graph.update(frame_to='chassis-bulkhead.mid', frame_from=scene.graph.base_frame,
                       matrix=trimesh.transformations.translation_matrix([-HIP_X, 0, 0]), geometry='chassis-bulkhead')
    x0, x1, hw, (z0, z1) = SPINE
    spine = trimesh.creation.box(extents=[x1 - x0, 2 * hw, z1 - z0])
    spine.apply_translation([(x0 + x1) / 2, 0, (z0 + z1) / 2])
    scene.add_geometry(spine, geom_name='chassis-spine', node_name='chassis-spine')
    head = trimesh.creation.box(extents=[50, 70, 45])
    head.apply_translation([x1 + 2 * L.T + 40, 0, 35])
    neck = trimesh.creation.cylinder(radius=10, segment=[[x1 + 2 * L.T, 0, 20], [x1 + 2 * L.T + 16, 0, 30]])
    scene.add_geometry(trimesh.util.concatenate([head, neck]), geom_name='head', node_name='head')
    out = os.path.join(A, 'quad.glb')
    scene.export(out)
    print(f'quad: {out} ({os.path.getsize(out) / 1e6:.1f} MB)')


if __name__ == '__main__':
    main()
