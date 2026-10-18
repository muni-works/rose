"""Whole-quad architectures built from the one quad00 leg, for side-by-side comparison in the viewer.

  corner  -- Mini Cheetah layout. One body box as wide as the legs; the four leg clusters sit at its
             corners, just past each end. Every ab/ad RS00 points back into the body and bolts to an
             end wall, so the front legs carry their yoke + ab/ad mirrored fore-aft.
  side    -- Go2 layout. Legs hang along the body's sides, all facing forward. Each ab/ad sits ahead
             of its hip inside the body, on a cross wall; the body is pocketed where the hip clusters
             tuck in.

Body frame: X forward, Y left, Z up, hip axes at z = 0. Run leg.py --assembly first.
  python3 quad.py  ->  out/assembly/quad-<name>.glb
"""
import glob
import os

import numpy as np
import trimesh
from manifold3d import CrossSection, FillRule, Manifold

import leg as L

HERE = os.path.dirname(os.path.abspath(__file__))
A = os.path.join(HERE, 'out', 'assembly')
YOKE_SIDE = ('yoke', 'rs00-abad')          # the parts that flip fore-aft on a corner-pod front leg
BODY_Z = (-40.0, 70.0)


def wall(ys, t=2 * L.T):
    """YZ end/cross wall carrying ab/ad RS00s (back pattern + cable hole) at each y in ys."""
    half = max(abs(y) for y in ys) + 36
    b = L.rounded(-half, BODY_Z[0], half, BODY_Z[1], 8)
    for y in ys:
        b = b.difference(L.holes_on(L.BACK_PCD, L.BACK_N, L.M3, phase=45, at=(y, 0))).difference(L.circle(y, 0, 16))
    return L.tri(L.to_world(CrossSection(L.rings(b), FillRule.EvenOdd).extrude(t), 'yz'))


def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))


def corner():
    hip_x, plane = 260.0, 74.0
    ya = plane + L.ABAD_Y                                  # ab/ad axis y: 32
    wall_x = hip_x - L.X_BULK[0]                           # end wall inner... the motor back face: 170
    legs = [(t, x, s, x > 0) for t, x, s in (('fl', hip_x, 1), ('fr', hip_x, -1), ('rl', -hip_x, 1), ('rr', -hip_x, -1))]
    shell = box(-wall_x + 2 * L.T, wall_x - 2 * L.T, -137.6, 137.6, *BODY_Z)
    walls = [(wall([ya, -ya]), [wall_x - 2 * L.T, 0, 0]),                        # front wall, motors on its face
             (wall([ya, -ya]), [-wall_x, 0, 0])]
    return dict(hip_x=hip_x, plane=plane, legs=legs, body=L.tri(shell), walls=walls,
                note='body 327 x 275 x 110, overall ~600 x 275')


def side():
    hip_x, plane = 210.0, 102.0
    ya = plane + L.ABAD_Y                                  # 60
    half = ya + 32                                         # body half-width: holds the ab/ad motors
    legs = [(t, x, s, False) for t, x, s in (('fl', hip_x, 1), ('fr', hip_x, -1), ('rl', -hip_x, 1), ('rr', -hip_x, -1))]
    front = hip_x + L.X_BULK[1]
    shell = box(-hip_x - 60, front, -half, half, *BODY_Z)
    inner = plane - (L.Y_FLANGE + L.YOKE_T) - 3            # pocket for each hip cluster, open to the side
    for x in (hip_x, -hip_x):
        for s in (1, -1):
            y0, y1 = sorted((s * inner, s * (half + 1)))
            shell = shell - box(x - 42, x + L.X_YOKE + L.YOKE_T + 2, y0, y1, BODY_Z[0] - 1, BODY_Z[1] + 1)
    walls = [(wall([ya, -ya]), [x + L.X_BULK[0], 0, 0]) for x in (hip_x, -hip_x)]
    return dict(hip_x=hip_x, plane=plane, legs=legs, body=L.tri(shell), walls=walls,
                note=f'body {front + hip_x + 60:.0f} x {2 * half:.0f} x 110 with hip pockets, overall ~{2 * (plane + L.Y_FLANGE + L.YOKE_T):.0f} wide')


def build(arch):
    scene = trimesh.Scene()
    files = sorted(f for f in glob.glob(os.path.join(A, '*.stl'))
                   if not os.path.basename(f).startswith(('step', 'bulkhead')))
    geoms = {os.path.basename(f)[:-4]: trimesh.load(f) for f in files}
    for n, g in geoms.items():
        g.merge_vertices()
        scene.add_geometry(g, geom_name=n, node_name=f'{n}.proto', transform=np.diag([1e-6, 1e-6, 1e-6, 1.0]))
    for tag, x, s, flip in arch['legs']:
        place = trimesh.transformations.translation_matrix([x, s * arch['plane'], 0]) @ np.diag([1, s, 1, 1.0])
        for n in geoms:
            m = place @ np.diag([-1, 1, 1, 1.0]) if (flip and n in YOKE_SIDE) else place
            scene.graph.update(frame_to=f'{n}.{tag}', frame_from=scene.graph.base_frame, matrix=m, geometry=n)
    scene.add_geometry(arch['body'], geom_name='chassis-body', node_name='chassis-body')
    for i, (w, at) in enumerate(arch['walls']):
        scene.add_geometry(w, geom_name=f'chassis-wall-{i}', node_name=f'chassis-wall-{i}',
                           transform=trimesh.transformations.translation_matrix(at))
    return scene


def main():
    for name, fn in (('corner', corner), ('side', side)):
        arch = fn()
        out = os.path.join(A, f'quad-{name}.glb')
        build(arch).export(out)
        print(f'{name}: {arch["note"]} -> {out} ({os.path.getsize(out) / 1e6:.1f} MB)')


if __name__ == '__main__':
    main()
