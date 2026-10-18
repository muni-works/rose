"""Whole-quad architectures built from the one quad00 leg, for side-by-side comparison in the viewer.

  corner  -- Mini Cheetah layout. One body box as wide as the legs; the four leg clusters sit at its
             corners, just past each end. Every ab/ad RS00 points back into the body and bolts to an
             end wall, so the front legs carry their yoke + ab/ad mirrored fore-aft.
  side    -- Go2 layout, the one being developed: body.py. Legs hang along the body's sides, all
             facing forward; two flat decks + cross ribs, necked where the hip clusters tuck in.

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
BODY_Z = (-36.0, 46.0)                     # low: just clears the ab/ad motors (r 28.5) with a 3 mm skin


def wall(ys, t=2 * L.T):
    """YZ end/cross wall carrying ab/ad RS00s (back pattern + cable hole) at each y in ys."""
    half = max(abs(y) for y in ys) + 30            # just past the motor body (r 28.5)
    b = L.rounded(-half, BODY_Z[0], half, BODY_Z[1], 8)
    for y in ys:
        b = b.difference(L.holes_on(L.BACK_PCD, L.BACK_N, L.M3, phase=45, at=(y, 0))).difference(L.circle(y, 0, 16))
    return L.tri(L.to_world(CrossSection(L.rings(b), FillRule.EvenOdd).extrude(t), 'yz'))


def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))


def softbox(x0, x1, y0, y1, z0, z1, r_plan=18.0, r_side=14.0):
    """a box rounded in plan (corners) and in elevation (top and bottom edges): plan profile
    extruded in z, intersected with the side profile extruded in y."""
    plan = CrossSection(L.rings(L.rounded(x0, y0, x1, y1, r_plan)), FillRule.EvenOdd).extrude(z1 - z0).translate((0, 0, z0))
    side = L.to_world(CrossSection(L.rings(L.rounded(x0 - 1, z0, x1 + 1, z1, r_side)), FillRule.EvenOdd)
                      .extrude(y1 - y0 + 2).translate((0, 0, y0 - 1)), 'xz')
    return plan ^ side


def corner():
    """a low slab as wide as the legs; it runs out to the ab/ad output faces, so each leg plugs
    straight into a corner of it."""
    hip_x, plane = 260.0, 74.0
    ya = plane + L.ABAD_Y                                  # ab/ad axis y: 32
    wall_x = hip_x - L.X_BULK[0]                           # ab/ad back faces: 169.8
    end = hip_x - L.X_ABAD[0] - 1.0                        # slab ends 1 mm short of the ab/ad output face
    half = plane + L.Y_FLANGE + L.YOKE_T                   # as wide as the outer yoke flanges
    legs = [(t, x, s, x > 0) for t, x, s in (('fl', hip_x, 1), ('fr', hip_x, -1), ('rl', -hip_x, 1), ('rr', -hip_x, -1))]
    shell = softbox(-end, end, -half, half, *BODY_Z, r_plan=24, r_side=16)
    walls = [(wall([ya, -ya]), [wall_x - 2 * L.T, 0, 0]), (wall([ya, -ya]), [-wall_x, 0, 0])]
    return dict(hip_x=hip_x, plane=plane, legs=legs, body=L.tri(shell), walls=walls,
                note=f'slab {2 * end:.0f} x {2 * half:.0f} x {BODY_Z[1] - BODY_Z[0]:.0f}, overall ~{2 * (hip_x + 40):.0f} x {2 * half:.0f}')


def side():
    """the real side-hung body (body.py): two flat decks + cross ribs, open sides, internals."""
    import body as B
    legs = [(t, x, s, False) for t, x, s in (('fl', B.HIP_X, 1), ('fr', B.HIP_X, -1), ('rl', -B.HIP_X, 1), ('rr', -B.HIP_X, -1))]
    extra = [(f'chassis-{n}', L.tri(m)) for n, m, _, _ in B.plates()]
    extra += [(f'int-{n}', L.tri(m)) for n, m, _ in B.internals()]
    return dict(hip_x=B.HIP_X, plane=B.PLANE, legs=legs, body=None, walls=[], extra=extra,
                note=f'decks + ribs, {B.X_NOSE + 2 * B.T - B.X_TAIL:.0f} long, {2 * B.WIDE:.0f} wide at chest and belly')


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
    if arch.get('body') is not None:
        scene.add_geometry(arch['body'], geom_name='chassis-body', node_name='chassis-body')
    for n, m in arch.get('extra', []):
        scene.add_geometry(m, geom_name=n, node_name=n)
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
