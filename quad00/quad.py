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


# --- inboard hip: both pitch motors inboard of the thigh. Hip motor above the hip drives a gear on the
#     thigh; nothing crosses the leg plane, so the thigh turns all the way round.
HIP_RATIO = (16, 32)                                    # hip pinion, thigh gear: 2:1, m2.5, centres 60
HIP_C = L.M * sum(HIP_RATIO) / 2
Y_HG = (-(L.Y_PLATE[1] + 0.4 + 3 * L.T), -(L.Y_PLATE[1] + 0.4))   # thigh gear layer
Y_MOT = (Y_HG[0] - 1.0 - L.RS00_L, Y_HG[0] - 1.0)       # both pitch motors
Y_LFL = (Y_MOT[0] - L.T, Y_MOT[0])                      # L-yoke flange


def inboard_parts(th_t):
    """leg-frame meshes for the inboard-hip leg's new parts. th_t: thigh pose angle (deg)."""
    xz = lambda prof, y0, t: L.tri(L.to_world(CrossSection(L.rings(prof), FillRule.EvenOdd).extrude(t).translate((0, 0, y0)), 'xz'))
    zp, zh = HIP_RATIO
    gear = L.gear(zh).difference(L.circle(0, 0, 52)).difference(L.holes_on(62, 6, L.M3))   # 6808 bearing bore
    pin = L.gear(zp, at=(0, HIP_C)).difference(L.holes_on(L.OUT_PCD, L.OUT_N, L.M3, at=(0, HIP_C)))
    fl = L.unary_union([L.circle(0, 0, L.RS00_D + 14), L.circle(0, HIP_C, L.RS00_D + 14),
                        L.box(0, -22, L.X_YOKE + L.YOKE_T, 40)]).convex_hull
    for c in ((0, 0), (0, HIP_C)):
        fl = fl.difference(L.holes_on(L.BACK_PCD, L.BACK_N, L.M3, phase=45, at=c)).difference(L.circle(*c, 16))
    base = L.box(Y_LFL[0], -22, -(L.Y_PLATE[1] + 11), 40).difference(
        L.holes_on(L.OUT_PCD, L.OUT_N, L.M3, phase=90, at=(L.ABAD_Y, 0)))
    base_m = L.tri(L.to_world(CrossSection(L.rings(base), FillRule.EvenOdd).extrude(L.YOKE_T).translate((0, 0, L.X_YOKE)), 'yz'))
    ry = trimesh.transformations.rotation_matrix(np.radians(th_t), [0, 1, 0])
    cyl = lambda r, y0, y1, x=0, z=0: trimesh.creation.cylinder(radius=r, segment=[[x, y0, z], [x, y1, z]], sections=64)
    return {
        'pinion-thighgear': xz(gear, Y_HG[0], 3 * L.T).apply_transform(ry),
        'pinion-hip': xz(pin, Y_HG[0], 3 * L.T),
        'rs00-hip': cyl(L.RS00_D / 2, *Y_MOT, z=HIP_C),
        'rs00-knee': cyl(L.RS00_D / 2, *Y_MOT),
        'yoke': trimesh.util.concatenate([xz(fl, Y_LFL[0], L.YOKE_T), base_m]),
    }


def inboard():
    import importlib
    import body as B
    importlib.reload(B)
    plane = 102.0
    B.configure(plane, flange_outer=-Y_LFL[0], ends=False)          # belly only: nothing beside a hip cluster
    legs = [(t, x, s, x > 0) for t, x, s in (('fl', B.HIP_X, 1), ('fr', B.HIP_X, -1), ('rl', -B.HIP_X, 1), ('rr', -B.HIP_X, -1))]
    l1, hh = -L.Z_KNEEAX, 255.0
    th_t = np.degrees(np.arccos((l1 ** 2 + hh ** 2 - L.SHIN ** 2) / (2 * l1 * hh)))
    extra = [(f'chassis-{n}', L.tri(m)) for n, m, _, _ in B.plates()]
    extra += [(f'int-{n}', L.tri(m)) for n, m, _ in B.internals()]
    gap = (B.HIP_X - L.M * (HIP_RATIO[1] + 2) / 2) - B.X_END              # thigh-gear tip to belly end, in x
    note = (f'inboard hip, belly only: {2 * (plane + L.Y_PLATE[1] + 1.5):.0f} wide overall, body {2 * B.X_END:.0f} x {2 * B.WIDE:.0f}; '
            f'hip {HIP_RATIO[1] // HIP_RATIO[0]}:1, full circle; clusters clear the body by {gap:.1f} mm in x at any abduction')
    B.configure(87.0)
    return dict(hip_x=210.0, plane=plane, legs=legs, body=None, walls=[], extra=extra, note=note,
                replace=inboard_parts(th_t), drop=('bulkhead',))


def build(arch):
    scene = trimesh.Scene()
    files = sorted(f for f in glob.glob(os.path.join(A, '*.stl'))
                   if not os.path.basename(f).startswith(('step', 'bulkhead')))
    geoms = {os.path.basename(f)[:-4]: trimesh.load(f) for f in files}
    for n in arch.get('drop', ()):
        geoms.pop(n, None)
    geoms.update(arch.get('replace', {}))
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
    for name, fn in (('corner', corner), ('side', side), ('inboard', inboard)):
        arch = fn()
        out = os.path.join(A, f'quad-{name}.glb')
        build(arch).export(out)
        print(f'{name}: {arch["note"]} -> {out} ({os.path.getsize(out) / 1e6:.1f} MB)')


if __name__ == '__main__':
    main()
