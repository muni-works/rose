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
import math
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


# --- zero gears: a motor at every joint. Hip RS00 coaxial and inboard, bolted to the thigh; knee RS00
#     at the knee, outboard, bolted to the shin. Links 130 / 130. Same L-yoke idea as the inboard hip.
DG_L = 130.0
DG_SHIN_T = 2 * L.T                                     # shin: 2 layers, centred on the leg plane
DG_PLATE = (DG_SHIN_T / 2 + 0.4, DG_SHIN_T / 2 + 0.4 + L.T)   # thigh fork plates at +/- this
DG_HIP = (-(DG_PLATE[1] + 1.0 + L.RS00_L), -(DG_PLATE[1] + 1.0))
DG_KNEE = (DG_PLATE[1] + 1.0, DG_PLATE[1] + 1.0 + L.RS00_L)
DG_LFL = (DG_HIP[0] - L.T, DG_HIP[0])


def direct_parts(th_t, rel):
    xz = lambda prof, y0, t: L.tri(L.to_world(CrossSection(L.rings(prof), FillRule.EvenOdd).extrude(t).translate((0, 0, y0)), 'xz'))
    ry = lambda d: trimesh.transformations.rotation_matrix(np.radians(d), [0, 1, 0])
    cyl = lambda r, y0, y1, x=0, z=0: trimesh.creation.cylinder(radius=r, segment=[[x, y0, z], [x, y1, z]], sections=64)
    thigh = L.stadium([(0, 0, 44), (0, -DG_L, 44)]).difference(L.holes_on(L.OUT_PCD, L.OUT_N, L.M3)) \
        .difference(L.circle(0, -DG_L, 38)).difference(L.rounded(-8, -DG_L + 40, 8, -40, 8))
    shin = L.stadium([(0, 0, 36), (0, -DG_L, 22)]).difference(L.holes_on(L.OUT_PCD, L.OUT_N, L.M3)) \
        .difference(L.rounded(-5, -DG_L + 30, 5, -30, 5))
    knee_T = trimesh.transformations.translation_matrix([0, 0, -DG_L])
    th = ry(th_t)
    fl = L.unary_union([L.circle(0, 0, L.RS00_D + 14), L.box(0, -22, L.X_YOKE + L.YOKE_T, 30)]).convex_hull
    fl = fl.difference(L.holes_on(L.BACK_PCD, L.BACK_N, L.M3, phase=45)).difference(L.circle(0, 0, 16))
    base = L.box(DG_LFL[0], -22, DG_HIP[1] - 1, 30).difference(L.holes_on(L.OUT_PCD, L.OUT_N, L.M3, phase=90, at=(L.ABAD_Y, 0)))
    base_m = L.tri(L.to_world(CrossSection(L.rings(base), FillRule.EvenOdd).extrude(L.YOKE_T).translate((0, 0, L.X_YOKE)), 'yz'))
    paw = trimesh.creation.icosphere(subdivisions=3, radius=L.PAW_D / 2).apply_translation([0, 0, -DG_L])
    shin_m = trimesh.util.concatenate([xz(shin, -DG_SHIN_T / 2, DG_SHIN_T)])
    return {
        'thigh-outer': xz(thigh, DG_PLATE[0], L.T).apply_transform(th),
        'thigh-inner': xz(thigh, -DG_PLATE[1], L.T).apply_transform(th),
        'shin': shin_m.apply_transform(ry(-rel)).apply_transform(knee_T).apply_transform(th),
        'paw': paw.apply_transform(ry(-rel)).apply_transform(knee_T).apply_transform(th),
        'rs00-hip': cyl(L.RS00_D / 2, *DG_HIP),
        'rs00-knee': cyl(L.RS00_D / 2, *DG_KNEE).apply_transform(knee_T).apply_transform(th),
        'yoke': trimesh.util.concatenate([xz(fl, DG_LFL[0], L.YOKE_T), base_m]),
        'rs00-abad': cyl(L.RS00_D / 2, L.ABAD_Y, L.ABAD_Y, 0, 0).apply_transform(np.eye(4)) if False else
                     trimesh.creation.cylinder(radius=L.RS00_D / 2, segment=[[L.X_ABAD[0], L.ABAD_Y, 0], [L.X_ABAD[1], L.ABAD_Y, 0]], sections=64),
    }


def direct():
    import importlib
    import body as B
    importlib.reload(B)
    plane, hip_x = 80.0, 190.0
    B.configure(plane, flange_outer=-DG_LFL[0], hip_x=hip_x, ends=False)
    B.CLIP = plane - DG_PLATE[1]                         # no E-clips: the thigh's inner face
    B.WIDE = B.CLIP - 4.0
    wall = hip_x - L.X_BULK[0]
    B.RIBS = {'nose': (wall - 2 * L.T, 2 * L.T, B.WIDE), 'mid': (-wall, 2 * L.T, B.WIDE)}
    B.INTERNALS = [('battery', (140, 126, 44), (20, 0, -18), 1.70), ('compute', (95, 63, 28), (-40, 0, 22), 0.12),
                   ('power', (60, 45, 22), (-60, 0, -26), 0.10)]
    hh = 255.0 * DG_L / 150.0
    th_t = np.degrees(np.arccos((hh / 2) / DG_L))
    rel = 2 * th_t                                       # shin back under the hip
    legs = [(t, x, s, x > 0) for t, x, s in (('fl', hip_x, 1), ('fr', hip_x, -1), ('rl', -hip_x, 1), ('rr', -hip_x, -1))]
    extra = [(f'chassis-{n}', L.tri(m)) for n, m, _, _ in B.plates()]
    extra += [(f'int-{n}', L.tri(m)) for n, m, _ in B.internals()]
    note = (f'zero gears: {2 * (plane + DG_KNEE[1]):.0f} wide at the knees, {2 * (plane + DG_PLATE[1]):.0f} at the hips; '
            f'body {2 * B.X_END:.0f} x {2 * B.WIDE:.0f}; hips {hh:.0f} high')
    B.configure(87.0)
    return dict(hip_x=hip_x, plane=plane, legs=legs, body=None, walls=[], extra=extra, note=note,
                replace=direct_parts(th_t, rel), only=True)


# --- hybrid: zero gears, both motors inside the thigh. The thigh is a box -- two plates with the hip
#     and knee RS00 as its spacers. Leg frame y = 0 at the inner thigh plate's inner face, outboard +.
HY_L = 130.0
HY_IN = (0.0, L.T)                                       # inner thigh plate
HY_MOT = (HY_IN[1] + 1.0, HY_IN[1] + 1.0 + L.RS00_L)     # both motors, back faces inboard
HY_OUT = (HY_MOT[1] + 1.0, HY_MOT[1] + 1.0 + L.T)        # outer thigh plate (on the hip motor's output)
HY_SHIN = (HY_OUT[1] + 1.0, HY_OUT[1] + 1.0 + 2 * L.T)   # shin, outside the box, on the knee output
HY_FL = (-L.T - 1.0, -1.0)                               # L-yoke flange, just inboard of the thigh
HY_ABAD = -35.0                                          # ab/ad axis: its body clears the thigh by 6.5


def hybrid_parts(th_t, rel):
    xz = lambda prof, y0, t: L.tri(L.to_world(CrossSection(L.rings(prof), FillRule.EvenOdd).extrude(t).translate((0, 0, y0)), 'xz'))
    ry = lambda d: trimesh.transformations.rotation_matrix(np.radians(d), [0, 1, 0])
    cyl = lambda r, y0, y1, x=0, z=0: trimesh.creation.cylinder(radius=r, segment=[[x, y0, z], [x, y1, z]], sections=64)
    box_thigh = L.stadium([(0, 0, 64), (0, -HY_L, 64)])
    inner = box_thigh.difference(L.circle(0, 0, 50)).difference(L.holes_on(L.BACK_PCD, L.BACK_N, L.M3, phase=45, at=(0, -HY_L)))
    inner = inner.difference(L.rounded(-12, -HY_L + 45, 12, -45, 10))
    outer = box_thigh.difference(L.holes_on(L.OUT_PCD, L.OUT_N, L.M3)).difference(L.circle(0, -HY_L, 40))
    outer = outer.difference(L.rounded(-12, -HY_L + 45, 12, -45, 10))
    shin = L.stadium([(0, 0, 40), (0, -HY_L, 22)]).difference(L.holes_on(L.OUT_PCD, L.OUT_N, L.M3)) \
        .difference(L.rounded(-5, -HY_L + 32, 5, -32, 5))
    knee = trimesh.transformations.translation_matrix([0, 0, -HY_L])
    th = ry(th_t)
    fl = L.unary_union([L.circle(0, 0, 56), L.box(0, -22, L.X_YOKE + L.YOKE_T, 28)]).convex_hull
    boss = L.circle(0, 0, 46).difference(L.holes_on(L.BACK_PCD, L.BACK_N, L.M3, phase=45))
    base = L.box(HY_ABAD - 22, -22, HY_FL[0], 28).difference(L.holes_on(L.OUT_PCD, L.OUT_N, L.M3, phase=90, at=(HY_ABAD, 0)))
    base_m = L.tri(L.to_world(CrossSection(L.rings(base), FillRule.EvenOdd).extrude(L.YOKE_T).translate((0, 0, L.X_YOKE)), 'yz'))
    paw = trimesh.creation.icosphere(subdivisions=3, radius=L.PAW_D / 2).apply_translation([0, (HY_SHIN[0] + HY_SHIN[1]) / 2, -HY_L])
    return {
        'thigh-inner': xz(inner, HY_IN[0], L.T).apply_transform(th),
        'thigh-outer': xz(outer, HY_OUT[0], L.T).apply_transform(th),
        'shin': xz(shin, HY_SHIN[0], 2 * L.T).apply_transform(ry(-rel)).apply_transform(knee).apply_transform(th),
        'paw': paw.apply_transform(ry(-rel)).apply_transform(knee).apply_transform(th),
        'rs00-hip': cyl(L.RS00_D / 2, *HY_MOT),
        'rs00-knee': cyl(L.RS00_D / 2, *HY_MOT).apply_transform(knee).apply_transform(th),
        'yoke': trimesh.util.concatenate([xz(fl, HY_FL[0], L.T), xz(boss, HY_FL[1], HY_MOT[0] - HY_FL[1]), base_m]),
        'rs00-abad': trimesh.creation.cylinder(radius=L.RS00_D / 2, segment=[[L.X_ABAD[0], HY_ABAD, 0], [L.X_ABAD[1], HY_ABAD, 0]], sections=64),
    }


def hybrid():
    import importlib
    import body as B
    importlib.reload(B)
    y0, hip_x = 70.0, 190.0                              # inner thigh plate at |y| = 70
    B.configure(y0, hip_x=hip_x, ends=False)
    B.PLANE, B.YA, B.CLIP = y0, y0 + HY_ABAD, y0
    B.WIDE = y0 - 4.0
    wall = hip_x - L.X_BULK[0]
    B.RIBS = {'nose': (wall - 2 * L.T, 2 * L.T, B.WIDE), 'mid': (-wall, 2 * L.T, B.WIDE)}
    B.INTERNALS = [('battery', (140, 120, 44), (20, 0, -18), 1.70), ('compute', (95, 63, 28), (-40, 0, 22), 0.12),
                   ('power', (60, 45, 22), (-60, 0, -26), 0.10)]
    hh = 255.0 * HY_L / 150.0
    th_t = np.degrees(np.arccos((hh / 2) / HY_L))
    legs = [(t, x, s, x > 0) for t, x, s in (('fl', hip_x, 1), ('fr', hip_x, -1), ('rl', -hip_x, 1), ('rr', -hip_x, -1))]
    extra = [(f'chassis-{n}', L.tri(m)) for n, m, _, _ in B.plates()]
    extra += [(f'int-{n}', L.tri(m)) for n, m, _ in B.internals()]
    pair_gap = 2 * (B.YA - L.RS00_D / 2)
    note = (f'hybrid: {2 * (y0 + HY_SHIN[1]):.0f} wide at the feet, {2 * (y0 + HY_OUT[1]):.0f} at the hips, nothing proud of '
            f'the leg; body {2 * B.X_END:.0f} x {2 * B.WIDE:.0f}; ab/ad pair gap {pair_gap:.0f}, thigh clears ab/ad by {-(HY_ABAD + L.RS00_D / 2):.1f}')
    B.configure(87.0)
    return dict(hip_x=hip_x, plane=y0, legs=legs, body=None, walls=[], extra=extra, note=note,
                replace=hybrid_parts(th_t, 2 * th_t), only=True)


# --- inline: the hybrid with the knee motor flipped (back on the outer plate, output through the inner
#     plate) so the shin runs down the INSIDE of the thigh box, nearly under the ab/ad axis. Knees point
#     inward ("><": front back, rear forward) so a folding foot rises on the far side of the hip from the
#     ab/ad and yoke base.
IL_SHIN = (-(1.0 + L.T + 1.0 + 2 * L.T + 1.0), -(1.0 + L.T + 2.0))   # inboard of the yoke flange layer
IL_ABAD = IL_SHIN[0] - 3.0 - L.RS00_D / 2                            # ab/ad body clears the shin layer by 3


def inline_parts(th_t, rel):
    xz = lambda prof, y0, t: L.tri(L.to_world(CrossSection(L.rings(prof), FillRule.EvenOdd).extrude(t).translate((0, 0, y0)), 'xz'))
    ry = lambda d: trimesh.transformations.rotation_matrix(np.radians(d), [0, 1, 0])
    cyl = lambda r, y0, y1, x=0, z=0: trimesh.creation.cylinder(radius=r, segment=[[x, y0, z], [x, y1, z]], sections=64)
    box_thigh = L.stadium([(0, 0, 64), (0, -HY_L, 64)])
    inner = box_thigh.difference(L.circle(0, 0, 50)).difference(L.circle(0, -HY_L, 40))
    inner = inner.difference(L.rounded(-12, -HY_L + 45, 12, -45, 10))
    outer = box_thigh.difference(L.holes_on(L.OUT_PCD, L.OUT_N, L.M3)).difference(L.holes_on(L.BACK_PCD, L.BACK_N, L.M3, phase=45, at=(0, -HY_L)))
    outer = outer.difference(L.rounded(-12, -HY_L + 45, 12, -45, 10))
    shin = L.stadium([(0, 0, 40), (0, -HY_L, 22)]).difference(L.holes_on(L.OUT_PCD, L.OUT_N, L.M3)) \
        .difference(L.rounded(-5, -HY_L + 32, 5, -32, 5))
    knee = trimesh.transformations.translation_matrix([0, 0, -HY_L])
    th = ry(th_t)
    fl = L.unary_union([L.circle(0, 0, 56), L.box(0, -22, L.X_YOKE + L.YOKE_T, 28)]).convex_hull
    boss = L.circle(0, 0, 46).difference(L.holes_on(L.BACK_PCD, L.BACK_N, L.M3, phase=45))
    base = L.box(IL_ABAD - 22, -22, HY_FL[0], 28).difference(L.holes_on(L.OUT_PCD, L.OUT_N, L.M3, phase=90, at=(IL_ABAD, 0)))
    base_m = L.tri(L.to_world(CrossSection(L.rings(base), FillRule.EvenOdd).extrude(L.YOKE_T).translate((0, 0, L.X_YOKE)), 'yz'))
    hub = L.circle(0, 0, 36)
    paw = trimesh.creation.icosphere(subdivisions=3, radius=L.PAW_D / 2).apply_translation([0, sum(IL_SHIN) / 2, -HY_L])
    return {
        'thigh-inner': xz(inner, HY_IN[0], L.T).apply_transform(th),
        'thigh-outer': xz(outer, HY_OUT[0], L.T).apply_transform(th),
        'shin': trimesh.util.concatenate([xz(shin, IL_SHIN[0], 2 * L.T), xz(hub, IL_SHIN[1], HY_MOT[0] - IL_SHIN[1])])
                .apply_transform(ry(-rel)).apply_transform(knee).apply_transform(th),
        'paw': paw.apply_transform(ry(-rel)).apply_transform(knee).apply_transform(th),
        'rs00-hip': cyl(L.RS00_D / 2, *HY_MOT),
        'rs00-knee': cyl(L.RS00_D / 2, *HY_MOT).apply_transform(knee).apply_transform(th),
        'yoke': trimesh.util.concatenate([xz(fl, HY_FL[0], L.T), xz(boss, HY_FL[1], HY_MOT[0] - HY_FL[1]), base_m]),
        'rs00-abad': trimesh.creation.cylinder(radius=L.RS00_D / 2, segment=[[L.X_ABAD[0], IL_ABAD, 0], [L.X_ABAD[1], IL_ABAD, 0]], sections=64),
    }


def inline():
    import importlib
    import body as B
    importlib.reload(B)
    hip_x = 190.0
    y0 = L.RS00_D / 2 + 3.0 - IL_ABAD                   # ab/ad pair 6 mm apart at the centre line
    ya = y0 + IL_ABAD
    B.configure(80.0, hip_x=hip_x, ends=False)
    B.PLANE, B.YA = y0, ya
    B.WIDE = y0 + IL_SHIN[0] - 4.0                       # 4 mm inside the shin layer
    wall = hip_x - L.X_BULK[0]
    B.RIBS = {'nose': (wall - 2 * L.T, 2 * L.T, B.WIDE), 'mid': (-wall, 2 * L.T, B.WIDE)}
    B.INTERNALS = [('battery', (140, 105, 44), (20, 0, -18), 1.70), ('compute', (95, 63, 28), (-40, 0, 22), 0.12),
                   ('power', (60, 45, 22), (-60, 0, -26), 0.10)]
    hh = 255.0 * HY_L / 150.0
    th_t = np.degrees(np.arccos((hh / 2) / HY_L))
    # front: yoke + ab/ad mirrored fore-aft (ab/ad behind the hip, into the body), knee back as drawn.
    # rear: yoke as drawn (ab/ad ahead, into the body), the rest of the leg mirrored -> knee forward.
    legs = [(t, x, s, 'yoke' if x > 0 else 'leg') for t, x, s in
            (('fl', hip_x, 1), ('fr', hip_x, -1), ('rl', -hip_x, 1), ('rr', -hip_x, -1))]
    extra = [(f'chassis-{n}', L.tri(m)) for n, m, _, _ in B.plates()]
    extra += [(f'int-{n}', L.tri(m)) for n, m, _ in B.internals()]
    foot = y0 + sum(IL_SHIN) / 2
    lever = foot - ya
    note = (f'inline: {2 * (y0 + HY_OUT[1]):.0f} wide overall, feet at +/-{foot:.0f}, ab/ad lever {lever:.0f} mm '
            f'(hybrid 99); body {2 * B.X_END:.0f} x {2 * B.WIDE:.0f}; knees point inward')
    B.configure(87.0)
    return dict(hip_x=hip_x, plane=y0, legs=legs, body=None, walls=[], extra=extra, note=note,
                replace=inline_parts(th_t, 2 * th_t), only=True)


def dog():
    """the inline hybrid in dog proportions: a deep chest (depth / withers ~0.47) with the battery in a
    tray under the ab/ad motors, compute on the withers, hips pulled in as far as the inward knees allow:
    same-side thighs must clear through a +/-30 deg stride about the stance angle."""
    import importlib
    import body as B
    importlib.reload(B)
    hh = 255.0 * HY_L / 150.0
    th_t = np.degrees(np.arccos((hh / 2) / HY_L))
    stride = 30.0
    hip_x = math.ceil(HY_L * math.sin(math.radians(th_t + stride)) + 32 + 3)   # thigh box r32, 3 mm between
    y0 = L.RS00_D / 2 + 3.0 - IL_ABAD
    B.configure(80.0, hip_x=hip_x, ends=False)
    B.PLANE, B.YA = y0, y0 + IL_ABAD
    B.WIDE = y0 + IL_SHIN[0] - 4.0
    B.H_TOP, B.H_BOT = 50.0 - L.T, 77.0 - L.T            # deck outer faces at +50 / -77
    wall = hip_x - L.X_BULK[0]
    B.RIBS = {'nose': (wall - 2 * L.T, 2 * L.T, B.WIDE), 'mid': (-wall, 2 * L.T, B.WIDE)}
    B.INTERNALS = [('battery', (126, 70, 42), (0, 20.5, -51), 0.85),     # 12S1P 21700 (44.4 V, 5 Ah) under the ab/ad
                   ('power', (60, 40, 22), (0, -35.5, -61), 0.10)]
    legs = [(t, x, s, 'yoke' if x > 0 else 'leg') for t, x, s in
            (('fl', hip_x, 1), ('fr', hip_x, -1), ('rl', -hip_x, 1), ('rr', -hip_x, -1))]
    extra = [(f'chassis-{n}', L.tri(m)) for n, m, _, _ in B.plates()]
    extra += [(f'int-{n}', L.tri(m)) for n, m, _ in B.internals()]
    withers_pack = Manifold.cube((95, 63, 28)).translate((-47.5, -31.5, 50.0))   # Pi 5 + CAN hat, on the withers
    extra.append(('int-compute', L.tri(withers_pack)))
    ok = B.check()
    withers = hh + 50 + 28
    length = 2 * hip_x + 2 * 35
    note = (f'dog: withers {withers:.0f}, length/withers {length / (hh + 50):.2f}, chest/withers {127 / (hh + 50):.2f}; '
            f'hips {2 * hip_x:.0f} apart (thighs clear a +/-{stride:.0f} deg stride), body {2 * B.X_END:.0f} long; '
            f'checks {"pass" if ok else "FAIL"}')
    B.configure(87.0)
    B.H_TOP = B.H_BOT = B.H_IN
    return dict(hip_x=hip_x, plane=y0, legs=legs, body=None, walls=[], extra=extra, note=note,
                replace=inline_parts(th_t, 2 * th_t), only=True)


def humanoid():
    """the hybrid limb module on a biped, mounted exactly as on the quad: every limb hangs from a
    fore-aft ab/ad RS00 whose back face bolts to the torso's back plate. Legs: whole module mirrored
    fore-aft (knee bends forward). Arms: only the yoke flipped (elbow points back)."""
    y0 = 70.0                                             # inner thigh plate at |y| = 70, as on the quad
    ya = y0 + HY_ABAD                                     # ab/ad axes at +/-35
    torso_h = 200.0                                       # hip axis -> shoulder axis
    leg_bend, arm_up, arm_bend = 12.0, 10.0, 80.0         # pose (deg)
    x_back = -L.X_BULK[0]                                 # ab/ad back faces, behind the limb axes
    hip_z = 2 * HY_L * np.cos(np.radians(leg_bend)) + L.PAW_D / 2
    legs = [('ll', 0.0, 1, 'all', hip_z), ('lr', 0.0, -1, 'all', hip_z),
            ('al', 0.0, 1, 'yoke', hip_z + torso_h), ('ar', 0.0, -1, 'yoke', hip_z + torso_h)]
    poses = {'all': hybrid_parts(leg_bend, 2 * leg_bend), 'yoke': hybrid_parts(arm_up, arm_bend)}
    ext = lambda prof, plane, w0, t: L.tri(L.to_world(CrossSection(L.rings(prof), FillRule.EvenOdd).extrude(t).translate((0, 0, w0)), plane))
    half = ya + L.RS00_D / 2 + 4                          # back plate holds both ab/ad of a pair
    back = L.rounded(-half, hip_z - 40, half, hip_z + torso_h + 40, 10)
    for z in (hip_z, hip_z + torso_h):
        for s in (1, -1):
            back = back.difference(L.holes_on(L.BACK_PCD, L.BACK_N, L.M3, phase=45, at=(s * ya, z))).difference(L.circle(s * ya, z, 16))
    wide = y0 - 8.0                                        # torso sides 8 mm inside the limbs
    x_front = 40.0
    side = L.rounded(x_back, hip_z - 30, x_front, hip_z + torso_h + 30, 10)
    cap = L.rounded(x_back, -wide, x_front, wide, 10)
    extra = [('chassis-back', ext(back, 'yz', x_back - 2 * L.T, 2 * L.T)),
             ('chassis-front', ext(L.rounded(-wide, hip_z + 50, wide, hip_z + torso_h + 30, 10), 'yz', x_front, L.T)),
             ('chassis-side-l', ext(side, 'xz', wide - L.T, L.T)), ('chassis-side-r', ext(side, 'xz', -wide, L.T)),
             ('chassis-cap', ext(cap, 'xy', hip_z + torso_h + 30, L.T))]
    extra.append(('int-battery', L.tri(Manifold.cube((70, 2 * wide - 20, 126)).translate((x_back + 6, -wide + 10, hip_z + 50)))))
    extra.append(('int-compute', L.tri(Manifold.cube((28, 63, 95)).translate((x_back + 84, -31.5, hip_z + 70)))))
    head = Manifold.cube((70, 80, 60)).translate((-35 - 20, -40, hip_z + torso_h + 30 + L.T + 25))
    neck = Manifold.cylinder(25, 14, 14, 32).translate((-20, 0, hip_z + torso_h + 30 + L.T))
    extra.append(('head', L.tri(head + neck)))
    note = (f'humanoid: hybrid limbs x4, hips {hip_z:.0f} high, overall ~{hip_z + torso_h + 30 + 90:.0f} tall, '
            f'{2 * (y0 + HY_SHIN[1]):.0f} wide; legs and arms hang from fore-aft ab/ad on one back plate')
    return dict(hip_x=0.0, plane=y0, legs=legs, body=None, walls=[], extra=extra, note=note, posed=poses, only=True)


def build(arch):
    scene = trimesh.Scene()
    files = sorted(f for f in glob.glob(os.path.join(A, '*.stl'))
                   if not os.path.basename(f).startswith(('step', 'bulkhead')))
    geoms = {os.path.basename(f)[:-4]: trimesh.load(f) for f in files}
    if arch.get('only'):
        geoms = {}
    for n in arch.get('drop', ()):
        geoms.pop(n, None)
    geoms.update(arch.get('replace', {}))
    for n, g in geoms.items():
        g.merge_vertices()
        scene.add_geometry(g, geom_name=n, node_name=f'{n}.proto', transform=np.diag([1e-6, 1e-6, 1e-6, 1.0]))
    for n_pose, gset in arch.get('posed', {}).items():
        for n, g in gset.items():
            scene.add_geometry(g, geom_name=f'{n}~{n_pose}', node_name=f'{n}~{n_pose}.proto',
                               transform=np.diag([1e-6, 1e-6, 1e-6, 1.0]))
    for leg in arch['legs']:
        tag, x, s, flip = leg[:4]
        z = leg[4] if len(leg) > 4 else 0.0
        place = trimesh.transformations.translation_matrix([x, s * arch['plane'], z]) @ np.diag([1, s, 1, 1.0])
        if 'posed' in arch:
            for n in arch['posed'][flip]:
                mirrored = flip == 'all' or (flip == 'yoke' and n in YOKE_SIDE)
                m = place @ np.diag([-1, 1, 1, 1.0]) if mirrored else place
                scene.graph.update(frame_to=f'{n}.{tag}', frame_from=scene.graph.base_frame, matrix=m, geometry=f'{n}~{flip}')
            continue
        for n in geoms:
            if flip == 'leg':
                mirrored = n not in YOKE_SIDE
            else:
                mirrored = bool(flip) and n in YOKE_SIDE
            m = place @ np.diag([-1, 1, 1, 1.0]) if mirrored else place
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
    for name, fn in (('corner', corner), ('side', side), ('inboard', inboard), ('direct', direct), ('hybrid', hybrid), ('inline', inline), ('dog', dog), ('humanoid', humanoid)):
        arch = fn()
        out = os.path.join(A, f'quad-{name}.glb')
        build(arch).export(out)
        print(f'{name}: {arch["note"]} -> {out} ({os.path.getsize(out) / 1e6:.1f} MB)')


if __name__ == '__main__':
    main()
