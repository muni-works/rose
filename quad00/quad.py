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
YOKE_SIDE = ('yoke', 'rs00-abad', 'rs02-abad', 'rs04-abad', 'rs06-abad')          # the parts that flip fore-aft on a corner-pod front leg
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
HY_L = 130.0                                            # thigh
HY_SHIN_L = 170.0                                       # shin: longer than the thigh (knee lever 78 mm at hips 255)


def stance(hh, L1=HY_L, L2=None):
    """thigh angle and shin-relative angle (deg, this file's R_y convention) with the foot under the hip."""
    L2 = HY_SHIN_L if L2 is None else L2
    a1 = math.acos((L1 ** 2 + hh ** 2 - L2 ** 2) / (2 * L1 * hh))
    a2 = math.asin(L1 * math.sin(a1) / L2)
    return math.degrees(a1), math.degrees(a1 + a2)
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
    shin = L.stadium([(0, 0, 40), (0, -HY_SHIN_L, 22)]).difference(L.holes_on(L.OUT_PCD, L.OUT_N, L.M3)) \
        .difference(L.rounded(-5, -HY_SHIN_L + 32, 5, -32, 5))
    knee = trimesh.transformations.translation_matrix([0, 0, -HY_L])
    th = ry(th_t)
    fl = L.unary_union([L.circle(0, 0, 56), L.box(0, -22, L.X_YOKE + L.YOKE_T, 28)]).convex_hull
    boss = L.circle(0, 0, 46).difference(L.holes_on(L.BACK_PCD, L.BACK_N, L.M3, phase=45))
    base = L.box(HY_ABAD - 22, -22, HY_FL[0], 28).difference(L.holes_on(L.OUT_PCD, L.OUT_N, L.M3, phase=90, at=(HY_ABAD, 0)))
    base_m = L.tri(L.to_world(CrossSection(L.rings(base), FillRule.EvenOdd).extrude(L.YOKE_T).translate((0, 0, L.X_YOKE)), 'yz'))
    paw = trimesh.creation.icosphere(subdivisions=3, radius=L.PAW_D / 2).apply_translation([0, (HY_SHIN[0] + HY_SHIN[1]) / 2, -HY_SHIN_L])
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
    th_t, rel = stance(255.0)
    legs = [(t, x, s, x > 0) for t, x, s in (('fl', hip_x, 1), ('fr', hip_x, -1), ('rl', -hip_x, 1), ('rr', -hip_x, -1))]
    extra = [(f'chassis-{n}', L.tri(m)) for n, m, _, _ in B.plates()]
    extra += [(f'int-{n}', L.tri(m)) for n, m, _ in B.internals()]
    pair_gap = 2 * (B.YA - L.RS00_D / 2)
    note = (f'hybrid: {2 * (y0 + HY_SHIN[1]):.0f} wide at the feet, {2 * (y0 + HY_OUT[1]):.0f} at the hips, nothing proud of '
            f'the leg; body {2 * B.X_END:.0f} x {2 * B.WIDE:.0f}; ab/ad pair gap {pair_gap:.0f}, thigh clears ab/ad by {-(HY_ABAD + L.RS00_D / 2):.1f}')
    B.configure(87.0)
    return dict(hip_x=hip_x, plane=y0, legs=legs, body=None, walls=[], extra=extra, note=note,
                replace=hybrid_parts(th_t, rel), only=True)


# --- inline: the hybrid with the knee motor flipped (back on the outer plate, output through the inner
#     plate) so the shin runs down the INSIDE of the thigh box, nearly under the ab/ad axis. Knees point
#     inward ("><": front back, rear forward) so a folding foot rises on the far side of the hip from the
#     ab/ad and yoke base.
IL_SHIN = (-(1.0 + L.T + 1.0 + 2 * L.T + 1.0), -(1.0 + L.T + 2.0))   # inboard of the yoke flange layer
IL_ABAD = IL_SHIN[0] - 3.0 - L.RS00_D / 2                            # ab/ad body clears the shin layer by 3


IP_AB = (-L.RS00_L / 2, L.RS00_L / 2)                  # ab/ad stator, x
IP_OUT = IP_AB[1] + 1.0                                # output face: 26.5 ahead of the limb axis
IP_BASE = (IP_AB[0] - L.YOKE_T, IP_AB[0])              # yoke base, behind the axis


def pocket(prof, keep, rim=8.0, rib=7.0, struts=(), min_area=120.0):
    """take weight out of a plate: keep a `rim`-wide edge, a ring round every bolt pattern or bearing
    (`keep`: (x, y, r) discs), and `rib`-wide struts (segments) between them; everything else is cut.
    Windows smaller than `min_area` mm^2 stay solid (not worth a cut)."""
    from shapely.geometry import LineString
    solid = L.unary_union([L.circle(x, y, 2 * r) for x, y, r in keep] +
                          [LineString(s).buffer(rib / 2, cap_style=1) for s in struts])
    win = prof.buffer(-rim, join_style=1).difference(solid).buffer(-1.5, join_style=1).buffer(1.5, join_style=1)
    geoms = getattr(win, 'geoms', [win])
    win = L.unary_union([g for g in geoms if g.area >= min_area])
    out = prof.difference(win) if not win.is_empty else prof
    # DFM: no web thinner than 1.5 x stock -- thicken any (they appear where a strut meets a ring at a shallow angle)
    w = 1.5 * L.T
    thin = out.difference(out.buffer(-w / 2, join_style=1).buffer(w / 2, join_style=1))
    if thin.area > 0.5:
        out = out.union(thin.buffer(w / 2, join_style=1).intersection(prof))
    POCKETED.append(out)
    return out


POCKETED = []                                           # every pocketed profile, for dfm.py
SHIN_PROFILES = []                                      # last two-motor shin's plates (profile, y0, t), for shin_check.py
LIMB_PROFILES = {}                                      # last limb module's plates by segment, for limb_check.py


def shapely_mirror(p):
    """mirror a profile across u = 0 (the body's centre plane, for the right-hand copy)."""
    from shapely import affinity
    return affinity.scale(p, xfact=-1, yfact=1, origin=(0, 0))


def warren(p0, p1, half, n):
    """struts of a Warren truss between p0 and p1 (a beam of half-width `half`): n bays of diagonals."""
    (x0, y0), (x1, y1) = p0, p1
    dx, dy = x1 - x0, y1 - y0
    ln = math.hypot(dx, dy) or 1.0
    ux, uy, nx, ny = dx / ln, dy / ln, -dy / ln, dx / ln
    pts = [(x0 + ux * ln * i / n + nx * half * (1 if i % 2 else -1), y0 + uy * ln * i / n + ny * half * (1 if i % 2 else -1)) for i in range(n + 1)]
    return [(pts[i], pts[i + 1]) for i in range(n)]


# RobStride motors the limb module can take. RS00 from its drawing; RS02 from the arm's fork model; RS06:
# body + front pattern from its manual (8 x M3 on 82), output/back patterns ASSUMED until measured.
MOTORS = {
    'rs00': dict(D=57.0, L=51.0, out=(27.0, 6), back=(38.0, 4), front=(50.0, 6), hole=3.4, kg=0.310, peak=14.0, cont=3.6),
    'rs02': dict(D=78.5, L=41.5, out=(24.0, 6), back=(52.0, 4), front=(73.0, 9), hole=4.5, kg=0.380, peak=17.0, cont=4.4),
    'rs06': dict(D=88.0, L=64.0, out=(36.0, 8), back=(60.0, 8), front=(82.0, 8), hole=3.4, kg=0.621, peak=36.0, cont=8.0),
    'rs04': dict(D=120.0, L=52.2, out=(36.0, 8), back=(106.0, 10), front=(106.0, 10), hole=4.5, kg=1.420, peak=120.0, cont=25.0),
}


def module_dims(mot='rs00', ab=None):
    """the module's layer stack: thigh-box motor (hip pitch + knee) `mot`, ab/ad motor `ab` (default the same),
    by the same rules as the RS00 module, solved for their bodies."""
    m, a = MOTORS[mot], MOTORS[ab or mot]
    mot_y = (HY_IN[1] + 1.0, HY_IN[1] + 1.0 + m['L'])
    out_y = (mot_y[1] + 1.0, mot_y[1] + 1.0 + L.T)
    abad = IL_SHIN[0] - 3.0 - a['D'] / 2
    ip_ab = (-a['L'] / 2, a['L'] / 2)
    return dict(m=m, a=a, MOT=mot_y, OUT=out_y, ABAD=abad, AB=ip_ab, IPOUT=ip_ab[1] + 1.0, BASE=(ip_ab[0] - L.YOKE_T, ip_ab[0]))


def inline_parts(th_t, rel, end=None, LT=HY_L, LS=HY_SHIN_L, DM=None, CR=None, mot='rs00', end_mot=None, dual=False, ab_mot=None, ank=(0.0, 0.0)):
    """THE limb module, all limbs: thigh box (hip + knee motor as spacers), inline shin, ab/ad centred
    fore-aft on the hip (stator on the yoke base behind, output face IPOUT ahead -> the body wall).
    Any RobStride in MOTORS; the RS00 build is the reference (plate widths scale with the motor body)."""
    d = module_dims(mot, ab_mot)
    m, ma_, MOT, OUTY, ABAD, AB, IPO, BASE = d['m'], d['a'], d['MOT'], d['OUT'], d['ABAD'], d['AB'], d['IPOUT'], d['BASE']
    ab_n = ab_mot or mot
    ka = ma_['D'] / 57.0
    k = m['D'] / 57.0                                        # plate-width scale vs the RS00 reference
    H = m['hole']
    hon = lambda pat, **kw: L.holes_on(pat[0], pat[1], H, **kw)
    xz = lambda prof, y0, t: L.tri(L.to_world(CrossSection(L.rings(prof), FillRule.EvenOdd).extrude(t).translate((0, 0, y0)), 'xz'))
    ry = lambda d_: trimesh.transformations.rotation_matrix(np.radians(d_), [0, 1, 0])
    cyl = lambda r, y0, y1, x=0, z=0: trimesh.creation.cylinder(radius=r, segment=[[x, y0, z], [x, y1, z]], sections=64)
    W = m['D'] + 7.0                                         # thigh box width: the motor + 3.5 a side
    boss_d = m['back'][0] + 8.0
    hub_d = m['out'][0] + 9.0
    box_thigh = L.stadium([(0, 0, W), (0, -LT, W)])
    win = lambda: L.rounded(-12 * k, -LT + 45 * k, 12 * k, -45 * k, 10)
    r_hip, r_knee = (boss_d + 4) / 2 + 7, max(hub_d + 4, m['back'][0] + 2 * H) / 2 + 7
    tr = warren((0, -r_hip), (0, -LT + r_knee), W / 2 - 8, max(2, int((LT - r_hip - r_knee) / (0.8 * W))))
    thigh_keep = [(0, 0, r_hip), (0, -LT, r_knee)]
    inner = pocket(box_thigh, thigh_keep, struts=tr).difference(L.circle(0, 0, boss_d + 4)).difference(L.circle(0, -LT, hub_d + 4))
    outer = pocket(box_thigh, thigh_keep, struts=tr).difference(hon(m['out'])).difference(hon(m['back'], phase=45, at=(0, -LT)))
    shin = L.stadium([(0, 0, hub_d + 4), (0, -LS, 22 * k)]).difference(hon(m['out'])) \
        .difference(L.rounded(-5, -LS + 32 * k, 5, -32 * k, 5))
    LIMB_PROFILES['thigh'] = [(inner, HY_IN[0], L.T), (outer, OUTY[0], L.T)]
    knee = trimesh.transformations.translation_matrix([0, 0, -LT])
    th = ry(th_t)
    fl = L.unary_union([L.circle(0, 0, W - 8), L.box(BASE[0], -26 * k, 0, 26 * k)]).convex_hull
    fl = pocket(fl, [(0, 0, boss_d / 2 + 6)], struts=[((0, 0), (BASE[0], 20 * k)), ((0, 0), (BASE[0], -20 * k))])
    boss = L.circle(0, 0, boss_d).difference(hon(m['back'], phase=45)).difference(L.circle(0, 0, max(10.0, m['back'][0] - 2 * H - 10)))   # a ring
    # ab/ad reversed: its stator back-mounts on the yoke base, its output bolts to the body wall, so the
    # limb module carries all three of its motors and the body is only plates with bolt patterns
    base = L.unary_union([L.circle(ABAD, 0, 29 * ka), L.box(ABAD, -32 * ka, HY_FL[0], 32 * ka)])
    rb = ma_['back'][0] / 2 + 6
    base = pocket(base, [(ABAD, 0, rb)], rim=7.0, struts=[((ABAD, 0), (HY_FL[0], 28 * ka)), ((ABAD, 0), (HY_FL[0], -28 * ka))]) \
        .difference(L.holes_on(ma_['back'][0], ma_['back'][1], ma_['hole'], phase=45, at=(ABAD, 0)))   # round about the ab/ad axis: rolls in its own shadow
    base_m = L.tri(L.to_world(CrossSection(L.rings(base), FillRule.EvenOdd).extrude(L.YOKE_T).translate((0, 0, BASE[0])), 'yz'))
    hub = L.circle(0, 0, hub_d)
    paw = trimesh.creation.icosphere(subdivisions=3, radius=L.PAW_D / 2).apply_translation([0, sum(IL_SHIN) / 2, -LS])
    e = MOTORS[end_mot or mot]
    if end:
        DM = DM or max(80.0, (W / 2 + e['D'] / 2 + 4.0) / math.sin(math.radians(127))) + (20.0 if end == 'wrist' and dual else 0.0)   # clears the thigh to a 127 deg knee (K1's stop); a 2-rod wrist sits lower so its rods clear the upper arm
        CR = CR or 0.4 * e['D']
        shin = L.stadium([(0, 0, hub_d + 4), (0, -DM, e['D'] + 7), (0, -LS, 30 * k)])
        rh, rm, rp = hub_d / 2 + 6, e['back'][0] / 2 + 8, 15 * k
        st = warren((0, -rh), (0, -DM + rm), e['D'] / 2 - 6, 2) + warren((0, -DM - rm), (0, -LS + rp), 8 * k, max(2, int((LS - DM) / 70)))
        shin = pocket(shin, [(0, 0, rh), (0, -DM, rm), (0, -LS, rp)], rim=7.0, struts=st).difference(hon(m['out'])) \
            .difference(L.holes_on(e['back'][0], e['back'][1], e['hole'], phase=45, at=(0, -DM))).difference(L.circle(0, -LS, 8.4 * k))
    LIMB_PROFILES['shin'] = [(shin, IL_SHIN[0], 2 * L.T)]
    parts = {
        'thigh-inner': xz(inner, HY_IN[0], L.T).apply_transform(th),
        'thigh-outer': xz(outer, OUTY[0], L.T).apply_transform(th),
        'shin': trimesh.util.concatenate([xz(shin, IL_SHIN[0], 2 * L.T), xz(hub, IL_SHIN[1], MOT[0] - IL_SHIN[1])])
                .apply_transform(ry(-rel)).apply_transform(knee).apply_transform(th),
        'paw': paw.apply_transform(ry(-rel)).apply_transform(knee).apply_transform(th),
        f'{mot}-hip': cyl(m['D'] / 2, *MOT),
        f'{mot}-knee': cyl(m['D'] / 2, *MOT).apply_transform(knee).apply_transform(th),
        'yoke': trimesh.util.concatenate([xz(fl, HY_FL[0], L.T), xz(boss, HY_FL[1], MOT[0] - HY_FL[1]), base_m]),
        f'{ab_n}-abad': trimesh.util.concatenate([
            trimesh.creation.cylinder(radius=ma_['D'] / 2, segment=[[AB[0], ABAD, 0], [AB[1], ABAD, 0]], sections=64),
            trimesh.creation.cylinder(radius=ma_['out'][0] / 2 + 4, segment=[[AB[1], ABAD, 0], [IPO, ABAD, 0]], sections=48)]),
    }
    if not end:
        return parts
    # mini: the end joint (ankle / wrist) is driven, not carried: its motor sits high in the shin, DM below the
    # knee (clear of the thigh box at any knee angle), and a pushrod parallel to the shin turns the foot / hand
    # crank -- a parallelogram, 1:1. Light extremities, the knee idiom kept: motor inline in the limb's band.
    del parts['paw']
    en = end_mot or mot
    shin_T = th @ knee @ ry(-rel)
    m0, m1 = IL_SHIN[1] + 1.0, IL_SHIN[1] + 1.0 + e['L']
    a = (rel - th_t if end == 'ankle' else 0.0) + ank[0]     # foot level (+ commanded pitch), hand inline
    cx, cz = CR * math.cos(math.radians(a)), -CR * math.sin(math.radians(a))   # crank vector, shin frame
    yl = m1 + 1.0                                            # crank layer (motor output face / foot side plate)
    yr_ = yl + L.T + 0.5                                     # rod layer
    ke = e['D'] / 57.0
    parts[f'{en}-{end}'] = cyl(e['D'] / 2, m0, m1, z=-DM).apply_transform(shin_T)
    crank = L.stadium([(0, 0, e['out'][0] + 13), (cx, cz, 16 * ke)]).difference(L.holes_on(e['out'][0], e['out'][1], e['hole'])).difference(L.circle(cx, cz, 6.4))
    parts[f'crank-{end}'] = xz(crank, yl, L.T).apply_transform(trimesh.transformations.translation_matrix([0, 0, -DM])).apply_transform(shin_T)
    rod = L.stadium([(cx, cz - DM, 14 * ke), (cx, cz - LS, 14 * ke)]).difference(L.circle(cx, cz - DM, 6.4)).difference(L.circle(cx, cz - LS, 6.4))
    pins = [trimesh.creation.cylinder(radius=3, segment=[[cx, yl - 0.5, cz + zz], [cx, yr_ + 2 * L.T + (L.T + 1.5 if end == 'wrist' and zz == -LS else 1), cz + zz]], sections=24) for zz in (-DM, -LS)]
    parts[f'link-{end}'] = trimesh.util.concatenate([xz(rod, yr_, 2 * L.T)] + pins).apply_transform(shin_T)
    parts[f'pivot-{end}'] = trimesh.creation.cylinder(radius=4 * k, segment=[[0, IL_SHIN[0] - 1, -LS], [0, (yr_ + 3 * L.T + 1.0) if end == 'wrist' else yl + L.T, -LS]], sections=24).apply_transform(shin_T)
    if dual:
        return ankle2(parts, th, knee, rel, th_t, LS, DM, CR, e, en, hub_d, m, k, xz, ank, end=end)
    eT = shin_T @ trimesh.transformations.translation_matrix([0, 0, -LS]) @ ry(a)
    lobe = (CR, 0)                                           # foot / hand crank pin, in its own frame
    if end == 'ankle':
        fz = 46 * k                                          # sole below the ankle axis
        fl_ = 0.6 * LS                                       # foot length
        ank = L.unary_union([L.circle(0, 0, 40 * k), L.stadium([(0, 0, 30 * k), (*lobe, 16 * ke)]), L.box(-0.3 * fl_, -fz, 0.46 * fl_, -fz + 16 * k)])
        ank = pocket(ank, [(0, 0, 14 * k), (*lobe, 8 * ke)], rim=6.0, struts=[((0, 0), (-0.25 * fl_, -fz)), ((0, 0), (0.4 * fl_, -fz)), ((0, 0), (0, -fz))]) \
            .difference(L.circle(0, 0, 8.4 * k)).difference(L.circle(*lobe, 6.4))
        y_in = IL_SHIN[0] - 8
        sw0, sw1 = y_in - 4, yl + L.T + 4
        sp = L.rounded(-0.41 * fl_, sw0, 0.59 * fl_, sw1, 10)
        sp = pocket(sp, [(0, (sw0 + sw1) / 2, 14 * k)], rim=10.0, rib=8.0,
                    struts=warren((-0.38 * fl_, (sw0 + sw1) / 2), (0.56 * fl_, (sw0 + sw1) / 2), (sw1 - sw0) / 2 - 10, 6))
        sole = L.tri(L.to_world(CrossSection(L.rings(sp), FillRule.EvenOdd).extrude(2 * L.T).translate((0, 0, -fz - 2 * L.T)), 'xy'))
        foot = [xz(ank, yl, L.T), sole]
        foot = trimesh.util.concatenate(foot)
    else:
        hl = 70 * k
        hand = L.unary_union([L.stadium([(0, 0, 30 * k), (0, -hl, 22 * k)]), L.stadium([(0, 0, 30 * k), (*lobe, 16 * ke)])]) \
            .difference(L.circle(0, 0, 8.4 * k)).difference(L.circle(*lobe, 6.4)).difference(L.rounded(-8 * k, -hl + 12 * k, 8 * k, -30 * k, 6))
        foot = xz(hand, yr_ + 2 * L.T + 1.0, L.T)              # its own layer, outboard of the rod: clears the crank
    parts['foot' if end == 'ankle' else 'hand'] = foot.apply_transform(eT)
    return parts



def ankle2_geom(LS, DM, CR, e, beta=-12.0, phi=180.0, h=25.0, gap=9.0, cgap=7.0, tuck=False):
    """the two-motor ankle in the shin frame (x fore, y out, z up the shin; knee at the origin): a shin BOX of
    two plates with both ankle motors as spacers, facing opposite ways (the thigh-box idiom). Motor A (upper)
    drives a crank + rod OUTSIDE the outer plate, motor B (lower) a crank + rod INSIDE the inner plate."""
    m0 = IL_SHIN[1] + 1.0
    DM1 = DM
    DM2 = DM + e['D'] + 2.0
    # tuck: the box is widened by a slot so the lower motor (facing in) stops short of the inner plate and its
    # crank + rod run INSIDE the box -- nothing stands inboard of the limb (used on the wrists)
    slot = 3 * L.T + 3.0 if tuck else 0.0
    y_os = m0 + e['L'] + 1.0 + slot                          # outer shin plate
    yA = y_os + L.T + cgap                                   # crank A layer (also the foot's post), on a spacer: posts roll clear of the box
    yRA = yA + L.T + gap                                     # rod A layer (2T); `gap` lets the rod lean as the foot rolls
    if tuck:
        yB = y_os - 1.0 - e['L'] - 1.0 - L.T                 # crank B, inside the box on B's output
        yRB = yB - 0.5 - 2 * L.T                             # rod B, inside the box, against the inner plate
    else:
        yB = IL_SHIN[0] - cgap - L.T                         # crank B layer (foot's inner post)
        yRB = yB - gap - 2 * L.T                             # rod B layer
    yc = (yB + yA + L.T) / 2                                 # roll axis, centred in the U
    # cranks and foot lobes both sit `beta` off square at neutral, so the K1 pitch range (-50..20) is centred
    # on the crank's best transmission instead of starting at one end of it
    return dict(m0=m0, DM1=DM1, DM2=DM2, y_os=y_os, yA=yA, yRA=yRA, yB=yB, yRB=yRB, yc=yc,
                rods=((DM1, yRA + L.T), (DM2, yRB + L.T)), CR=CR, LS=LS, beta=beta, phi=phi, h=h, tuck=tuck,
                foot_pin=(CR * math.cos(math.radians(beta + phi)), -h))
    # the foot pin sits level with the ROLL axis, under the crank pin: rolling the foot then barely moves it
    # sideways (the rods stay clear of the shin plates) and the posts stay low (clear of the lower motor)
    # h: the roll axis sits h below the pitch axis (a dog-bone U-joint), keeping the foot's lugs low
    # phi = 180: cranks, rods and foot posts BEHIND the shin (a calf + Achilles), so toe-up pitch swings the
    # posts down, away from the lower motor, instead of up into it


def ankle2_ik(G, q, r):
    """crank angles (deg) that put the foot at pitch q (relative to the shin, this file's R_y sense) and roll r
    (about x). Returns [(alpha, transmission)] per rod, or None where the linkage can't reach."""
    LS, CR, yc, b = G['LS'], G['CR'], G['yc'], math.radians(G['beta'] + G['phi'])
    fx, fz = G['foot_pin']
    Ry_ = trimesh.transformations.rotation_matrix(np.radians(q), [0, 1, 0])[:3, :3]
    Rx_ = trimesh.transformations.rotation_matrix(np.radians(r), [1, 0, 0])[:3, :3]
    cz = np.array([0, 0, -G['h']])
    out = []
    for DMi, yi in G['rods']:
        v = np.array([fx, yi - yc, fz])
        Q = np.array([0, yc, -LS]) + Ry_ @ (Rx_ @ (v - cz) + cz)
        P0 = np.array([CR * math.cos(b), yi, -DMi - CR * math.sin(b)])
        ell = np.linalg.norm(P0 - (np.array([0, yc, -LS]) + v))   # rod length: neutral crank to neutral foot pin
        f = lambda al: np.linalg.norm(np.array([CR * math.cos(al), yi, -DMi - CR * math.sin(al)]) - Q) - ell
        grid = np.radians(np.arange(G['phi'] - 170, G['phi'] + 171, 1.0))
        vals = np.array([f(g_) for g_ in grid])
        roots = []
        for i in range(len(grid) - 1):
            if vals[i] == 0 or vals[i] * vals[i + 1] < 0:
                lo, hi = grid[i], grid[i + 1]
                for _ in range(50):
                    mid = (lo + hi) / 2
                    if f(lo) * f(mid) <= 0: hi = mid
                    else: lo = mid
                roots.append((lo + hi) / 2)
        if not roots:
            out.append(None); continue
        al = min(roots, key=lambda x: abs(x - math.radians(q) - b))
        P = np.array([CR * math.cos(al), yi, -DMi - CR * math.sin(al)])
        arm = P - np.array([0, yi, -DMi]); rod = Q - P
        tm = np.linalg.norm(np.cross(arm, rod)) / (np.linalg.norm(arm) * np.linalg.norm(rod))
        out.append((math.degrees(al), tm, P, Q))
    return out


def _bar(P, Q, w, t):
    """a flat link from P to Q, its thickness along y."""
    d = (Q - P); ln = np.linalg.norm(d); d = d / ln
    y = np.array([0, 1.0, 0]); y = y - d * (y @ d); y /= np.linalg.norm(y)
    x = np.cross(y, d)
    M = np.eye(4); M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = x, y, d, (P + Q) / 2
    return trimesh.creation.box(extents=[w, t, ln + w]).apply_transform(M)


def ankle2(parts, th, knee, rel, th_t, LS, DM, CR, e, en, hub_d, m, k, xz, ank, end='ankle'):
    if end == 'ankle':
        CR = (0.8 if e['D'] > 70 else 1.2) * e['D']           # long cranks: less swing per degree of roll (swept: RS06 0.8 D, RS00 1.2 D; beta -12)
        G = ankle2_geom(LS, DM, CR, e)
    else:                                                    # wrist: pitch +/-45 x yaw +/-20 (swept: 1.0 D, beta -5)
        CR = 1.0 * e['D']
        G = ankle2_geom(LS, DM, CR, e, beta=-5.0, tuck=True)
    ry = lambda d_: trimesh.transformations.rotation_matrix(np.radians(d_), [0, 1, 0])
    cyl = lambda r, y0, y1, z: trimesh.creation.cylinder(radius=r, segment=[[0, y0, z], [0, y1, z]], sections=64)
    shin_T = th @ knee @ ry(-rel)
    ke = e['D'] / 57.0
    DM1, DM2, m0, y_os, yA, yRA, yB, yRB, yc = (G[x_] for x_ in ('DM1', 'DM2', 'm0', 'y_os', 'yA', 'yRA', 'yB', 'yRB', 'yc'))
    W = e['D'] + 7.0
    bk = lambda z: L.holes_on(e['back'][0], e['back'][1], e['hole'], phase=45, at=(0, z))
    pas = lambda z: L.circle(0, z, e['out'][0] + 11)
    # inner shin plate: knee hub .. A's stator (back) .. B's output (pass) .. pivot
    rh, rm, rp = hub_d / 2 + 6, e['back'][0] / 2 + 8, 15 * k
    inner = L.stadium([(0, 0, W), (0, -DM1, W), (0, -DM2, W), (0, -LS, 30 * k)])
    st = warren((0, -DM2 - rm), (0, -LS + rp), 8 * k, 2)
    # knee .. upper motor: the shin is ONE plate here (the outer plate can't reach up past the thigh box when
    # the knee folds), so it stays solid and full width -- it carries the landing load off-centre (shin_check.py)
    solid_top = inner.intersection(L.box(-W, -DM1, W, W))
    inner = pocket(inner, [(0, 0, rh), (0, -DM1, rm), (0, -DM2, rm), (0, -LS, rp)], rim=7.0, struts=st + [((0, -DM1), (0, -DM2))]).union(solid_top)
    inner = inner.difference(L.holes_on(m['out'][0], m['out'][1], m['hole'])).difference(L.circle(0, -LS, 8.4 * k))
    if not G['tuck']:
        inner = inner.difference(pas(-DM2)).difference(bk(-DM1))
    outer = L.stadium([(0, -DM1, W), (0, -DM2, W), (0, -LS, 30 * k)])
    outer = pocket(outer, [(0, -DM1, rm), (0, -DM2, rm), (0, -LS, rp)], rim=7.0, struts=[((0, -DM1), (0, -DM2))] + warren((0, -DM2 - rm), (0, -LS + rp), 8 * k, 2))
    outer = outer.difference(pas(-DM1)).difference(bk(-DM2)).difference(L.circle(0, -LS, 8.4 * k))
    if G['tuck']:      # tucked: A hangs from the outer plate on its front flange, so the slot is clear end to end
        outer = outer.difference(L.holes_on(e['front'][0], e['front'][1], e['hole'], at=(0, -DM1)))
    hub = L.circle(0, 0, hub_d)
    SHIN_PROFILES[:] = [(inner, IL_SHIN[0], 2 * L.T), (outer, y_os, L.T)]     # for shin_check.py
    LIMB_PROFILES['shin'] = list(SHIN_PROFILES)
    parts['shin'] = trimesh.util.concatenate([xz(inner, IL_SHIN[0], 2 * L.T), xz(hub, IL_SHIN[1], HY_MOT[0] - IL_SHIN[1]),
                                              xz(outer, y_os, L.T)]).apply_transform(shin_T)
    oring = e['out'][0] / 2 + 4
    a0 = y_os - 1.0 - e['L'] if G['tuck'] else m0              # A's back face
    parts[f'{en}-{end}'] = trimesh.util.concatenate([cyl(e['D'] / 2, a0, a0 + e['L'], -DM1), cyl(oring, a0 + e['L'], yA, -DM1)]).apply_transform(shin_T)
    b0 = y_os - 1.0 - e['L'] if G['tuck'] else m0              # B's output face
    parts[f'{en}-{end}-b'] = trimesh.util.concatenate([cyl(e['D'] / 2, b0, b0 + e['L'], -DM2), cyl(oring, yB + L.T, b0, -DM2)]).apply_transform(shin_T)
    # foot pose: level (+ commanded pitch) and roll
    a = (rel - th_t if end == 'ankle' else 0.0) + ank[0]   # foot level / hand inline, + command
    sol = ankle2_ik(G, a, ank[1])
    if any(s_ is None for s_ in sol):
        raise ValueError(f'{end} linkage cannot reach pitch {a:.0f} roll {ank[1]:.0f}')
    for (al, tm, P, Qp), (DMi, yi), yl_, tag in zip(sol, G['rods'], (yA, yB), ('', '-b')):
        cx, cz = CR * math.cos(math.radians(al)), -CR * math.sin(math.radians(al))
        crank = L.stadium([(0, 0, e['out'][0] + 13), (cx, cz, 16 * ke)]).difference(L.holes_on(e['out'][0], e['out'][1], e['hole'])).difference(L.circle(cx, cz, 6.4))
        parts[f'crank-{end}{tag}'] = xz(crank, yl_, L.T).apply_transform(trimesh.transformations.translation_matrix([0, 0, -DMi])).apply_transform(shin_T)
        y0_, y1_ = (yA, yRA + 2 * L.T) if tag == '' else (yRB, yB + L.T)      # pin: crank / foot post layer .. rod layer (ball ends)
        pins = [trimesh.creation.cylinder(radius=3, segment=[[pt[0], y0_ - 0.5, pt[2]], [pt[0], y1_ + 0.5, pt[2]]], sections=20) for pt in (P, Qp)]
        parts[f'link-{end}{tag}'] = trimesh.util.concatenate([_bar(P, Qp, 14 * ke, 2 * L.T)] + pins).apply_transform(shin_T)
    # ankle: a universal joint at the shin's end -- a cross block on the pitch pin (through both shin plates)
    # carries the roll pin (fore-aft); the foot hangs from the roll pin on two lugs, and the rods land on
    # posts rising from the sole IN FRONT of the shin box, so the foot rolls without touching the box
    sb = 12 * ke                                             # cross block half-size: fits between the shin plates
    parts[f'pivot-{end}'] = trimesh.util.concatenate([
        trimesh.creation.cylinder(radius=4 * k, segment=[[0, IL_SHIN[0] - 0.5, -LS], [0, y_os + L.T + 0.5, -LS]], sections=24),
        (lambda y0_, y1_: trimesh.creation.box(extents=[2 * sb, y1_ - y0_, 2 * sb]).apply_translation([0, (y0_ + y1_) / 2, -LS - G['h'] / 2]))(
            (yB + L.T + 1.0) if G['tuck'] else IL_SHIN[1] + 1.0, y_os - 1.0)]).apply_transform(shin_T)
    fz, fl_ = (G['h'] + 2 * sb + 6.0, 0.6 * LS) if end == 'ankle' else (30 * k, 0.35 * LS)   # sole just under the U-joint's lugs
    lx, lz = G['foot_pin']             # rod pins on the foot / hand, joint frame (x fore, z up)
    sw0, sw1 = yRB - 4, yRA + 2 * L.T + 4
    sp = L.rounded(-0.41 * fl_, sw0, 0.59 * fl_, sw1, 10) if end == 'ankle' else L.rounded(lx - 12 * ke, sw0, 0.45 * fl_, sw1, 8)
    sp = pocket(sp, [(0, yc, 14 * k), (lx, yRA + L.T, 10), (lx, yRB + L.T, 10)], rim=10.0, rib=8.0,
                struts=warren((-0.38 * fl_, yc), (0.56 * fl_, yc), (sw1 - sw0) / 2 - 10, 6) if end == 'ankle' else [((lx, yc), (0.4 * fl_, yc))])
    sole = L.tri(L.to_world(CrossSection(L.rings(sp), FillRule.EvenOdd).extrude(2 * L.T).translate((0, 0, -fz - 2 * L.T)), 'xy'))
    post = L.unary_union([L.stadium([(lx, lz, 14 * ke), (lx, -fz, 24 * ke)]), L.box(lx - 18 * ke, -fz, lx + 18 * ke, -fz + 10)]).difference(L.circle(lx, lz, 6.4))
    hh = G['h']
    lug = L.unary_union([L.circle(yc, -hh, 2 * sb), L.box(yc - sb, -fz, yc + sb, -hh)]).difference(L.circle(yc, -hh, 8.4 * k))
    yz = lambda prof, x0, t: L.tri(L.to_world(CrossSection(L.rings(prof), FillRule.EvenOdd).extrude(t).translate((0, 0, x0)), 'yz'))
    rollpin = trimesh.creation.cylinder(radius=4 * k, segment=[[-sb - L.T - 1.5, yc, -hh], [sb + L.T + 1.5, yc, -hh]], sections=24)
    body_ = [sole, xz(post, yA, L.T), xz(post, yB, L.T), yz(lug, sb + 1.0, L.T), yz(lug, -sb - 1.0 - L.T, L.T), rollpin]
    grip = None
    if end != 'ankle':
        # hand: a gripper in the house idiom -- under the U-joint's palm, an RS00 is a spacer in a palm BOX
        # (two plates), axis across the hand; the box plates run on into a fixed two-tine finger, and the RS00's
        # output carries the moving finger; TPU pads on both tips meet on the hand's centre plane
        g_ = MOTORS['rs00']
        zg = -fz - 2 * L.T - 8.0 - g_['D'] / 2                  # gripper axis, below the palm
        yg0, yg1 = yc - g_['L'] / 2, yc + g_['L'] / 2            # motor
        ya_, yb_ = yg0 - 1.0 - L.T, yg1 + 1.0                    # palm box plates (back on ya_, output through yb_)
        fl_len = 95.0 * k / 1.54
        box_p = L.unary_union([L.stadium([(0, zg, g_['D'] + 10), (0.25 * fl_, -fz - 2 * L.T, 20)]),
                               L.stadium([(-6, zg, 26), (-0.28 * fl_len, zg - fl_len, 14)])])     # fixed finger: the box runs on
        box_p = box_p.difference(L.circle(0, zg, g_['out'][0] + 11))
        box_b = box_p.difference(L.holes_on(g_['back'][0], g_['back'][1], g_['hole'], phase=45, at=(0, zg)))
        box_p = pocket(box_p, [(0, zg, g_['D'] / 2 + 4)], rim=6.0, struts=[((0, zg), (-0.2 * fl_len, zg - fl_len + 10))])
        body_ += [xz(box_b, ya_, L.T), xz(box_p, yb_, L.T)]
        gmot = trimesh.creation.cylinder(radius=g_['D'] / 2, segment=[[0, yg0, zg], [0, yg1, zg]], sections=48)
        mov = L.unary_union([L.circle(0, zg, g_['out'][0] + 13), L.stadium([(6, zg, 22), (0.3 * fl_len, zg - fl_len, 14)])]) \
            .difference(L.holes_on(g_['out'][0], g_['out'][1], g_['hole'], at=(0, zg)))
        mov = pocket(mov, [(0, zg, g_['out'][0] / 2 + 6)], rim=5.0, struts=[((0, zg), (0.25 * fl_len, zg - fl_len + 10))])
        grip = trimesh.util.concatenate([xz(mov, yb_ + L.T + 1.0, L.T)])
        pad = lambda x_: trimesh.creation.box(extents=[10, yb_ + 2 * L.T + 1 - ya_, 22]).apply_translation([x_, (ya_ + yb_ + 2 * L.T + 1) / 2, zg - fl_len + 4])
        body_.append(pad(-0.28 * fl_len + 7))
        grip = trimesh.util.concatenate([grip, pad(0.3 * fl_len - 7)])
    foot = trimesh.util.concatenate(body_)
    Rx = trimesh.transformations.rotation_matrix(np.radians(ank[1]), [1, 0, 0], [0, yc, -hh])
    hT = shin_T @ trimesh.transformations.translation_matrix([0, 0, -LS]) @ ry(a) @ Rx
    parts['foot' if end == 'ankle' else 'hand'] = foot.apply_transform(hT)
    if grip is not None:
        parts['finger'] = grip.apply_transform(hT)
        parts['rs00-grip'] = gmot.apply_transform(hT)
    return parts

AB_OUT = IP_OUT                                         # ab/ad output face (body side) from the hip: 26.5


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
    B.WALL_TAKES_OUTPUT = True
    wall = hip_x - AB_OUT                                # wall's outer face = the ab/ad output face
    B.X_END = wall                                       # the body is just the box between the two walls
    B.RIBS = {'nose': (wall - 2 * L.T, 2 * L.T, B.WIDE), 'mid': (-wall, 2 * L.T, B.WIDE)}
    B.INTERNALS = [('battery', (140, 105, 44), (10, 0, -18), 1.70), ('compute', (95, 63, 28), (-30, 0, 22), 0.12),
                   ('power', (60, 45, 22), (-55, 0, -26), 0.10)]
    hh = 255.0
    th_t, rel_t = stance(hh)
    # front: yoke + ab/ad mirrored fore-aft (ab/ad behind the hip, into the body), knee back as drawn.
    # rear: yoke as drawn (ab/ad ahead, into the body), the rest of the leg mirrored -> knee forward.
    legs = [(t, x, s, 'yoke' if x > 0 else 'leg') for t, x, s in
            (('fl', hip_x, 1), ('fr', hip_x, -1), ('rl', -hip_x, 1), ('rr', -hip_x, -1))]
    extra = [(f'chassis-{n}', L.tri(m)) for n, m, _, _ in B.plates()]
    extra += [(f'int-{n}', L.tri(m)) for n, m, _ in B.internals()]
    foot = y0 + sum(IL_SHIN) / 2
    lever = foot - ya
    note = (f'mini quad: {2 * (y0 + HY_OUT[1]):.0f} wide overall, feet at +/-{foot:.0f}, ab/ad lever {lever:.0f} mm; '
            f'body {2 * B.X_END:.0f} x {2 * B.WIDE:.0f} between two plain walls; knees point inward')
    B.configure(87.0)
    return dict(hip_x=hip_x, plane=y0, legs=legs, body=None, walls=[], extra=extra, note=note,
                replace=inline_parts(th_t, rel_t), only=True)


def dog():
    """the inline hybrid in dog proportions: a deep chest (depth / withers ~0.47) with the battery in a
    tray under the ab/ad motors, compute on the withers, hips pulled in as far as the inward knees allow:
    same-side thighs must clear through a +/-30 deg stride about the stance angle."""
    import importlib
    import body as B
    importlib.reload(B)
    hh = 255.0
    th_t, rel_t = stance(hh)
    stride = 30.0
    hip_x = math.ceil(LT * math.sin(math.radians(th_t + stride)) + 32 + 3)   # thigh box r32, 3 mm between
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
                replace=inline_parts(th_t, rel_t), only=True)


def humanoid():
    """the hybrid limb module on a biped, mounted exactly as on the quad: every limb hangs from a
    fore-aft ab/ad RS00 whose back face bolts to the torso's back plate. Legs: whole module mirrored
    fore-aft (knee bends forward). Arms: only the yoke flipped (elbow points back)."""
    y0 = L.RS00_D / 2 + 3.0 - IL_ABAD                     # as on the mini quad
    ya = y0 + IL_ABAD
    torso_h = 200.0                                       # hip axis -> shoulder axis
    arm_up, arm_bend = 10.0, 80.0                         # arm pose (deg)
    leg_hh = 0.96 * (LT + LS)                    # nearly straight legs
    leg_th, leg_rel = stance(leg_hh)
    x_back = -AB_OUT                                      # ab/ad output faces, behind the limb axes
    hip_z = leg_hh + L.PAW_D / 2
    legs = [('ll', 0.0, 1, 'all', hip_z), ('lr', 0.0, -1, 'all', hip_z),
            ('al', 0.0, 1, 'yoke', hip_z + torso_h), ('ar', 0.0, -1, 'yoke', hip_z + torso_h)]
    poses = {'all': inline_parts(leg_th, leg_rel), 'yoke': inline_parts(arm_up, arm_bend)}
    ext = lambda prof, plane, w0, t: L.tri(L.to_world(CrossSection(L.rings(prof), FillRule.EvenOdd).extrude(t).translate((0, 0, w0)), plane))
    half = ya + L.RS00_D / 2 + 4                          # back plate holds both ab/ad of a pair
    back = L.rounded(-half, hip_z - 40, half, hip_z + torso_h + 40, 10)
    for z in (hip_z, hip_z + torso_h):
        for s in (1, -1):
            back = back.difference(L.holes_on(L.OUT_PCD, L.OUT_N, L.M3, phase=90, at=(s * ya, z))).difference(L.circle(s * ya, z, 16))
    wide = y0 + IL_SHIN[0] - 8.0                           # torso sides 8 mm inside the shins
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
            f'{2 * (y0 + HY_OUT[1]):.0f} wide; every limb carries its ab/ad, the back plate takes their outputs')
    return dict(hip_x=0.0, plane=y0, legs=legs, body=None, walls=[], extra=extra, note=note, posed=poses, only=True)


def humanoid2():
    """the quad's chassis rule applied to a biped: every ab/ad points INTO the body. Arms: ab/ad ahead of the
    shoulder, output on a chest wall; legs: ab/ad behind the hip, output on a pelvis wall. The torso is the
    box joining the two walls -- chest forward, pelvis back -- so nothing sits behind the limbs."""
    y0 = L.RS00_D / 2 + 3.0 - IL_ABAD
    ya = y0 + IL_ABAD
    torso_h = 200.0
    arm_up, arm_bend = 10.0, 80.0
    leg_hh = 0.96 * (LT + LS)
    leg_th, leg_rel = stance(leg_hh)
    hip_z = leg_hh + L.PAW_D / 2
    sh_z = hip_z + torso_h
    legs = [('ll', 0.0, 1, 'all', hip_z), ('lr', 0.0, -1, 'all', hip_z),
            ('al', 0.0, 1, 'none', sh_z), ('ar', 0.0, -1, 'none', sh_z)]
    poses = {'all': inline_parts(leg_th, leg_rel), 'none': inline_parts(arm_up, arm_bend)}
    ext = lambda prof, plane, w0, t: L.tri(L.to_world(CrossSection(L.rings(prof), FillRule.EvenOdd).extrude(t).translate((0, 0, w0)), plane))
    wide = ya + L.RS00_D / 2 + 3.0                         # covers the ab/ad motors, clears the limbs
    half = wide
    def wall(z, x0):
        w = L.rounded(-half, z - 42, half, z + 42, 10)
        for s in (1, -1):
            w = w.difference(L.holes_on(L.OUT_PCD, L.OUT_N, L.M3, phase=90, at=(s * ya, z))).difference(L.circle(s * ya, z, 16))
        return ext(w, 'yz', x0, 2 * L.T)
    chest_x, pelvis_x = AB_OUT, -AB_OUT                    # wall faces = ab/ad output faces
    # side profile (x, z): pelvis box behind the hips, chest box ahead of the shoulders, an abdomen joining them
    prof = L.unary_union([L.rounded(pelvis_x, hip_z - 42, 34, hip_z + 70, 14),
                          L.rounded(-34, sh_z - 70, chest_x, sh_z + 42, 14),
                          L.rounded(-34, hip_z + 20, 34, sh_z - 20, 10)])
    prof = prof.buffer(8, join_style=1).buffer(-8, join_style=1)
    extra = [('chassis-wall-chest', wall(sh_z, chest_x)), ('chassis-wall-pelvis', wall(hip_z, pelvis_x - 2 * L.T)),
             ('chassis-side-l', ext(prof, 'xz', wide - L.T, L.T)), ('chassis-side-r', ext(prof, 'xz', -wide, L.T))]
    extra.append(('int-battery', L.tri(Manifold.cube((60, 2 * wide - 20, 126)).translate((-30, -wide + 10, hip_z + 45)))))
    head = Manifold.cube((70, 80, 60)).translate((-35, -40, sh_z + 42 + 25))
    neck = Manifold.cylinder(25, 14, 14, 32).translate((0, 0, sh_z + 42))
    extra.append(('head', L.tri(head + neck)))
    note = (f'humanoid, S-torso: chest {chest_x:.0f} ahead of the shoulders, pelvis {-pelvis_x:.0f} behind the hips, '
            f'nothing behind the limbs; ~{sh_z + 42 + 85:.0f} tall, {2 * (y0 + HY_OUT[1]):.0f} wide')
    return dict(hip_x=0.0, plane=y0, legs=legs, body=None, walls=[], extra=extra, note=note, posed=poses, only=True)


# --- in-plane: the ab/ad RS00 sits level with its limb -- centred fore-aft on the hip/shoulder axis, just
#     inboard of the shin layer -- so limb, ab/ad and torso share one ~66 mm slab. Its stator back-mounts on
#     the yoke base behind the axis; its output (front) bolts to the torso wall in front of the axis.
inplane_parts = inline_parts                            # one module now


def humanoid3():
    """slab humanoid: every ab/ad in the limb's own plane. Arms (as drawn: elbow back) put their ab/ad
    output on a FRONT wall; legs (mirrored fore-aft: knee forward) on a BACK wall. The torso is the two
    walls and the sides between them -- the same depth as the limbs."""
    y0 = L.RS00_D / 2 + 3.0 - IL_ABAD
    ya = y0 + IL_ABAD
    torso_h = 200.0
    arm_up, arm_bend = 6.0, 70.0
    leg_hh = 0.96 * (HY_L + HY_SHIN_L)
    leg_th, leg_rel = stance(leg_hh)
    hip_z = leg_hh + L.PAW_D / 2
    sh_z = hip_z + torso_h
    legs = [('ll', 0.0, 1, 'all', hip_z), ('lr', 0.0, -1, 'all', hip_z),
            ('al', 0.0, 1, 'none', sh_z), ('ar', 0.0, -1, 'none', sh_z)]
    poses = {'all': inplane_parts(leg_th, leg_rel), 'none': inplane_parts(arm_up, arm_bend)}
    ext = lambda prof, plane, w0, t: L.tri(L.to_world(CrossSection(L.rings(prof), FillRule.EvenOdd).extrude(t).translate((0, 0, w0)), plane))
    wide = ya + L.RS00_D / 2 + 3.0
    z0, z1 = hip_z - 40, sh_z + 40
    def wall(z_ports):
        w = L.rounded(-wide, z0, wide, z1, 12)
        for s in (1, -1):
            w = w.difference(L.holes_on(L.OUT_PCD, L.OUT_N, L.M3, phase=90, at=(s * ya, z_ports))).difference(L.circle(s * ya, z_ports, 16))
        return w.difference(L.rounded(-wide + 14, (z0 + z1) / 2 - 50, wide - 14, (z0 + z1) / 2 + 50, 10))
    side = L.rounded(-IP_OUT - 2 * L.T, z0, IP_OUT + 2 * L.T, z1, 12)
    extra = [('chassis-wall-front', ext(wall(sh_z), 'yz', IP_OUT, 2 * L.T)),
             ('chassis-wall-back', ext(wall(hip_z), 'yz', -IP_OUT - 2 * L.T, 2 * L.T)),
             ('chassis-side-l', ext(side, 'xz', wide - L.T, L.T)), ('chassis-side-r', ext(side, 'xz', -wide, L.T)),
             ('chassis-cap', ext(L.rounded(-IP_OUT - 2 * L.T, -wide, IP_OUT + 2 * L.T, wide, 8), 'xy', z1, L.T))]
    extra.append(('int-battery', L.tri(Manifold.cube((44, 2 * wide - 16, 120)).translate((-22, -wide + 8, hip_z + 40)))))
    head = Manifold.cube((60, 80, 60)).translate((-30, -40, z1 + L.T + 25))
    neck = Manifold.cylinder(25, 14, 14, 32).translate((0, 0, z1 + L.T))
    extra.append(('head', L.tri(head + neck)))
    depth = 2 * (IP_OUT + 2 * L.T)
    note = (f'slab humanoid: ab/ad in each limb\'s plane; torso {depth:.0f} deep x {2 * wide:.0f} wide, the limbs\' own depth; '
            f'~{z1 + 90:.0f} tall')
    return dict(hip_x=0.0, plane=y0, legs=legs, body=None, walls=[], extra=extra, note=note, posed=poses, only=True)


def humanoid4(leg_pose=(84.0, -12.0), arm_pose=(100.0, 70.0), hip_x=140.0):
    """the mini quad reared up: its own body (two walls taking the ab/ad outputs + decks) and its own four
    limbs, mounted exactly as on the quad; every hip pitches ~90 deg so the limbs hang along the spine, and
    the whole robot stands on its rear legs. ab/ad axes now run along the spine: hip/shoulder yaw."""
    import importlib
    import body as B
    importlib.reload(B)
    y0 = L.RS00_D / 2 + 3.0 - IL_ABAD
    B.configure(80.0, hip_x=hip_x, ends=False)
    B.PLANE, B.YA = y0, y0 + IL_ABAD
    B.WIDE = y0 + IL_SHIN[0] - 4.0
    B.WALL_TAKES_OUTPUT = True
    wall = hip_x - AB_OUT
    B.X_END = wall
    B.RIBS = {'nose': (wall - 2 * L.T, 2 * L.T, B.WIDE), 'mid': (-wall, 2 * L.T, B.WIDE)}
    B.INTERNALS = [('battery', (70, 105, 44), (0, 0, -18), 0.85), ('compute', (60, 63, 28), (0, 0, 22), 0.12)]
    # rear pair (legs): yoke as drawn, ab/ad ahead into the body; front pair (arms): yoke flipped, ab/ad behind
    legs = [('ll', -hip_x, 1, 'none'), ('lr', -hip_x, -1, 'none'), ('al', hip_x, 1, 'yoke'), ('ar', hip_x, -1, 'yoke')]
    poses = {'none': inline_parts(*leg_pose), 'yoke': inline_parts(*arm_pose)}
    extra = [(f'chassis-{n}', L.tri(m)) for n, m, _, _ in B.plates()]
    extra += [(f'int-{n}', L.tri(m)) for n, m, _ in B.internals()]
    head = Manifold.cube((60, 80, 60)).translate((hip_x + 40, -40, -30))         # above the shoulders
    neck = Manifold.cylinder(hip_x + 40 - wall, 14, 14, 32).rotate((0, 90, 0)).translate((wall + 2 * L.T, 0, 0))
    extra.append(('head', L.tri(head + neck)))
    note = (f'mini quad reared up: same body ({2 * wall:.0f} long between its walls) and limbs; '
            f'shoulder-to-hip {2 * hip_x:.0f}; ab/ad become yaw')
    B.configure(87.0)
    # stand it up: body x -> world z (nose up), body z -> world -x
    R = trimesh.transformations.rotation_matrix(np.radians(-90), [0, 1, 0])
    return dict(hip_x=hip_x, plane=y0, legs=legs, body=None, walls=[], extra=extra, note=note,
                posed=poses, only=True, world=R, ground=True)


def humanoid5(hip_a=85.0, arm_pose=(100.0, 70.0)):
    """the mini quad's chassis, upright, with hip roll. Built in the quad's body frame (x = spine), then
    stood up. Arms: the quad's front pair, exactly as on the quad (ab/ad output on the top wall -> shoulder
    yaw). Legs: a hip-yaw RS00 hangs from the pelvis wall (the quad's rear wall); an L-bracket on its output
    takes the quad limb module's ab/ad output, so that ab/ad is hip ROLL, as on the quad: yaw-roll-pitch-knee."""
    import importlib
    import body as B
    importlib.reload(B)
    y0 = L.RS00_D / 2 + 3.0 - IL_ABAD                     # 75.6: same limb plane as the quad
    ya = y0 + IL_ABAD                                     # arm ab/ad axes
    wall = hip_a - AB_OUT                                 # walls at +/-49.8, as the reared quad
    B.configure(80.0, hip_x=hip_a, ends=False)
    y0a = y0 + HY_OUT[1] - IL_SHIN[0] + L.PAW_D / 2       # arms outboard of the legs: the hands hang past the thighs
    ya = y0a + IL_ABAD                                    # arm ab/ad (shoulder) axes
    chest = ya + 20.0                                     # chest half-width: the arm's thigh box yaws +/-30 past it
    B.PLANE, B.YA = y0, ya
    B.WIDE = y0 + IL_SHIN[0] - 4.0
    B.X_END = wall
    B.WALL_TAKES_OUTPUT = True
    B.RIBS = {'nose': (wall - 2 * L.T, 2 * L.T, chest)}                    # top wall = shoulder bar
    B.INTERNALS = [('battery', (70, 105, 44), (0, 0, -18), 0.85), ('compute', (60, 63, 28), (0, 0, 22), 0.12)]
    extra = [(f'chassis-{n}', L.tri(m)) for n, m, _, _ in B.plates()]
    extra += [(f'int-{n}', L.tri(m)) for n, m, _ in B.internals()]
    ext = lambda prof, plane, w0, t: L.tri(L.to_world(CrossSection(L.rings(prof), FillRule.EvenOdd).extrude(t).translate((0, 0, w0)), plane))
    # chest: the belly box widens under the shoulder bar -- front/back skins + a floor, a T-torso
    ch_x = wall - 2 * L.T - 70.0
    skin = L.rounded(ch_x, -chest, wall, chest, 10).difference(L.rounded(ch_x + 14, -chest + 14, wall - 16, chest - 14, 8))
    extra.append(('chassis-chest-front', ext(skin, 'xy', B.H_TOP, L.T)))
    extra.append(('chassis-chest-back', ext(skin, 'xy', -B.H_BOT - L.T, L.T)))
    fl = L.box(-chest, -B.H_BOT, chest, B.H_TOP).difference(L.rounded(-B.WIDE + 4, -B.H_BOT + 12, B.WIDE - 4, B.H_TOP - 12, 8))
    extra.append(('chassis-chest-floor', ext(fl, 'yz', ch_x, 2 * L.T)))
    # legs, in body coordinates: leg frame (x_l fore-aft, y_l out, z_l down) -> body (z, y, -x)
    leg_hh = 0.96 * (HY_L + HY_SHIN_L)
    th, rel = stance(leg_hh)
    parts = inplane_parts(th, rel)                        # the same module as every other limb
    hip_x = -wall - 2 * L.T - 40.0                        # hip axis 40 under the pelvis wall (thigh box r32)
    zr = IP_OUT                                           # roll output face, 26.5 ahead of the hips
    yr = y0 + IL_ABAD                                     # roll axes
    # pelvis: ONE plate in the torso's mid-depth, between the decks, hanging below the pelvis wall; both
    # roll outputs bolt to it -- no yaw stage, no brackets: 12 RS00, as the quad
    pp = L.rounded(hip_x - 24, -yr - 24, -wall + 60, yr + 24, 10)          # port rings only: the thighs roll past it
    for s in (1, -1):
        pp = pp.difference(L.holes_on(L.OUT_PCD, L.OUT_N, L.M3, phase=90, at=(hip_x, s * yr))).difference(L.circle(hip_x, s * yr, 16))
    pp = pp.difference(L.rounded(hip_x + 28, -yr + 14, -wall + 40, yr - 14, 8))
    extra.append(('chassis-pelvis', ext(pp, 'xy', zr, 2 * L.T)))
    Tl = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1.0]])
    for tag, s in (('ll', 1), ('lr', -1)):
        place = trimesh.transformations.translation_matrix([hip_x, s * y0, 0]) @ np.diag([1, s, 1, 1.0]) @ Tl
        for n, g in parts.items():
            extra.append((f'{n}.{tag}', g.copy().apply_transform(place)))
    legs = [('al', hip_a, 1, 'yoke'), ('ar', hip_a, -1, 'yoke')]
    plane_arm = y0a
    poses = {'yoke': inline_parts(*arm_pose)}
    note = (f'mini humanoid: the mini quad chassis upright; legs roll-pitch-knee on one pelvis plate (the '
            f'quad limb module, 12 RS00), arms as the quad\'s front pair; shoulder-to-hip {hip_a - hip_x:.0f}')
    B.configure(87.0)
    R = trimesh.transformations.rotation_matrix(np.radians(-90), [0, 1, 0])
    return dict(hip_x=hip_a, plane=plane_arm, legs=legs, body=None, walls=[], extra=extra, note=note,
                posed=poses, only=True, world=R, ground=True)



def humanoid_family(name, leg, arm, chest_h, legm='rs00', armm='rs00', yawm='rs00', waistm='rs00', wristm=None,
                    arm_pose=(90.0, -60.0), stand=0.97, ankle_roll=False, head=None, legpk=None, rect=False,
                    arm_chain=False, arm_yawm='rs02', arm_drop=30.0, shoulder=(0.0, 0.0, 0.0), head_h=None, covers=False):
    """the humanoid style, for any RobStride set: flush shoulder bar, a chest with the waist motor inside,
    a pelvis box with the two hip-yaw motors as spacers, hip flange + roll plate, then the limb module
    (thigh box, inline shin, ab/ad centred on the hip) with the end joint driven by a pushrod."""
    legpk = legpk or legm                                  # hip pitch + knee (the thigh box); legm: ab/ad (roll) + ankle
    dl, da = module_dims(legpk, legm), module_dims(armm)
    ml, ma, my, mw = MOTORS[legm], MOTORS[armm], MOTORS[yawm], MOTORS[waistm]
    mpk = MOTORS[legpk]
    roll_r = math.hypot(ml['D'] / 2, ml['L'] / 2)         # a roll motor yaws inside this circle
    yy = max(roll_r + 4.0, my['D'] / 2 + 8.0)             # hip-yaw axes = roll axes
    y0 = yy - dl['ABAD']                                   # leg plane
    y0a = y0 + dl['OUT'][1] - IL_SHIN[0] + 8.0             # arms just outboard of the legs (thigh box's outer plate)
    ya = y0a + da['ABAD']
    chest = ya + 20.0 * ma['D'] / 57.0
    hp = yy + my['D'] / 2 + 6.0                            # pelvis half-width
    ka = ma['D'] / 57.0
    if rect:
        # one rectangular torso: the arms come in over the legs' line, the shoulder motors as close to the
        # neck/waist motor as they fit; the pelvis widens to the chest, and the torso grows until the
        # hanging hands clear the thighs (instead of the arms hanging outside them)
        mid = MOTORS[head] if head and head != 'fixed' else mw
        ya = max(mid['D'] / 2 + 5.0 + ma['D'] / 2, hp - 20.0 * ka)
        y0a = ya - da['ABAD']
        chest = ya + 20.0 * ka
        hp = chest
        T2_ = 2 * L.T
        top = (-T2_ - 6.0 - ma['D'] / 2) if arm_chain else (-T2_ - da['IPOUT'])     # shoulder axis below the bar's top face
        s2h0 = top - ((-T2_ - 1.0 - mw['L'] - 1.0) - T2_ - 1.0 - T2_ - 1.0 - my['L'] - 1.0 - T2_ - 1.0 - L.T
                      - ((MOTORS[legpk]['D'] + 7.0) / 2 + 8.0))
        if arm_chain:   # pitch axis .. roll motor .. yaw motor .. cap .. upper arm .. elbow motor's bottom
            reach = ma['D'] / 2 + 4.0 + MOTORS[arm_yawm]['L'] + 1.0 + L.T + (ma['D'] + 7.0) + arm[0]
        else:
            reach = arm[0] + (ma['D'] + 7.0) / 2               # shoulder axis to the bottom of the elbow: only the elbows
                                                              # must clear the thighs; forearms work in front of them
        chest_h = max(chest_h, reach + (MOTORS[legpk]['D'] + 7.0) / 2 + 8.0 - s2h0)   # elbow box 8 mm over the thigh box
    H = max(41.0, ma['D'] / 2 + 12.0, mw['D'] / 2 + 12.0, my['D'] / 2 + 12.0)
    T2 = 2 * L.T
    ext = lambda prof, plane, w0, t: L.tri(L.to_world(CrossSection(L.rings(prof), FillRule.EvenOdd).extrude(t).translate((0, 0, w0)), plane))
    cyl = lambda r, x0, x1, y=0.0: trimesh.creation.cylinder(radius=r, segment=[[x0, y, 0], [x1, y, 0]], sections=64)
    extra = []
    def wallp(half, ports):
        w = L.rounded(-half, -H, half, H, 8)
        keep = [(y, 0, pat[0] / 2 + hd + 5) for (y, pat, ph, hole, hd) in ports]
        ys = sorted([-half + 10] + [k_[0] for k_ in keep] + [half - 10])
        st = [s_ for a_, b_ in zip(ys, ys[1:]) for s_ in warren((a_, 0), (b_, 0), H - 8, max(1, int((b_ - a_) / (1.2 * H))))]
        w = pocket(w, keep, rim=8.0, struts=st)
        for (y, pat, ph, hole, hd) in ports:
            w = w.difference(L.holes_on(pat[0], pat[1], hd, phase=ph, at=(y, 0))).difference(L.circle(y, 0, hole))
        return w
    # chest: shoulder bar (arm ab/ad outputs on its UNDERSIDE: shoulder-yaw motors inside the chest's top
    # corners, nothing above the shoulder line) .. floor; the waist motor inside on its front flange, output down
    xt = 0.0
    xf = xt - T2 - 1.0 - mw['L'] - 1.0 - chest_h
    neck_port = [(0, MOTORS[head]['front'], 0, MOTORS[head]['out'][0] + 11, MOTORS[head]['hole'])] if head and head != 'fixed' else []
    if arm_chain:
        bar = wallp(chest, neck_port)
    else:
        bar = wallp(chest, [(s * ya, ma['out'], 90, ma['out'][0] * 0.6, ma['hole']) for s in (1, -1)] + neck_port)
    extra.append(('chassis-shoulder-bar', ext(bar, 'yz', xt - T2, T2)))
    extra.append(('chassis-chest-floor', ext(wallp(chest, [(0, mw['front'], 0, mw['out'][0] + 11, mw['hole'])]), 'yz', xf - T2, T2)))
    skin = L.rounded(xf - T2, -chest, xt, chest, 10)
    xm = (xf + xt) / 2
    skin = pocket(skin, [], rim=12.0, rib=9.0, struts=warren((xm, -chest + 12), (xm, chest - 12), (xt - xf) / 2 - 14, max(2, int(chest / 45))))
    extra += [('chassis-chest-front', ext(skin, 'xy', H, L.T)), ('chassis-chest-back', ext(skin, 'xy', -H - L.T, L.T))]
    extra.append((f'{waistm}-waist', cyl(mw['D'] / 2, xf, xf + mw['L'])))
    extra.append((f'{waistm}-waist-out', cyl(mw['out'][0] / 2 + 4, xf - T2 - 1.0, xf)))
    bw = 0.9 * H
    for s in (1, -1):
        bh = (0.35 * chest_h + 10) if rect and not arm_chain else (0.6 * chest_h + 20)   # stay under the arms' yokes
        extra.append((f'int-battery.{"l" if s > 0 else "r"}', L.tri(Manifold.cube((bh, 0.4 * chest, 2 * bw * 0.85))
                      .translate((xf + 4, s * (mw['D'] / 2 + 6 + 0.2 * chest) - 0.2 * chest, -bw * 0.85)))))
    # pelvis: top wall takes the waist output; the two hip-yaw motors stand between its walls, outputs down
    xp = xf - T2 - 1.0
    xpb = xp - T2 - 1.0 - my['L'] - 1.0
    extra.append(('chassis-pelvis-top', ext(wallp(hp, [(0, mw['out'], 90, mw['out'][0] * 0.6, mw['hole'])]), 'yz', xp - T2, T2)))
    extra.append(('chassis-pelvis-bottom', ext(wallp(hp, [(s * yy, my['front'], 0, my['out'][0] + 11, my['hole']) for s in (1, -1)]), 'yz', xpb - T2, T2)))
    pskin = L.rounded(xpb - T2, -hp, xp, hp, 8).difference(L.circle((xp + xpb) / 2 - L.T, 0, 0.5 * (xp - xpb)))
    extra += [('chassis-pelvis-front', ext(pskin, 'xy', H, L.T)), ('chassis-pelvis-back', ext(pskin, 'xy', -H - L.T, L.T))]
    xo = xpb - T2 - 1.0
    th, rel = stance(stand * sum(leg), *leg)
    th, rel = -th, -rel                                   # knees bend FORWARD: toward the toes and the face
    parts = inline_parts(th, rel, end='ankle', LT=leg[0], LS=leg[1], mot=legpk, ab_mot=legm, end_mot=legm, dual=ankle_roll)
    hip_x = xo - L.T - ((mpk['D'] + 7.0) / 2 + 8.0)
    zr, yr = dl['IPOUT'], y0 + dl['ABAD']
    rf, rl = my['out'][0] + 33, ml['out'][0] + 33         # flange / roll-plate port discs
    Tl = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1.0]])
    for tag, s in (('ll', 1), ('lr', -1)):
        extra.append((f'{yawm}-hipyaw.{tag}', cyl(my['D'] / 2, xpb + 1.0, xpb + 1.0 + my['L'], s * yy)))
        extra.append((f'{yawm}-hipyaw-out.{tag}', cyl(my['out'][0] / 2 + 4, xo, xpb + 1.0, s * yy)))
        y_lo, y_hi = sorted((s * (yy + rf / 2.5), s * (yr - rl / 2.5)))
        fl = L.unary_union([L.circle(s * yy, 0, rf), L.rounded(y_lo, -10, y_hi, zr + T2, 6)]).difference(L.holes_on(my['out'][0], my['out'][1], my['hole'], at=(s * yy, 0)))
        extra.append((f'chassis-hipflange.{tag}', ext(fl, 'yz', xo - L.T, L.T)))
        rp = L.unary_union([L.circle(hip_x, s * yr, rl), L.rounded(hip_x, y_lo, xo - L.T, y_hi, 6)])
        rp = rp.difference(L.holes_on(ml['out'][0], ml['out'][1], ml['hole'], phase=90, at=(hip_x, s * yr))).difference(L.circle(hip_x, s * yr, ml['out'][0] * 0.6))
        extra.append((f'chassis-rollplate.{tag}', ext(rp, 'xy', zr, T2)))
        place = trimesh.transformations.translation_matrix([hip_x, s * y0, 0]) @ np.diag([1, s, 1, 1.0]) @ Tl
        for n, g in parts.items():
            extra.append((f'{n}.{tag}', g.copy().apply_transform(place)))
    if head == 'fixed':
        # a fixed head (no DoF, for a torque-limited size): the same shell on a short neck plate bolted to the bar
        hh = head_h or 75.0
        hw, dep = 0.33 * hh, 0.8 * hh
        x0 = xt + 14.0
        x1 = x0 + hh
        zf, zb = 0.55 * dep, -0.45 * dep
        extra.append(('chassis-neck', ext(L.rounded(-hw + L.T, zb, hw - L.T, zf, 6), 'yz', xt, L.T)))
        post = L.rounded(xt + L.T, -0.3 * dep, x0 + 0.4 * hh, 0.3 * dep, 6)
        extra.append(('chassis-neck-post', ext(post, 'xz', -L.T, 2 * L.T)))
        ex_ = x0 + 0.55 * hh
        face = L.rounded(x0, -hw, x1, hw, 0.18 * hh).difference(L.circle(ex_, -0.42 * hw, 0.16 * hh)).difference(L.circle(ex_, 0.42 * hw, 0.16 * hh))
        extra.append(('head-face', ext(face, 'xy', zf, L.T)))
        extra.append(('head-back', ext(L.rounded(x0, -hw, x1, hw, 0.18 * hh), 'xy', zb - L.T, L.T)))
        extra.append(('head-crown', ext(L.rounded(-hw, zb, hw, zf, 0.1 * hh), 'yz', x1, L.T)))
        cheek = L.rounded(x0, zb, x1, zf, 0.15 * hh)
        extra += [('head-cheek.r', ext(cheek, 'xz', hw - L.T, L.T)), ('head-cheek.l', ext(cheek, 'xz', -hw, L.T))]
        extra.append(('int-camera', L.tri(Manifold.cube((18, 2 * hw - 4 * L.T, 14)).translate((ex_ - 9, -hw + 2 * L.T, zf - 15)))))
    elif head:
        # head: neck yaw motor INSIDE the chest, hung from the shoulder bar on its front flange, output up
        # through the bar (the waist idiom); on it a neck bracket holding the pitch motor as a spacer between
        # two plates (the thigh-box idiom); the head is a U on the pitch output + a plain pivot, camera in front
        mn, mp = MOTORS[head], MOTORS[head]
        xn0 = xt - T2 - 1.0 - mn['L']
        extra.append((f'{head}-neckyaw', cyl(mn['D'] / 2, xn0, xn0 + mn['L'])))
        extra.append((f'{head}-neckyaw-out', cyl(mn['out'][0] / 2 + 4, xn0 + mn['L'], xt + 1.0)))
        bar_holes = L.holes_on(mn['front'][0], mn['front'][1], mn['hole']).union(L.circle(0, 0, mn['out'][0] + 11))
        kp = mp['D'] / 57.0
        xb = xt + 1.0                                       # neck plate on the yaw output
        xh = xb + L.T + 8.0 + mp['D'] / 2                   # pitch axis
        ya_, yb_ = mp['L'] / 2 + 1.0, -(mp['L'] / 2 + 1.0)  # output face +y, back face -y
        neck = L.rounded(-mp['L'] / 2 - 1 - L.T, -24 * kp, mp['L'] / 2 + 1 + L.T, 24 * kp, 6).difference(L.holes_on(mn['out'][0], mn['out'][1], mn['hole']))
        extra.append(('chassis-neck', ext(neck, 'yz', xb, L.T)))
        pa = L.stadium([(xb + L.T + 20 * kp, 0, 40 * kp), (xh, 0, mp['D'] + 7)]).difference(L.holes_on(mp['back'][0], mp['back'][1], mp['hole'], phase=45, at=(xh, 0)))
        extra.append(('chassis-neck-plate', ext(pa, 'xz', yb_ - L.T, L.T)))
        extra.append((f'{head}-neckpitch', trimesh.creation.cylinder(radius=mp['D'] / 2, segment=[[xh, yb_ + 1.0, 0], [xh, ya_ - 1.0, 0]], sections=64)))
        if head_h:
            # a human-proportioned head: height head_h (~1/7.5 of standing height), width 0.66 h, depth 0.8 h --
            # a closed plate shell (face, back, crown, cheeks) on the pitch motor's two plates, eyes in the face
            hh = head_h
            hw, dep = 0.33 * hh, 0.8 * hh
            x0 = xb + L.T + 6.0
            x1 = x0 + hh
            zf, zb = 0.55 * dep, -0.45 * dep
            side = L.unary_union([L.circle(xh, 0, mp['D'] + 10), L.rounded(x0, zb + L.T + 1, x1 - L.T - 1, zf - 1, 10)])
            side = pocket(side, [(xh, 0, mp['D'] / 2 + 8)], rim=8.0, struts=[((xh, 0), (x1 - 12, zf - 12)), ((xh, 0), (x1 - 12, zb + 12))])
            extra.append(('head-side.r', ext(side.difference(L.holes_on(mp['out'][0], mp['out'][1], mp['hole'], at=(xh, 0))), 'xz', ya_, L.T)))
            extra.append(('head-side.l', ext(side.difference(L.circle(xh, 0, 12)), 'xz', yb_ - 2 * L.T - 1.0, L.T)))
            extra.append(('head-pivot', trimesh.creation.cylinder(radius=5, segment=[[xh, yb_ - 2 * L.T - 1.5, 0], [xh, yb_ - L.T, 0]], sections=24)))
            ex_ = x0 + 0.55 * hh
            face = L.rounded(x0, -hw, x1, hw, 0.18 * hh).difference(L.circle(ex_, -0.42 * hw, 0.16 * hh)).difference(L.circle(ex_, 0.42 * hw, 0.16 * hh))
            extra.append(('head-face', ext(face, 'xy', zf, L.T)))
            extra.append(('head-back', ext(L.rounded(x0, -hw, x1, hw, 0.18 * hh), 'xy', zb - L.T, L.T)))
            extra.append(('head-crown', ext(L.rounded(-hw, zb, hw, zf, 0.1 * hh), 'yz', x1, L.T)))
            cheek = L.rounded(x0, zb, x1, zf, 0.15 * hh)
            cheek = pocket(cheek, [], rim=9.0, struts=warren((x0 + 10, (zb + zf) / 2), (x1 - 10, (zb + zf) / 2), (zf - zb) / 2 - 10, 2))
            extra += [('head-cheek.r', ext(cheek, 'xz', hw - L.T, L.T)), ('head-cheek.l', ext(cheek, 'xz', -hw, L.T))]
            extra.append(('int-camera', L.tri(Manifold.cube((26, ya_ - (yb_ - L.T) - 4, 22)).translate((ex_ - 13, yb_ - L.T + 2, zf - 22 - 1)))))
        else:
            hw = mp['L'] / 2 + 1 + 2 * L.T + 6.0                # head half-width
            side = L.unary_union([L.circle(xh, 0, mp['D'] + 10), L.rounded(xh - 20 * kp, -10, xh + 60 * kp, 50 * kp, 10)])
            extra.append(('head-side.r', ext(side.difference(L.holes_on(mp['out'][0], mp['out'][1], mp['hole'], at=(xh, 0))), 'xz', ya_, L.T)))
            extra.append(('head-side.l', ext(side.difference(L.circle(xh, 0, 12)), 'xz', yb_ - 2 * L.T - 1.0, L.T)))
            extra.append(('head-pivot', trimesh.creation.cylinder(radius=5, segment=[[xh, yb_ - 2 * L.T - 1.5, 0], [xh, yb_ - L.T, 0]], sections=24)))
            face = L.rounded(xh - 20 * kp, -hw, xh + 60 * kp, hw, 10).difference(L.circle(xh + 25 * kp, -hw * 0.45, 18 * kp)).difference(L.circle(xh + 25 * kp, hw * 0.45, 18 * kp))
            extra.append(('head-face', ext(face, 'xy', 50 * kp, L.T)))
            extra.append(('int-camera', L.tri(Manifold.cube((22 * kp, ya_ - yb_ + L.T - 4, 20 * kp)).translate((xh + 14 * kp, yb_ - L.T + 2, 50 * kp - 20 * kp - 1)))))
    hip_a = xt - T2 - da['IPOUT']
    legs = [('al', hip_a, 1, 'none'), ('ar', hip_a, -1, 'none')]
    poses = {} if arm_chain else {'none': inline_parts(*arm_pose[:2], end='wrist', LT=arm[0], LS=arm[1], mot=armm, end_mot=wristm)}
    if arm_chain:
        # shoulder, pitch-roll-yaw (as a person's): the PITCH motor sits inside the chest corner, stator on a
        # chest side wall, output out through it; on its output an L-bracket holds the ROLL motor (axis
        # fore-aft) beside the torso; the roll output's L-bracket hangs the upper-arm YAW motor (axis down the
        # arm); the yaw output caps the thigh box, which is now only the upper arm + elbow motor.
        mya, W_ = MOTORS[arm_yawm], ma['D'] + 7.0
        P_, R_, Y_ = shoulder
        c = chest
        xp = xt - T2 - 6.0 - ma['D'] / 2                                 # pitch axis (lateral), just under the bar
        yr = c + 1.0 + L.T + 22.0 + ma['D'] / 2                          # roll axis (fore-aft), beside the torso; the arm clears its side
        xr = xp
        xc = xr - ma['D'] / 2 - 4.0                                      # roll bracket's yaw plate, under the roll motor
        x_cap = xc - mya['L'] - 1.0                                      # yaw output face = top of the upper arm
        y0a = yr - da['OUT'][1] / 2                                      # upper-arm box centred on the yaw axis
        oa = ma['out'][0] / 2 + 4
        side = L.rounded(xf, -H, xt - T2, H, 8).difference(L.holes_on(ma['out'][0], ma['out'][1], ma['hole'], at=(xp, 0))).difference(L.circle(xp, 0, ma['out'][0] + 11))
        side = pocket(side, [(xp, 0, ma['out'][0] / 2 + 12)], rim=10.0, struts=warren((xp - 40, 0), (xf, 0), H - 10, 3))
        extra += [('chassis-chest-side.l', ext(side, 'xz', c - T2, T2)), ('chassis-chest-side.r', ext(side, 'xz', -c, T2))]
        apart = inline_parts(0.0, arm_pose[1], end='wrist', LT=arm[0], LS=arm[1], mot=armm, ab_mot=armm, end_mot=wristm, dual=True, ank=tuple(arm_pose[2:4]) if len(arm_pose) > 2 else (0.0, 0.0))
        for n_ in [n_ for n_ in apart if n_ == 'yoke' or n_.endswith(('-abad', '-hip'))]:
            del apart[n_]
        rot = lambda deg, axis, pt: trimesh.transformations.rotation_matrix(np.radians(deg), axis, pt)
        cylx = lambda r, x0, x1, y, z=0.0: trimesh.creation.cylinder(radius=r, segment=[[x0, y, z], [x1, y, z]], sections=64)
        cyly = lambda r, y0_, y1_, x, z=0.0: trimesh.creation.cylinder(radius=r, segment=[[x, y0_, z], [x, y1_, z]], sections=64)
        cylz = lambda r, z0, z1, x, y: trimesh.creation.cylinder(radius=r, segment=[[x, y, z0], [x, y, z1]], sections=64)
        zb = -ma['L'] / 2 - 1.0                                         # roll stator back face (behind)
        zf = ma['L'] / 2 + 1.0                                          # roll output face (front)
        for tag, s in (('al', 1), ('ar', -1)):
            Mp = rot(P_, [0, 1, 0], [xp, 0, 0])
            Mr = Mp @ rot(s * R_, [0, 0, 1], [xr, s * yr, 0])
            My = Mr @ rot(s * Y_, [1, 0, 0], [0, s * yr, 0])
            add = lambda n_, m_, M_: extra.append((f'{n_}.{tag}', m_.apply_transform(M_)))
            # pitch motor, inside the chest
            y_in, y_out = c - T2 - 1.0 - ma['L'], c - T2 - 1.0
            extra.append((f'{armm}-shoulderpitch.{tag}', cyly(ma['D'] / 2, s * y_in, s * y_out, xp)))
            add(f'{armm}-shoulderpitch-out', cyly(oa, s * y_out, s * (c + 1.0), xp), Mp)
            # pitch bracket: B1 on the pitch output (xz), B2 holding the roll stator's back (xy, behind)
            b1 = L.unary_union([L.circle(xp, 0, ma['out'][0] + 33), L.rounded(xp - ma['D'] / 2 - 2, zb, xp + 18, 18, 6)]) \
                .difference(L.holes_on(ma['out'][0], ma['out'][1], ma['hole'], at=(xp, 0)))
            add('chassis-shoulder-pitchplate', ext(b1, 'xz', (c + 1.0) if s > 0 else -(c + 1.0) - L.T, L.T), Mp)
            b2 = L.unary_union([L.circle(xr, s * yr, ma['back'][0] + 16), L.rounded(xr - ma['D'] / 2 - 2, min(s * (c + 1.0 + L.T), s * yr), xr + 18, max(s * (c + 1.0 + L.T), s * yr), 6)]) \
                .difference(L.holes_on(ma['back'][0], ma['back'][1], ma['hole'], phase=45, at=(xr, s * yr)))
            add('chassis-shoulder-rollmount', ext(b2, 'xy', zb - L.T, L.T), Mp)
            add(f'{armm}-shoulderroll', cylz(ma['D'] / 2, -ma['L'] / 2, ma['L'] / 2, xr, s * yr), Mp)
            add(f'{armm}-shoulderroll-out', cylz(oa, ma['L'] / 2, zf, xr, s * yr), Mr)
            # roll bracket: C1 on the roll output (xy, front), C2 under the roll motor holding the yaw stator (yz)
            c1 = L.unary_union([L.circle(xr, s * yr, ma['out'][0] + 33), L.rounded(xc + L.T, s * yr - 26, xr, s * yr + 26, 6)]) \
                .difference(L.holes_on(ma['out'][0], ma['out'][1], ma['hole'], at=(xr, s * yr)))
            add('chassis-shoulder-rollplate', ext(c1, 'xy', zf, L.T), Mr)
            c2 = L.unary_union([L.circle(s * yr, 0, mya['back'][0] + 16), L.rounded(s * yr - 26, -10, s * yr + 26, zf, 6)]) \
                .difference(L.holes_on(mya['back'][0], mya['back'][1], mya['hole'], phase=45, at=(s * yr, 0)))
            add('chassis-shoulder-yawmount', ext(c2, 'yz', xc, L.T), Mr)
            add(f'{arm_yawm}-armyaw', cylx(mya['D'] / 2, xc - mya['L'], xc, s * yr), Mr)
            add(f'{arm_yawm}-armyaw-out', cylx(mya['out'][0] / 2 + 4, x_cap, xc - mya['L'], s * yr), My)
            # upper arm: the yaw output caps the thigh box
            ylo, yhi = sorted((s * (y0a + HY_IN[0]), s * (y0a + da['OUT'][1])))
            cap = L.rounded(ylo, -0.8 * W_ / 2, yhi, 0.8 * W_ / 2, 8).difference(L.holes_on(mya['out'][0], mya['out'][1], mya['hole'], at=(s * yr, 0)))
            add('chassis-arm-cap', ext(cap, 'yz', x_cap - L.T, L.T), My)
            place = My @ trimesh.transformations.translation_matrix([x_cap - L.T - W_ / 2, s * y0a, 0]) @ np.diag([1, s, 1, 1.0]) @ Tl
            for n_, g_ in apart.items():
                extra.append((f'{n_}.{tag}', g_.copy().apply_transform(place)))
        hip_a, legs, poses = xp, [], {}
        y0a = y0a  # for the width note
    count = {}
    arms_ = ([armm] * 6 + [wristm] * 4 + ['rs00'] * 2 + [arm_yawm] * 2) if arm_chain else ([armm] * 6 + [wristm or armm] * 2)
    for mm in [legpk] * 4 + [legm] * (6 if ankle_roll else 4) + arms_ + [yawm] * 2 + [waistm] + ([head] * 2 if head and head != 'fixed' else []):
        count[mm] = count.get(mm, 0) + 1
    kg = sum(MOTORS[k_]['kg'] * n_ for k_, n_ in count.items())
    mix = ' + '.join(f'{n_} {k_.upper()}' for k_, n_ in count.items())
    note = (f'{name}: {sum(count.values())} joints ({mix}, motors {kg:.1f} kg); thigh {leg[0]:.0f} / shin {leg[1]:.0f}; '
            f'shoulder-to-hip {hip_a - hip_x:.0f}, shoulders {2 * (y0a + da["OUT"][1]):.0f} wide')
    if covers:
        add_covers(extra)
    R = trimesh.transformations.rotation_matrix(np.radians(-90), [0, 1, 0])
    return dict(hip_x=hip_a, plane=y0a, legs=legs, body=None, walls=[], extra=extra, note=note,
                posed=poses, only=True, world=R, ground=True, cover_posed=covers)


def humanoid_k1(s2h=261.0, c=76.0, leg_lt=5.0, shin=245.0, arm_lt=57.0, forearm=150.0, head_h=125.0, hip_y=None,
                knee_bend=12.0, elbow=12.0, ankle=(0.0, 0.0), leg_pose=None, arm_pose=None):
    """MINI humanoid = the Booster K1 reverse-engineered on our stack: its 22 DoF and its dimensions.
    Every limb is the same pitch-roll-yaw chain (pitch motor inside a torso corner, roll L-bracket, yaw
    motor down the limb) ending in our limb box: legs RS06 pitch/roll/knee + RS02 yaw + a shin box with two
    RS00 driving a 2-rod ankle (pitch + roll); arms all RS00 (the K1's arms are 14 N.m) to the elbow and a
    ball hand; a 2-DoF RS00 head. One rectangular torso box, no waist."""
    T2 = 2 * L.T
    m06, m02, m00 = MOTORS['rs06'], MOTORS['rs02'], MOTORS['rs00']
    H = max(41.0, m06['D'] / 2 + 12.0)
    ext = lambda prof, plane, w0, t: L.tri(L.to_world(CrossSection(L.rings(prof), FillRule.EvenOdd).extrude(t).translate((0, 0, w0)), plane))
    Tl = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1.0]])
    extra = []
    xh, xs = 0.0, s2h                                          # hip / shoulder pitch axes
    xbot = xh - m06['D'] / 2 - 2.0 - T2                      # floor's bottom face, under the hip pitch motors
    xtop = xs + m00['D'] / 2 + 6.0 + T2                        # bar's top face
    # torso: side walls carry all four pitch motors' outputs; bar, floor, skins
    # pelvis carved around each hip: the roll motor (under the pitch) sweeps a circle about the pitch axis, so
    # the side walls end in an arc round the pitch port and the floor + skins narrow between the hips
    d06_ = module_dims('rs06')
    hip_y_ = hip_y or (d06_['OUT'][1] / 2 - IL_SHIN[0] + 7.0 + L.T + 9.0 + 2 * L.T + 4.0)
    xr_ = xh - m06['D'] / 2 - 2.0 - T2 - 3.0 - m06['D'] / 2
    r_cut = abs(xr_ - xh) - m06['D'] / 2 - 4.0                 # nearest the roll motor comes to the pitch axis, less 4
    y_cut = hip_y_ - m06['D'] / 2 - 4.0                        # inboard edge of the roll motor's sweep
    side = L.rounded(xh, -H, xtop - T2, H, 8).union(L.circle(xh, 0, 2 * r_cut))
    side = pocket(side, [(xh, 0, m06['out'][0] / 2 + 12), (xs, 0, m00['out'][0] / 2 + 12)], rim=10.0, struts=warren((xh + 30, 0), (xs - 30, 0), H - 10, 3))
    for x_, m_ in ((xh, m06), (xs, m00)):
        side = side.difference(L.holes_on(m_['out'][0], m_['out'][1], m_['hole'], at=(x_, 0))).difference(L.circle(x_, 0, m_['out'][0] + 11))
    extra += [('chassis-side.l', ext(side, 'xz', c - T2, T2)), ('chassis-side.r', ext(side, 'xz', -c, T2))]
    for nm, x_ in (('chassis-bar', xtop - T2), ('chassis-floor', xbot)):
        w = L.rounded(-c, -H, c, H, 8) if nm == 'chassis-bar' else L.rounded(-y_cut, -H, y_cut, H, 8)
        w = pocket(w, [(0, 0, 30)] if nm == 'chassis-bar' else [], rim=9.0, struts=warren((-w.bounds[2] + 10, 0), (w.bounds[2] - 10, 0), H - 9, 2))
        if nm == 'chassis-bar':
            w = w.difference(L.holes_on(m00['back'][0], m00['back'][1], m00['hole'], phase=45))
        extra.append((nm, ext(w, 'yz', x_, T2)))
    skin = L.unary_union([L.rounded(xh, -c, xtop, c, 10), L.rounded(xbot, -y_cut, xh + 20, y_cut, 10)])
    skin = pocket(skin, [], rim=12.0, rib=9.0, struts=warren(((xbot + xtop) / 2, -c + 12), ((xbot + xtop) / 2, c - 12), (xtop - xbot) / 2 - 14, 2))
    extra += [('chassis-front', ext(skin, 'xy', H, L.T)), ('chassis-back', ext(skin, 'xy', -H - L.T, L.T))]
    extra.append(('int-battery', L.tri(Manifold.cube((xs - xh - m06['D'] / 2 - m00['D'] / 2 - 16, 2 * c - 24, 2 * H - 24))
                                      .translate((xh + m06['D'] / 2 + 8, -c + 12, -H + 12)))))
    # legs: stance solved with the chain's own hip-to-knee length; pitch at the torso, knee in the box
    d06 = module_dims('rs06')
    # hips K1-style: the roll motor under the pitch, the legs as close as their inboard ankle rods allow
    hip_y = hip_y or (d06['OUT'][1] / 2 - IL_SHIN[0] + 7.0 + L.T + 9.0 + 2 * L.T + 4.0)
    drop = m06['D'] / 2 + 2.0 + T2 + 3.0 + m06['D'] / 2         # pitch axis -> roll axis
    hip_knee = drop + m06['D'] / 2 + 4.0 + m02['L'] + 1.0 + L.T + (m06['D'] + 7.0) / 2 + leg_lt
    th, rel = stance((hip_knee + shin) * math.cos(math.radians(knee_bend / 2)), hip_knee, shin)
    lp = leg_pose or (th, rel)
    for tag, s in (('ll', 1), ('lr', -1)):
        best = None
        for sg in (1, -1):                                     # pick the knee-forward, foot-level solution
            for sa in (1, -1):
                try:
                    apart = inline_parts(0.0, sg * lp[1], end='ankle', LT=leg_lt, LS=shin, mot='rs06', end_mot='rs00', dual=True,
                                         ank=(sa * (-sg * lp[0]) + ankle[0], ankle[1]))
                except ValueError:                             # foot can't stay level at this hip pitch: ankle neutral
                    apart = inline_parts(0.0, sg * lp[1], end='ankle', LT=leg_lt, LS=shin, mot='rs06', end_mot='rs00', dual=True,
                                         ank=(-sg * lp[1] * 0 + ankle[0] - (sg * lp[1]) * 0, ankle[1])) if False else None
                    if apart is None:
                        try:
                            apart = inline_parts(0.0, sg * lp[1], end='ankle', LT=leg_lt, LS=shin, mot='rs06', end_mot='rs00', dual=True,
                                                 ank=(-sg * lp[1], ankle[1]))
                        except ValueError:
                            continue
                for n_ in [n_ for n_ in apart if n_ == 'yoke' or n_.endswith(('-abad', '-hip'))]:
                    del apart[n_]
                parts_, info = pry_limb(tag, s, xh, c, m06, m06, m02, apart, m06['D'] + 7.0, d06['OUT'][1], pose=(-sg * lp[0], 0.0, 0.0), ext=ext, Tl=Tl, H=H,
                                        roll_below=hip_y)
                pm = dict(parts_)
                knee_z = pm[f'rs06-knee.{tag}'].centroid[2]
                foot = pm[f'foot.{tag}']
                tilt = np.ptp(foot.vertices[foot.vertices[:, 0] < foot.vertices[:, 0].min() + 2, 2]) if True else 0
                sole = foot.vertices[:, 0].min()
                flat = np.ptp(foot.vertices[np.abs(foot.vertices[:, 0] - sole) < 3][:, 0]) if True else 0
                score = (knee_z > 0, -abs(foot.centroid[2]) , -foot.extents[0])
                if best is None or score > best[0]:
                    best = (score, parts_)
        extra += best[1]
    # arms: all RS00 to the elbow, ball hand (paw); elbow forward
    d00 = module_dims('rs00')
    for tag, s in (('al', 1), ('ar', -1)):
        best = None
        for sg in (1, -1):
            apart = inline_parts(0.0, sg * elbow, LT=arm_lt, LS=forearm, mot='rs00')
            for n_ in [n_ for n_ in apart if n_ == 'yoke' or n_.endswith(('-abad', '-hip'))]:
                del apart[n_]
            parts_, info = pry_limb(tag, s, xs, c, m00, m00, m00, apart, m00['D'] + 7.0, d00['OUT'][1], pose=arm_pose or (0.0, 0.0, 0.0), ext=ext, Tl=Tl, H=H,
                                    gap=max((d00['OUT'][1] / 2 - IL_SHIN[0] + 6.0) - 1.0 - L.T - m00['D'] / 2,
                                            (hip_y + d06['OUT'][1] / 2 + 6.0 - c) + (d00['OUT'][1] / 2 - IL_SHIN[0]) - 1.0 - L.T - m00['D'] / 2))   # hang outside the legs
            pz = dict(parts_)[f'paw.{tag}'].centroid[2]
            if best is None or pz > best[0]:
                best = (pz, parts_)
        extra += best[1]
    # head: neck yaw RS00 standing on the bar, a pitch RS00 between two plates, the shell
    xn = xtop
    extra.append(('rs00-neckyaw', trimesh.creation.cylinder(radius=m00['D'] / 2, segment=[[xn, 0, 0], [xn + m00['L'], 0, 0]], sections=64)))
    xb = xn + m00['L'] + 1.0
    extra.append(('chassis-neck', ext(L.rounded(-m00['L'] / 2 - 1 - L.T, -24, m00['L'] / 2 + 1 + L.T, 24, 6).difference(L.holes_on(m00['out'][0], m00['out'][1], m00['hole'])), 'yz', xb, L.T)))
    xhp = xb + L.T + 8.0 + m00['D'] / 2
    ya_, yb_ = m00['L'] / 2 + 1.0, -(m00['L'] / 2 + 1.0)
    pa = L.stadium([(xb + L.T + 20, 0, 40), (xhp, 0, m00['D'] + 7)]).difference(L.holes_on(m00['back'][0], m00['back'][1], m00['hole'], phase=45, at=(xhp, 0)))
    extra.append(('chassis-neck-plate', ext(pa, 'xz', yb_ - L.T, L.T)))
    extra.append(('rs00-neckpitch', trimesh.creation.cylinder(radius=m00['D'] / 2, segment=[[xhp, yb_ + 1.0, 0], [xhp, ya_ - 1.0, 0]], sections=64)))
    hh = head_h
    hw, dep = 0.33 * hh, 0.8 * hh
    x0, x1 = xb + L.T + 6.0, xb + L.T + 6.0 + hh
    zf, zb = 0.55 * dep, -0.45 * dep
    hs = L.unary_union([L.circle(xhp, 0, m00['D'] + 10), L.rounded(x0, zb + L.T + 1, x1 - L.T - 1, zf - 1, 10)])
    hs = pocket(hs, [(xhp, 0, m00['D'] / 2 + 8)], rim=8.0, struts=[((xhp, 0), (x1 - 12, zf - 12)), ((xhp, 0), (x1 - 12, zb + 12))])
    extra.append(('head-side.r', ext(hs.difference(L.holes_on(m00['out'][0], m00['out'][1], m00['hole'], at=(xhp, 0))), 'xz', ya_, L.T)))
    extra.append(('head-side.l', ext(hs.difference(L.circle(xhp, 0, 12)), 'xz', yb_ - 2 * L.T - 1.0, L.T)))
    ex_ = x0 + 0.55 * hh
    face = L.rounded(x0, -hw, x1, hw, 0.18 * hh).difference(L.circle(ex_, -0.42 * hw, 0.16 * hh)).difference(L.circle(ex_, 0.42 * hw, 0.16 * hh))
    extra += [('head-face', ext(face, 'xy', zf, L.T)), ('head-back', ext(L.rounded(x0, -hw, x1, hw, 0.18 * hh), 'xy', zb - L.T, L.T)),
              ('head-crown', ext(L.rounded(-hw, zb, hw, zf, 0.1 * hh), 'yz', x1, L.T))]
    cheek = pocket(L.rounded(x0, zb, x1, zf, 0.15 * hh), [], rim=9.0, struts=warren((x0 + 10, (zb + zf) / 2), (x1 - 10, (zb + zf) / 2), (zf - zb) / 2 - 10, 2))
    extra += [('head-cheek.r', ext(cheek, 'xz', hw - L.T, L.T)), ('head-cheek.l', ext(cheek, 'xz', -hw, L.T))]
    extra.append(('int-camera', L.tri(Manifold.cube((26, ya_ - (yb_ - L.T) - 4, 22)).translate((ex_ - 13, yb_ - L.T + 2, zf - 22 - 1)))))
    count = {'rs06': 6 + 2, 'rs02': 2, 'rs00': 4 + 8 + 2}       # legs: pitch, roll, knee x2 + ... see note
    count = {'rs06': 6, 'rs02': 2, 'rs00': 4 + 8 + 2}
    kg = sum(MOTORS[k_]['kg'] * n_ for k_, n_ in count.items())
    import design as ID
    extra += ID.design(extra)
    extra[:] = [(n, m) for n, m in extra if not n.startswith('paw.')]     # the mitten replaces the ball paw
    note = (f'mini humanoid (K1 on our stack): 22 DoF ({count["rs06"]} RS06 + {count["rs02"]} RS02 + {count["rs00"]} RS00, motors {kg:.1f} kg); '
            f'hip-knee {hip_knee:.0f} / shin {shin:.0f}; shoulder-to-hip {s2h:.0f}; torso {2 * c:.0f} wide')
    R = trimesh.transformations.rotation_matrix(np.radians(-90), [0, 1, 0])
    return dict(hip_x=0.0, plane=0.0, legs=[], body=None, walls=[], extra=extra, note=note, posed={}, only=True, world=R, ground=True)


COVER_SEGMENTS = (
    # segment, name patterns (regex on the part's base name); first match wins
    ('foot', r'^(foot|pivot-ankle)$'),
    ('shin', r'^(shin|rs0\d-(ankle|wrist)(-b)?|crank-(ankle|wrist)(-b)?|link-(ankle|wrist)(-b)?|pivot-wrist)$'),
    ('thigh', r'^(thigh-(inner|outer)|rs0\d-(knee|hip)|chassis-(cap|arm-cap)|yoke|rs0\d-abad)$'),
    ('head', r'^(head-(face|back|crown|cheek|side)|rs00-neckpitch|chassis-neck-plate)$'),
    ('_joint', r'^(chassis-(pitchplate|rollmount|rollplate|yawmount|hipflange|rollplate|shoulderflange|shoulderplate|shoulder-(pitchplate|rollmount|rollplate|yawmount))|rs0\d-(.pitch-out|.roll|.roll-out|.yaw|.yaw-out|hipyaw|hipyaw-out|shoulderyaw|shoulderyaw-out|armyaw|armyaw-out|shoulderroll|shoulderroll-out|shoulderpitch-out))$'),
)
TORSO = r'^(chassis-(side|bar|floor|front|back|chest-.*|pelvis-.*|shoulder-bar|shoulder-floor|shoulder-front|shoulder-back|neck-post)|int-battery|rs0\d-(waist|waist-out|.pitch|shoulderpitch))$'


def add_covers(extra, margin=4.0):
    """cover shells for a finished look: one smooth hull per limb segment (and the torso), a few mm proud of the
    structure. Real covers would be printed/thermoformed shells clipped to the plates; here they are hulls."""
    import re
    from manifold3d import Manifold
    groups = {}
    for n, m in extra:
        base, _, tag = n.partition('.')
        if base.startswith(('cover', 'paw', 'hand', 'finger')):
            continue
        seg = None
        if re.match(TORSO, base) and not tag:
            seg = 'torso'
        elif re.match(TORSO, base) and tag and base.endswith(('pitch', 'shoulderpitch')):
            seg = 'torso'
        else:
            for s_, pat in COVER_SEGMENTS:
                if re.match(pat, base):
                    seg = s_
                    break
        if seg is None or seg.startswith('_'):                  # joints stay bare: dark motors as accents
            continue
        key = f'cover-{seg}' + (f'.{tag}' if tag and seg not in ('torso', 'head') else '')
        groups.setdefault(key, []).append(np.asarray(m.vertices))
    offs = np.array([[0, 0, 0]] + [[sx * margin, sy * margin, sz * margin] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
    out = []
    for key, vs in groups.items():
        P = np.concatenate(vs)
        P = (P[:, None, :] + offs[None, :, :]).reshape(-1, 3)
        h = Manifold.hull_points(P.astype(np.float32).tolist() if False else P.astype(np.float32))
        try:
            h = h.smooth_out(60.0).refine(3)
        except Exception:
            pass
        g = h.to_mesh()
        out.append((key, trimesh.Trimesh(np.asarray(g.vert_properties)[:, :3], np.asarray(g.tri_verts), process=False)))
    extra += out
    return extra


def humanoid_mini():
    return humanoid_k1()


def pry_limb(tag, s, xp, c, mp, mr, my, apart, W_, out1, pose=(0.0, 0.0, 0.0), ext=None, Tl=None, H=41.0, gap=2.0, roll_below=None):
    """one limb, pitch-roll-yaw (the shoulder chain, used for hips too): the PITCH motor inside the torso
    corner (stator toward the centre, output out through the side wall at y = c), an L-bracket holding the
    ROLL motor (axis fore-aft) beside the torso, an L-bracket under it hanging the YAW motor (axis down the
    limb), whose output caps the limb module's box. Returns [(name, mesh)] in the body frame (x up, y out, z fore)."""
    T2 = 2 * L.T
    P_, R_, Y_ = pose
    xr = xp
    yr = c + 1.0 + L.T + gap + mr['D'] / 2                     # roll axis; `gap` sets how far the limb hangs off the torso side
    if roll_below is not None:                                  # K1-style hip: roll motor UNDER the pitch, at y = roll_below
        xr = xp - mp['D'] / 2 - 2.0 - T2 - 3.0 - mr['D'] / 2    # just under the torso floor
        yr = roll_below
    xc = xr - mr['D'] / 2 - 4.0                                 # yaw-mount plate, under the roll motor
    x_cap = xc - my['L'] - 1.0                                  # yaw output face = top of the limb box
    y0 = yr - out1 / 2                                          # limb box centred on the yaw axis
    rot = lambda deg, axis, pt: trimesh.transformations.rotation_matrix(np.radians(deg), axis, pt)
    cylx = lambda r, x0, x1, y, z=0.0: trimesh.creation.cylinder(radius=r, segment=[[x0, y, z], [x1, y, z]], sections=64)
    cyly = lambda r, y0_, y1_, x, z=0.0: trimesh.creation.cylinder(radius=r, segment=[[x, y0_, z], [x, y1_, z]], sections=64)
    cylz = lambda r, z0, z1, x, y: trimesh.creation.cylinder(radius=r, segment=[[x, y, z0], [x, y, z1]], sections=64)
    out = []
    Mp = rot(P_, [0, 1, 0], [xp, 0, 0])
    Mr = Mp @ rot(s * R_, [0, 0, 1], [xr, s * yr, 0])
    My = Mr @ rot(s * Y_, [1, 0, 0], [0, s * yr, 0])
    add = lambda n_, m_, M_: out.append((f'{n_}.{tag}', m_.apply_transform(M_)))
    zb, zf = -mr['L'] / 2 - 1.0, mr['L'] / 2 + 1.0
    y_in, y_out = c - T2 - 1.0 - mp['L'], c - T2 - 1.0
    out.append((f'{mp_name(mp)}-{tag[0]}pitch.{tag}', cyly(mp['D'] / 2, s * y_in, s * y_out, xp)))
    add(f'{mp_name(mp)}-{tag[0]}pitch-out', cyly(mp['out'][0] / 2 + 4, s * y_out, s * (c + 1.0), xp), Mp)
    b1_lo = (xr + mr['D'] / 2 + 2.0) if roll_below is not None else (xp - mr['D'] / 2 - 2)   # stop above a roll motor underneath
    b1 = L.unary_union([L.circle(xp, 0, mp['out'][0] + 33), L.rounded(b1_lo, zb, xp + 18, 18, 6)]) \
        .difference(L.holes_on(mp['out'][0], mp['out'][1], mp['hole'], at=(xp, 0)))
    add('chassis-pitchplate', ext(b1, 'xz', (c + 1.0) if s > 0 else -(c + 1.0) - L.T, L.T), Mp)
    if roll_below is not None:     # back plate: from the pitch plate's lower edge down and in to the roll stator
        y_side = c + 1.0 + L.T                                   # outside the torso's side wall
        x_floor = xr + mr['D'] / 2 + 2.0                          # just under the torso floor
        vs = L.rounded(x_floor - 20, min(s * y_side, s * (y_side + 26)), xp + 18, max(s * y_side, s * (y_side + 26)), 6)   # up beside the torso
        hs = L.rounded(xr - 12, min(s * yr, s * (y_side + 26)), x_floor, max(s * yr, s * (y_side + 26)), 6)              # under it, over the roll motor
        b2 = L.unary_union([L.circle(xr, s * yr, mr['back'][0] + 16), vs, hs]).intersection(
            L.box(-1e4, -1e4, x_floor, 1e4).union(L.box(-1e4, s * y_side if s > 0 else -1e4, 1e4, 1e4 if s > 0 else s * y_side)))
    else:
        b2 = L.unary_union([L.circle(xr, s * yr, mr['back'][0] + 16), L.rounded(xr - mr['D'] / 2 - 2, min(s * (c + 1.0 + L.T), s * yr), xr + 18, max(s * (c + 1.0 + L.T), s * yr), 6)])
    b2 = b2.difference(L.holes_on(mr['back'][0], mr['back'][1], mr['hole'], phase=45, at=(xr, s * yr)))
    add('chassis-rollmount', ext(b2, 'xy', zb - L.T, L.T), Mp)
    add(f'{mp_name(mr)}-{tag[0]}roll', cylz(mr['D'] / 2, -mr['L'] / 2, mr['L'] / 2, xr, s * yr), Mp)
    add(f'{mp_name(mr)}-{tag[0]}roll-out', cylz(mr['out'][0] / 2 + 4, mr['L'] / 2, zf, xr, s * yr), Mr)
    c1 = L.unary_union([L.circle(xr, s * yr, mr['out'][0] + 33), L.rounded(xc + L.T, s * yr - 26, xr, s * yr + 26, 6)]) \
        .difference(L.holes_on(mr['out'][0], mr['out'][1], mr['hole'], at=(xr, s * yr)))
    add('chassis-rollplate', ext(c1, 'xy', zf, L.T), Mr)
    c2 = L.unary_union([L.circle(s * yr, 0, my['back'][0] + 16), L.rounded(s * yr - 26, -10, s * yr + 26, zf, 6)]) \
        .difference(L.holes_on(my['back'][0], my['back'][1], my['hole'], phase=45, at=(s * yr, 0)))
    add('chassis-yawmount', ext(c2, 'yz', xc, L.T), Mr)
    add(f'{mp_name(my)}-{tag[0]}yaw', cylx(my['D'] / 2, xc - my['L'], xc, s * yr), Mr)
    add(f'{mp_name(my)}-{tag[0]}yaw-out', cylx(my['out'][0] / 2 + 4, x_cap, xc - my['L'], s * yr), My)
    ylo, yhi = sorted((s * (y0 + HY_IN[0]), s * (y0 + out1)))
    cap = L.rounded(ylo, -0.8 * W_ / 2, yhi, 0.8 * W_ / 2, 8).difference(L.holes_on(my['out'][0], my['out'][1], my['hole'], at=(s * yr, 0)))
    add('chassis-cap', ext(cap, 'yz', x_cap - L.T, L.T), My)
    place = My @ trimesh.transformations.translation_matrix([x_cap - L.T - W_ / 2, s * y0, 0]) @ np.diag([1, s, 1, 1.0]) @ Tl
    for n_, g_ in apart.items():
        out.append((f'{n_}.{tag}', g_.copy().apply_transform(place)))
    return out, dict(yr=yr, y0=y0, x_top_box=x_cap - L.T, x_origin=x_cap - L.T - W_ / 2)


def mp_name(m):
    return next(k for k, v in MOTORS.items() if v is m)


def humanoid_tiny():
    """TINY humanoid, ~56 cm: the largest all-RS00 humanoid once its plates are weighed (10.2 kg est.; walking
    knee 94% of 0.7 x 14 N.m). The 70 cm version was 134% -- shrinking barely lightens it (the motors and
    their rings are fixed), so the size came down to x0.72."""
    return humanoid_family('tiny humanoid', leg=(122.0, 158.0), arm=(108.0, 122.0), chest_h=62.0, rect=True, head='fixed', head_h=75.0, covers=True)


def humanoid_mini_v1():
    """MINI humanoid, a full Booster K1 replacement (~95 cm, K1's 22 DoF + a waist = 23): RS04 hip pitch + knee
    (an RS06 knee walks at ~120% of 0.7 x peak once the plates are weighed), RS06 hip roll, ankles + waist,
    RS02 arms + hip yaw, RS00 wrists + head; ankle pitch AND roll from two shin motors and two pushrods. Lengths scaled 1.36x from the tiny; knee sized like the tiny (walking ~0.7 x RS06 peak)."""
    return humanoid_family('mini humanoid', leg=(210.0, 300.0), arm=(80.0, 220.0), chest_h=60.0,
                           legm='rs06', armm='rs06', yawm='rs02', waistm='rs06', wristm='rs00', ankle_roll=True, head='rs00',
                           legpk='rs04', rect=True, arm_chain=True, arm_yawm='rs02', arm_pose=(0.0, 60.0), head_h=145.0)


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
                mirrored = flip == 'all' or (flip == 'yoke' and n in YOKE_SIDE)       # 'none': as drawn
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
    if arch.get('cover_posed') and 'posed' in arch:          # covers for posed (instanced) limbs, in the body frame
        placed = []
        for node in list(scene.graph.nodes_geometry):
            if '.' not in node or node.endswith('.proto'):
                continue
            T_, g_ = scene.graph[node]
            placed.append((node.replace('~', '.').split('.')[0] + '.' + node.split('.')[-1], scene.geometry[g_].copy().apply_transform(T_)))
        n0 = len(placed)
        for n, m in add_covers(placed)[n0:]:
            if n.endswith(('.al', '.ar')):                     # the instanced arms only; the rest come with `extra`
                scene.add_geometry(m, geom_name=n, node_name=n)
    if arch.get('body') is not None:
        scene.add_geometry(arch['body'], geom_name='chassis-body', node_name='chassis-body')
    for n, m in arch.get('extra', []):
        scene.add_geometry(m, geom_name=n, node_name=n)
    for i, (w, at) in enumerate(arch['walls']):
        scene.add_geometry(w, geom_name=f'chassis-wall-{i}', node_name=f'chassis-wall-{i}',
                           transform=trimesh.transformations.translation_matrix(at))
    if 'world' in arch:
        scene.apply_transform(arch['world'])
    if arch.get('ground'):
        scene.apply_translation([0, 0, -scene.bounds[0][2]])
    return scene


def main():
    for name, fn in (('inline', inline), ('humanoid_tiny', humanoid_tiny), ('humanoid_mini', humanoid_mini)):          # the mini quad, the mini humanoid
        arch = fn()
        out = os.path.join(A, f'quad-{name}.glb')
        build(arch).export(out)
        print(f'{name}: {arch["note"]} -> {out} ({os.path.getsize(out) / 1e6:.1f} MB)')


if __name__ == '__main__':
    main()
