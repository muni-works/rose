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
    return prof.difference(win) if not win.is_empty else prof


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



def ankle2_geom(LS, DM, CR, e, beta=-12.0, phi=180.0, h=25.0, gap=9.0, cgap=7.0):
    """the two-motor ankle in the shin frame (x fore, y out, z up the shin; knee at the origin): a shin BOX of
    two plates with both ankle motors as spacers, facing opposite ways (the thigh-box idiom). Motor A (upper)
    drives a crank + rod OUTSIDE the outer plate, motor B (lower) a crank + rod INSIDE the inner plate."""
    m0 = IL_SHIN[1] + 1.0
    DM1 = DM
    DM2 = DM + e['D'] + 2.0
    y_os = m0 + e['L'] + 1.0                                 # outer shin plate
    yA = y_os + L.T + cgap                                   # crank A layer (also the foot's post), on a spacer: posts roll clear of the box
    yRA = yA + L.T + gap                                     # rod A layer (2T); `gap` lets the rod lean as the foot rolls
    yB = IL_SHIN[0] - cgap - L.T                             # crank B layer (foot's inner post)
    yRB = yB - gap - 2 * L.T                                 # rod B layer
    yc = (yB + yA + L.T) / 2                                 # roll axis, centred in the U
    # cranks and foot lobes both sit `beta` off square at neutral, so the K1 pitch range (-50..20) is centred
    # on the crank's best transmission instead of starting at one end of it
    return dict(m0=m0, DM1=DM1, DM2=DM2, y_os=y_os, yA=yA, yRA=yRA, yB=yB, yRB=yRB, yc=yc,
                rods=((DM1, yRA + L.T), (DM2, yRB + L.T)), CR=CR, LS=LS, beta=beta, phi=phi, h=h,
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
        CR = 0.8 * e['D']                                    # long cranks: less crank swing per degree of roll (swept: 0.8 D, beta -12)
        G = ankle2_geom(LS, DM, CR, e)
    else:                                                    # wrist: pitch +/-45 x yaw +/-20 (swept: 1.0 D, beta -5)
        CR = 1.0 * e['D']
        G = ankle2_geom(LS, DM, CR, e, beta=-5.0)
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
    inner = L.stadium([(0, 0, hub_d + 4), (0, -DM1, W), (0, -DM2, W), (0, -LS, 30 * k)])
    st = warren((0, -rh), (0, -DM1 + rm), e['D'] / 2 - 6, 2) + warren((0, -DM2 - rm), (0, -LS + rp), 8 * k, 2)
    inner = pocket(inner, [(0, 0, rh), (0, -DM1, rm), (0, -DM2, rm), (0, -LS, rp)], rim=7.0, struts=st + [((0, -DM1), (0, -DM2))])
    inner = inner.difference(L.holes_on(m['out'][0], m['out'][1], m['hole'])).difference(bk(-DM1)).difference(pas(-DM2)).difference(L.circle(0, -LS, 8.4 * k))
    outer = L.stadium([(0, -DM1, W), (0, -DM2, W), (0, -LS, 30 * k)])
    outer = pocket(outer, [(0, -DM1, rm), (0, -DM2, rm), (0, -LS, rp)], rim=7.0, struts=[((0, -DM1), (0, -DM2))] + warren((0, -DM2 - rm), (0, -LS + rp), 8 * k, 2))
    outer = outer.difference(pas(-DM1)).difference(bk(-DM2)).difference(L.circle(0, -LS, 8.4 * k))
    hub = L.circle(0, 0, hub_d)
    parts['shin'] = trimesh.util.concatenate([xz(inner, IL_SHIN[0], 2 * L.T), xz(hub, IL_SHIN[1], HY_MOT[0] - IL_SHIN[1]),
                                              xz(outer, y_os, L.T)]).apply_transform(shin_T)
    oring = e['out'][0] / 2 + 4
    parts[f'{en}-{end}'] = trimesh.util.concatenate([cyl(e['D'] / 2, m0, m0 + e['L'], -DM1), cyl(oring, m0 + e['L'], yA, -DM1)]).apply_transform(shin_T)
    parts[f'{en}-{end}-b'] = trimesh.util.concatenate([cyl(e['D'] / 2, m0, m0 + e['L'], -DM2), cyl(oring, yB + L.T, m0, -DM2)]).apply_transform(shin_T)
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
        trimesh.creation.box(extents=[2 * sb, y_os - IL_SHIN[1] - 2.0, 2 * sb]).apply_translation([0, (y_os + IL_SHIN[1]) / 2, -LS - G['h'] / 2])]).apply_transform(shin_T)
    fz, fl_ = (46 * k, 0.6 * LS) if end == 'ankle' else (30 * k, 0.35 * LS)
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
    if end != 'ankle':
        # hand: the same U-joint and posts on a short palm, with a paddle (tool plate) carrying on along the forearm
        hl = 70 * k
        paddle = L.unary_union([L.rounded(-0.2 * fl_, -fz - hl, 0.45 * fl_, -fz, 10)]).difference(L.rounded(0, -fz - hl + 14, 0.3 * fl_, -fz - 14, 6))
        body_.append(xz(paddle, yc - L.T, 2 * L.T))
    foot = trimesh.util.concatenate(body_)
    Rx = trimesh.transformations.rotation_matrix(np.radians(ank[1]), [1, 0, 0], [0, yc, -hh])
    parts['foot' if end == 'ankle' else 'hand'] = foot.apply_transform(shin_T @ trimesh.transformations.translation_matrix([0, 0, -LS]) @ ry(a) @ Rx)
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
                    arm_chain=False, arm_yawm='rs02', arm_drop=30.0):
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
        mid = MOTORS[head] if head else mw
        ya = max(mid['D'] / 2 + 5.0 + ma['D'] / 2, hp - 20.0 * ka)
        y0a = ya - da['ABAD']
        chest = ya + 20.0 * ka
        hp = chest
        T2_ = 2 * L.T
        s2h0 = (-T2_ - da['IPOUT']) - ((-T2_ - 1.0 - mw['L'] - 1.0) - T2_ - 1.0 - T2_ - 1.0 - my['L'] - 1.0 - T2_ - 1.0 - L.T
                                       - ((MOTORS[legpk]['D'] + 7.0) / 2 + 8.0))
        reach = arm[0] + (ma['D'] + 7.0) / 2                   # shoulder axis to the bottom of the elbow: only the elbows
                                                              # must clear the thighs; forearms work in front of them
        chest_h = max(chest_h, reach + (MOTORS[legpk]['D'] + 7.0) / 2 + 20.0 - s2h0)
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
    neck_port = [(0, MOTORS[head]['front'], 0, MOTORS[head]['out'][0] + 11, MOTORS[head]['hole'])] if head else []
    if arm_chain:
        # arms = the leg chain, hung from the shoulders: a yaw motor as a spacer in a shoulder box at each end
        # of the bar (the pelvis idiom), its flange + roll plate, then the limb module (ab/ad = shoulder roll)
        mya = MOTORS[arm_yawm]
        ys = chest + math.hypot(ma['D'] / 2, ma['L'] / 2) + 4.0      # yaw = roll axis, the roll motor yaws outside the chest
        y0a = ys - da['ABAD']
        hs = ys + mya['D'] / 2 + 6.0                                  # bar half-width: over the shoulder boxes
        bar = wallp(hs, neck_port)
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
        extra.append((f'int-battery.{"l" if s > 0 else "r"}', L.tri(Manifold.cube((0.6 * chest_h + 20, 0.4 * chest, 2 * bw * 0.85))
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
    if head:
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
    poses = {'none': inline_parts(*arm_pose, end='wrist', LT=arm[0], LS=arm[1], mot=armm, end_mot=wristm)}
    if arm_chain:
        xsb = xt - T2 - 1.0 - mya['L'] - 1.0                  # shoulder floor's top face (yaw motor between bar and floor)
        xso = xsb - T2 - 1.0                                   # yaw output face
        sh_x = xso - L.T - ((ma['D'] + 7.0) / 2 + 8.0) - arm_drop   # shoulder pitch axis; the drop buys abduction
        zra = da['IPOUT']
        rfa = mya['out'][0] + 33
        rla = ma['out'][0] + 33
        apart = inline_parts(*arm_pose, end='wrist', LT=arm[0], LS=arm[1], mot=armm, ab_mot=armm, end_mot=wristm, dual=True)
        for tag, s in (('al', 1), ('ar', -1)):
            # shoulder box: the bar above, a floor below (front flange of the yaw motor), skins fore and aft
            fl0 = L.unary_union([L.circle(ys, 0, mya['front'][0] + 16), L.rounded(chest - 20.0, -0.6 * H, ys, 0.6 * H, 8)]).difference(L.holes_on(mya['front'][0], mya['front'][1], mya['hole'], at=(ys, 0))).difference(L.circle(ys, 0, mya['out'][0] + 11))
            if s < 0:
                fl0 = shapely_mirror(fl0)
            extra.append((f'chassis-shoulder-floor.{tag}', ext(fl0, 'yz', xsb - T2, T2)))
            sk = L.rounded(xsb - T2, chest + 0.5, xt, hs, 6)
            if s < 0:
                sk = shapely_mirror(sk)
            extra += [(f'chassis-shoulder-front.{tag}', ext(sk, 'xy', H, L.T)), (f'chassis-shoulder-back.{tag}', ext(sk, 'xy', -H - L.T, L.T))]
            extra.append((f'{arm_yawm}-shoulderyaw.{tag}', cyl(mya['D'] / 2, xsb + 1.0, xsb + 1.0 + mya['L'], s * ys)))
            extra.append((f'{arm_yawm}-shoulderyaw-out.{tag}', cyl(mya['out'][0] / 2 + 4, xso, xsb + 1.0, s * ys)))
            fla = L.unary_union([L.circle(s * ys, 0, rfa), L.rounded(s * ys - 12, -10, s * ys + 12, zra + T2, 6)]).difference(L.holes_on(mya['out'][0], mya['out'][1], mya['hole'], at=(s * ys, 0)))
            extra.append((f'chassis-shoulderflange.{tag}', ext(fla, 'yz', xso - L.T, L.T)))
            rpa = L.unary_union([L.circle(sh_x, s * ys, rla), L.rounded(sh_x, s * ys - 12, xso - L.T, s * ys + 12, 6)])
            rpa = rpa.difference(L.holes_on(ma['out'][0], ma['out'][1], ma['hole'], phase=90, at=(sh_x, s * ys))).difference(L.circle(sh_x, s * ys, ma['out'][0] * 0.6))
            extra.append((f'chassis-shoulderplate.{tag}', ext(rpa, 'xy', zra, T2)))
            place = trimesh.transformations.translation_matrix([sh_x, s * y0a, 0]) @ np.diag([1, s, 1, 1.0]) @ Tl
            for n, g in apart.items():
                extra.append((f'{n}.{tag}', g.copy().apply_transform(place)))
        hip_a, legs, poses = sh_x, [], {}
    count = {}
    arms_ = ([armm] * 6 + [wristm] * 4 + [arm_yawm] * 2) if arm_chain else ([armm] * 6 + [wristm or armm] * 2)
    for mm in [legpk] * 4 + [legm] * (6 if ankle_roll else 4) + arms_ + [yawm] * 2 + [waistm] + ([head] * 2 if head else []):
        count[mm] = count.get(mm, 0) + 1
    kg = sum(MOTORS[k_]['kg'] * n_ for k_, n_ in count.items())
    mix = ' + '.join(f'{n_} {k_.upper()}' for k_, n_ in count.items())
    note = (f'{name}: {sum(count.values())} joints ({mix}, motors {kg:.1f} kg); thigh {leg[0]:.0f} / shin {leg[1]:.0f}; '
            f'shoulder-to-hip {hip_a - hip_x:.0f}, shoulders {2 * (y0a + da["OUT"][1]):.0f} wide')
    R = trimesh.transformations.rotation_matrix(np.radians(-90), [0, 1, 0])
    return dict(hip_x=hip_a, plane=y0a, legs=legs, body=None, walls=[], extra=extra, note=note,
                posed=poses, only=True, world=R, ground=True)


def humanoid_tiny():
    """TINY humanoid, ~56 cm: the largest all-RS00 humanoid once its plates are weighed (10.2 kg est.; walking
    knee 94% of 0.7 x 14 N.m). The 70 cm version was 134% -- shrinking barely lightens it (the motors and
    their rings are fixed), so the size came down to x0.72."""
    return humanoid_family('tiny humanoid', leg=(122.0, 158.0), arm=(108.0, 122.0), chest_h=62.0)


def humanoid_mini():
    """MINI humanoid, a full Booster K1 replacement (~95 cm, K1's 22 DoF + a waist = 23): RS04 hip pitch + knee
    (an RS06 knee walks at ~120% of 0.7 x peak once the plates are weighed), RS06 hip roll, ankles + waist,
    RS02 arms + hip yaw, RS00 wrists + head; ankle pitch AND roll from two shin motors and two pushrods. Lengths scaled 1.36x from the tiny; knee sized like the tiny (walking ~0.7 x RS06 peak)."""
    return humanoid_family('mini humanoid', leg=(230.0, 300.0), arm=(205.0, 230.0), chest_h=120.0,
                           legm='rs06', armm='rs06', yawm='rs02', waistm='rs06', wristm='rs00', ankle_roll=True, head='rs00',
                           legpk='rs04', rect=True, arm_chain=True, arm_yawm='rs02', arm_pose=(10.0, 60.0))


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
