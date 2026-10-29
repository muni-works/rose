"""Structure for the K1-layout mini: 3.2 mm router-cut aluminium plates inside the bones, RobStrides bolted
to plate pairs, printed shells over. LEGS first. K1 frame: x fore, y left, z up (mm).

Packaging on the K1's joint axes with our motors:
  hip pitch  RS06, axis y, at the K1 hip-pitch point; stator back on the pelvis side plate (inboard),
             output outboard;
  hip roll   RS06, axis x, on the K1 roll AXIS but moved FORWARD along it, clear of the pitch can
             (the K1 nests a smaller roll 26 mm under its pitch; two RS06 cannot); output faces aft;
  hip yaw    RS02, axis z, at the K1 yaw point, stator up;
  knee       RS06, axis y, at the K1 knee point, stator on the thigh's inner plate, output outboard to
             the shin;
  ankle      2 x RS00 in the shin box (lateral axes), rods to the foot (as the K1 does from its shank).
"""
import math, os
import numpy as np, trimesh
from manifold3d import Manifold, CrossSection, FillRule, OpType
import leg as L
from shapely.geometry import Polygon
import quad as Q
import k1 as K, k1_teardown as KT

T = L.T
G = 1.0                      # assembly gap

# one design, two sizes. R = layout scale relative to the mini (absolute mm that track the body scale by R;
# motor- and plate-driven dims are computed from the motors and don't scale)
VARIANTS = {
    'mini': dict(scale=100.0 / 94.0, big='rs06', yaw='rs02', arm='rs00', ankle='rs00', head='rs00', arm_out=16.0, ankle_f=(0.42, 0.66), head_z=205.0, flat=1.3, shin_gap=0.0, web_x=-20.0, hub_bore=None, elbow_dz=0.0, foot_drop=0.0, pocket=(40.0, 46.0), lighten=dict(rim=7.0, rib=5.0, pitch=28.0, boss=6.0), leg_in=20.0, torso_lean=0.25),
    'tiny': dict(scale=0.75, big='rs00', yaw='rs00', arm='rs00', ankle='rs00', head='rs00', arm_out=30.0, ankle_f=None, head_z=None, flat=1.6, shin_gap=6.4, web_x=None, hub_bore=22.0, elbow_dz=-20.0, foot_drop=14.0, pocket=(44.0, 50.0)),
    # tiny: upper arm 20 mm longer (forearm shorter) so the elbow folds past the RS00 rotation drum; foot 14 mm deeper so the
    # low ankle drum clears the sole in roll
    # tiny: the RS00 knee is as long as the ankle RS00s, so the inner shin plate stands off 6.4 mm inboard of the thigh's
    # full: the 2 m humanoid (Booster T2 / Unitree H2 class) in the mini's design language. RobStride for now (rana
    # in v2): the RS04 (120 N.m) is the largest they make, so the legs are torque-short (hip/knee need ~530 N.m at
    # 2 m, full/SIZING.md) -- this is a packaging/kinematics/look model until the rana actuators exist.
    'full': dict(scale=170.3 / 94.0, big='rs04', yaw='rs04', arm='rs04', ankle='rs04', head='rs02', arm_out=32.0,
                 ankle_f=None, head_z=395.5, flat=1.3, shin_gap=6.4, web_x=None, hub_bore=None, elbow_dz=0.0,
                 foot_drop=0.0, pocket=(68.0, 79.0), plate_t=6.35, waist='rs04', waist_dz=100.0, wrist='rs00', wrist_ujoint=True, ky=0.85, trunk_kx=0.88, leg_in=30.0, torso_lean=0.25),
}
V = dict(VARIANTS['mini'], name='mini')


def use(name):
    """switch every builder (structure + design + rom) to a variant."""
    global T
    V.clear(); V.update(VARIANTS[name], name=name)
    T = V.get('plate_t', L.T)                                  # plate stock: 3.2 mm, 6.35 mm at full size (buckling goes as t^3)


def R():
    return V['scale'] / VARIANTS['mini']['scale']


def BIG():
    return Q.MOTORS[V['big']]


def YAW():
    return Q.MOTORS[V['yaw']]


def ARM():
    return Q.MOTORS[V['arm']]


def ANK():
    return Q.MOTORS[V['ankle']]


def HEAD():
    return Q.MOTORS[V['head']]


def WAIST():
    return Q.MOTORS[V['waist']] if V.get('waist') else None


def waist_layout():
    """the 3-DoF waist (H2/T2-style), stacked on the pelvis deck: yaw (z, stator down), roll (x, output fore),
    pitch (y, output +y). None when the variant has no waist."""
    m = WAIST()
    if m is None:
        return None
    zy = Z_D0() + T + G + m['L'] / 2                                       # yaw stator on the pelvis deck
    zr = zy + m['L'] / 2 + G + T + G + m['D'] / 2                          # roll can on the yaw output plate
    zp = zr + m['D'] / 2 + 12.0 + m['D'] / 2                               # pitch can above the roll (12 mm: the W2/W3 corner)
    return dict(yaw=(np.array([0.0, 0, zy]), np.array([0, 0, 1.0])), roll=(np.array([0.0, 0, zr]), np.array([1.0, 0, 0])),
                pitch=(np.array([0.0, 0, zp]), np.array([0, 1.0, 0])))


def WRIST():
    return Q.MOTORS[V['wrist']] if V.get('wrist') else None


def wrist_layout(J, side, M=None):
    """the 3-DoF wrist (H2/T2 7-DoF arm): roll (forearm axis, mid-forearm, output down), pitch (y) at the wrist,
    yaw (x) under it in the hand. None without a wrist."""
    mw = WRIST()
    if mw is None:
        return None
    if M is None:
        J, M = world()
    U = upper_layout(J, side); s = U['s']; ma = ARM()
    ey = U['ey'][0]
    f_in, f_out = -(ma['L'] / 2 + G) + 3.5, ma['L'] / 2 + G + T + 3.3                 # the forearm's y span (k1_design)
    r_sl = math.hypot(12.0, mw['L'] / 2 + G + T) + 3.0                    # the round sleeve over R0/R1 (k1_design)
    yc = ey[1] + s * (f_in + r_sl)                                         # forearm keeps its inboard face, grows outboard
    wz = ey[2] + hang(J, M, side)[2] * 0.62
    rz = (ey[2] - ma['D'] / 2 + wz) / 2                                    # roll mid-forearm
    yz = wz - mw['D'] / 2 - 12.0 - mw['D'] / 2                              # yaw under the pitch, 12 mm apart (the P0/P1 corner)
    if V.get('wrist_ujoint'):                                              # hands pass 2026-10-28: the yaw can leaves the palm --
        rz = ey[2] - ma['D'] / 2 - G - T - mw['L'] / 2 - 4.0                # the roll goes up under the elbow, and pitch + yaw
        yz = wz                                                            # become one U-joint at the wrist, driven from the forearm
    return dict(s=s, r_sl=r_sl, ujoint=bool(V.get('wrist_ujoint')), roll=(np.array([ey[0], yc, rz]), np.array([0, 0, -1.0])), pitch=(np.array([ey[0], yc, wz]), np.array([0, s * 1.0, 0])),
                yaw=(np.array([ey[0], yc, yz]), np.array([1.0, 0, 0])))


def wrist(J, side, M):
    """wrist motors + plates: F2 (under F1) -> roll -> R0/R1 -> pitch -> P0/P1 -> yaw -> H0 (the hand)."""
    W = wrist_layout(J, side, M)
    if W is None:
        return []
    mw, mn, s = WRIST(), V['wrist'], W['s']
    tag = side[0].lower()
    U = upper_layout(J, side); ey = U['ey'][0]; ma = ARM()
    rc, pc, yc = W['roll'][0], W['pitch'][0], W['yaw'][0]
    out = [(f'{mn}-wrist{k}.{tag}a', motor(W[k][0], W[k][1], mn)) for k in ('roll', 'pitch', 'yaw')]
    y_f1 = abs(ey[1]) + ma['L'] / 2 + G                                   # F1's inner face (|y|)
    z_f2 = rc[2] + mw['L'] / 2 + G
    F2 = L.unary_union([L.circle(rc[0], rc[1], mw['D'] / 2), Rr(rc[0] - 22, rc[1], rc[0] + 22, s * (y_f1 + T))])
    out.append((f'wrist-F2.{tag}a', plate(F2.difference(holes(mw, 'back', (rc[0], rc[1]), 45)), 'xy', z_f2)))
    z_r0 = rc[2] - mw['L'] / 2 - G - T
    if W.get('ujoint'):
        return out[:1] + [out_ for out_ in [(f'wrist-F2.{tag}a', plate(F2.difference(holes(mw, 'back', (rc[0], rc[1]), 45)), 'xy', z_f2))]] + \
            ujoint_wrist(W, mw, s, tag, z_r0, ey + hang(J, M, side))
    yr1 = abs(pc[1]) - mw['L'] / 2 - G - T                                # pitch stator back (inboard)
    R0 = L.unary_union([L.circle(rc[0], rc[1], mw['D'] / 2), Rr(rc[0] - 12, rc[1], rc[0] + 12, s * yr1)])
    out.append((f'wrist-R0.{tag}a', plate(R0.difference(holes(mw, 'out', (rc[0], rc[1]))), 'xy', z_r0)))
    R1 = L.unary_union([L.circle(pc[0], pc[2], mw['back'][0] / 2 + 8), Rr(pc[0] - 12, pc[2], pc[0] + 12, z_r0)])
    out.append((f'wrist-R1.{tag}a', plate(R1.difference(holes(mw, 'back', (pc[0], pc[2]), 45)), 'xz', min(s * yr1, s * (yr1 + T)))))
    yp0 = abs(pc[1]) + mw['L'] / 2 + G                                    # pitch output (outboard)
    x_p1 = yc[0] - mw['L'] / 2 - G - T                                    # yaw stator back (aft)
    z_top = yc[2] + mw['D'] / 2 + G                                       # P0 stays over the yaw can
    P0 = L.unary_union([L.circle(pc[0], pc[2], mw['out'][0] / 2 + 14), Rr(x_p1 + T, z_top, pc[0] + 18, pc[2])])
    out.append((f'wrist-P0.{tag}a', plate(P0.difference(holes(mw, 'out', (pc[0], pc[2]))), 'xz', min(s * yp0, s * (yp0 + T)))))
    P1 = L.unary_union([L.circle(yc[1], yc[2], mw['back'][0] / 2 + 8), Rr(yc[1] - s * 18, yc[2], s * (yp0 + T), z_top + 10.0)])
    out.append((f'wrist-P1.{tag}a', plate(P1.difference(holes(mw, 'back', (yc[1], yc[2]), 45)), 'yz', x_p1)))
    x_h0 = yc[0] + mw['L'] / 2 + G                                        # yaw output (fore): the hand's spine
    tip = ey + hang(J, M, side)
    H0 = L.unary_union([L.circle(yc[1], yc[2], mw['out'][0] / 2 + 14), Rr(yc[1] - 16, yc[2], yc[1] + 16, tip[2] + 40.0 * R())])
    out.append((f'wrist-H0.{tag}a', plate(H0.difference(holes(mw, 'out', (yc[1], yc[2]))), 'yz', x_h0)))
    return out


LA_WRIST = dict(D=20.0, L=70.0, stroke=24.0, N=300.0)      # forearm linear actuators (ASSUMED: a 20 mm micro leadscrew class)
LA_HAND = dict(D=10.0, L=42.0, stroke=10.0, N=60.0)         # palm linear actuators (ASSUMED: the dexterous-hand micro class)
HAND_FX = (28.0, 9.5, -9.5, -28.0)                           # finger centres along x from the palm centre (k1_design)


def _rod(p0, p1, r):
    return trimesh.creation.cylinder(radius=r, segment=[np.asarray(p0, float), np.asarray(p1, float)], sections=32)


def ujoint_wrist(W, mw, s, tag, z_r0, tip):
    """the 7-DoF arm's wrist with nothing in the palm (hands pass 2026-10-28): roll under the elbow -> R0 -> a yoke down to
    one U-joint (pitch y, yaw x, the same centre) -> the hand frame. Pitch and yaw are differential: two linear actuators
    in the rolling forearm, aft of the axis and +-16 in y, push rods to the hand frame -- together they pitch, against
    each other they yaw. The palm carries six more: one per finger (a four-bar curls its three phalanges together) and
    two for the thumb (flex, and the swing across the palm)."""
    rc, wc = W['roll'][0], W['pitch'][0]
    x0, yc, wz = wc[0], wc[1], wc[2]
    out = []
    R0 = L.unary_union([L.circle(rc[0], rc[1], mw['D'] / 2), Rr(rc[0] - 30, rc[1] - 26, rc[0] + 22, rc[1] + 26)])
    out.append((f'wrist-R0.{tag}a', plate(R0.difference(holes(mw, 'out', (rc[0], rc[1]))), 'xy', z_r0)))
    for k, sy in (('a', 1), ('b', -1)):                                    # the yoke: two lugs either side of the cross
        yl = yc + sy * 16.0
        lug = L.unary_union([L.circle(x0, wz, 13.0), Rr(x0 - 13, wz, x0 + 13, z_r0)])
        out.append((f'wrist-R1{k}.{tag}a', plate(lug.difference(L.circle(x0, wz, 3.0)), 'xz', yl if sy > 0 else yl - T, t=T)))
    out.append((f'wrist-P0x.{tag}a', trimesh.creation.box(extents=[18.0, 18.0, 18.0]).apply_translation([x0, yc, wz])))   # the cross
    zla0 = z_r0 - 2.0
    zb = zla0 - LA_WRIST['L']
    for k, sy in (('a', 1), ('b', -1)):                                    # the differential pair, aft of the axis
        c = np.array([x0 - 24.0, yc + sy * 14.0])
        out.append((f'wrist-R1la{k}.{tag}a', _rod([c[0], c[1], zla0], [c[0], c[1], zb], LA_WRIST['D'] / 2)))
        out.append((f'wrist-R1rod{k}.{tag}a', _rod([c[0], c[1], zb], [c[0], c[1], wz - 16.0], 3.0)))
    zh = wz - 24.0                                                         # the hand frame, under the cross
    for k, sx in (('a', 1), ('b', -1)):
        lug = L.unary_union([L.circle(yc, wz, 11.0), Rr(yc - 11, wz, yc + 11, zh)])
        out.append((f'wrist-H0{k}.{tag}a', plate(lug.difference(L.circle(yc, wz, 3.0)), 'yz', x0 + sx * 12.0 if sx > 0 else x0 - 12.0 - 4.0, t=4.0)))
    out.append((f'wrist-H0f.{tag}a', trimesh.creation.box(extents=[40.0, 30.0, 4.0]).apply_translation([x0 - 2.0, yc, zh - 6.0])))
    zt = tip[2] - 50.0; Lh = (wz - 0.0) - zt; zk = wz - 0.50 * Lh             # the knuckle line (k1_design draws the shell)
    palm = Rr(x0 + 4 - 36, zh - 8, x0 + 4 + 32, zk + 8, 10.0)
    out.append((f'wrist-H0palm.{tag}a', plate(palm, 'xz', yc + s * 6.0 if s > 0 else yc + s * 6.0 - 3.0, t=3.0)))
    yla = yc - s * 4.0                                                     # on the palm side of the frame
    for i, fx in enumerate(HAND_FX):
        x = x0 + 4.0 + fx * 0.96
        out.append((f'wrist-H0la{i}.{tag}a', _rod([x, yla, zh - 8.0], [x, yla, zh - 8.0 - LA_HAND['L']], LA_HAND['D'] / 2)))
        out.append((f'wrist-H0link{i}.{tag}a', _rod([x, yla, zh - 8.0 - LA_HAND['L']], [x, yla - s * 2.0, zk + 2.0], 1.6)))   # to the MCP crank
    xt = x0 + 4.0 + 34.0
    out.append((f'wrist-H0la4.{tag}a', _rod([xt, yla - s * 2.0, zh - 6.0], [xt, yla - s * 2.0, zh - 6.0 - 32.0], LA_HAND['D'] / 2)))   # thumb flex
    out.append((f'wrist-H0la5.{tag}a', _rod([xt - 34.0, yla + s * 3.0, zh - 12.0], [xt - 2.0, yla + s * 3.0, zh - 12.0], LA_HAND['D'] / 2)))   # thumb swing
    return out


def Z_CD():
    """the chest's bottom deck: on the waist pitch output when there's a waist, else the one torso deck."""
    W = waist_layout()
    return Z_D0() if W is None else W['pitch'][0][2] + WAIST()['D'] / 2 + G


def waist():
    """waist motors + plates: pelvis deck (on the hip-roll mounts) -> yaw -> W0 -> roll -> W1/W2 -> pitch -> W3 -> chest deck."""
    W = waist_layout()
    if W is None:
        return []
    m, mn = WAIST(), V['waist']
    yc, rc, pc = W['yaw'][0], W['roll'][0], W['pitch'][0]
    out = [(f'{mn}-waist{k}', motor(W[k][0], W[k][1], mn)) for k in ('yaw', 'roll', 'pitch')]
    yh = m['D'] / 2 + 8.0
    r_drum = math.hypot(m['L'] / 2 + G + T + 2.0, m['D'] / 2) + 4.0       # the yaw drum's radius (k1_design): the deck follows its edge
    out.append(('pelvis-deck', plate(L.circle(yc[0], yc[1], 2 * r_drum).difference(holes(m, 'back', (yc[0], yc[1]), 45)), 'xy', Z_D0())))
    z_w0 = yc[2] + m['L'] / 2 + G
    out.append(('waist-W0', plate(L.circle(yc[0], yc[1], yh).difference(holes(m, 'out', (yc[0], yc[1]))), 'xy', z_w0)))
    x_w1 = rc[0] - m['L'] / 2 - G - T                                     # roll stator back (aft), standing on W0
    W1 = L.unary_union([L.circle(rc[1], rc[2], m['back'][0] / 2 + 14), Rr(-yh * 0.7, z_w0 + T, yh * 0.7, rc[2])])
    out.append(('waist-W1', plate(W1.difference(holes(m, 'back', (rc[1], rc[2]), 45)), 'yz', x_w1)))
    x_w2 = rc[0] + m['L'] / 2 + G                                         # roll output (fore), up to the pitch stator plate
    y_w3 = -(m['L'] / 2 + G + T)                                           # pitch stator back (-y)
    z_w3 = rc[2] + m['D'] / 2 + G                                         # W3's lower edge, just over the roll can
    W2 = L.unary_union([L.circle(rc[1], rc[2], m['out'][0] + 30), Rr(y_w3, rc[2], y_w3 + T + 24.0, pc[2] - m['D'] / 2 - G)])
    out.append(('waist-W2', plate(W2.difference(holes(m, 'out', (rc[1], rc[2]))), 'yz', x_w2)))
    W3 = L.unary_union([L.circle(pc[0], pc[2], m['back'][0] / 2 + 14), Rr(pc[0] - 20.0, z_w3, x_w2, pc[2])]).intersection(L.box(-1e3, z_w3, x_w2, 1e3))
    out.append(('waist-W3', plate(W3.difference(holes(m, 'back', (pc[0], pc[2]), 45)), 'xz', y_w3)))
    y_w4 = m['L'] / 2 + G                                                 # pitch output (+y), up to the chest deck
    W4 = L.unary_union([L.circle(pc[0], pc[2], m['out'][0] + 30), Rr(pc[0] - 30.0, pc[2], pc[0] + 30.0, Z_CD())])
    out.append(('waist-W4', plate(W4.difference(holes(m, 'out', (pc[0], pc[2]))), 'xz', y_w4)))
    return out


def plate(profile, plane, w0, t=None, light=True):
    """extrude a shapely profile into a plate: plane 'xz' (normal y), 'yz' (normal x), 'xy' (normal z)."""
    t = T if t is None else t
    if light and V.get('lighten'):
        profile = lighten(profile, **V['lighten'])
    cs = CrossSection(L.rings(profile), FillRule.EvenOdd)
    m = L.to_world(cs.extrude(t).translate((0, 0, w0)), plane)
    return L.tri(m)


def lighten(profile, rim, rib, pitch, boss):
    """router pocketing: a +-45 deg rib lattice inside a rim, solid bosses round every hole. Through-cut (3.2 mm stock)."""
    from shapely.geometry import LineString
    inner = profile.buffer(-rim, join_style=2)
    if inner.is_empty:
        return profile
    polys = list(profile.geoms) if profile.geom_type == 'MultiPolygon' else [profile]
    keep = L.unary_union([Polygon(h).buffer(boss) for p_ in polys for h in p_.interiors] or [Polygon()])
    x0, y0, x1, y1 = inner.bounds
    span = (x1 - x0) + (y1 - y0)
    lines = []
    for c in np.arange(-span, span, pitch):                       # +45 and -45 families through the plate's own box
        lines.append(LineString([(x0 + c - span, y0 - span), (x0 + c + span, y0 + span)]))
        lines.append(LineString([(x0 + c - span, y1 + span), (x0 + c + span, y1 - span)]))
    ribs = L.unary_union(lines).buffer(rib / 2, cap_style=2)
    pockets = inner.difference(ribs).difference(keep)
    geoms = list(pockets.geoms) if hasattr(pockets, 'geoms') else [pockets]
    cut = [g.buffer(-2.0).buffer(2.0) for g in geoms if g.area > 90.0]    # router: 4 mm min radius, no slivers
    cut = [g for g in cut if not g.is_empty]
    return profile.difference(L.unary_union(cut)) if cut else profile


def motor(c, ax, mot):
    m = Q.MOTORS[mot]
    a = np.asarray(ax, float) / np.linalg.norm(ax)
    return trimesh.creation.cylinder(radius=m['D'] / 2, segment=[np.asarray(c) - a * m['L'] / 2, np.asarray(c) + a * m['L'] / 2], sections=64)


def layout(J, side):
    """our joint placements (motor centre, axis, model) for one leg, on the K1's axes."""
    s = 1 if side == 'Left' else -1
    m06, m02, m00 = BIG(), YAW(), ANK()
    hp = J[f'{side}_Hip_Pitch']['p'].copy()
    hr = J[f'{side}_Hip_Roll']['p'].copy()
    hy = J[f'{side}_Hip_Yaw']['p'].copy()
    kn = J[f'{side}_Knee_Pitch']['p'].copy()
    an = J[f'{side}_Ankle_Pitch']['p'].copy()
    # roll: inside the pelvis, axis fore/aft, inboard of the hip line (hidden); pitch: one drum under the pelvis flank;
    # yaw: straight under the pitch, then the thigh.  (the K1 hides its roll in the trunk the same way)
    roll_c = np.array([5.0 * R(), s * (m06['D'] / 2 + 4.0), -30.0 * R()])   # centred in the pelvis block
    hp = np.array([0.0, hp[1], roll_c[2] - m06['D'] / 2 - G - m06['D'] / 2])
    hy = np.array([0.0, hp[1], hp[2] - m06['D'] / 2 - G - T - G - m02['L'] / 2])
    return dict(s=s,
                pitch=(hp, np.array([0, s * 1.0, 0]), V['big']),     # output outboard (+s y)
                roll=(roll_c, np.array([1.0, 0, 0]), V['big']),      # output forward (+x)
                yaw=(hy, np.array([0, 0, -1.0]), V['yaw']),          # output down
                knee=(kn, np.array([0, s * 1.0, 0]), V['big']),      # output outboard
                ankle=an)


def leg(J, side):
    """the leg's plates and motors."""
    P = layout(J, side)
    s = P['s']
    m06, m02, m00 = BIG(), YAW(), ANK()
    out = []
    hp, _, _ = P['pitch']; rc, _, _ = P['roll']; yc, _, _ = P['yaw']; kc, _, _ = P['knee']; ac = P['ankle']
    # -- motors
    for k_ in ('pitch', 'roll', 'yaw', 'knee'):
        c, a, mo = P[k_]
        out.append((f'{mo}-{k_}.{side[0].lower()}l', motor(c, a, mo)))
    tag = side[0].lower()
    span = lambda a_, b_: (min(a_, b_), max(a_, b_))
    face = lambda c_, d_: c_ if d_ > 0 else c_ - T              # plate w0 for a plate on the +/- side of a face at c_
    # -- roll mount (pelvis frame): yz plate on the roll stator back (aft)
    x_rb = rc[0] - m06['L'] / 2 - G
    rm = L.unary_union([L.circle(rc[1], rc[2], m06['back'][0] + 22), L.rounded(*span(0.0, rc[1])[:1], rc[2] - 30, span(0.0, rc[1])[1], Z_D0(), 8)]).convex_hull   # up to the torso deck: one clean teardrop
    rm = rm.difference(L.holes_on(m06['back'][0], m06['back'][1], m06['hole'], phase=45, at=(rc[1], rc[2])))
    out.append((f'chassis-rollmount.{tag}l', plate(rm, 'yz', x_rb - T)))
    # -- hip R: yz plate on the roll output (fore), down + outboard to the pitch's inner plate
    x_ro = rc[0] + m06['L'] / 2 + G
    y_pf = hp[1] - s * (m06['L'] / 2 + G)                 # pitch back face (inboard)
    y_pi = face(y_pf, -s)                                  # inner hip plate w0
    y0, y1 = span(rc[1] - s * 20, y_pf - s * T)        # butts against the inner hip plate
    R1 = L.unary_union([L.circle(rc[1], rc[2], m06['out'][0] + 30), L.rounded(y0, hp[2] + (4 if V['name'] == 'mini' else -10), y1, rc[2], 8)])   # mini: stops above the pitch axis, the rest would show inboard of the hip drum
    R1 = R1.difference(L.holes_on(m06['out'][0], m06['out'][1], m06['hole'], at=(rc[1], rc[2])))
    out.append((f'chassis-hipR.{tag}l', plate(R1, 'yz', x_ro, light=False)))
    # -- hip inner: xz plate on the pitch stator back, forward to hip R; stays below the roll can and above the yaw
    z_top = yc[2] + m02['L'] / 2 + G                       # yaw stator top
    z_rb = rc[2] - m06['D'] / 2 - G                        # roll can bottom
    HI = L.unary_union([L.circle(hp[0], hp[2], m06['back'][0] + 22), L.rounded(hp[0] - 30, hp[2] - 20, x_ro, z_rb, 8)])
    HI = HI.intersection(L.box(-1e3, z_top + T + 2, 1e3, z_rb))
    r_sw = hp[2] - (yc[2] + m02['L'] / 2) - 3.0                         # the yaw can swings round the pitch axis at this radius
    HI = HI.intersection(L.unary_union([L.circle(hp[0], hp[2], 2 * r_sw), L.box(-1e3, hp[2], 1e3, 1e3)]))
    HI = HI.difference(L.holes_on(m06['back'][0], m06['back'][1], m06['hole'], phase=45, at=(hp[0], hp[2])))
    out.append((f'chassis-hipinner.{tag}l', plate(HI, 'xz', y_pi, light=False)))
    # -- hip A: xz plate on the pitch output (outboard), down to the yaw-top plate
    y_out = hp[1] + s * (m06['L'] / 2 + G)
    A1 = L.unary_union([L.circle(hp[0], hp[2], m06['out'][0] + 30), L.rounded(hp[0] - 40, z_top, hp[0] + 40, hp[2], 10)])
    A1 = A1.intersection(L.box(-1e3, z_top + T, 1e3, 1e3))
    A1 = A1.difference(L.holes_on(m06['out'][0], m06['out'][1], m06['hole'], at=(hp[0], hp[2])))
    out.append((f'chassis-hipA.{tag}l', plate(A1, 'xz', face(y_out, s), light=False)))
    # -- hip B2: xy plate on the yaw stator top, out to hip A
    yb0, yb1 = span(yc[1] - s * 40, y_out)
    B2 = L.unary_union([L.circle(yc[0], yc[1], m02['back'][0] + 22), L.rounded(yc[0] - 40, yb0, yc[0] + 40, yb1, 8)])
    B2 = B2.difference(L.holes_on(m02['back'][0], m02['back'][1], m02['hole'], phase=45, at=(yc[0], yc[1])))
    out.append((f'chassis-hipB2.{tag}l', plate(B2, 'xy', z_top, light=False)))
    # -- thigh: a top plate on the yaw output, an inner plate (knee stator) + outer plate down to the knee
    z_yo = yc[2] - m02['L'] / 2 - G                 # yaw output face (down)
    yk_in = kc[1] - s * (m06['L'] / 2 + G)          # knee back face (inboard)
    yk_out = kc[1] + s * (m06['L'] / 2 + G)         # knee output face (outboard)
    y_ti, y_to = (yk_in - T if s > 0 else yk_in), (yk_out + G + T + 5.0 if s > 0 else yk_out - G - T - 5.0 - T)   # outer thigh plate: outside the shin plate + the calf shell wall (clevis)
    top = L.rounded(min(y_ti, y_to) - 2, -40 + yc[0], max(y_ti, y_to) + T + 2, 40 + yc[0], 10)
    top = L.unary_union([L.rounded(yc[0] - 40, min(y_ti, y_to), yc[0] + 40, max(y_ti, y_to) + T, 10)])
    top = top.difference(L.holes_on(m02['out'][0], m02['out'][1], m02['hole'], at=(yc[0], yc[1])))
    out.append((f'chassis-thightop.{side[0].lower()}l', plate(top, 'xy', z_yo - T)))
    W = m06['D'] + 8
    side_p = L.stadium([(yc[0], z_yo - T - 40, 80), (kc[0], kc[2], W)])   # top edge meets the top plate's underside
    th_in = side_p.difference(L.holes_on(m06['back'][0], m06['back'][1], m06['hole'], phase=45, at=(kc[0], kc[2])))
    th_out = side_p.difference(L.circle(kc[0], kc[2], V['hub_bore'] or m06['out'][0] + 30))      # bearing seat around the shin hub (tiny: a 608)
    out.append((f'thigh-inner.{side[0].lower()}l', plate(th_in, 'xz', y_ti)))
    out.append((f'thigh-outer.{side[0].lower()}l', plate(th_out, 'xz', y_to)))
    # -- shin: outer plate on the knee output (between the knee and the outer thigh plate), a shin box below
    y_so = yk_out if s > 0 else yk_out - T
    # ankle motors: two RS00 down the shin, lateral, between the outer shin plate and an inner shin plate
    y_si = (yk_out - G - m00['L'] - G - T - V['shin_gap']) if s > 0 else (yk_out + G + m00['L'] + G + V['shin_gap'])
    a_mot = []
    if V['ankle_f']:
        fs = V['ankle_f']
    else:                                                  # stacked from the knee down: knee can, gap, two RS00 cans
        Lsh = np.linalg.norm(ac - kc)
        f0 = (m06['D'] / 2 + m00['D'] / 2 + 6.0) / Lsh
        fs = (f0, f0 + (m00['D'] + 4.0) / Lsh)
    for f in fs:
        c = kc + (ac - kc) * f
        cm = np.array([c[0], (y_so - G - m00['L'] / 2) if s > 0 else (y_so + T + G + m00['L'] / 2), c[2]])
        a_mot.append(cm)
        out.append((f"{V['ankle']}-ankle{(42, 66)[len(a_mot) - 1]}.{side[0].lower()}l", motor(cm, [0, s * 1.0, 0], V['ankle'])))
    shin_o = L.stadium([(kc[0], kc[2], W - 4), (a_mot[0][0], a_mot[0][2], m00['D'] + 10), (a_mot[1][0], a_mot[1][2], m00['D'] + 10), (ac[0], ac[2], 40)])
    shin_o = shin_o.difference(L.holes_on(m06['out'][0], m06['out'][1], m06['hole'], at=(kc[0], kc[2])))
    out.append((f'shin-outer.{side[0].lower()}l', plate(shin_o, 'xz', y_so, light=False)))   # shins stay solid: buckling-critical
    z_box = kc[2] - m06['D'] / 2 - 6               # the inner shin plate starts below the knee can
    shin_i = L.stadium([(a_mot[0][0], z_box - 34, m00['D'] + 22), (a_mot[1][0], a_mot[1][2], m00['D'] + 22), (ac[0], ac[2], 48)])
    out.append((f'shin-inner.{side[0].lower()}l', plate(shin_i, 'xz', y_si, light=False)))
    # bridge from the outer shin plate to the inner one under the knee can (a yz web), keeps the box closed
    if V['web_x'] is not None:
        web = L.rounded(min(y_si, y_so) + T, z_box - 16, max(y_si, y_so), z_box, 4)
        x_web = kc[0] + V['web_x']
    else:                                                  # ankle cans crowd the knee: close the box below them instead
        zb0, zb1 = ac[2] + 22.0, a_mot[1][2] - m00['D'] / 2 - 2.0
        web = L.rounded(min(y_si, y_so) + T, zb0, max(y_si, y_so), zb1, 3)
        x_web = ac[0] - T / 2
    out.append((f'shin-web.{side[0].lower()}l', plate(web, 'yz', x_web)))
    return out


def Rr(a0, b0, a1, b1, r=6.0):
    return L.rounded(min(a0, a1), min(b0, b1), max(a0, a1), max(b0, b1), r)


def holes(m, which, at, phase=0.0):
    pcd, n = m[which]
    return L.holes_on(pcd, n, m['hole'], phase=phase, at=at)


def hang(J, M, side):
    """elbow -> fingertip, straight down (the K1's length, our plumb arm)."""
    Hd = M[f'{side.lower()}_hand_link']
    tip = Hd.vertices[np.argmin(Hd.vertices[:, 2])]
    return np.array([0, 0, -np.linalg.norm(tip - J[f'{side}_Elbow_Yaw']['p']) - V['elbow_dz']])


def upper_layout(J, side):
    """our arm motor placements on the K1 layout. Axes squared up to x/y/z."""
    s = 1 if side == 'Left' else -1
    m = ARM()
    sr = J[f'{side}_Shoulder_Roll']['p'].copy()
    sr = sr + np.array([0, s * V['arm_out'], 0])                          # plumb arms need the hands clear of the hip drums
    ey = J[f'{side}_Elbow_Yaw']['p'].copy()
    sp = J[f'A{side}_Shoulder_Pitch']['p'].copy()
    sp = np.array([sp[0], s * (CHEST() - 5.0 - G - m['L'] / 2), sp[2]])   # output face just outside the chest wall
    ep = np.array([0.0, sr[1], sr[2] - m['D'] / 2 - G - T - G - m['L'] / 2])   # straight under the shoulder drum: the arm hangs plumb
    ey = np.array([ey[0], sr[1], ey[2] + V['elbow_dz']])
    return dict(s=s, sp=(sp, np.array([0, s * 1.0, 0])), sr=(sr, np.array([1.0, 0, 0])),
                ep=(ep, np.array([0, 0, -1.0])), ey=(np.array([ey[0], ey[1], ey[2]]), np.array([0, s * 1.0, 0])))


def head_layout(J):
    m = HEAD()
    if V['head_z']:
        z = V['head_z'] + V.get('waist_dz', 0.0)                                                    # yaw down in the chest top, between the shoulder cans
    else:                                                                  # no room between them: up above the shoulder cans
        sp = upper_layout(J, 'Left')['sp'][0]
        z = sp[2] + ARM()['D'] / 2 + G + T + G + m['L'] / 2
    hy = np.array([J['AAHead_yaw']['p'][0], 0.0, z])
    hpz = hy[2] + m['L'] / 2 + G + T + G + m['D'] / 2
    return dict(yaw=(hy, np.array([0, 0, 1.0])), pitch=(np.array([hy[0], 0.0, hpz]), np.array([0, 1.0, 0])))


def arm(J, side, M):
    P = upper_layout(J, side)
    s, m = P['s'], ARM()
    tag = side[0].lower()
    out = [(f"{V['arm']}-{k}.{tag}a", motor(P[k][0], P[k][1], V['arm'])) for k in ('sp', 'sr', 'ep', 'ey')]
    sp, sr, ep, ey = (P[k][0] for k in ('sp', 'sr', 'ep', 'ey'))
    yw = lambda y0: min(s * y0, s * (y0 + T))                            # w0 of an xz plate starting at |y| = y0, growing outboard
    # shoulder yoke: P1 on the pitch output (xz), P2 on the roll stator back (yz)
    y_p1 = abs(sp[1]) + m['L'] / 2 + G
    x_p2 = sr[0] - m['L'] / 2 - G - T
    P1 = L.unary_union([L.circle(sp[0], sp[2], m['out'][0] + 30), Rr(x_p2, sr[2] - 16, sp[0] + 12, sp[2] + 18)])
    out.append((f'arm-yokeP1.{tag}a', plate(P1.difference(holes(m, 'out', (sp[0], sp[2]))), 'xz', yw(y_p1))))
    P2 = L.unary_union([L.circle(sr[1], sr[2], m['back'][0] + 22), Rr(s * (y_p1 + T), sr[2] - 12, sr[1], sr[2] + 24)])
    out.append((f'arm-yokeP2.{tag}a', plate(P2.difference(holes(m, 'back', (sr[1], sr[2]), 45)), 'yz', x_p2)))
    # upper arm: U1 on the roll output (yz, fore), U2 on the rotation motor's stator top (xy)
    x_u1 = sr[0] + m['L'] / 2 + G
    z_u2 = ep[2] + m['L'] / 2 + G
    U1 = L.unary_union([L.circle(sr[1], sr[2], m['out'][0] + 30), Rr(ep[1] - s * 22, z_u2 + T, ep[1] + s * 22, sr[2])])
    out.append((f'arm-upperU1.{tag}a', plate(U1.difference(holes(m, 'out', (sr[1], sr[2]))), 'yz', x_u1)))
    U2 = L.unary_union([L.circle(ep[0], ep[1], m['back'][0] + 22), Rr(ep[0], ep[1] - s * 22, x_u1 + T, ep[1] + s * 22)])
    U2 = U2.intersection(L.box(x_p2 + T + 2 * G + 1, -1e3, 1e3, 1e3))       # running clearance to the yoke (P2 is on the other side of the roll)
    out.append((f'arm-upperU2.{tag}a', plate(U2.difference(holes(m, 'back', (ep[0], ep[1]), 45)), 'xy', z_u2)))
    # elbow fork: E1 on the rotation output (xy, below), E2 on the elbow stator back (xz, inboard)
    z_e1 = ep[2] - m['L'] / 2 - G - T
    y_e2 = abs(ey[1]) - m['L'] / 2 - G - T
    E1 = L.unary_union([L.circle(ep[0], ep[1], m['out'][0] + 30), Rr(ey[0] - 20, s * y_e2, ey[0] + 20, ep[1])])
    out.append((f'arm-elbowE1.{tag}a', plate(E1.difference(holes(m, 'out', (ep[0], ep[1]))), 'xy', z_e1)))
    E2 = L.unary_union([L.circle(ey[0], ey[2], m['back'][0] + 22), Rr(ey[0] - 12, ey[2], ey[0] + 12, z_e1)])   # neck stays inside the rotation drum
    out.append((f'arm-elbowE2.{tag}a', plate(E2.difference(holes(m, 'back', (ey[0], ey[2]), 45)), 'xz', yw(y_e2))))
    # forearm: F1 on the elbow output (xz, outboard), down to the wrist
    y_f1 = abs(ey[1]) + m['L'] / 2 + G
    wr = ey + hang(J, M, side) * 0.62
    Wr = wrist_layout(J, side, M)
    if Wr is not None:                                                    # down to the wrist roll's mounting plate
        z_f2 = Wr['roll'][0][2] + WRIST()['L'] / 2 + G
        F1 = L.stadium([(ey[0], ey[2], m['out'][0] + 30), (wr[0], z_f2 + T + 15, 30)]).intersection(L.box(-1e3, z_f2 + T, 1e3, 1e3)).difference(holes(m, 'out', (ey[0], ey[2])))
    else:
        F1 = L.stadium([(ey[0], ey[2], m['out'][0] + 30), (wr[0], wr[2], 30)]).difference(holes(m, 'out', (ey[0], ey[2])))
    out.append((f'arm-forearmF1.{tag}a', plate(F1, 'xz', yw(y_f1))))
    return out


def torso(J):
    """the spine: two side plates carrying the shoulder-pitch stators, a bottom deck on the roll mounts, a top deck under the neck yaw."""
    m, mh = ARM(), HEAD()
    out = []
    U = upper_layout(J, 'Left'); H = head_layout(J)
    sp = U['sp'][0]
    y_s = abs(sp[1]) - m['L'] / 2 - G - T                                # side plate inner face
    hy = H['yaw'][0]
    z_d1 = hy[2] - mh['L'] / 2 - G - T
    z_d0 = Z_CD()
    x0, x1 = -55.0 * R(), 65.0 * R()
    if V['name'] == 'mini':                                                # inside the slimmer shell (3 mm wall + clearance)
        xm_, hw_, _ = shell_x(J)
        x0, x1 = max(x0, xm_ - hw_ + 5.0), min(x1, xm_ + hw_ - 5.0)
    for s, tag in ((1, 'l'), (-1, 'r')):
        S = L.unary_union([L.circle(sp[0], sp[2], m['back'][0] + 22), Rr(x0, z_d0, x1, max(sp[2], z_d1))]).intersection(L.box(-1e3, -1e3, 1e3, z_d1 if z_d1 > sp[2] else 1e3))
        S = S.difference(holes(m, 'back', (sp[0], sp[2]), 45))
        if V['name'] == 'mini' and s < 0:                                     # the spine's pass-through (harness), 10 mm
            S = S.difference(L.circle(*PASS_THROUGH(), 10.0))
        out.append((f'torso-side.{tag}', plate(S, 'xz', y_s if s > 0 else -y_s - T)))
    out.append(('torso-deck0', plate(Rr(x0, -y_s, x1, y_s), 'xy', z_d0)))
    yd = max(y_s, mh['back'][0] / 2 + 8.0)                                 # rests on the side plates' tops when the gap is narrow
    D1 = Rr(hy[0] - 40, -yd, hy[0] + 45, yd).difference(holes(mh, 'back', (hy[0], hy[1]), 45))
    out.append(('torso-deck1', plate(D1, 'xy', z_d1)))
    return out


def head(J):
    m = HEAD()
    H = head_layout(J)
    hy, hp = H['yaw'][0], H['pitch'][0]
    out = [(f"{V['head']}-headyaw", motor(hy, H['yaw'][1], V['head'])), (f"{V['head']}-headpitch", motor(hp, H['pitch'][1], V['head']))]
    z_n0 = hy[2] + m['L'] / 2 + G
    y_n1 = -(m['L'] / 2 + G + T)                                         # pitch stator back (-y)
    N0 = L.unary_union([L.circle(hy[0], hy[1], m['out'][0] + 30), Rr(hy[0] - 20, y_n1 + T, hy[0] + 20, 18)]).difference(holes(m, 'out', (hy[0], hy[1])))
    out.append(('neck-N0', plate(N0, 'xy', z_n0)))
    N1 = L.unary_union([L.circle(hp[0], hp[2], m['back'][0] + 22), Rr(hp[0] - 20, z_n0 + T, hp[0] + 20, hp[2])]).difference(holes(m, 'back', (hp[0], hp[2]), 45))
    out.append(('neck-N1', plate(N1.intersection(L.box(-1e3, z_n0 + T, 1e3, 1e3)), 'xz', y_n1)))
    zd = DOCK_Z(J)
    Hh = L.unary_union([L.circle(hp[0], hp[2], m['out'][0] + 30), Rr(hp[0] - 22 * R(), hp[2], hp[0] + 22 * R(), zd, 6)]).difference(holes(m, 'out', (hp[0], hp[2])))
    out.append(('head-H1', plate(Hh, 'xz', m['L'] / 2 + G)))
    # the dock: the cortex head (Jetson + its own battery) lifts off this plate and can live on a desk; flat bottom.
    # This plate IS the dock face of cortex.loon (dock-face): pogo window, latch window, cone studs.
    yd = m['L'] / 2 + G + T
    dk = Rr(hp[0] - 34 * R(), -yd, hp[0] + 34 * R(), yd, 8)
    out.append(('head-dock', plate(dock_cut(dk, hp[0]), 'xy', zd)))
    out += cx_parts(CX_NECK, (hp[0], 0.0, zd + T))
    return out


# ---- the cortex head. cortex/cortex.loon owns the product; cortex/export.sh writes one STL per part to cortex/stl,
# and the body loads those as its head proxies (ROM sweeps, harness check). Frame: the DOCK ORIGIN, the centre of the
# head's flat floor; x fore, y left, z up.
CX_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cortex')
CX_SIZE = dict(x_face=46.0, x_back=-58.0, hh=120.6, hw=81.8)      # cortex.loon x-face, x-back, hh, hw (mini)
CX_SHELL = ('tub', 'glass', 'ear-grilles', 'exhaust-grille', 'floor')
CX_INSIDE = ('padboard', 'pads', 'strikes', 'keeper', 'display', 'cams', 'dockboard', 'power', 'battery', 'sled',
             'compute', 'heatsink', 'speakers', 'mics', 'amps')
CX_NECK = ('dock-pogo', 'dock-pogo-block', 'dock-cones', 'dock-magnets', 'dock-pawl', 'dock-release')   # dock-face = head-dock plate
DOCK_UNDER = 12.0          # free depth kept under the dock face, over the pitch drum (pogo block + release rod reach 9)
DOCK_WIN = [(-6.2, -10.2, 6.2, 10.2), (17.5, -6.5, 23.5, 6.5)]  # dock-face windows (dock origin): pogo block, latch pawl
DOCK_HOLES = [(0.0, 18.0, 1.6), (0.0, -18.0, 1.6)]               # M3 cone studs. OPEN: cortex.loon's magnets (y +-30, D8)
                                                                  # overhang this plate (y +-29.7) and its trim (+-32.7)
_CX = {}


def cx_mesh(name):
    if name not in _CX:
        m = trimesh.load(os.path.join(CX_DIR, 'stl', name + '.stl'), force='mesh')
        if m.bounds[0][0] < -99.0:                                  # drop a fold's seed cube (pads, pogo pins: x -100)
            m = trimesh.util.concatenate([c for c in m.split(only_watertight=False) if c.bounds[1][0] > -99.0])
        _CX[name] = m
    return _CX[name]


def cx_parts(names, origin, scale=(1.0, 1.0, 1.0)):
    """cortex.loon parts (cortex/stl) placed at a dock origin; scale fits the mini-size head to another frame."""
    out = []
    for n in names:
        m = cx_mesh(n if n.startswith('dock-') else 'cortex-' + n).copy()
        m.apply_scale(scale)
        m.apply_translation(origin)
        out.append((n if n.startswith('dock-') else 'cortex-' + n, m))
    return out


def cx_scale(HF):
    """the head's per-axis scale against cortex.loon's mini size (1 on the mini)."""
    k = (HF['hd'] * 2 / (CX_SIZE['x_face'] - CX_SIZE['x_back']), HF['hw'] / CX_SIZE['hw'], HF['hh'] / CX_SIZE['hh'])
    return tuple(1.0 if abs(v - 1.0) < 1e-3 else v for v in k)          # the mini's head is cortex.loon's size to 0.05 mm


def dock_cut(profile, x0):
    """cut the dock face's windows and holes (cortex.loon dock-face) out of a profile, dock origin at (x0, 0)."""
    for a, b, c, d in DOCK_WIN:
        profile = profile.difference(Rr(x0 + a, b, x0 + c, d, 1.0))
    for x, y, r in DOCK_HOLES:
        profile = profile.difference(L.circle(x0 + x, y, r))
    return profile


def DOCK_Z(J):
    """the neck's dock plate (top of the head-pitch stage): the cortex head's flat bottom sits on it."""
    hp = head_layout(J)['pitch'][0]
    r_drum = max(33.0, HEAD()['D'] / 2 + 4.5)                      # the pogo block + latch live under the dock face, over the drum
    return hp[2] + r_drum + DOCK_UNDER + 2.0


def head_frame(J, M):
    """the cortex head's frame on this body: its dock origin sits on the neck's dock face."""
    hlo, hhi = M['Head_2'].bounds
    hp = head_layout(J)['pitch'][0]
    ox, oy, oz = hp[0], 0.0, DOCK_Z(J) + T + 0.5
    hh, hw, Rk = (hhi[2] - hlo[2]) * 0.62, (hhi[1] - hlo[1]) / 2 * 0.96, R()
    x_face, x_back = ox + CX_SIZE['x_face'] * Rk, ox + CX_SIZE['x_back'] * Rk
    hd = (x_face - x_back) / 2
    return dict(ox=ox, oy=oy, zb=oz, hh=hh, hw=hw, hd=hd, x_face=x_face, x_back=x_back, R=Rk,
                rc_p=0.30 * min(hw, hd), rf=0.42 * min(hw, hd), rb=0.24 * min(hw, hd))


def cortex(J, M):
    """the cortex's insides on this body (mini), from cortex.loon."""
    if V['name'] != 'mini':
        return []
    HF = head_frame(J, M)
    return cx_parts(CX_INSIDE, (HF['ox'], HF['oy'], HF['zb']), cx_scale(HF))


def harness(J, M):
    """the cortex + spine wiring (mini): the head's own looms (cortex.loon cortex-cables), the spine from the pogo
    block down behind the pitch drum, through the yaw service loop into the chest, through the right side plate
    (a 10 mm pass-through), to the limbic board's top edge."""
    if V['name'] != 'mini':
        return []
    HF = head_frame(J, M)
    out = [('cable-hcortex', cx_parts(['cables'], (HF['ox'], HF['oy'], HF['zb']), cx_scale(HF))[0][1])]
    H = head_layout(J); hp, hy = H['pitch'][0], H['yaw'][0]
    r_drum = max(33.0, HEAD()['D'] / 2 + 4.5)
    xs = hp[0] - r_drum - 4.0                                       # just behind the pitch drum
    zf = DOCK_Z(J) - 4.5                                            # under the dock trim, out of the pogo block's back
    ys = -8.0                                                       # beside the release rod and button (y +-4)
    out.append(cable('cable-hspine', [[hp[0] - 6.0 + 1.0, ys, zf], [xs, ys, zf], [xs, ys, hp[2] + 8.0]], r=3.0))    # off the pogo block's back, down behind the drum; the pitch flex loop is the gap below
    out.append(cable('cable-nspine', [[xs, 0, hp[2] - 8.0], [xs, 0, hy[2] + 48.0]], r=3.0))                         # rides the neck (yaw)
    xh, zh = PASS_THROUGH()
    out.append(cable('cable-tspine', [[xs, 0, hy[2] + 40], [xs - 7, -10, zh + 5], [xh, -25, zh], [xh, -50, zh - 2], [xh, -52, 167.0]], r=3.0))   # the yaw service loop lives at the top of this run
    return out


def PASS_THROUGH():
    """the spine's pass-through in the right torso side plate (x, z), behind the shoulder-pitch can."""
    return -48.0 * R(), 210.0 * R()                                    # behind the shoulder drum, above the exhaust fans


def cable(name, pts, r=3.0, flat=None):
    """a cable as a swept tube (r) or a flat FFC/FPC ribbon (flat = width) along a polyline."""
    pts = [np.asarray(p, float) for p in pts]
    segs = []
    for a, b in zip(pts, pts[1:]):
        if flat:                                                   # ribbon: hull of two thin rectangles, broad side horizontal-ish
            d = b - a; d = d / np.linalg.norm(d)
            w = np.cross(d, [0, 0, 1.0]); w = w / (np.linalg.norm(w) or 1.0)
            if np.linalg.norm(np.cross(d, [0, 0, 1.0])) < 1e-6:
                w = np.array([0, 1.0, 0])
            n = np.cross(d, w)
            q = [c + sw * w * flat / 2 + sn * n * 0.5 for c in (a, b) for sw in (-1, 1) for sn in (-1, 1)]
            segs.append(Manifold.hull_points(np.array(q, np.float32)))
        else:
            sph = lambda c: Manifold.sphere(r, 16).translate(list(c))
            segs.append(Manifold.hull(sph(a) + sph(b)) if hasattr(Manifold, 'hull') else (sph(a) + sph(b)).hull())
    m = Manifold.batch_boolean(segs, OpType.Add) if len(segs) > 1 else segs[0]
    mm = m.to_mesh()
    return (name, trimesh.Trimesh(vertices=np.asarray(mm.vert_properties)[:, :3], faces=np.asarray(mm.tri_verts), process=False))


# electronics in the torso bay. PLACEHOLDERS until the limbic-board spec (rose branch claude/limbic-spec, LIMBIC.md)
# lands: cells, pack arrangement, Jetson and board envelopes are assumptions, sized to the bay; swap numbers here.
ELEC = dict(cell_d=21.0, cell_h=70.0, cell_kg=0.070, S=12, P=2,     # 12S2P 21700 (Molicel P50B class): 432 Wh, ~1.7 h walking
            # ONE swappable pack inside the chassis, K1-style: it slides in from the back into the bay between the torso side
            # plates, on two guide strips on deck0, until its front face blind-mates the dock on a cross plate; latch at the
            # hatch, whose face IS the pack's back (flush with the shell). Cells lie in 3 (y) x 4 (x) x 2 tiers, upright.
            # (Retired: the back-mounted option C. Option D (12S3P) no longer fits the bay.)
            pack=dict(cells=(4, 3, 2), kg=2.0),                     # cells x, y, z
            monitor=6.0,                                            # BQ76952 board on the pack's dock (front) face (was a 10 mm slab)
            jetson=(50.0, 52.0, 87.0), jetson_kg=0.40,              # Seeed A603 + Orin NX 16 GB, upright (x, y, z); height ESTIMATE
            fan=(10.0, 30.0, 30.0), brake=(28.0, 50.0, 20.0),       # 30 mm exhaust fan (blowing out, above the pack, under the leaning back); brake resistor (size TBD)
            estop=None,                                             # no hard e-stop (the K1 has none): wireless ELRS e-stop instead
            elrs=(20.0, 14.0, 5.0),                                 # ELRS 2.4 GHz RX (RP1/RP2 class), chest top; antenna along the PA12 wall
            vent_in=dict(z=(28.0, 82.0), y=(48.0, 78.0)),   # two low front grilles, one per side bay (>=12 cm2 open for 4 CFM)
            vent_out=dict(z=(172.0, 200.0)),                # exhaust grille: chest back, over the pack
            foot_node=(20.0, 15.0, 6.0),                            # STM32G0 + FSR node in each sole (v1)
            board=(110.0, 88.0, 15.0), board_kg=0.12)   # 110 x 88 (claude/limbic-spec 0a00f7b), tall parts outboard               # limbic board 110 x 74 (LIMBIC.md), with parts (x, y, z)


SHELL_DEPTH = {'mini': (0.88, 0.90), 'full': (0.70, 0.84)}   # torso depth at the waist / the chest, of the trunk envelope
                                                                # (torso pass 2026-10-28; the mini's is the repacked bay's)


def shell_x(J=None, M=None):
    """the torso shell's x centre and its outer half-depths at the waist and the chest (k1_design.torso_shell)."""
    if M is None:
        J, M = world()
    lo, hi = M['Trunk'].bounds
    xm, h = ((lo[0] + 10.0) + (hi[0] - 12.0)) / 2, ((hi[0] - 12.0) - (lo[0] + 10.0)) / 2
    d0, d1 = SHELL_DEPTH.get(V['name'], (1.0, 1.0))
    return xm, h * d0, h * d1


def pack_box(J=None):
    """the pack's envelope in the bay (mini): ((x0, y0, z0), (x1, y1, z1)) incl. the 2 mm PA12 case."""
    E = ELEC
    if J is None:
        J, _ = world()
    sp = upper_layout(J, 'Left')['sp'][0]
    y_s = abs(sp[1]) - ARM()['L'] / 2 - G - T                 # side plate inner face
    cx, cy, cz = E['pack']['cells']
    xm, hw, _ = shell_x()
    x1 = xm - hw + 0.5                                        # the hatch: flush with the (slimmer) back skin at the waist
    x0 = x1 + 2.0 + cx * E['cell_d'] + E['monitor'] + 2.0
    z0 = Z_D0() + T + 3.0                                     # on 3 mm guide strips on deck0
    return (np.array([x1, -(y_s - 1.0), z0]), np.array([x0, y_s - 1.0, z0 + 4.0 + cz * E['cell_h'] + 4.0]))


def electronics(J):
    """the in-bay pack + dock, Jetson and limbic board in the side bays, fan, brake, ELRS (mini only)."""
    if V['name'] != 'mini':
        return []
    E, out = ELEC, []
    sp = upper_layout(J, 'Left')['sp'][0]
    y_s = abs(sp[1]) - ARM()['L'] / 2 - G - T                 # side plate inner face
    lo, hi = pack_box(J)
    for s_, t in ((1, 'l'), (-1, 'r')):                         # guide strips the pack slides in on
        out.append((f'torso-rail.{t}', trimesh.creation.box(bounds=[[lo[0] + 4.0, s_ * (y_s - 14.0) - 4.0, Z_D0() + T], [hi[0], s_ * (y_s - 14.0) + 4.0, lo[2]]])))
    x_dp = hi[0] + 6.0                                          # dock cross plate, between the side plates
    out.append(('torso-dock', plate(Rr(-y_s, Z_D0() + T, y_s, lo[2] + 70.0), 'yz', x_dp)))
    out.append(('elec-dock', trimesh.creation.box(bounds=[[hi[0] + 0.5, -16.0, lo[2] + 4.0], [x_dp, 16.0, lo[2] + 24.0]])))
    out.append(('torso-latch', trimesh.creation.box(bounds=[[lo[0] + 30.0, 22.0, hi[2] + 0.5], [lo[0] + 42.0, 38.0, hi[2] + 3.5]])))
    cx, cy, cz = E['pack']['cells']
    xm = hi[0] - 2.0 - E['monitor']
    out.append(('elec-bms', trimesh.creation.box(bounds=[[xm, lo[1] + 2.0, lo[2] + 2.0], [hi[0] - 2.0, hi[1] - 2.0, hi[2] - 2.0]])))
    out.append(('elec-battery', trimesh.creation.box(bounds=[[xm - cx * E['cell_d'], -cy * E['cell_d'] / 2, lo[2] + 2.0], [xm, cy * E['cell_d'] / 2, hi[2] - 2.0]])))
    # side bays (outboard of the side plates, under the shoulder cans): Jetson left, limbic board + brake right
    y_o = y_s + T + 2.0
    # (the Jetson lives in the cortex head now: the left side bay is free)
    bx_, by_, bz_ = E['board']
    xm_, hw_, _ = shell_x(J)
    out.append(('elec-limbic', trimesh.creation.box(bounds=[[xm_ - by_ / 2, -(y_o + bz_), 56.0], [xm_ + by_ / 2, -y_o, 56.0 + bx_]])))   # on end, z 56..166
    rx, ry, rz = E['brake']                                   # the left bay (free since the Jetson went to the head), lying flat
    out.append(('elec-brake', trimesh.creation.box(bounds=[[xm_ - ry / 2, y_o + 2.0, 118.0], [xm_ + ry / 2, y_o + 2.0 + 20.0, 118.0 + rx - 2.0]])))   # up where the V is wide
    fx, fy, fz = E['fan']; zo = E['vent_out']['z']
    x_wall = shell_x(J)[0] - shell_x(J)[2] + 4.0               # inside the chest back wall, over the hatch
    for i, yf in enumerate((-(fy / 2 + 1.0), fy / 2 + 1.0)):  # 2 x 30 mm, side by side (>= 4 CFM with the front intake)
        out.append((f'elec-fan.{"r" if yf < 0 else "l"}', trimesh.creation.box(extents=[fx, fy, fz]).apply_translation([x_wall + 2 + fx / 2, yf, (zo[0] + zo[1]) / 2])))
    ex, ey_, ez = E['elrs']
    z_d1 = head_layout(J)['yaw'][0][2] - HEAD()['L'] / 2 - G - T
    out.append(('elec-elrs', trimesh.creation.box(extents=[ex, ez, ey_]).apply_translation([xm_, y_o + 2.0 + ez / 2, 160.0])))   # on end, top of the left bay
    return out


def foot_nodes(J):
    """v1 foot contact: a small CAN node in each sole (in the ankle body)."""
    if V['name'] != 'mini':
        return []
    nx, ny, nz = ELEC['foot_node']
    out = []
    for side in ('Left', 'Right'):
        a = layout(J, side)['ankle']
        out.append((f'node-foot.{side[0].lower()}l', trimesh.creation.box(extents=[nx, ny, nz]).apply_translation([a[0] + 40.0, a[1], a[2] - 27.0])))
    return out


def CHEST():
    return 101.0 * R() * V.get('ky', 1.0)          # chest half-width (mini 101)


def Z_D0():
    return 16.0 * R()                  # torso bottom deck


def world():
    """the K1 joint layout (arms down), scaled to our height; link meshes scaled with it."""
    J, M = KT.world(K.arms_down())
    J = {n: dict(j, p=np.asarray(j['p']) * V['scale']) for n, j in J.items()}
    M = {n: m.copy().apply_scale(V['scale']) for n, m in M.items()}
    ky, kx = V.get('ky', 1.0), V.get('trunk_kx', 1.0)                    # full: slimmer across (H2 is 456 mm wide), shallower trunk
    if ky != 1.0:
        J = {n: dict(j, p=j['p'] * np.array([1.0, ky, 1.0])) for n, j in J.items()}
        M = {n: m.apply_scale([1.0, ky, 1.0]) for n, m in M.items()}
    if kx != 1.0:
        M['Trunk'] = M['Trunk'].apply_scale([kx, 1.0, 1.0])
    li = V.get('leg_in', 0.0)                                             # H2/T2: legs closer in, the hip drums at the pelvis corners
    if li:                                                                 # (20 mm at the mini: the most that keeps 15 deg inward hip roll)
        leg = lambda n: any(k in n for k in ('Hip', 'Knee', 'Ankle', 'Shank', 'foot'))
        sh = lambda n: np.array([0, -li if ('Left' in n or 'left' in n) else li, 0])
        J = {n: dict(j, p=j['p'] + sh(n)) if leg(n) else j for n, j in J.items()}
        M = {n: m.apply_translation(sh(n)) if leg(n) else m for n, m in M.items()}
    dz = V.get('waist_dz', 0.0)                                           # the waist stack lengthens the trunk: lift arms + head
    if dz:
        up = lambda n: any(k in n for k in ('Shoulder', 'Elbow', 'Head', 'Arm', 'hand'))
        J = {n: dict(j, p=j['p'] + np.array([0, 0, dz])) if up(n) else j for n, j in J.items()}
        M = {n: m.apply_translation([0, 0, dz]) if up(n) else m for n, m in M.items()}
    return J, M


def structure():
    J, M = world()
    out = []
    for side in ('Left', 'Right'):
        out += leg(J, side) + arm(J, side, M) + wrist(J, side, M)
    return out + torso(J) + waist() + head(J) + electronics(J) + cortex(J, M) + harness(J, M) + foot_nodes(J)
