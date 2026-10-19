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
import math
import numpy as np, trimesh
from manifold3d import Manifold, CrossSection, FillRule
import leg as L
import quad as Q
import k1 as K, k1_teardown as KT

T = L.T
G = 1.0                      # assembly gap


def plate(profile, plane, w0, t=T):
    """extrude a shapely profile into a plate: plane 'xz' (normal y), 'yz' (normal x), 'xy' (normal z)."""
    cs = CrossSection(L.rings(profile), FillRule.EvenOdd)
    m = L.to_world(cs.extrude(t).translate((0, 0, w0)), plane)
    return L.tri(m)


def motor(c, ax, mot):
    m = Q.MOTORS[mot]
    a = np.asarray(ax, float) / np.linalg.norm(ax)
    return trimesh.creation.cylinder(radius=m['D'] / 2, segment=[np.asarray(c) - a * m['L'] / 2, np.asarray(c) + a * m['L'] / 2], sections=64)


def layout(J, side):
    """our joint placements (motor centre, axis, model) for one leg, on the K1's axes."""
    s = 1 if side == 'Left' else -1
    m06, m02, m00 = Q.MOTORS['rs06'], Q.MOTORS['rs02'], Q.MOTORS['rs00']
    hp = J[f'{side}_Hip_Pitch']['p'].copy()
    hr = J[f'{side}_Hip_Roll']['p'].copy()
    hy = J[f'{side}_Hip_Yaw']['p'].copy()
    kn = J[f'{side}_Knee_Pitch']['p'].copy()
    an = J[f'{side}_Ankle_Pitch']['p'].copy()
    # roll: forward along x until its rear face clears the pitch can (+ gap for the output plate)
    # roll can hugs the inside of bracket A (on the pitch output) and sits forward of the pitch can
    y_out = hp[1] + s * (m06['L'] / 2 + G)
    roll_c = np.array([m06['D'] / 2 + G + T + G + m06['L'] / 2, y_out - s * (G + m06['D'] / 2), hr[2]])
    hy = np.array([0.0, hy[1], hy[2]])                        # yaw straight under the pitch, clear of the roll can
    return dict(s=s,
                pitch=(hp, np.array([0, s * 1.0, 0]), 'rs06'),     # output outboard (+s y)
                roll=(roll_c, np.array([-1.0, 0, 0]), 'rs06'),     # output aft (-x)
                yaw=(hy, np.array([0, 0, -1.0]), 'rs02'),          # output down
                knee=(kn, np.array([0, s * 1.0, 0]), 'rs06'),      # output outboard
                ankle=an)


def leg(J, side):
    """the leg's plates and motors."""
    P = layout(J, side)
    s = P['s']
    m06, m02, m00 = Q.MOTORS['rs06'], Q.MOTORS['rs02'], Q.MOTORS['rs00']
    out = []
    hp, _, _ = P['pitch']; rc, _, _ = P['roll']; yc, _, _ = P['yaw']; kc, _, _ = P['knee']; ac = P['ankle']
    # -- motors
    for k_ in ('pitch', 'roll', 'yaw', 'knee'):
        c, a, mo = P[k_]
        out.append((f'{mo}-{k_}.{side[0].lower()}l', motor(c, a, mo)))
    yo = lambda y: y if s > 0 else -y - T          # plate y position for this side (plates extrude +y)
    # -- pelvis side plate: the hip-pitch stator's back, inboard of the can
    y_in = hp[1] - s * (m06['L'] / 2 + G)          # pitch back face
    pel = L.unary_union([L.circle(hp[0], hp[2], m06['back'][0] + 22), L.rounded(hp[0] - 50, hp[2] - 10, hp[0] + 50, hp[2] + 110, 10)])
    x_cut = rc[0] - m06['L'] / 2 - G - T - 2.0                # stop short of the roll bracket + can
    pel = pel.intersection(L.box(-1e3, -1e3, x_cut, 1e3))
    pel = pel.difference(L.holes_on(m06['back'][0], m06['back'][1], m06['hole'], phase=45, at=(hp[0], hp[2])))
    out.append((f'chassis-pelvisside.{side[0].lower()}l', plate(pel, 'xz', (y_in - T) if s > 0 else (y_in))))
    # -- bracket A: on the pitch output (outboard, xz), forward to a yz plate behind the roll can's back
    y_out = hp[1] + s * (m06['L'] / 2 + G)         # pitch output face
    x_back = rc[0] + m06['L'] / 2 + G               # roll back face (forward end)
    z_ytop = yc[2] + m02['L'] / 2
    A1 = L.unary_union([L.circle(hp[0], hp[2], m06['out'][0] + 30), L.rounded(hp[0], max(rc[2] - 30, z_ytop + 2), x_back, hp[2] + 18, 10)])
    A1 = A1.intersection(L.box(-1e3, z_ytop + 2, 1e3, 1e3))
    A1 = A1.difference(L.rounded(rc[0] - m06['L'] / 2 - G - T - 1, -1e3, rc[0] + m06['L'] / 2, rc[2] - 30 + 1e-3, 0.5)) if False else A1
    A1 = A1.difference(L.holes_on(m06['out'][0], m06['out'][1], m06['hole'], at=(hp[0], hp[2])))
    out.append((f'chassis-hipA.{side[0].lower()}l', plate(A1, 'xz', y_out if s > 0 else y_out - T)))
    ylo, yhi = sorted((y_out + s * T, rc[1] - s * (m06['D'] / 2 - 4)))
    A2 = L.unary_union([L.circle(rc[1], rc[2], m06['back'][0] + 22), L.rounded(ylo, rc[2] - 30, yhi, rc[2] + 30, 8)])
    A2 = A2.difference(L.holes_on(m06['back'][0], m06['back'][1], m06['hole'], phase=45, at=(rc[1], rc[2])))
    out.append((f'chassis-hipA2.{side[0].lower()}l', plate(A2, 'yz', x_back)))
    # -- bracket B: on the roll output (aft face, yz), down to a horizontal plate on the yaw stator
    x_out = rc[0] - m06['L'] / 2 - G                # roll output face (aft)
    z_top = yc[2] + m02['L'] / 2 + G                # yaw stator top (back)
    B1 = L.unary_union([L.circle(rc[1], rc[2], m06['out'][0] + 30), L.rounded(rc[1] - 30, z_top + T, rc[1] + 30, rc[2], 8)])
    B1 = B1.difference(L.holes_on(m06['out'][0], m06['out'][1], m06['hole'], at=(rc[1], rc[2])))
    out.append((f'chassis-hipB.{side[0].lower()}l', plate(B1, 'yz', x_out - T)))
    yb0, yb1 = sorted((yc[1] - s * 34, y_out - s * (G + 1)))
    B2 = L.unary_union([L.circle(yc[0], yc[1], m02['back'][0] + 22), L.rounded(yc[0], yb0, x_out - T, yb1, 8)])
    B2 = B2.intersection(L.box(-1e3, -1e3 if s > 0 else y_out + G, 1e3, y_out - G if s > 0 else 1e3))
    B2 = B2.difference(L.holes_on(m02['back'][0], m02['back'][1], m02['hole'], phase=45, at=(yc[0], yc[1])))
    out.append((f'chassis-hipB2.{side[0].lower()}l', plate(B2, 'xy', z_top)))
    # -- thigh: a top plate on the yaw output, an inner plate (knee stator) + outer plate down to the knee
    z_yo = yc[2] - m02['L'] / 2 - G                 # yaw output face (down)
    yk_in = kc[1] - s * (m06['L'] / 2 + G)          # knee back face (inboard)
    yk_out = kc[1] + s * (m06['L'] / 2 + G)         # knee output face (outboard)
    y_ti, y_to = (yk_in - T if s > 0 else yk_in), (yk_out + G + T + G if s > 0 else yk_out - G - T - G - T)   # outer thigh plate rides outside the shin plate
    top = L.rounded(min(y_ti, y_to) - 2, -40 + yc[0], max(y_ti, y_to) + T + 2, 40 + yc[0], 10)
    top = L.unary_union([L.rounded(yc[0] - 40, min(y_ti, y_to), yc[0] + 40, max(y_ti, y_to) + T, 10)])
    top = top.difference(L.holes_on(m02['out'][0], m02['out'][1], m02['hole'], at=(yc[0], yc[1])))
    out.append((f'chassis-thightop.{side[0].lower()}l', plate(top, 'xy', z_yo - T)))
    W = m06['D'] + 8
    side_p = L.stadium([(yc[0], z_yo - T - 40, 80), (kc[0], kc[2], W)])   # top edge meets the top plate's underside
    th_in = side_p.difference(L.holes_on(m06['back'][0], m06['back'][1], m06['hole'], phase=45, at=(kc[0], kc[2])))
    th_out = side_p.difference(L.circle(kc[0], kc[2], m06['out'][0] + 30))      # bearing seat around the shin hub
    out.append((f'thigh-inner.{side[0].lower()}l', plate(th_in, 'xz', y_ti)))
    out.append((f'thigh-outer.{side[0].lower()}l', plate(th_out, 'xz', y_to)))
    # -- shin: outer plate on the knee output (between the knee and the outer thigh plate), a shin box below
    y_so = yk_out if s > 0 else yk_out - T
    # ankle motors: two RS00 down the shin, lateral, between the outer shin plate and an inner shin plate
    y_si = (yk_out - G - m00['L'] - G - T) if s > 0 else (yk_out + G + m00['L'] + G)
    a_mot = []
    for f in (0.42, 0.66):
        c = kc + (ac - kc) * f
        cm = np.array([c[0], (y_so - G - m00['L'] / 2) if s > 0 else (y_so + T + G + m00['L'] / 2), c[2]])
        a_mot.append(cm)
        out.append((f'rs00-ankle{int(f * 100)}.{side[0].lower()}l', motor(cm, [0, s * 1.0, 0], 'rs00')))
    shin_o = L.stadium([(kc[0], kc[2], W - 4), (a_mot[0][0], a_mot[0][2], m00['D'] + 10), (a_mot[1][0], a_mot[1][2], m00['D'] + 10), (ac[0], ac[2], 40)])
    shin_o = shin_o.difference(L.holes_on(m06['out'][0], m06['out'][1], m06['hole'], at=(kc[0], kc[2])))
    out.append((f'shin-outer.{side[0].lower()}l', plate(shin_o, 'xz', y_so)))
    z_box = kc[2] - m06['D'] / 2 - 6               # the inner shin plate starts below the knee can
    shin_i = L.stadium([(a_mot[0][0], z_box - 34, m00['D'] + 10), (a_mot[1][0], a_mot[1][2], m00['D'] + 10), (ac[0], ac[2], 40)])
    out.append((f'shin-inner.{side[0].lower()}l', plate(shin_i, 'xz', y_si)))
    # bridge from the outer shin plate to the inner one under the knee can (a yz web), keeps the box closed
    web = L.rounded(min(y_si, y_so) + T, z_box - 16, max(y_si, y_so), z_box, 4)
    out.append((f'shin-web.{side[0].lower()}l', plate(web, 'yz', kc[0] - 20)))
    return out


def structure():
    J, M = KT.world(K.arms_down())
    out = []
    for side in ('Left', 'Right'):
        out += leg(J, side)
    return out
