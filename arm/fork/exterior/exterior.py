"""lymb exterior: the cortex / quad00 design language over the fork arm (v9). EXTERIOR ONLY.

Nothing here moves a plate, a motor, a joint or the harness. Every part is a cover that sits over the
structure rose_fork.build() already makes, placed in its link's frame, and its shape is one of the three
loon functions in shell.loon (pad, flush perforated insert, accent ring). The loops come from the arm
model's own numbers, so ROSE_ARM=mini dresses the mini arm with the same code.

The rule that keeps the fork's range: a cover lives only in a y-slab its link owns at that joint. The
straddler is outermost at a pitch joint, so it gets the cheek fairing; the link it straddles gets flush
inserts in its own plate windows (zero thickness added outside the plate) and nothing on its sides.

  base    drum      a soft drum over the yaw RS04 and its collar, bored for the cassette
  turret  cheeks    two dead-front fairings over the tower plates (they enclose the shoulder hub cap)
  upper   inserts   flush perforated dot-grid inserts filling both plates' windows
  fore    cheeks    two fairings over the forearm plates (elbow hub cap and end-plate edges inside)
          sleeve    (full arm) a tube over the J5 roll motor
  wrist   hubs      two round dead-front hubs over the wrist lobes (wrist hub cap inside)
          sleeve    a tube over the J7 tool-roll motor, harness inside, a rolled lip at the tool
          accent    the one neon ring, at the root of the J7 sleeve

Run:  python3 arm/fork/exterior/exterior.py --summary        grams per joint
      python3 arm/fork/exterior/exterior.py --clash [step]   covers vs structure, vs each other, vs harness
      python3 arm/fork/exterior/exterior.py '{"J2_sho_pitch": -40}' [cream|black] > pose.loon
"""
import itertools
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', '..'))
import rose_fork as F
from rose_arm_geom import add, sub, mul, dot, cross, norm, unit, mv, mm_

T = F.T_PL
WALL = 0.002          # moulded PC/ABS wall (cortex tub: 2.5)
SKIN, SKIN_GAP = 0.0012, 0.0003   # turned skins (drum, motor sleeves, ring): +1.5 mm of radius. The tool and wrist
                                  # can pass within ~3 mm of the base, so these stay as thin as a print allows
RHO = 1150.0          # PC/ABS, kg/m^3
STAND = 0.0005        # a cover stands this far off the plate face it covers
CHEEK_R = 0.010       # the generous face round
INS_CLR = 0.00015     # insert to window clearance, each side
DOT_P, DOT_R = 0.0030, 0.0008     # arm-scale dot grid: 1.6 mm holes on a 3.0 mm hex (cortex: 1.4 on 2.2)

# master-plan palette (sRGB): warm black #1e1813, cream #e8e1d2, one small neon family accent
SHELL = {'cream': (0.910, 0.882, 0.824), 'black': (0.118, 0.094, 0.075)}
NEON = (0.78, 1.0, 0.24)


# ---------------------------------------------------------------- loops (m), convex, CCW
def circle_pts(cu, cv, r, n=48):
    return [(cu + r * math.cos(2 * math.pi * k / n), cv + r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def hull(pts):
    pts = sorted(set((round(p[0], 7), round(p[1], 7)) for p in pts))
    cr = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, hi = [], []
    for p in pts:
        while len(lo) >= 2 and cr(lo[-2], lo[-1], p) <= 1e-12:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi) >= 2 and cr(hi[-2], hi[-1], p) <= 1e-12:
            hi.pop()
        hi.append(p)
    return lo[:-1] + hi[:-1]           # CCW


def resample(loop, step=0.004):
    """Densify long straight edges so the miter offset stays exact and lofts stay smooth."""
    out = []
    for i, a in enumerate(loop):
        b = loop[(i + 1) % len(loop)]
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
        out += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(n)]
    return out


def to_uv(pts_xz, u, v, o):
    """World-zero points given in a plate's plane -> the cover's (u, v), CCW."""
    P = [(dot(sub(p, o), u), dot(sub(p, o), v)) for p in pts_xz]
    a = sum(P[i][0] * P[(i + 1) % len(P)][1] - P[(i + 1) % len(P)][0] * P[i][1] for i in range(len(P)))
    return P if a > 0 else P[::-1]


def area_perim(loop):
    a = sum(loop[i][0] * loop[(i + 1) % len(loop)][1] - loop[(i + 1) % len(loop)][0] * loop[i][1] for i in range(len(loop))) / 2
    p = sum(math.hypot(loop[(i + 1) % len(loop)][0] - loop[i][0], loop[(i + 1) % len(loop)][1] - loop[i][1]) for i in range(len(loop)))
    return abs(a), p


def stadium_pts(s0, s1, h, n=12):
    a, b = s0 + h, s1 - h
    pts = [(b + h * math.cos(-math.pi / 2 + math.pi * k / n), h * math.sin(-math.pi / 2 + math.pi * k / n)) for k in range(n + 1)]
    pts += [(a + h * math.cos(math.pi / 2 + math.pi * k / n), h * math.sin(math.pi / 2 + math.pi * k / n)) for k in range(n + 1)]
    return pts


def quarter(cx, cz, r, a0, a1, n=8):
    """Arc points from angle a0 to a1 (rad) about (cx, cz), both ends included."""
    return [(cx + r * math.cos(a0 + (a1 - a0) * k / n), cz + r * math.sin(a0 + (a1 - a0) * k / n)) for k in range(n + 1)]


def cup_profile(rb, ro, h, r, t):
    """(radius, height) profiles of a turned cup: outer radius ro, depth h, face round r, wall t, bored rb.
    Returns (material, envelope), both closed CCW loops in the (radius, height) half plane."""
    ri = max(r - t, 0.0006)
    mat = [(ro - t, 0.0), (ro, 0.0)] + quarter(ro - r, h - r, r, 0.0, math.pi / 2)[:-1] + [(ro - r, h), (rb, h), (rb, h - t)] \
        + quarter(ro - r, h - r, ri, math.pi / 2, 0.0)[1:]
    env = [(rb, 0.0), (ro, 0.0)] + quarter(ro - r, h - r, r, 0.0, math.pi / 2)[:-1] + [(ro - r, h), (rb, h)]
    return mat, env


def tube_profile(ri, ro, h):
    """A sleeve: wall ri..ro, height h, its far outer edge rolled with a full wall-radius round."""
    t = ro - ri
    mat = [(ri, 0.0), (ro, 0.0)] + quarter(ri, h - t, t, 0.0, math.pi / 2)[:-1] + [(ri, h)]
    return mat, mat


# ---------------------------------------------------------------- the covers
def cover(name, link, kind, o, u, v, loop, **kw):
    n = cross(u, v)
    return dict(name=name, link=link, kind=kind, o=o, u=unit(u), v=unit(v), n=unit(n), loop=loop, **kw)


def cheek(name, link, side, y_face, pts_xz, h, r=CHEEK_R):
    """A pad over a plate whose outer face is the plane y = side * y_face; pts_xz are (x, z) points of the
    pad's footprint on the shoulder-axis plane (z measured from Z_SH)."""
    o = (0.0, side * (y_face + STAND), F.Z_SH)
    u = (1.0, 0.0, 0.0)
    v = (0.0, 0.0, -1.0) if side > 0 else (0.0, 0.0, 1.0)      # u x v = +-y, outward
    pts = [(x, side * (y_face + STAND), F.Z_SH + z) for x, z in hull(pts_xz)]
    loop = resample(to_uv(pts, u, v, o))
    return cover(name, link, 'pad', o, u, v, loop, h=h, r=r, t=WALL, mat='shell')


def covers():
    C = []
    J1M, J2M, J4M, J5M, J6M, J7M = F.J1M, F.J2M, F.J4M, F.J5M, F.J6M, F.J7M
    S = F.stations()
    x5, x_cp, x_ep, xt = S['x5'], S['x_cp'], S['x_ep'], S['xt']
    # ---- base: the drum. Inner wall 2 mm outside the printed collar; top 3 mm under the turret disc;
    #      bored inside the yaw motor's rim so the harness and the cassette pass up to the turret.
    zo = F.T_FOOT + J1M['L']
    r_drum = J1M['d'] / 2 + 0.010 + SKIN_GAP + SKIN
    o = (0.0, 0.0, F.T_FOOT + STAND)
    h = zo - 0.003 - o[2]
    mat, env = cup_profile(J1M['d'] / 2 + 0.0065, r_drum, h, min(CHEEK_R, h / 3), SKIN)
    C.append(cover('base drum', 'base', 'turned', o, (1, 0, 0), (0, 1, 0), [], prof=mat, env=env, mat='shell'))
    # ---- turret: cheeks over both tower plates, foot clipped 1 mm over the turret disc
    rd = J1M['d'] / 2 + 0.010
    h_t = F.Z_SH - (zo + T)
    m = 0.002
    foot = -h_t + 0.001
    tower = circle_pts(0, 0, F.R_LOBE_SH + m) + [(-(rd + m), foot), (rd + m, foot)]
    tower = [p for p in tower if p[1] >= foot]
    cap_h = 0.012
    for side, yc in ((1, F.SH_CA), (-1, F.SH_CB)):
        C.append(cheek('turret cheek %s' % ('+y' if side > 0 else '-y'), 'turret', side, yc + T / 2, tower,
                       h=cap_h + 0.0045))
    # ---- upper: flush perforated inserts in both plates' windows (same thickness as the plate)
    for side, yo, keep0, keep1 in ((1, F.SH_PA, J2M['nose_d'] / 2 + 0.002, J4M['nose_d'] / 2 + 0.002),
                                   (-1, F.SH_PB, J2M['back'][1] / 2 + J2M['back'][2], J4M['back'][1] / 2 + J4M['back'][2])):
        h_cv = F.R_LOBE_EL - 0.012
        w = F.windows(F.UPPER, F.R_LOBE_SH, F.R_LOBE_EL, keep0, keep1, T, h_cv - T - 0.0062)
        if not w:
            continue
        hh = w[0][0][1][1]                      # stadium(s0, s1, h): first line (s0 + h, h) -> (s1 - h, h)
        s0, s1 = w[0][0][1][0] - hh, w[0][0][2][0] + hh
        loop_s = stadium_pts(s0 + INS_CLR, s1 - INS_CLR, hh - INS_CLR)
        o = (0.0, side * (yo - T / 2), F.Z_SH)          # from the plate's inner face, flush with both faces
        u = (1.0, 0.0, 0.0)
        v = (0.0, 0.0, -1.0) if side > 0 else (0.0, 0.0, 1.0)
        pts = [(s, o[1], F.Z_SH + z) for s, z in loop_s]
        loop = to_uv(pts, u, v, o)
        cu = (s0 + s1) / 2
        # the dot field: a centred pill like the cortex ear (~280 holes), the rest of the insert a calm dead-front
        # plane. The whole window full of dots was ~1200 holes and would not evaluate in reasonable time.
        a, b = min((s1 - s0) / 2 - 0.006, 0.040), min(hh - 0.006, 0.015)
        ports = []
        if side < 0:     # the harness crosses plate B just outside each motor (rose_fork.cable_points)
            for xs in (J2M['d'] / 2 + 0.004, F.UPPER - J4M['d'] / 2 - 0.004):
                pu, pv = dot(sub((xs, o[1], F.Z_SH), o), u), dot(sub((xs, o[1], F.Z_SH), o), v)
                pr = F.R_CABLE + 0.001
                if poly_inside(pu, pv, loop) or edge_dist(pu, pv, loop) < pr:     # a notch when it grazes the edge
                    ports.append((pu, pv, pr))
        C.append(cover('upper insert %s' % ('+y' if side > 0 else '-y'), 'upper', 'insert', o, u, v, loop,
                       th=T, field=(cu, 0.0, a, b), ports=ports, mat='insert'))
    # ---- forearm: cheeks over both plates, past the end plate's edges by the wall
    m3 = 0.003
    # past the end plate's edges by the wall -- unless a J5 roll motor sits right behind it: it is wider than
    # the forearm's plates (RS02 r 39 vs 35), so the cheek stops at the end plate and the plate shows as a seam
    x_end = x_cp + T + WALL + 0.0005 if F.NO_J5 else x_cp - 0.0005
    fore = circle_pts(F.UPPER, 0, F.R_LOBE_EL + m) + [(x_end, -(0.026 if F.NO_J5 else 0.036) - m3), (x_end, (0.026 if F.NO_J5 else 0.036) + m3)]
    for side, yc in ((1, F.EL_CA), (-1, F.EL_CB)):
        C.append(cheek('fore cheek %s' % ('+y' if side > 0 else '-y'), 'fore', side, yc + T / 2, fore, h=cap_h + 0.0045))
    if not F.NO_J5:     # J5 roll motor: a sleeve between the forearm end plate and the yoke bridge
        r_in = J5M['d'] / 2 + SKIN_GAP
        o = (x_cp + T + 0.0005, 0.0, F.Z_SH)
        L = x5 - 0.005 - o[0]       # stops short of where the harness leaves for the J5 cassette
        mat, env = tube_profile(r_in, r_in + SKIN, L)
        C.append(cover('fore J5 sleeve', 'fore', 'turned', o, (0, 1, 0), (0, 0, 1), [], prof=mat, env=env, mat='shell'))
    # ---- wrist: round hubs over the wrist lobes. A full cheek here would sweep the forearm cheeks at
    #      J6 +-106 on the mini (its corners reach 74 mm from the axis); a disc on the axis sweeps onto itself.
    for side, yc in ((1, F.WR_CA), (-1, F.WR_CB)):
        C.append(cheek('wrist hub %s' % ('+y' if side > 0 else '-y'), 'wrist', side, yc + T / 2,
                       circle_pts(F.W, 0, F.R_LOBE6W + m), h=0.010 + 0.0045, r=0.008))
    # ---- J7 sleeve over the motor only, and the accent ring at its root. (Sized round the harness, 6 mm under
    #      the motor, it grew the wrist 10 mm and cost folded poses; the harness stays where it runs today.)
    r_in = J7M['d'] / 2 + SKIN_GAP
    x0 = x_ep + T + 0.0005                                    # just past the wrist end plate
    ring_t = 0.002
    o = (x0, 0.0, F.Z_SH)
    C.append(cover('wrist accent ring', 'wrist', 'ring', o, (0, 1, 0), (0, 0, 1), circle_pts(0, 0, r_in + SKIN),
                   r0=r_in, r1=r_in + SKIN, th=ring_t, mat='accent'))
    o = (x0 + ring_t + 0.0003, 0.0, F.Z_SH)
    mat, env = tube_profile(r_in, r_in + SKIN, xt - 0.0055 - o[0])     # ends before the harness turns to the tool
    C.append(cover('wrist J7 sleeve', 'wrist', 'turned', o, (0, 1, 0), (0, 0, 1), [], prof=mat, env=env, mat='shell'))
    return C


# ---------------------------------------------------------------- inside tests (cover-local, m)
def poly_inside(x, y, P):
    c = False
    for i in range(len(P)):
        (x1, y1), (x2, y2) = P[i], P[(i + 1) % len(P)]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            c = not c
    return c


def edge_dist(x, y, P):
    best = 1e9
    for i in range(len(P)):
        (x1, y1), (x2, y2) = P[i], P[(i + 1) % len(P)]
        dx, dy = x2 - x1, y2 - y1
        t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy or 1e-18)))
        best = min(best, math.hypot(x - x1 - t * dx, y - y1 - t * dy))
    return best


def round_inset(w, h, r):
    """Inset of a pad's outline at height w under a face at h with a quarter round r (0 below the round)."""
    if w <= h - r:
        return 0.0
    rho = h - w
    return r - math.sqrt(max(0.0, r * r - (r - rho) ** 2))


def solid_depth(c, q, hollow=False):
    """How deep the cover-local point q = (u, v, w) is inside the cover's material (m, <= 0 outside).
    hollow=False tests the cover's whole envelope (what another link must not enter); True its material only."""
    x, y, w = q
    L = c['loop']
    if c['kind'] == 'insert':
        if not (0.0 <= w <= c['th']) or not poly_inside(x, y, L):
            return 0.0
        if any(math.hypot(x - pu, y - pv) < pr for pu, pv, pr in c.get('ports', ())):
            return 0.0
        return min(w, c['th'] - w, edge_dist(x, y, L))
    if c['kind'] == 'turned':
        P = c['prof'] if hollow else c['env']
        rr = math.hypot(x, y)
        return edge_dist(rr, w, P) if poly_inside(rr, w, P) else 0.0
    if c['kind'] == 'ring':
        rr = math.hypot(x, y)
        if not (0.0 <= w <= c['th']) or not (c['r0'] <= rr <= c['r1']):
            return 0.0
        return min(w, c['th'] - w, rr - c['r0'], c['r1'] - rr)
    h, r, t = c['h'], c['r'], c['t']
    if not (0.0 <= w <= h) or not poly_inside(x, y, L):
        return 0.0
    ed = edge_dist(x, y, L)
    d_out = min(ed - round_inset(w, h, r), h - w)
    if d_out <= 0:
        return 0.0
    if 'bore' in c:
        cu, cv, rb = c['bore']
        if math.hypot(x - cu, y - cv) < rb:
            return 0.0
        d_out = min(d_out, math.hypot(x - cu, y - cv) - rb)
    if not hollow:
        return d_out
    # material = envelope minus the cavity (inset t, its face at h - t, its round r - t)
    m_in = (ed - t - round_inset(w, h - t, max(r - t, 0.0006))) if w <= h - t else -(w - (h - t))
    return 0.0 if m_in > 0 else min(d_out, -m_in)


def surface_pts(c, step=0.004):
    """Sample points on a cover's outer surface (cover-local, m)."""
    out = []
    L = c['loop']
    if c['kind'] == 'insert':
        A = [(x, y, w) for x, y in resample(L, step) for w in (0.0002, c['th'] / 2, c['th'] - 0.0002)]
        return A
    if c['kind'] == 'turned':
        out = []
        for rr, w in resample(c['env'], 0.003):
            n = max(12, int(2 * math.pi * rr / step))
            out += [(rr * math.cos(2 * math.pi * k / n), rr * math.sin(2 * math.pi * k / n), w) for k in range(n)]
        return out
    if c['kind'] == 'ring':
        return [(c['r1'] * math.cos(a) * s, c['r1'] * math.sin(a) * s, w) for a in [2 * math.pi * k / 48 for k in range(48)]
                for s in (c['r0'] / c['r1'] + 0.01, 0.995) for w in (0.0002, c['th'] - 0.0002)]
    h, r = c['h'], c['r']
    ws = [k * h / max(1, int(h / step)) for k in range(int(h / step) + 1)]
    ws = sorted(set([min(max(w, 0.0002), h - 0.0002) for w in ws] + [h - r * (1 - math.cos(a)) for a in (0.3, 0.7, 1.1, 1.4)]))
    for w in ws:
        d = round_inset(w, h, r) + 0.0002
        out += [(x, y, w) for x, y in resample(inset_loop(L, d), step)]
    # the face
    face = inset_loop(L, r + 0.0005)
    xs = [p[0] for p in face]
    ys = [p[1] for p in face]
    k = 0.008
    x = min(xs)
    while x < max(xs):
        y = min(ys)
        while y < max(ys):
            if poly_inside(x, y, face) and not ('bore' in c and math.hypot(x - c['bore'][0], y - c['bore'][1]) < c['bore'][2]):
                out.append((x, y, h - 0.0002))
            y += k
        x += k
    return out


def inset_loop(P, d):
    """Python twin of shell.loon's x-inset."""
    n = len(P)
    def nrm(a, b):
        du, dv = b[0] - a[0], b[1] - a[1]
        l = math.hypot(du, dv) or 1e-12
        return (-dv / l, du / l)
    out = []
    for i in range(n):
        p = P[i]
        a = nrm(P[(i - 1) % n], p)
        b = nrm(p, P[(i + 1) % n])
        k = d / (1.0 + a[0] * b[0] + a[1] * b[1])
        out.append((p[0] + k * (a[0] + b[0]), p[1] + k * (a[1] + b[1])))
    return out


# ---------------------------------------------------------------- placement
def frame_R(c):
    """Rotation taking canonical (x, y, z) to the cover's (u, v, n), as rows."""
    u, v, n = c['u'], c['v'], c['n']
    return ((u[0], v[0], n[0]), (u[1], v[1], n[1]), (u[2], v[2], n[2]))


def world_of(Tk, c, q):
    """Cover-local q -> posed world."""
    p0 = add(c['o'], add(mul(c['u'], q[0]), add(mul(c['v'], q[1]), mul(c['n'], q[2]))))
    R, t = Tk[c['link']]
    return add(mv(R, sub(p0, F.ORIGIN[c['link']])), t)


def local_of(Tk, c, p):
    """Posed world p -> cover-local."""
    R, t = Tk[c['link']]
    d = sub(p, t)
    pz = add(tuple(sum(R[k][i] * d[k] for k in range(3)) for i in range(3)), F.ORIGIN[c['link']])
    d = sub(pz, c['o'])
    return (dot(d, c['u']), dot(d, c['v']), dot(d, c['n']))


# ---------------------------------------------------------------- loon
SHELL_LOON = open(os.path.join(HERE, 'shell.loon')).read()


def f3(x):
    return '%.3f' % (x * 1e3)


def body_loon(c):
    pts = '#[%s]' % ' '.join('#[%s %s]' % (f3(p[0]), f3(p[1])) for p in c['loop'])
    if c['kind'] == 'pad':
        if 'bore' in c:
            cu, cv, rb = c['bore']
            return '[x-pad-bored %s %s %s %s %s %s %s]' % (pts, f3(c['h']), f3(c['r']), f3(c['t']), f3(cu), f3(cv), f3(rb))
        return '[x-pad %s %s %s %s]' % (pts, f3(c['h']), f3(c['r']), f3(c['t']))
    if c['kind'] == 'insert':
        cu, cv, a, b = c['field']
        ni, nk = int(a / DOT_P) + 1, int(b / (2 * DOT_P * 0.866025)) + 1
        uc = dot(sub((cu, c['o'][1], F.Z_SH + cv), c['o']), c['u'])
        vc = dot(sub((cu, c['o'][1], F.Z_SH + cv), c['o']), c['v'])
        ports = '#[%s]' % ' '.join('#[%s %s %s]' % (f3(pu), f3(pv), f3(pr)) for pu, pv, pr in c.get('ports', ()))
        return '[x-insert %s 0.0 %s %s %s %s %s %s %s %d %d %s]' % (pts, f3(c['th']), f3(DOT_P), f3(DOT_R), f3(uc), f3(vc), f3(a), f3(b), ni, nk, ports)
    if c['kind'] == 'turned':
        return '[x-turned #[%s]]' % ' '.join('#[%s %s]' % (f3(p[0]), f3(p[1])) for p in c['prof'])
    if c['kind'] == 'ring':
        return '[x-ring %s %s 0.0 %s]' % (f3(c['r0']), f3(c['r1']), f3(c['th']))
    raise ValueError(c['kind'])


def materials(shell='cream'):
    s = SHELL[shell]
    ins = SHELL['black' if shell == 'cream' else 'cream']
    return ('[material "shell" %.3f %.3f %.3f 0.0 0.42]\n[material "insert" %.3f %.3f %.3f 0.0 0.55]\n'
            '[material "accent" %.3f %.3f %.3f 0.0 0.30]\n' % (s + ins + NEON))


def covers_loon(q, at=(0.0, 0.0, 0.0), prefix='', R0=F.I3, C=None):
    """The posed covers as loon lets + roots (shell.loon's functions must be in the file once)."""
    T0 = F.fk(q)
    Tk = {k: (mm_(R0, Rk), add(mv(R0, tk), at)) for k, (Rk, tk) in T0.items()}
    L, roots = [], []
    for i, c in enumerate(C or covers()):
        R, t = Tk[c['link']]
        Rw = mm_(R, frame_R(c))
        tw = add(mv(R, sub(c['o'], F.ORIGIN[c['link']])), t)
        L.append('[let %se%d %s]' % (prefix, i, F.place_loon(Rw, tw, body_loon(c))))
        roots.append('[root %se%d "%s"]' % (prefix, i, c['mat']))
    return '\n'.join(L), '\n'.join(roots)


def world_loon(q, shell='cream', with_structure=True):
    lets, roots = covers_loon(q)
    head = '; lymb exterior over the fork arm. GENERATED by arm/fork/exterior/exterior.py\n' + SHELL_LOON + '\n' + materials(shell)
    struct = F.world_loon(q) if with_structure else ''
    return head + '\n' + struct + '\n' + lets + '\n\n' + roots + '\n'


# ---------------------------------------------------------------- numbers
JOINT_OF = {'base': 'J1 base yaw', 'turret': 'J1 / J2 turret', 'upper': 'J2 / J4 upper arm', 'fore': 'J4 / J5 forearm',
            'yoke': 'J5 / J6 yoke', 'wrist': 'J6 / J7 wrist', 'tool': 'J7 tool'}


def mass(c):
    L = c['loop']
    if c['kind'] == 'insert':
        A, _ = area_perim(L)
        cu, cv, a, b = c['field']
        n_holes = A / (DOT_P * DOT_P * 0.866025) * (4 * a * b - (4 - math.pi) * b * b) / A
        return RHO * c['th'] * (A - n_holes * math.pi * DOT_R ** 2)
    if c['kind'] == 'ring':
        return RHO * c['th'] * math.pi * (c['r1'] ** 2 - c['r0'] ** 2)
    if c['kind'] == 'turned':     # Pappus: 2 pi * centroid radius * area
        P = c['prof']
        A = cx = 0.0
        for i in range(len(P)):
            (x1, y1), (x2, y2) = P[i], P[(i + 1) % len(P)]
            cr = x1 * y2 - x2 * y1
            A += cr / 2
            cx += (x1 + x2) * cr / 6
        return RHO * 2 * math.pi * abs(cx)
    def vol(L, h, r, d0):
        A, P = area_perim(L)
        n = 40
        v = 0.0
        for k in range(n):
            w = h * (k + 0.5) / n
            d = d0 + round_inset(w, h, r)
            v += max(0.0, A - P * d + math.pi * d * d) * h / n
        return v
    h, r, t = c['h'], c['r'], c['t']
    v = vol(L, h, r, 0.0) - vol(L, h - t, max(r - t, 0.0006), t)
    if 'bore' in c:
        rb = c['bore'][2]
        A, P = area_perim(L)
        face_t = t
        v -= math.pi * rb * rb * face_t if rb < math.sqrt(A / math.pi) - t - 1e-4 else 0.0
    return RHO * v


def summary():
    C = covers()
    by = {}
    print('lymb exterior (%s arm): %d covers' % (F.SIZE, len(C)))
    for c in C:
        g = mass(c) * 1e3
        by.setdefault(c['link'], []).append((c['name'], g))
        print('  %-22s %-7s %6.0f g' % (c['name'], c['kind'], g))
    print('  per joint (the link the cover rides on):')
    tot = 0.0
    for link, items in by.items():
        s = sum(g for _, g in items)
        tot += s
        print('    %-20s %6.0f g' % (JOINT_OF[link], s))
    print('  total %.0f g' % tot)
    return by


if __name__ == '__main__':
    a = sys.argv[1:]
    if a and a[0] == '--summary':
        summary()
    elif a and a[0] == '--clash':
        import clash
        clash.main(float(a[1]) if len(a) > 1 else 10.0)
    else:
        q = {}
        if a:
            q = {k: math.radians(v) for k, v in json.loads(open(a[0]).read() if os.path.exists(a[0]) else a[0]).items()}
        sys.stdout.write(world_loon(q, a[1] if len(a) > 1 else 'cream'))
