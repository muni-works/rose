"""Emit the rose arm as a vcad loon program, straight from rose_arm_layout.py.

Zero pose: arm along +x, Z-up, mm. Actuators are envelope cylinders (body +
output boss) from joints(); links are the closed flat-pack boxes from boxes(),
drawn as their outer envelope (windows, tabs and doublers are not modelled yet).
One root per body, never unioned. Nothing here generates geometry; it generates
vcad source.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rose_arm_layout as A

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cad', 'rose-arm.loon')
ROT = {(1, 0, 0): '[rotate 0 90 0 %s]', (-1, 0, 0): '[rotate 0 -90 0 %s]',
       (0, 1, 0): '[rotate -90 0 0 %s]', (0, -1, 0): '[rotate 90 0 0 %s]',
       (0, 0, 1): '%s', (0, 0, -1): '[rotate 180 0 0 %s]'}


def act(j):
    label, a, b, r, model = A.seg(j)
    bdir = j[4]
    cyl = '[cylinder %.1f %.1f]' % (r * 1e3, A.LN(model) * 1e3)
    return '[translate %.1f %.1f %.1f %s]' % (a[0] * 1e3, a[1] * 1e3, a[2] * 1e3, ROT[tuple(bdir)] % cyl)


R_END = 0.008      # corner radius where a link meets a roll drum
R_WAIST = 0.120    # radius that smooths the waist into a curve


def _tangent(P, C, r, upper):
    """Tangent point on circle (C, r) seen from external point P, on the upper or lower side."""
    dx, dz = P[0] - C[0], P[1] - C[1]
    d = math.hypot(dx, dz)
    a = math.atan2(dz, dx)
    b = math.acos(min(1.0, r / d))
    cands = [(C[0] + r * math.cos(t), C[1] + r * math.sin(t)) for t in (a + b, a - b)]
    return max(cands, key=lambda q: q[1]) if upper else min(cands, key=lambda q: q[1])


def outline(prof):
    """Clockwise outline as a list of ('v', (x, z), fillet_r) vertices and ('lobe', (xc, r)) markers.
    Lines join consecutive vertices; a lobe marker is an arc between its neighbours."""
    label, (x0, x1), (y0, y1), ends, waist = prof
    xm = (x0 + x1) / 2
    items = []
    # top edge, left to right
    if ends[0][0] == 'flat':
        items.append(('v', (x0, ends[0][1]), R_END))
    if waist:
        items.append(('v', (xm, waist), R_WAIST))
    if ends[1][0] == 'flat':
        items.append(('v', (x1, ends[1][1]), R_END))
        items.append(('v', (x1, -ends[1][1]), R_END))
    else:
        items.append(('lobe', ends[1]))
    if waist:
        items.append(('v', (xm, -waist), R_WAIST))
    if ends[0][0] == 'flat':
        items.append(('v', (x0, -ends[0][1]), R_END))
    else:
        items.append(('lobe', ends[0]))
    # resolve lobes into tangent points (upper/lower) from their neighbouring vertices
    n = len(items)
    res = []
    for i, it in enumerate(items):
        if it[0] == 'v':
            res.append(it)
            continue
        _, xc, r = it[1]
        prev = items[(i - 1) % n][1]
        nxt = items[(i + 1) % n][1]
        right = xc >= xm
        a = _tangent(prev, (xc, 0.0), r, upper=right)
        b = _tangent(nxt, (xc, 0.0), r, upper=not right)
        res.append(('t', a, 0.0))
        res.append(('arc', (xc, 0.0), r))
        res.append(('t', b, 0.0))
    return res


def profile_solid(prof, origin=(0.0, 0.0, 0.0)):
    """One link segment: its x-z side profile extruded across y, in mm relative to `origin` (m).
    Tangent lines into concentric lobes, filleted corners, smooth waist. Sketch on the y1 face, extruded -y."""
    label, (x0, x1), (y0, y1), ends, waist = prof
    ox, oy, oz = origin
    F = lambda p: '%.3f %.3f' % ((p[0] - ox) * 1e3, p[1] * 1e3)
    items = outline(prof)
    # expand filleted vertices into (in-point, arc, out-point)
    pts = [it for it in items]
    n = len(pts)
    path = []            # sequence of ('p', xy) and ('a', centre, ccw) between consecutive p's
    for i, it in enumerate(pts):
        if it[0] == 'arc':
            path.append(('a', it[1], False))
            continue
        P = it[1]
        rf = it[2]
        prv = pts[(i - 1) % n]
        nxt = pts[(i + 1) % n]
        if rf <= 0 or prv[0] == 'arc' or nxt[0] == 'arc':
            path.append(('p', P))
            continue
        A_, B_ = prv[1], nxt[1]
        u1 = ((A_[0] - P[0]), (A_[1] - P[1])); l1 = math.hypot(*u1); u1 = (u1[0] / l1, u1[1] / l1)
        u2 = ((B_[0] - P[0]), (B_[1] - P[1])); l2 = math.hypot(*u2); u2 = (u2[0] / l2, u2[1] / l2)
        cosang = max(-1.0, min(1.0, u1[0] * u2[0] + u1[1] * u2[1]))
        th = math.acos(cosang)
        if th > math.pi - 1e-3:
            path.append(('p', P))
            continue
        d = rf / math.tan(th / 2)
        dmax = 0.45 * min(l1, l2)
        if d > dmax:
            d = dmax
            rf = d * math.tan(th / 2)
        pa = (P[0] + u1[0] * d, P[1] + u1[1] * d)
        pb = (P[0] + u2[0] * d, P[1] + u2[1] * d)
        bis = (u1[0] + u2[0], u1[1] + u2[1]); bl = math.hypot(*bis); bis = (bis[0] / bl, bis[1] / bl)
        c = (P[0] + bis[0] * rf / math.sin(th / 2), P[1] + bis[1] * rf / math.sin(th / 2))
        # clockwise outline: a right turn (convex) arcs clockwise
        din = (-u1[0], -u1[1])
        turn = din[0] * u2[1] - din[1] * u2[0]
        path += [('p', pa), ('a', c, turn > 0), ('p', pb)]
    # emit segments
    segs = []
    m = len(path)
    for i, it in enumerate(path):
        if it[0] != 'p':
            continue
        j = (i + 1) % m
        if path[j][0] == 'a':
            k = (i + 2) % m
            c, ccw = path[j][1], path[j][2]
            segs.append('[arc %s %s %s %s]' % (F(it[1]), F(path[k][1]), F(c), 'true' if ccw else 'false'))
        else:
            segs.append('[line %s %s]' % (F(it[1]), F(path[j][1])))
    zc = (A.BASE_H - oz) * 1e3
    return ('[extrude 0.0 %.3f 0.0 [sketch 0.0 %.3f %.3f  1.0 0.0 0.0  0.0 0.0 1.0  #[%s]]]'
            % (-(y1 - y0) * 1e3, (y1 - oy) * 1e3, zc, ' '.join(segs)))


def rounded_rect_z(x0, y0, x1, y1, z0, h, r, origin=(0.0, 0.0, 0.0)):
    """A plan-view rounded rectangle extruded up in z (mm in, origin in m)."""
    ox, oy, oz = (v * 1e3 for v in origin)
    x0, x1, y0, y1, z0 = x0 - ox, x1 - ox, y0 - oy, y1 - oy, z0 - oz
    P = lambda x, y: '%.3f %.3f' % (x, y)
    s = ['[line %s %s]' % (P(x0 + r, y0), P(x1 - r, y0)), '[arc %s %s %s true]' % (P(x1 - r, y0), P(x1, y0 + r), P(x1 - r, y0 + r)),
         '[line %s %s]' % (P(x1, y0 + r), P(x1, y1 - r)), '[arc %s %s %s true]' % (P(x1, y1 - r), P(x1 - r, y1), P(x1 - r, y1 - r)),
         '[line %s %s]' % (P(x1 - r, y1), P(x0 + r, y1)), '[arc %s %s %s true]' % (P(x0 + r, y1), P(x0, y1 - r), P(x0 + r, y1 - r)),
         '[line %s %s]' % (P(x0, y1 - r), P(x0, y0 + r)), '[arc %s %s %s true]' % (P(x0, y0 + r), P(x0 + r, y0), P(x0 + r, y0 + r))]
    return '[extrude 0.0 0.0 %.3f [sketch 0.0 0.0 %.3f  1.0 0.0 0.0  0.0 1.0 0.0  #[%s]]]' % (h, z0, ' '.join(s))


def rounded_box(label, p0, p1, origin=(0.0, 0.0, 0.0)):
    """Base pieces: plan-rounded. Radius follows the part's smaller plan side."""
    mm = [v * 1e3 for v in (*p0, *p1)]
    w = min(mm[3] - mm[0], mm[4] - mm[1])
    r = {'base_plate': 30.0, 'col_floor': 12.0, 'column': 18.0, 'grip_plate': 6.0}.get(label, 0.0)
    if r <= 0 or r >= w / 2:
        return None
    return rounded_rect_z(mm[0], mm[1], mm[3], mm[4], mm[2], mm[5] - mm[2], r, origin)


def box(p0, p1):
    return '[translate %.1f %.1f %.1f [cube %.1f %.1f %.1f]]' % (
        p0[0] * 1e3, p0[1] * 1e3, p0[2] * 1e3, (p1[0] - p0[0]) * 1e3, (p1[1] - p0[1]) * 1e3, (p1[2] - p0[2]) * 1e3)


def main(dof7=True):
    roots = []
    out = ['; rose arm - 7-DOF RobStride table arm, tapered flat-pack links (lobes concentric with pitch motors) in 1.5 mm 5052 (Anolex 4040).',
           '; GENERATED by arm/rose_arm_loon.py from arm/rose_arm_layout.py - edit the layout, not this.',
           '; zero pose: arm along +x, shoulder pitch axis at z %.0f. Links never overlap: every pancake' % (A.BASE_H * 1e3),
           '; bolts stator to one box and output to the next. Boxes drawn as envelopes.',
           '[material "actuator" 0.10 0.10 0.11 0.6 0.5]',
           '[material "sheet" 0.78 0.79 0.80 0.9 0.35]',
           '[material "base" 0.55 0.56 0.58 0.9 0.4]', '']
    for j in A.joints(dof7):
        n = 'act_' + j[0]
        out.append('[let %s %s]' % (n, act(j)))
        roots.append((n, 'actuator'))
    prof = {p[0]: p for p in A.profiles(dof7)}
    for label, p0, p1, link in A.boxes(dof7):
        geo = profile_solid(prof[label]) if label in prof else (rounded_box(label, p0, p1) or box(p0, p1))
        out.append('[let %s %s]' % (label, geo))
        roots.append((label, 'base' if link == 'base' else 'sheet'))
    out.append('')
    out += ['[root %s "%s"]' % r for r in roots]
    src = '\n'.join(out) + '\n'
    with open(OUT, 'w') as f:
        f.write(src)
    print('wrote %s  (%d roots)' % (OUT, len(roots)))


if __name__ == '__main__':
    main()
