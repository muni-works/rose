"""Elbow test rig for the fork joint: one RS04 on a bench stand, a test forearm straddling it.
(v5, 2026-10-13: the elbow is an RS04 - RS03 is not available. Shoulder and elbow are the same joint,
so this rig also qualifies the shoulder.)

What it proves before a whole arm gets cut: the stator-held-both-ends mount, the spacer disc
through the parent plate, the plain bushing on the sleeve (wobble), cable twist through the
sleeve (life), and real torque / heat against the sim.

Writes, from one set of dimensions (interfaces from ../INTERFACES.md):
  cut/*.dxf          one per plate, R12 LINE/ARC/CIRCLE, mm, outline + holes + windows
  cut/sheet_3mm.dxf  the 3 mm plates nested on the Anolex 4040 bed
  cut/*.svg          previews
  ../rig/BOM.md, ../rig/TURNED.md are hand-written next to this script

Run:  python3 arm/fork/rig/elbow_rig.py
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'cut')
BED = (390.0, 390.0)

# ---- interfaces (mm), RS04, from the user manual drawing (RS04 manual p. 9)
OUT_BC, OUT_N = 36.0, 8           # output: 8x M5 deep 6.5 on 36 +-0.1 (2x D5 dowels on 35.86)
OUT_HOLE = 5.5
RING_BC, RING_N = 106.0, 10       # stator front: 10x M4 EQS blind (5 min) on 106 +-0.1
BACK_BC, BACK_N = 106.0, 10       # back: 10x M4 EQS deep 6 on 106 +-0.1
MOTOR_D = 120.0
STATOR_SPAN = 52.2                # 13.2 + 39
CLEAR_D = 70.0                    # parent plate A bore round the output flange and spacer disc
M3, M4, M5, M8 = 3.4, 4.5, 5.5, 8.5   # clearance holes

# ---- rig choices
T = 3.0                           # plates, 5052
AXIS_H = 200.0                    # bench to joint axis
FOOT_W = 180.0
R_LOBE = MOTOR_D / 2 + 6.0
ARM_L = 270.0                     # joint axis to load point (RS04 lobe; fits the 4040 bed with margin)
R_TIP = 34.0
SPACER_D, SPACER_T = 48.0, 3.0   # on the output flange, through parent plate A's bore
SLEEVE_D = 16.0                   # sleeve spigot; child plate B rides a flanged bushing on it
BUSH_OD = 18.0                    # igus GFM-1618 style: ID 16 / OD 18 / flange 24
WIN_H, WIN_FLANGE = 30.0, 20.0


# ---------------------------------------------------------------- geometry helpers
def tangent(P, C, r, upper):
    dx, dz = P[0] - C[0], P[1] - C[1]
    d = math.hypot(dx, dz)
    a = math.atan2(dz, dx)
    b = math.acos(min(1.0, r / d))
    cands = [(C[0] + r * math.cos(t), C[1] + r * math.sin(t)) for t in (a + b, a - b)]
    return max(cands, key=lambda q: q[1]) if upper else min(cands, key=lambda q: q[1])


def capsule(L, r1, r2):
    th = math.acos(max(-1.0, min(1.0, (r1 - r2) / L)))
    p1t = (r1 * math.cos(th), r1 * math.sin(th))
    p2t = (L + r2 * math.cos(th), r2 * math.sin(th))
    p2b, p1b = (p2t[0], -p2t[1]), (p1t[0], -p1t[1])
    return [('line', p1t, p2t), ('arc', p2t, p2b, (L, 0.0), False),
            ('line', p2b, p1b), ('arc', p1b, p1t, (0.0, 0.0), False)]


def tower():
    """Stand plate: lobe on the axis (0, 0), tangent flanks down to a foot at -AXIS_H."""
    fl, fr = (-FOOT_W / 2, -AXIS_H), (FOOT_W / 2, -AXIS_H)
    tl, tr = tangent(fl, (0, 0), R_LOBE, True), tangent(fr, (0, 0), R_LOBE, True)
    return [('line', fl, fr), ('line', fr, tr), ('arc', tr, tl, (0, 0), True), ('line', tl, fl)]


def stadium(x0, x1, z, h):
    a, b = x0 + h, x1 - h
    return [('line', (a, z + h), (b, z + h)), ('arc', (b, z + h), (b, z - h), (b, z), False),
            ('line', (b, z - h), (a, z - h)), ('arc', (a, z - h), (a, z + h), (a, z), False)]


def bolt_circle(bc, n, d, phase=0.0, c=(0.0, 0.0)):
    return [('circle', (c[0] + bc / 2 * math.cos(phase + 2 * math.pi * k / n),
                        c[1] + bc / 2 * math.sin(phase + 2 * math.pi * k / n)), d / 2) for k in range(n)]


# ---------------------------------------------------------------- the parts
def parent_A():
    """Stand plate on the stator front: 10x M4 on 106, bore round the output."""
    holes = [('circle', (0, 0), CLEAR_D / 2)] + bolt_circle(RING_BC, RING_N, M4, math.pi / RING_N)
    holes += [('circle', (x, -AXIS_H + 12), M5 / 2) for x in (-60, -20, 20, 60)]       # to the 2020 foot rails
    holes += [stadium(-30, 30, -125, 18)]
    return tower(), holes


def parent_B():
    """Stand plate on the back cover. The sleeve boss bolts through it with the same 4x M4."""
    holes = [('circle', (0, 0), 10.0)] + bolt_circle(BACK_BC, BACK_N, M4, math.pi / BACK_N)
    holes += [('circle', (x, -AXIS_H + 12), M5 / 2) for x in (-60, -20, 20, 60)]
    holes += [stadium(-30, 30, -125, 18)]
    return tower(), holes


def child(kind):
    """Test forearm plate. A bolts to the spacer disc; B rides the bushing on the sleeve."""
    ol = capsule(ARM_L, R_LOBE, R_TIP)
    if kind == 'A':
        holes = bolt_circle(OUT_BC, OUT_N, OUT_HOLE)
    else:
        holes = [('circle', (0, 0), BUSH_OD / 2)]
    holes += [('circle', (ARM_L, 0), M8 / 2)]                                         # load bolt
    # bottom 2020 bar, T-nuts: 12 mm in from the tapered bottom edge (the edge runs from R_LOBE to R_TIP)
    edge = lambda x: R_LOBE + (R_TIP - R_LOBE) * x / ARM_L
    holes += [('circle', (x, -edge(x) + 12), M5 / 2) for x in (100, 150, 200, 235)]
    holes += [stadium(R_LOBE + 12, 180, 6, 18), stadium(190, ARM_L - R_TIP - 10, 6, 12)]
    return ol, holes


def spacer_disc():
    return [('circle', (0, 0), SPACER_D / 2)], bolt_circle(OUT_BC, OUT_N, OUT_HOLE)


PARTS = {
    'parent_A': (parent_A, 3.0, 1), 'parent_B': (parent_B, 3.0, 1),
    'child_A': (lambda: child('A'), 3.0, 1), 'child_B': (lambda: child('B'), 3.0, 1),
    'spacer_disc': (spacer_disc, 3.0, 1),
}


# ---------------------------------------------------------------- DXF / SVG
def bounds(items):
    xs, ys = [], []
    for it in items:
        if it[0] == 'circle':
            (cx, cy), r = it[1], it[2]
            xs += [cx - r, cx + r]
            ys += [cy - r, cy + r]
        elif it[0] == 'line':
            xs += [it[1][0], it[2][0]]
            ys += [it[1][1], it[2][1]]
        else:
            p, q, c = it[1], it[2], it[3]
            r = math.hypot(p[0] - c[0], p[1] - c[1])
            xs += [c[0] - r, c[0] + r]
            ys += [c[1] - r, c[1] + r]
    return min(xs), min(ys), max(xs), max(ys)


def flatten(outline, holes):
    items = list(outline)
    for h in holes:
        if isinstance(h, list):
            items += h
        else:
            items.append(h)
    return items


def dxf_entities(items, dx=0.0, dy=0.0, layer='CUT'):
    out = []
    for it in items:
        if it[0] == 'line':
            (x1, y1), (x2, y2) = it[1], it[2]
            out += ['0', 'LINE', '8', layer, '10', '%.4f' % (x1 + dx), '20', '%.4f' % (y1 + dy),
                    '11', '%.4f' % (x2 + dx), '21', '%.4f' % (y2 + dy)]
        elif it[0] == 'circle':
            (cx, cy), r = it[1], it[2]
            out += ['0', 'CIRCLE', '8', layer, '10', '%.4f' % (cx + dx), '20', '%.4f' % (cy + dy), '40', '%.4f' % r]
        else:
            p, q, c, ccw = it[1], it[2], it[3], it[4]
            r = math.hypot(p[0] - c[0], p[1] - c[1])
            a0 = math.degrees(math.atan2(p[1] - c[1], p[0] - c[0]))
            a1 = math.degrees(math.atan2(q[1] - c[1], q[0] - c[0]))
            s, e = (a0, a1) if ccw else (a1, a0)     # DXF arcs run counter-clockwise from start to end
            out += ['0', 'ARC', '8', layer, '10', '%.4f' % (c[0] + dx), '20', '%.4f' % (c[1] + dy),
                    '40', '%.4f' % r, '50', '%.4f' % (s % 360), '51', '%.4f' % (e % 360)]
    return out


def write_dxf(path, ents):
    with open(path, 'w') as f:
        f.write('\n'.join(['0', 'SECTION', '2', 'HEADER', '9', '$INSUNITS', '70', '4', '0', 'ENDSEC',
                           '0', 'SECTION', '2', 'ENTITIES'] + ents + ['0', 'ENDSEC', '0', 'EOF']) + '\n')


def svg_path(items, dx, dy, H):
    d = []
    for it in items:
        if it[0] == 'line':
            (x1, y1), (x2, y2) = it[1], it[2]
            d.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f"/>' % (x1 + dx, H - (y1 + dy), x2 + dx, H - (y2 + dy)))
        elif it[0] == 'circle':
            (cx, cy), r = it[1], it[2]
            d.append('<circle cx="%.2f" cy="%.2f" r="%.2f"/>' % (cx + dx, H - (cy + dy), r))
        else:
            p, q, c, ccw = it[1], it[2], it[3], it[4]
            r = math.hypot(p[0] - c[0], p[1] - c[1])
            a0 = math.atan2(p[1] - c[1], p[0] - c[0])
            a1 = math.atan2(q[1] - c[1], q[0] - c[0])
            sweep = (a1 - a0) % (2 * math.pi) if ccw else (a0 - a1) % (2 * math.pi)
            large = 1 if sweep > math.pi else 0
            sf = 0 if ccw else 1          # SVG y is flipped
            d.append('<path d="M %.2f %.2f A %.2f %.2f 0 %d %d %.2f %.2f"/>' % (
                p[0] + dx, H - (p[1] + dy), r, r, large, sf, q[0] + dx, H - (q[1] + dy)))
    return d


def write_svg(path, placed, W, H, title):
    body = []
    for items, dx, dy in placed:
        body += svg_path(items, dx, dy, H)
    with open(path, 'w') as f:
        f.write('<svg xmlns="http://www.w3.org/2000/svg" viewBox="-5 -5 %.0f %.0f" width="%.0fmm" height="%.0fmm">\n'
                '<title>%s</title><rect x="0" y="0" width="%.0f" height="%.0f" fill="none" stroke="#bbb" stroke-dasharray="4 3"/>\n'
                '<g fill="none" stroke="#e8590c" stroke-width="0.6">\n%s\n</g></svg>\n'
                % (W + 10, H + 10, W + 10, H + 10, title, W, H, '\n'.join(body)))


def nest(parts, gap=8.0, margin=5.0):
    """Shelf-pack parts (largest first) onto as many bed-sized sheets as needed. Rotates a part
    90 deg when that is the only way it fits a sheet."""
    sheets = []
    for name, items in sorted(parts, key=lambda p: -(bounds(p[1])[2] - bounds(p[1])[0]) * (bounds(p[1])[3] - bounds(p[1])[1])):
        placed = False
        for rot in (False, True):
            its = [rotate_item(it) for it in items] if rot else items
            x0, y0, x1, y1 = bounds(its)
            w, h = x1 - x0, y1 - y0
            if w > BED[0] - 2 * margin or h > BED[1] - 2 * margin:
                continue
            for sh in sheets:
                if sh['x'] + w <= BED[0] - margin and sh['y'] + h <= BED[1] - margin:
                    sh['parts'].append((name, its, sh['x'] - x0, sh['y'] - y0))
                    sh['x'] += w + gap
                    sh['row'] = max(sh['row'], h)
                    placed = True
                    break
                if sh['y'] + sh['row'] + gap + h <= BED[1] - margin and margin + w <= BED[0] - margin:
                    sh['y'] += sh['row'] + gap
                    sh['x'], sh['row'] = margin, 0.0
                    sh['parts'].append((name, its, sh['x'] - x0, sh['y'] - y0))
                    sh['x'] += w + gap
                    sh['row'] = h
                    placed = True
                    break
            if placed:
                break
            if not placed:
                sheets.append(dict(x=margin + w + gap, y=margin, row=h, parts=[(name, its, margin - x0, margin - y0)]))
                placed = True
                break
        if not placed:
            raise SystemExit('%s does not fit the %.0f x %.0f bed' % (name, BED[0], BED[1]))
    return sheets


def rotate_item(it):
    r = lambda p: (-p[1], p[0])
    if it[0] == 'line':
        return ('line', r(it[1]), r(it[2]))
    if it[0] == 'circle':
        return ('circle', r(it[1]), it[2])
    return ('arc', r(it[1]), r(it[2]), r(it[3]), it[4])


def main():
    os.makedirs(OUT, exist_ok=True)
    by_t = {}
    print('%-14s %5s %8s %8s %6s' % ('part', 't', 'w', 'h', 'holes'))
    for name, (fn, t, qty) in PARTS.items():
        ol, holes = fn()
        items = flatten(ol, holes)
        x0, y0, x1, y1 = bounds(items)
        write_dxf(os.path.join(OUT, '%s_%gmm.dxf' % (name, t)), dxf_entities(items, -x0, -y0))
        write_svg(os.path.join(OUT, '%s.svg' % name), [(items, -x0, -y0)], x1 - x0, y1 - y0, name)
        print('%-14s %4.0f  %6.1f  %6.1f  %5d' % (name, t, x1 - x0, y1 - y0, sum(1 for it in items if it[0] == 'circle')))
        for _ in range(qty):
            by_t.setdefault(t, []).append((name, items))
    for t, parts in by_t.items():
        sheets = nest(parts)
        for i, sh in enumerate(sheets, 1):
            ents = []
            for name, items, dx, dy in sh['parts']:
                ents += dxf_entities(items, dx, dy)
            base = os.path.join(OUT, 'sheet_%gmm_%d' % (t, i))
            write_dxf(base + '.dxf', ents)
            write_svg(base + '.svg', [(it, dx, dy) for _, it, dx, dy in sh['parts']], BED[0], BED[1],
                      '%g mm 5052, sheet %d of %d' % (t, i, len(sheets)))
            print('%g mm sheet %d/%d: %s' % (t, i, len(sheets), ', '.join(n for n, _, _, _ in sh['parts'])))


if __name__ == '__main__':
    main()
