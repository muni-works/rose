"""Fabrication package for the fork arm, straight from the model (rose_fork.build()).

Writes arm/fab/:
  parts/<name>.dxf    one per unique router part: outline, bolt patterns, bores, windows, slots (R12, mm)
  parts/<name>.svg    preview
  nest/<stock>_<n>.dxf / .svg   every router part of ONE arm nested on the 4040 bed (390 x 390,
                      5 mm margin, 6 mm between parts); free rotation, so long plates go on the diagonal
  BOM.md              motors, stock sheets, purchased parts and every fastener, by interface
  SLEEVES.md, sleeve_*.svg   the turned pitch-joint sleeves, dimensioned from the motor table

Run:  python3 arm/fab.py
"""
import collections
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, 'rig'))
import rose_fork as F
import audit as Au
import dfm as D
import elbow_rig as R      # DXF / SVG writers

OUT = os.environ.get('FAB_OUT') or os.path.join(HERE, 'fab-v10' if F.V10 else 'fab')   # v10 never overwrites the v9 package
BED = float(os.environ.get("SHEET_MM", 381.0))   # 15 x 15 in square sheet (fits the 390 mm 4040 bed); SHEET_MM overrides
MARGIN, GAP, CELL = 5.0, 6.0, 2.0


# ---------------------------------------------------------------- parts
def mm_items(items):
    out = []
    for it in items:
        if it[0] == 'line':
            out.append(('line', (it[1][0] * 1e3, it[1][1] * 1e3), (it[2][0] * 1e3, it[2][1] * 1e3)))
        else:
            out.append(('arc', (it[1][0] * 1e3, it[1][1] * 1e3), (it[2][0] * 1e3, it[2][1] * 1e3),
                        (it[3][0] * 1e3, it[3][1] * 1e3), it[4]))
    return out


ROLE = {('base', 'base'): 'foot', ('turret', 'base'): 'turret-disc', ('upper', 'cover'): 'upper-cover'}


def unique_parts():
    """[(name, thickness mm, qty, outline items, hole item-lists, kg each)] for router parts."""
    groups = collections.OrderedDict()
    for p in F.build():
        proc, t, _ = D.classify(p)
        if not proc.startswith('router'):
            continue
        groups.setdefault(D.key(p), []).append(p)
    out, seen = [], collections.Counter()
    for k, ps in groups.items():
        p = ps[0]
        link, mat = p[0], p[1]
        P = Au.poly(p[5], 0.004)
        w = (max(q[0] for q in P) - min(q[0] for q in P)) * 1e3
        h = (max(q[1] for q in P) - min(q[1] for q in P)) * 1e3
        is_disc = len(p[5]) == 2 and p[5][0][0] == 'arc'
        role = ROLE.get((link, mat)) or ('%s-%s' % (link, 'disc' if is_disc else 'plate'))
        seen[role] += 1
        name = role if seen[role] == 1 else '%s-%d' % (role, seen[role])
        holes = [mm_items(hh) for hh in (p[7] if len(p) > 7 else [])]
        out.append(dict(name=name, t=round(Au.norm(p[6]) * 1e3, 1), qty=len(ps), ol=mm_items(p[5]), holes=holes,
                        kg=F.prim_mass(p), w=w, h=h))
    # names that say what the part is
    rename = {'turret-plate': 'turret-tower-A', 'turret-plate-2': 'turret-tower-B', 'turret-plate-3': 'shoulder-spacer',
              'upper-plate': 'upper-A', 'upper-plate-2': 'upper-B',
              'fore-plate': 'fore-A', 'fore-plate-2': 'fore-B', 'fore-plate-3': 'fore-end-plate', 'fore-plate-4': 'elbow-spacer',
              'yoke-plate': 'yoke-bridge', 'yoke-plate-2': 'yoke-A', 'yoke-plate-3': 'yoke-B',
              'wrist-plate': 'wrist-A', 'wrist-plate-2': 'wrist-B', 'wrist-plate-3': 'wrist-end-plate', 'wrist-plate-4': 'wrist-spacer',
              'tool-disc': 'tool-flange', 'tool-plate': 'jaw', 'turret-disc-2': 'spacer-RS04', 'wrist-disc': 'spacer-RS02'}
    if F.V10:   # v10: the upper arm is two links; the forearm has no J5 cartridge; the desk pedestal
        rename.update({'upper-plate': 'upper-A', 'upper-plate-2': 'upper-B', 'upper-plate-3': 'upper-end-plate',
                       'upperd-plate': 'upperd-bridge', 'upperd-plate-2': 'upperd-A', 'upperd-plate-3': 'upperd-B',
                       'upperd-cover': 'upperd-cover', 'fore-plate-3': 'fore-end-plate', 'fore-plate-4': 'elbow-spacer',
                       'yoke-plate': 'yoke-A', 'yoke-plate-2': 'yoke-B',
                       'base-plate': 'pedestal-X', 'base-plate-2': 'pedestal-Y', 'foot-2': 'pedestal-base', 'upperd-plate-4': 'upperd-cover'})
    for d in out:
        d['name'] = rename.get(d['name'], d['name'])
    return out


# ---------------------------------------------------------------- nesting (bitmask raster, bottom-left)
def xform(items, ang, dx=0.0, dy=0.0):
    c, s = math.cos(ang), math.sin(ang)
    f = lambda p: (c * p[0] - s * p[1] + dx, s * p[0] + c * p[1] + dy)
    out = []
    for it in items:
        if it[0] == 'line':
            out.append(('line', f(it[1]), f(it[2])))
        else:
            out.append(('arc', f(it[1]), f(it[2]), f(it[3]), it[4]))
    return out


def raster(poly, dil):
    """Row bitmasks (cell = CELL mm) of a polygon in its own positive quadrant, dilated by dil cells."""
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    W = int(math.ceil(max(xs) / CELL)) + 1
    H = int(math.ceil(max(ys) / CELL)) + 1
    rows = [0] * H
    n = len(poly)
    for j in range(H):
        for yy in (j * CELL + 0.1, j * CELL + CELL / 2, (j + 1) * CELL - 0.1):
            cuts = []
            for i in range(n):
                (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
                if (y1 > yy) != (y2 > yy):
                    cuts.append(x1 + (yy - y1) * (x2 - x1) / (y2 - y1))
            cuts.sort()
            for a, b in zip(cuts[::2], cuts[1::2]):
                i0, i1 = max(0, int(a // CELL)), min(W - 1, int(b // CELL))
                rows[j] |= ((1 << (i1 - i0 + 1)) - 1) << i0
    # dilate: rows within dil, bits within dil
    grown = []
    for j in range(-dil, H + dil):
        m = 0
        for k in range(j - dil, j + dil + 1):
            if 0 <= k < H:
                m |= rows[k]
        g = m
        for s in range(1, dil + 1):
            g |= (m << s) | (m >> s)
        grown.append(g << dil)
    return grown, W + 2 * dil, H + 2 * dil


class Sheet:
    def __init__(self):
        self.N = int(BED // CELL)
        self.occ = [0] * self.N
        self.parts = []

    def place(self, mask, w, h):
        lo = int(math.ceil(MARGIN / CELL)) - int(round(GAP / 2 / CELL))
        lo = max(0, lo)
        hi = self.N - lo
        for y in range(lo, hi - h + 1):
            for x in range(lo, hi - w + 1):
                if all((self.occ[y + i] & (mask[i] << x)) == 0 for i in range(h)):
                    return x, y
        return None

    def commit(self, mask, x, y):
        for i, m in enumerate(mask):
            self.occ[y + i] |= m << x


def nest(parts):
    """parts: [(name, items, outline-poly)] - largest first, each to the lowest-then-leftmost spot
    on the first sheet that takes it, over 4 right-angle turns, or every 3 deg when it only fits on the diagonal."""
    dil = int(math.ceil(GAP / 2 / CELL))
    sheets = []
    usable = BED - 2 * MARGIN
    order = sorted(parts, key=lambda p: -abs(F.area2d([('line', p[2][i], p[2][(i + 1) % len(p[2])]) for i in range(len(p[2]))])))
    for name, items, poly in order:
        cands = []
        for ang in [k * math.pi / 2 for k in range(4)] + [math.radians(a) for a in range(3, 180, 3) if a % 90]:
            P = [(math.cos(ang) * x - math.sin(ang) * y, math.sin(ang) * x + math.cos(ang) * y) for x, y in poly]
            x0, y0 = min(p[0] for p in P), min(p[1] for p in P)
            P = [(x - x0, y - y0) for x, y in P]
            if max(p[0] for p in P) > usable or max(p[1] for p in P) > usable:
                continue
            cands.append((ang, x0, y0, P))
        right = [c for c in cands if abs(math.degrees(c[0]) % 90) < 1e-6]
        cands = right or cands
        if not cands:
            raise SystemExit('%s does not fit the bed at any angle' % name)
        masks = [(c, raster(c[3], dil)) for c in cands]
        done = False
        for sh in sheets + [None]:
            if sh is None:
                sh = Sheet()
                sheets.append(sh)
            best = None
            for c, (mask, w, h) in masks:
                pos = sh.place(mask, w, h)
                if pos and (best is None or (pos[1], pos[0]) < (best[1][1], best[1][0])):
                    best = (c, pos, mask)
            if best:
                (ang, x0, y0, _), (x, y), mask = best
                sh.commit(mask, x, y)
                ox = (x + dil) * CELL - x0
                oy = (y + dil) * CELL - y0
                sh.parts.append((name, xform(items, ang, ox, oy), math.degrees(ang)))
                done = True
                break
        assert done
    return sheets


# ---------------------------------------------------------------- fasteners, by interface
STD = (4, 5, 6, 8, 10, 12, 14, 16, 18, 20, 25)


def screw(d, stack, depth):
    """Longest standard socket-head screw that stays 0.5 mm short of the bottom of a blind thread."""
    L = max(x for x in STD if x <= stack + depth - 0.5)
    eng = L - stack
    return 'M%d x %d' % (d, L), eng


def fasteners():
    m = lambda pat: 'M%d' % {0.0034: 3, 0.0045: 4, 0.0055: 5}[pat[2]]
    T = S = F.T_PL * 1e3; FL = 2.0     # plate, spacer disc, sleeve flange (mm)
    # (joint, interface, pattern, stack mm, thread depth mm, depth source)
    rows = [
        ('J1 base yaw RS04', 'back face to the foot', F.J1M['back'], F.T_FOOT * 1e3, 6.0, 'manual'),
        ('J1 base yaw RS04', 'output to the turret disc', F.J1M['out'], T, 6.5, 'manual'),
        ('J2 shoulder RS04', 'back face to upper B + sleeve flange', F.J2M['back'], T + FL, 6.0, 'manual'),
        ('J2 shoulder RS04', 'output to spacer + turret tower A', F.J2M['out'], T + S, 6.5, 'manual'),
        ('J4 elbow RS04', 'back face to upper B + sleeve flange', F.J4M['back'], T + FL, 6.0, 'manual'),
        ('J4 elbow RS04', 'output to spacer + fore A', F.J4M['out'], T + S, 6.5, 'manual'),
    ]
    if F.V10:
        rows += [('J3 upper-arm roll RS02', 'back face to the upper end plate', F.J3M['back'], T, 6.0, 'assumed'),
                 ('J3 upper-arm roll RS02', 'output to the upperd bridge', F.J3M['out'], T, 6.0, 'assumed')]
    else:
        rows += [('J5 forearm roll RS02', 'back face to the forearm end plate', F.J5M['back'], T, 6.0, 'assumed'),
                 ('J5 forearm roll RS02', 'output to the yoke bridge', F.J5M['out'], T, 6.0, 'assumed')]
    rows += [
        ('J6 wrist pitch RS02', 'back face to yoke B + sleeve flange', F.J6M['back'], T + FL, 6.0, 'assumed'),
        ('J6 wrist pitch RS02', 'output to spacer + wrist A', F.J6M['out'], T + S, 6.0, 'assumed'),
        ('J7 tool roll RS00', 'back face to the wrist end plate', F.J7M['back'], T, 4.0, 'manual'),
        ('J7 tool roll RS00', 'output to tool flange + gripper top plate (shared screws)', F.J7M['out'], 2 * T, 5.0, 'manual'),
    ]
    out = []
    for j, what, pat, stack, depth, src in rows:
        d = int(m(pat)[1:])
        spec, eng = screw(d, stack, depth)
        out.append((j, what, pat[0], spec, stack, depth, eng, src))
    return out


def cable_length():
    pts = F.cable_points(F.fk({}))
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


# ---------------------------------------------------------------- main
def main():
    os.makedirs(os.path.join(OUT, 'parts'), exist_ok=True)
    os.makedirs(os.path.join(OUT, 'nest'), exist_ok=True)
    parts = unique_parts()
    print('%-18s %5s %4s %8s %8s %6s %7s' % ('part', 't mm', 'qty', 'w', 'h', 'holes', 'g'))
    by_t = collections.defaultdict(list)
    for d in parts:
        items = R.flatten(d['ol'], d['holes'])
        x0, y0, x1, y1 = R.bounds(items)
        fn = '%s_%gmm_x%d' % (d['name'], d['t'], d['qty'])
        R.write_dxf(os.path.join(OUT, 'parts', fn + '.dxf'), R.dxf_entities(items, -x0, -y0))
        R.write_svg(os.path.join(OUT, 'parts', fn + '.svg'), [(items, -x0, -y0)], x1 - x0, y1 - y0, fn)
        print('%-18s %5.1f %4d %8.1f %8.1f %6d %7.0f' % (d['name'], d['t'], d['qty'], x1 - x0, y1 - y0, len(d['holes']), d['kg'] * 1e3))
        poly = [(x, y) for x, y in Au.poly(d['ol'], 2.0)]
        for _ in range(d['qty']):
            by_t[d['t']].append((d['name'], items, poly))
    nests = {}
    for t, ps in sorted(by_t.items()):
        sheets = nest(ps)
        nests[t] = sheets
        for i, sh in enumerate(sheets, 1):
            stem = os.path.join(OUT, 'nest', '%gmm_sheet%d' % (t, i))
            ents = []
            for name, items, ang in sh.parts:
                ents += R.dxf_entities(items)
            R.write_dxf(stem + '.dxf', ents)
            R.write_svg(stem + '.svg', [(items, 0.0, 0.0) for _, items, _ in sh.parts], BED, BED, '%g mm sheet %d' % (t, i))
            print('nest %g mm sheet %d: %s' % (t, i, ', '.join('%s%s' % (n, '' if abs(a % 90) < 1e-6 else ' @%.0f deg' % a) for n, _, a in sh.parts)))
    write_bom(parts, nests)
    write_sleeves()


def write_bom(parts, nests):
    J = [F.J1M, F.J2M, F.J3M if F.V10 else F.J5M, F.J4M, F.J6M, F.J7M]
    name = {id(F.RS04): 'RS04', id(F.RS02): 'RS02', id(F.RS00): 'RS00'}
    motors = collections.Counter(name[id(m)] for m in J)
    fs = fasteners()
    tot = collections.Counter()
    for j, what, n, spec, *_ in fs:
        tot[spec] += n
    tot['M6 x (table) - foot to bench'] += 4
    Lc = cable_length()
    L = []
    w = L.append
    w('# Fork arm %s - bill of materials (one arm)' % ('v10' if F.V10 else 'v9'))
    w('')
    w('Generated by `fab.py` from the model. Router parts are in `parts/`, nested in `nest/`.')
    w('Fastener lengths are the longest standard length that stays 0.5 mm short of the bottom of')
    w('the motor\'s blind thread. Depths marked "assumed" are not on the RS02 drawing; measure before ordering.')
    w('')
    w('## Actuators')
    w('')
    w('| part | qty | where |')
    w('|---|---:|---|')
    w('| RobStride RS04 | %d | J1 base yaw, J2 shoulder, J4 elbow |' % motors['RS04'])
    w('| RobStride RS02 | %d | %s, J6 wrist pitch |' % (motors['RS02'], 'J3 upper-arm roll' if F.V10 else 'J5 forearm roll'))
    w('| RobStride RS00 | %d | J7 tool roll |' % motors['RS00'])
    w('| gripper servo (EDU class, Ø46 x 44) | 1 | jaws |')
    w('')
    w('## Router parts (5052-H32, Anolex 4040, 3.175 mm end mill)')
    w('')
    w('| part | t mm | qty | size mm | holes + slots | g each |')
    w('|---|---:|---:|---|---:|---:|')
    for d in parts:
        w('| %s | %g | %d | %.0f x %.0f | %d | %.0f |' % (d['name'], d['t'], d['qty'], d['w'], d['h'], len(d['holes']), d['kg'] * 1e3))
    w('')
    for t, sh in sorted(nests.items()):
        w('Stock, %g mm: **%d sheet%s** of %.0f x %.0f mm.' % (t, len(sh), "" if len(sh) == 1 else "s", BED, BED))
    w('')
    w('Covers and end plates are located by tabs in slots (10 mm tabs, slots 0.4 mm long and 0.2 mm wide')
    w('over). They are captured between the side plates once the motors are bolted, so they need no')
    w('screws; a drop of retaining compound on each tab stops rattle.')
    w('')
    w('## Turned, printed, purchased')
    w('')
    w('| part | qty | note |')
    w('|---|---:|---|')
    w('| sleeve, RS04 pitch joints (6061) | 2 | J2, J4 - see SLEEVES.md |')
    w('| sleeve, RS02 wrist pitch (6061) | 1 | J6 - see SLEEVES.md |')
    w('| flanged plain bushing 16 x 18 x 3, flange 24 (iglidur G GFM-1618 family) | 2 | J2, J4 straddler plate B |')
    w('| flanged plain bushing 10 x 12 x 3, flange 18 (iglidur G GFM-1012 family) | 1 | J6 straddler plate B |')
    w('| retaining ring DIN 471-16 | 2 | sleeve ends, J2, J4 |')
    w('| retaining ring DIN 471-10 | 1 | sleeve end, J6 |')
    w('| hub cap, closed cup (printed PETG, orange) | 3 | Ø40 x 12 at J2, J4; Ø32 x 10 at J6 |')
    w('| base collar (printed PETG) | 1 | round the J1 motor on the foot |')
    w('| flat harness: CAN pair + 48 V pair | %.1f m | model route at zero pose is %.2f m; +30 %% for the joint loops and service |' % (math.ceil(Lc * 1.3 * 10) / 10 + 0.5, Lc))
    w('| XT30 (2+2) connector pairs | 7 | one per actuator, daisy chain, plus the base lead |')
    w('')
    w('## Fasteners, by interface')
    w('')
    w('| joint | interface | qty | screw | stack mm | thread depth mm | engagement mm |')
    w('|---|---|---:|---|---:|---:|---:|')
    for j, what, n, spec, stack, depth, eng, src in fs:
        w('| %s | %s | %d | %s | %g | %g%s | %g |' % (j, what, n, spec, stack, depth, '' if src == 'manual' else ' (assumed)', eng))
    w('')
    w('### Order list')
    w('')
    w('| screw (ISO 4762 socket head, A2 or 12.9 black) | qty | + spares |')
    w('|---|---:|---:|')
    for spec, n in sorted(tot.items()):
        w('| %s | %d | %d |' % (spec, n, max(2, math.ceil(n * 0.1))))
    w('')
    w('Total: %d screws per arm.' % sum(tot.values()))
    with open(os.path.join(OUT, 'BOM.md'), 'w') as f:
        f.write('\n'.join(L) + '\n')
    print('BOM: %d screws, %d unique router parts' % (sum(tot.values()), len(parts)))



# ---------------------------------------------------------------- sleeves
def sleeve_dims(m, spigot, bush, ring, bore):
    """Turned sleeve for a pitch joint whose holder plate B bolts to motor m's back face."""
    n, bc, dh = m['back']
    bc, dh = bc * 1e3, dh * 1e3
    step = {id(F.RS04): 90.0, id(F.RS02): 36.0}[id(m)]       # = plate B's centre hole
    gap, T = F.CABLE_GAP * 1e3, F.T_PL * 1e3
    fl_t = 2.0
    fl_d = math.ceil(bc + dh + 8.0)
    L_sp = (gap - fl_t) + T + 1.5 + 0.2 + 1.1 + 1.5          # running gap, plate, bushing flange, float, groove, land
    return dict(n=n, bc=bc, dh=dh, step=step - 0.2, step_L=T - 0.2, fl_d=fl_d, fl_t=fl_t, spigot=spigot,
                L_sp=round(L_sp, 1), bore=bore, bush=bush, ring=ring, run=gap - fl_t)


def sleeve_kg(d):
    v = math.pi * ((d['step'] / 2) ** 2 - (d['step'] / 2 - 4) ** 2) * d['step_L'] + math.pi * ((d['fl_d'] / 2) ** 2 - (d['bore'] / 2) ** 2) * d['fl_t'] \
        + math.pi * ((d['spigot'] / 2) ** 2 - (d['bore'] / 2) ** 2) * d['L_sp']
    return 2700e-9 * v


SLEEVES = [
    ('RS04', 'J2 shoulder, J4 elbow (and the elbow rig)', 3, F.RS04, 16.0, '16 x 18 x 3, flange 24', 'DIN 471-16', 11.0),
    ('RS02', 'J6 wrist pitch', 1, F.RS02, 10.0, '10 x 12 x 3, flange 18', 'DIN 471-10', 7.0),
]


def sleeve_svg(path, d, title):
    """Half section, axis horizontal, motor side on the left. mm, scaled 4x."""
    k, X0, Y0 = 4.0, 30.0, 40.0
    r_fl, r_st, r_sp, r_b = d['fl_d'] / 2, d['step'] / 2, d['spigot'] / 2, d['bore'] / 2
    x_st0, x_fl0 = 0.0, d['step_L']
    x_fl1 = x_fl0 + d['fl_t']
    x_end = x_fl1 + d['L_sp']
    r_si = r_st - 4.0
    prof = [(x_st0, r_si), (x_st0, r_st), (x_fl0, r_st), (x_fl0, r_fl), (x_fl1, r_fl), (x_fl1, r_sp), (x_end, r_sp), (x_end, r_b), (x_fl0, r_b), (x_fl0, r_si)]
    H = Y0 + r_fl * k + 60
    W = X0 + x_end * k + 220
    P = lambda x, r: (X0 + x * k, Y0 + (r_fl - r) * k)
    pts = ' '.join('%.1f,%.1f' % P(x, r) for x, r in prof)
    ax0, ax1 = P(-3, 0), P(x_end + 3, 0)
    g = x_end - 1.5 - 1.1
    txt = [
        'flange Ø%.0f x %.0f, %dx Ø%.1f on Ø%.0f' % (d['fl_d'], d['fl_t'], d['n'], d['dh'], d['bc']),
        'step ring Ø%.1f / Ø%.1f x %.1f (locates in plate B)' % (d['step'], d['step'] - 8, d['step_L']),
        'spigot Ø%.0f g6 x %.1f, Ra 0.8' % (d['spigot'], d['L_sp']),
        'bore Ø%.0f through (harness), edges R1' % d['bore'],
        'cable slot 10 wide, radial through flange + step',
        '%s groove 1.5 from the end' % d['ring'],
        '6061-T6',
    ]
    body = ['<polygon points="%s" fill="#f3d9c9" stroke="#e8590c" stroke-width="1.2"/>' % pts,
            '<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#888" stroke-dasharray="8 3 2 3"/>' % (ax0 + ax1),
            '<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" fill="#fff" stroke="#e8590c" stroke-width="0.8"/>' % (
                P(g, r_sp)[0], P(g, r_sp)[1], 1.1 * k, 0.4 * k)]
    for i, t in enumerate(txt):
        body.append('<text x="%.0f" y="%.0f" font-family="Helvetica" font-size="13">%s</text>' % (X0 + x_end * k + 20, Y0 + 16 + 18 * i, t))
    body.append('<text x="%.0f" y="%.0f" font-family="Helvetica" font-size="15" font-weight="bold">%s</text>' % (X0, 22, title))
    with open(path, 'w') as f:
        f.write('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %.0f %.0f" width="%.0f" height="%.0f">'
                '<rect width="100%%" height="100%%" fill="#fff"/>%s</svg>\n' % (W, H, W, H, ''.join(body)))


def write_sleeves():
    L = ['# Pitch-joint sleeves (turned, 6061-T6)', '',
         'Generated by `fab.py`. One sleeve per pitch joint. It locates in the holder\'s plate B, clamps',
         'under the motor\'s back-face screws, and carries the straddler\'s plate B on a flanged plain',
         'bushing. The harness enters through the radial slot in the flange, inside the %.0f mm cable gap,' % (F.CABLE_GAP * 1e3),
         'twists along the bore, and turns under the hub cap. Nothing else moves against it.', '',
         'Stack, motor side first: motor back face, holder plate B (3) with the sleeve step in its centre',
         'hole, sleeve flange (2), running gap (2) where the flange faces the straddler plate (a safe pinch gap,',
         'under 4 mm), straddler plate B (3) on the bushing, bushing flange (1.5) outboard, retaining ring,',
         'then the hub cap, bonded to straddler plate B with a VHB ring.', '']
    for name, where, qty, m, sp, bush, ring, bore in SLEEVES:
        d = sleeve_dims(m, sp, bush, ring, bore)
        fn = 'sleeve_%s.svg' % name
        sleeve_svg(os.path.join(OUT, fn), d, 'sleeve %s - %s (%d off)' % (name, where, qty))
        L += ['## %s sleeve - %s, %d off' % (name, where, qty), '', '![%s](%s)' % (fn, fn), '',
              '| feature | size |', '|---|---|',
              '| step | ring Ø%.1f / Ø%.1f x %.1f, locates in plate B\'s Ø%.0f centre hole, stops 0.2 short of the motor |' % (d['step'], d['step'] - 8, d['step_L'], d['step'] + 0.2),
              '| mass | %.0f g |' % (sleeve_kg(d) * 1e3),
              '| flange | Ø%.0f x %.0f, %dx Ø%.1f on Ø%.0f, clamped with plate B by the back-face screws |' % (d['fl_d'], d['fl_t'], d['n'], d['dh'], d['bc']),
              '| spigot | Ø%.0f g6 x %.1f, Ra 0.8: running fit for the %s bushing |' % (d['spigot'], d['L_sp'], bush),
              '| bore | Ø%.0f through, edges broken R1 (harness) |' % d['bore'],
              '| cable slot | 10 wide, radial, through the flange and step into the bore |',
              '| groove | %s, 1.5 from the spigot end |' % ring, '']
    L += ['The RS02 sleeve\'s Ø7 bore takes the flat harness but not a connector: thread it before crimping.', '',
          'The elbow rig uses the RS04 sleeve, so the rig qualifies this exact part.']
    with open(os.path.join(OUT, 'SLEEVES.md'), 'w') as f:
        f.write('\n'.join(L) + '\n')


if __name__ == '__main__':
    main()
