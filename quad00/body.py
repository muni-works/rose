"""quad00 body, side-hung layout: two flat decks + cross ribs, open sides.

Plan view (X forward, Y left), the deck outline:

    tail | rear neck |        belly          | front neck | chest
     ±35 |   ±35     |         ±88           |    ±35     |  ±88

The necks are where the hip clusters tuck in (their yoke flanges stop 38.4 mm off the centre line).
The chest and the belly are as wide as the ab/ad motors they hold: the thighs swing just outside
(E-clips at |y| 92.1), so there is no room for a side wall -- the sides stay open and the decks do it.

Structure, all flat 3.2 mm, tab-and-slot:
  deck top / bottom   plan profile, split at x = 0 so every piece fits the 4040's 390 mm bed
  nose wall           2 layers, front ab/ad pair (back faces), the chest's front
  mid wall            2 layers, rear ab/ad pair, the belly's rear
  rib A, rib 0        belly front; the splice where the deck halves meet
  tail rib            the tail's end

  python3 body.py   ->  checks, mass + CoM, out/dxf/body-*.dxf
"""
import math
import os

import ezdxf
import numpy as np
from manifold3d import CrossSection, FillRule, Manifold
from shapely import affinity
from shapely.geometry import box
from shapely.ops import unary_union

import leg as L

HERE = os.path.dirname(os.path.abspath(__file__))

def configure(plane=87.0, flange_outer=None, hip_x=210.0, ends=True):
    """(re)derive every body dimension from the leg plane and how far inboard the hip cluster reaches.
    ends=False: belly only -- no chest, necks or tail; the front legs' ab/ad point back into the belly,
    and the body stops short of every hip cluster in x, so abduction can never bring a cluster into it."""
    global HIP_X, PLANE, YA, CLIP, WIDE, NECK, X_NOSE, X_MID, X_CHEST0, X_BELLY1, X_BELLY0, X_TAIL, RIBS, ENDS, X_END, INTERNALS
    HIP_X, PLANE = hip_x, plane
    flange_outer = L.Y_FLANGE + L.YOKE_T if flange_outer is None else flange_outer
    YA = PLANE + L.ABAD_Y                            # ab/ad axes
    CLIP = PLANE - L.Y_FASTEN                        # thigh's innermost point (E-clip)
    WIDE = CLIP - 4.0                                # chest / belly half-width: 4 mm from the swinging thigh
    NECK = PLANE - flange_outer - 3.0                # neck half-width: 3 mm from the hip cluster
    X_NOSE = HIP_X + L.X_BULK[0]                     # nose wall back face (front ab/ad back faces)
    X_MID = -HIP_X + L.X_BULK[0]                     # mid wall
    X_CHEST0 = HIP_X + L.X_ABAD[0]                   # chest starts at the front ab/ad body
    X_BELLY1 = HIP_X - 46.0                          # belly ends ahead of the front hip clusters
    X_BELLY0 = -HIP_X + L.X_ABAD[0]                  # belly starts at the rear ab/ad bodies
    X_TAIL = -HIP_X - 70.0
    RIBS = {'nose': (X_NOSE, 2 * T, WIDE), 'rib-a': (X_BELLY1 - T, T, WIDE), 'rib-0': (-T / 2, T, WIDE),
            'mid': (X_MID, 2 * T, WIDE), 'tail': (X_TAIL, T, NECK)}
    INTERNALS = INTERNALS_ENDS
    ENDS = ends
    X_END = HIP_X - 46.0                             # belly-only: ends 46 short of each hip (hip gear tip r42.5)
    if not ends:
        wall = HIP_X - L.X_BULK[0]                   # ab/ad back faces, both ends: 119.8
        RIBS = {'nose': (wall - 2 * T, 2 * T, WIDE), 'mid': (-wall, 2 * T, WIDE)}
        INTERNALS = INTERNALS_BELLY


H_IN = 41.0                                          # decks' inner faces at z = +/-41 (ab/ad motor r 28.5)
H_TOP, H_BOT = H_IN, H_IN                            # top deck inner face at +H_TOP, bottom at -H_BOT
T = L.T
# internals: (name, size x y z, centre, kg)
INTERNALS_ENDS = [
    ('battery', (140, 126, 44), (82, 0, -18), 1.70),       # 12S2P 21700, 44.4 V 10 Ah: 24 cells, 2 x 6 x 2 lying
    ('compute', (95, 63, 28), (-60, 0, 22), 0.12),         # Raspberry Pi 5 + CAN hat, on the top deck
    ('power', (60, 45, 22), (-60, 0, -26), 0.10),          # DC-DC 48 -> 5 V, e-stop relay, fuse
]
INTERNALS_BELLY = [('battery', (140, 126, 44), (40, 0, -18), 1.70),
                   ('compute', (95, 63, 28), (-60, 0, 22), 0.12),
                   ('power', (60, 45, 22), (-80, 0, -26), 0.10)]
configure(87.0)                                      # leg planes at +/-87: 301 overall (Go2 ~310)
TAB_W = 16.0


def deck_outline():
    """plan profile (x, y) of a deck, concave corners rounded r6, convex r10."""
    if not ENDS:
        return L.rounded(-X_END, -WIDE, X_END, WIDE, 22)
    p = unary_union([box(X_TAIL, -NECK, X_NOSE + 2 * T, NECK),
                     box(X_CHEST0, -WIDE, X_NOSE + 2 * T, WIDE),
                     box(X_BELLY0, -WIDE, X_BELLY1, WIDE)])
    return p.buffer(6, join_style=1).buffer(-16, join_style=1).buffer(10, join_style=1)


def tab_ys(half):
    return (-half / 2, half / 2) if half > 50 else (0.0,)


def deck(part):
    """one deck half ('front' x > 0, 'rear' x < 0) with slots for every rib's tabs."""
    d = deck_outline()
    if part != 'whole':
        d = d.intersection(box(0, -500, 1000, 500) if part == 'front' else box(-1000, -500, 0, 500))
    for x0, t, half in RIBS.values():
        for y in tab_ys(half):
            d = d.difference(box(x0 - 0.1, y - TAB_W / 2 - 0.1, x0 + t + 0.1, y + TAB_W / 2 + 0.1))
    spans = ((X_TAIL + 20, X_MID - 20), (X_MID + 30, -30), (30, X_BELLY1 - 30)) if ENDS else \
        ((-X_END + 70, -20), (20, X_END - 70))
    for x0, x1 in spans:                                                     # lightening
        if x1 - x0 > 40:
            w = NECK - 12 if (ENDS and x0 < X_MID) else WIDE - 26
            d = d.difference(L.rounded(x0, -w, x1, w, 12))
    return d


def rib(name):
    """YZ profile (u = y, v = z) of a rib or wall, tabs up and down into the decks."""
    x0, t, half = RIBS[name]
    r = box(-half, -H_BOT, half, H_TOP)
    for y in tab_ys(half):
        r = r.union(box(y - TAB_W / 2, -H_BOT - T, y + TAB_W / 2, H_TOP + T))
    if name in ('nose', 'mid'):
        for s in (1, -1):
            r = r.difference(L.holes_on(L.BACK_PCD, L.BACK_N, L.M3, phase=45, at=(s * YA, 0)))
            r = r.difference(L.circle(s * YA, 0, 16))
        r = r.difference(L.rounded(-18, -26, 18, 26, 6))                      # harness pass
    else:
        w = half - 14
        if w > 12:
            r = r.difference(L.rounded(-w, -26, w, 26, 8))
    return r


WIRING_KG = 0.25


def plates():
    """(name, manifold in the body frame, cut profile, qty) for every body plate."""
    out = []
    for part in (('front', 'rear') if ENDS else ('whole',)):
        prof = deck(part)
        cs = CrossSection(L.rings(prof), FillRule.EvenOdd)
        out.append((f'deck-{part}-top', cs.extrude(T).translate((0, 0, H_TOP)), prof, 1))
        out.append((f'deck-{part}-bottom', cs.extrude(T).translate((0, 0, -H_BOT - T)), prof, 1))
    for name, (x0, t, half) in RIBS.items():
        prof = rib(name)
        m = L.to_world(CrossSection(L.rings(prof), FillRule.EvenOdd).extrude(t).translate((0, 0, x0)), 'yz')
        out.append((name, m, prof, int(round(t / T))))
    return out


def internals():
    return [(n, Manifold.cube(s).translate(tuple(c[i] - s[i] / 2 for i in range(3))), kg) for n, s, c, kg in INTERNALS]


def leg_mass():
    """per leg, from leg.py's solids: printed-PETG or aluminium structure + 3 RS00."""
    P = L.parts()
    v = sum(L.solid_manifold(s).volume() * s['qty'] for n, s in P.items() if n != 'bulkhead') + L.yoke_manifold(L.YOKE_T).volume()
    return v / 1000, 3 * 0.310 + 0.08                  # cm3 of structure, kg of motors + hardware


def report(alu=True):
    rho = 2.70 if alu else 1.27 * 0.55
    items = []
    for name, m, _, _ in plates():
        items.append((name, m.volume() / 1000 * rho / 1000, L.tri(m).center_mass))
    for name, m, kg in internals():
        items.append((name, kg, L.tri(m).center_mass))
    items.append(('wiring', WIRING_KG, np.array([0, 0, 0])))
    vleg, kleg = leg_mass()
    for x in (HIP_X, -HIP_X):
        for s in (1, -1):
            items.append(('leg', vleg * rho / 1000 + kleg, np.array([x + 10, s * PLANE, -30])))   # leg CoM ~ ahead/below hip
    total = sum(k for _, k, _ in items)
    com = sum(k * c for _, k, c in items) / total
    body = sum(k for n, k, _ in items if n not in ('leg',))
    print(f'mass ({"aluminium" if alu else "printed PETG"} structure): total {total:.2f} kg '
          f'(legs {total - body:.2f}, body {body:.2f}), CoM x {com[0]:+.0f} mm from the hips\' midpoint, z {com[2]:+.0f}')
    return total, com


def check():
    ok = True
    if ENDS:
        print(f'body: length {X_NOSE + 2 * T - X_TAIL:.0f}, chest/belly +/-{WIDE:.1f}, necks +/-{NECK:.1f}, '
              f'height {2 * (H_IN + T):.0f}')
    else:
        print(f'body (belly only): {2 * X_END:.0f} x {2 * WIDE:.0f} x {H_TOP + H_BOT + 2 * T:.0f}; '
              f'ends {HIP_X - X_END:.0f} mm short of each hip axis')
    g = CLIP - WIDE
    print(f'thigh E-clip (y {CLIP:.1f}) to deck edge: {g:.1f} mm  (the thigh never crosses the body)')
    ok &= g >= 3
    if ENDS:
        g = PLANE - (L.Y_FLANGE + L.YOKE_T) - NECK
        print(f'yoke flange to neck: {g:.1f} mm')
        ok &= g >= 3
    reach = -L.Z_KNEEAX + L.M * (L.Z_KNEE + 2) / 2
    gap = 2 * HIP_X - 2 * reach
    print(f'front/rear thighs both swung fully toward each other: {"clear by %.0f mm" % gap if gap > 0 else "overlap %.0f mm (limit in software)" % -gap}')
    for n, (s, c, _) in [(n, (s, c, k)) for n, s, c, k in INTERNALS]:
        fits = abs(c[1]) + s[1] / 2 <= WIDE - T and c[2] - s[2] / 2 >= -H_BOT and c[2] + s[2] / 2 <= H_TOP
        print(f'  {n:8s} {s[0]}x{s[1]}x{s[2]} at x {c[0]:+.0f}: {"fits" if fits else "DOES NOT FIT"}')
        ok &= fits
    return ok


def write_dxf():
    d = os.path.join(HERE, 'out', 'dxf')
    os.makedirs(d, exist_ok=True)
    for f in os.listdir(d):
        if f.startswith('body-'):
            os.remove(os.path.join(d, f))
    worst = 0
    for name, _, prof, layers in plates():
        if name.endswith('-bottom'):
            continue                                             # same profile as the top deck: cut 2
        n = 2 if name.startswith('deck') else 1
        doc = ezdxf.new('R2000')
        msp = doc.modelspace()
        for ring in L.rings(prof):
            msp.add_lwpolyline([tuple(q) for q in ring], close=True)
        x0, y0, x1, y1 = prof.bounds
        worst = max(worst, x1 - x0, y1 - y0)
        doc.saveas(os.path.join(d, f'body-{name.replace("-top", "")}_{T}mm_x{n * layers}.dxf'))
    print(f'body DXFs written; largest plate {worst:.0f} mm (router bed 390)')
    return worst <= 390


if __name__ == '__main__':
    ok = check()
    report(alu=True)
    report(alu=False)
    ok &= write_dxf()
    raise SystemExit(0 if ok else 1)
