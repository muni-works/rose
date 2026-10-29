"""Printable node shell: hollow tub + floor plate, every hole cut. Run: python3 node_print.py

Builds straight from node_skull's section polygons in manifold3d (no vcad booleans: a loft minus a few hundred
holes is what kept breaking vcad), so the output is watertight by construction. Needs numpy, trimesh, manifold3d.

Writes ../fab/node/node-tub.stl and node-floor.stl in the model frame (dock origin, z up) and node-tub-print.stl
laid for the printer (crown up, open bottom on the bed... see NODE-BRIEF.md for supports).
"""
import node_skull as sk
import math, os
import numpy as np
import manifold3d as mf
import trimesh
from node_skull import section, BROW, FACE, ZS, VS

WALL = 2.0                      # printed shell wall: exactly 4 x 0.5 mm lines (2 loops each face), no gap fill
RAIN_DROP = 0.58                # tan 30: side holes fall 30 deg toward the outside
FLOOR = 3.0
N = 144                         # points per section
OUT = os.path.join(os.path.dirname(__file__) or '.', '..', 'fab', 'node')


def offset(pts, d):
    """Move each vertex of a CCW polygon inward by d along its vertex normal."""
    out = []
    n = len(pts)
    for i in range(n):
        (x0, y0), (x1, y1), (x2, y2) = pts[i - 1], pts[i], pts[(i + 1) % n]
        tx, ty = x2 - x0, y2 - y0
        l = math.hypot(tx, ty) or 1.0
        out.append((x1 - ty / l * d, y1 + tx / l * d))
    return out

def loft(rings):
    """Closed solid through rings of (z, pts), all with the same point count, capped at both ends."""
    v, f = [], []
    n = len(rings[0][1])
    for z, p in rings:
        v += [(x, y, z) for x, y in p]
    for k in range(len(rings) - 1):
        a, b = k * n, (k + 1) * n
        for i in range(n):
            j = (i + 1) % n
            f += [(a + i, a + j, b + j), (a + i, b + j, b + i)]
    for k, flip in ((0, True), (len(rings) - 1, False)):
        c = len(v)
        z, p = rings[k]
        v.append((sum(x for x, _ in p) / n, sum(y for _, y in p) / n, z))
        base = k * n
        for i in range(n):
            j = (i + 1) % n
            f.append((c, base + j, base + i) if flip else (c, base + i, base + j))
    m = mf.Manifold(mf.Mesh(vert_properties=np.array(v, np.float32), tri_verts=np.array(f, np.uint32)))
    if m.volume() <= 0 or m.status() != mf.Error.NoError:
        f = [(a, c, b) for a, b, c in f]
        m = mf.Manifold(mf.Mesh(vert_properties=np.array(v, np.float32), tri_verts=np.array(f, np.uint32)))
    assert m.volume() > 0, m.status()
    return m

def rod(p, d, r, length=12.0):
    """A cylinder of radius r centred on point p along direction d."""
    d = np.array(d, float) / np.linalg.norm(d)
    c = mf.Manifold.cylinder(length, r, r, 16, True)
    z = np.array([0, 0, 1.0])
    ax = np.cross(z, d)
    s, cth = np.linalg.norm(ax), float(np.dot(z, d))
    if s < 1e-9:
        R = np.eye(3) if cth > 0 else np.diag([1, -1, -1.0])
    else:
        k = ax / s
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        R = np.eye(3) + s * K + (1 - cth) * K @ K
    T = np.hstack([R, np.array(p, float).reshape(3, 1)])
    return c.transform(T)

def rrect_prism(x0, x1, cy, cz, hy, hz, r):
    """Rounded rectangle in the y-z plane, extruded along x from x0 to x1."""
    pts = []
    for cyc, czc, a0 in ((hy - r, hz - r, 0), (-(hy - r), hz - r, 90), (-(hy - r), -(hz - r), 180), (hy - r, -(hz - r), 270)):
        for k in range(9):
            a = math.radians(a0 + k * 90 / 8)
            pts.append((cy + cyc + r * math.cos(a), cz + czc + r * math.sin(a)))
    cs = mf.CrossSection([pts])
    m = mf.Manifold.extrude(cs, x1 - x0)           # along z, sketch in x-y = (y, z)
    # map (u, v, w) = (y, z, x) -> (x, y, z)
    T = np.array([[0, 0, 1, x0], [1, 0, 0, 0], [0, 1, 0, 0.0]])
    return m.transform(T)

def outline_at(z):
    return section(z, N)

def holes():
    out = []
    # Rain: every hole that isn't on the underside runs DOWN as it goes out (outer end lower, RAIN_DROP), so water
    # that lands in a hole runs back out instead of in. The underside intake already points 45 deg down.
    # ears: the 0.6-radius dots from node_skull, through along y, dropping toward the outside
    EX, EZ, EA, EB, P = -6.0, 78.0, 6.2, 11.2, 2.6
    for j in range(-6, 7):
        for i in range(-4, 4):
            x, z = EX + P * (i + 0.5 * (j % 2)), EZ + P * 0.866 * j
            if ((x - EX) / EA) ** 2 + ((z - EZ) / EB) ** 2 <= 1.0:
                for s in (1, -1):
                    out.append(rod((x, s * 60.0, z), (0, s, -RAIN_DROP), 0.6, 24.0))
    # intake: 3 staggered rows on the foot's bottom round, pointing out and 45 deg down
    for row, z in enumerate((4.0, 5.8, 7.6)):
        p = outline_at(z)
        per = sum(math.dist(p[i], p[(i + 1) % len(p)]) for i in range(len(p)))
        n = int(per / 2.2)
        acc, i, targets = 0.0, 0, [(k + 0.5 * (row % 2)) * per / n for k in range(n)]
        for t in targets:
            while acc + math.dist(p[i], p[(i + 1) % len(p)]) < t:
                acc += math.dist(p[i], p[(i + 1) % len(p)]); i += 1
            (x0, y0), (x1, y1) = p[i], p[(i + 1) % len(p)]
            u = (t - acc) / math.dist(p[i], p[(i + 1) % len(p)])
            x, y = x0 + u * (x1 - x0), y0 + u * (y1 - y0)
            ex, ey = (x1 - x0), (y1 - y0)
            l = math.hypot(ex, ey)
            nx, ny = ey / l, -ex / l                    # outward normal of a CCW outline
            if x > 50.0:
                continue                                # not under the face glass
            out.append(rod((x, y, z), (nx, ny, -1.0), 0.7, 12.0))
    # exhaust: a hex field on the back, just under the occiput's widest line (z 80), where the surface starts to face
    # down and the swell above shelters it; level with the heatsink's aft fins (z 64-72). Through along -x, dropping.
    for j in range(8):
        z = 63.0 + j * 2.2 * 0.866
        for i in range(-21, 22):
            y = (i + 0.5 * (j % 2)) * 2.2
            if abs(y) > 48.0:
                continue
            p = outline_at(z)
            xs = [a for a, b in zip(p, p[1:] + p[:1]) if (a[1] - y) * (b[1] - y) <= 0 and a[0] < 0]
            xb = min(q[0] for q in xs) if xs else -60.0
            out.append(rod((xb, y, z), (-1, 0, -RAIN_DROP), 0.7, 12.0))
    # chin grille: a band of fine dots under the face panel between the stereo camera's two Ø8 windows (Ø11 with the
    # chamfer), stopping 4 short of them -- the same rule as the full-size head. Mics behind it.
    for j in range(-2, 3):
        z = CHIN_Z + j * 2.2 * 0.866
        for i in range(-21, 22):
            y = (i + 0.5 * (j % 2)) * 2.2
            if abs(y) <= STEREO_Y - 9.5:
                out.append(rod((FACE, y, z), (1, 0, 0), 0.7, 10.0))
    return out

def prism_yz(poly, x0, x1):
    """A y-z polygon extruded along x from x0 to x1."""
    m = mf.Manifold.extrude(mf.CrossSection([poly]), x1 - x0)
    return m.transform(np.array([[0, 0, 1, x0], [1, 0, 0, 0], [0, 1, 0, 0.0]]))

def pcyl(x, y, r, z0, z1, n=32):
    """A z-up cylinder at (x, y) from z0 to z1, for on_plane."""
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate((x, y, z0))

def on_plane(m, x0):
    """Lay a part drawn in a z-up frame (z 0 = the outside face) onto the lean flat, its origin at plan x0, y 0."""
    ls, lc, ld = sk.lean_plane(sk.LEAN, sk.FLAT_X)
    return m.translate((x0 / lc, 0.0, 0.0)).rotate((0.0, sk.LEAN, 0.0)).translate((ld * ls, 0.0, ld * lc))

def box(x0, y0, z0, x1, y1, z1):
    return mf.Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))

# ---- the split -------------------------------------------------------------------------------------------------
# Two parts, split on a VERTICAL plane just behind the face (StackChan / Portal): a black FACE MODULE (panel, screen,
# chin grille + depth camera) that prints face-down, and a REAR SHELL (everything else, closed bottom) that prints
# upright. The devkit loads from the front through the whole cross-section. The joint is a 1 mm shadow gap framing
# the face; a lip on the rear shell locates the face module. The face module snaps on at the top (two
# cantilever hooks) and screws to the floor at the bottom (a tongue, 2 x M3 from below into heat-set inserts).
#
# Print 1 feedback: the old top tabs overlapped the rear lip by 0.17 cm3 when assembled (their ridge sat inside the
# lip's root and the tab had 0.1 mm clearance), so they snapped when forced. The seam also gapped at the sides,
# where the lip is cut for the devkit, and the two prints' skins never quite met flush. Changes:
# - hooks: a 20 mm beam, 1.8 thick, with 0.3 clearance under the lip. The 0.7 hook takes 0.45 % strain at the root,
#   under the interlayer limit, because the face prints face-down and its layers cross the beam. A 30 deg lead-in
#   and a 45 deg return let it snap on and pull off. It catches behind a keeper strip that is solid with the
#   rear's crown.
# - x stop: the hook bridges bear on the lip's front edge, which sets the shadow gap.
# - the face module carries its own side lips across the devkit's slot (z 35-74), so the seam is keyed all round.
# - the shadow gap goes 1.0 -> 0.5. A reveal cut from offsets of the skin left sliver edges that the slicer
#   pinched, so the gap stays a plain butt joint.
SPLIT_X, SPLIT_GAP = 40.0, 0.5
LIP_D, LIP_T = 5.0, 1.4                     # locating lip: reaches LIP_D forward inside the face module's wall
HOOK_Y, HOOK_HW = 10.0, 6.0                 # hook beams at y +-10, 12 wide (inboard of the ears' front bosses)
HOOK_CL, HOOK_T, HOOK_H = 0.3, 1.8, 0.7     # beam clearance under the lip, beam thickness, hook height
KEEPER_X = 28.0                             # the keeper's back face: the hook catches behind it

# Face layout (real z). The face flat runs z 21-137.
PANEL_HW, PANEL_Z0, PANEL_Z1, PANEL_R = 48.0, 37.5, 135.5, 8.0   # the face panel area (sets the grille's width)
PANEL_Z, PANEL_D = 84.0, 4.5     # 4in screen centre (glass z 42-126, flush) and its glass + LCD stack depth (ASSUMED)
CHIN_Z = 28.5                    # chin grille band centre (z ~24-33)
# Depth camera: Waveshare IMX219-83 stereo module (from its STEP, 0619.stp): board 85.2 x 24, two M12 lenses (Ø14
# barrels) 60 apart, lens axes 3.65 below the board's centre line, 18.4 deep lens-front to back. Mounted upright
# behind the chin grille, lenses looking out through Ø8 chamfered windows, board hanging down below them.
STEREO_Y, STEREO_Z = 30.0, 28.5

# Devkit, from the P3768 A04 Allegro extract: board x -4..96, y -17..62 (R0.6), M2.5 holes (Ø2.75 NPTH) at
# x {0, 92} x y {0, 58}; I/O edge at y -17. In the head: board x -> head y, board y -> head x (I/O aft).
DK_X0, DK_Z = -20.0, 38.0
DK_HOLES = [(by + DK_X0, bx - 46.0) for bx in (0.0, 92.0) for by in (0.0, 58.0)]
TONGUE_SCREWS = [(34.0, 12.0), (34.0, -12.0)]
# neck dock on the lean flat (plan x -48..12, ~100 wide): centre, insert circle radius, desk bumpons (about the centre)
DOCK_X, DOCK_BCD = -16.0, 14.1
# ears: reSpeaker XVF3800 (Ø100 x 1.2) flat under the crown, mics up: board bottom XVF_Z, centre XVF_X. From Seeed's
# STEP (respeaker_mic_array_xvf3800_1_with-xiao-0820.stp) and 2D drawing, board frame (centre, top view, USB-C at -y):
# 3 x Ø2.2 (M2) mounting holes at (0, -34) and (+-21.35, 24.5); bottom-port mics under Ø1.0 ports at (+-33, +-33).
# Every part is on the underside, down to 6.7 below it (XIAO, USB-C, 3.5 jack, edge buttons). The top has only the
# LED ring, 1.2 proud at r ~40. Turned so the USB-C faces aft (cable down the back to the devkit), the board maps
# (bx, by) -> head (XVF_X + by, -bx). Print 1 had the holes on the 66 square, which is where the mics are.
# Hung from three crown bosses (M2 self-tapping from below). Each mic port reaches the outside through a sealed
# printed channel (Ø5, a foam ring between its foot and the board) to a Ø1.6 port in the crown.
XVF_X, XVF_Z = 1.0, 120.0   # at 132 the edge buttons and USB-C hit the walls: 120 leaves ~4 per side and 6.6 behind the USB-C for a right-angle plug
XVF_HOLES = [(XVF_X + by, -bx) for bx, by in ((0.0, -34.0), (21.35, 24.5), (-21.35, 24.5))]
XVF_MICS = [(XVF_X + sx * 33.0, sy * 33.0) for sx in (1, -1) for sy in (1, -1)]


def inside_xy(poly, x, y):
    c = False
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            c = not c
    return c


def surface_z(x, y, lo=100.0, hi=200.0, sec=None):
    """Height of the outer skin above (x, y): the top of the head there."""
    sec = sec or (lambda z: section(z, 96))
    for _ in range(30):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if inside_xy(sec(mid), x, y) else (lo, mid)
    return lo


def halo(cx, half, mics, rows=4, pitch=2.2, n=6.0):
    """The crown grille: a rounded square of dots (Ø1.4 on a 2.2 hex, as the chin and ears) -- the screen's shape,
    on top of the head -- whose outer row runs through the four mics (its corners land on them), with rows stepping
    inward from there so its front edge stays a straight line behind the face module. Returns (blind dots, ports)."""
    a0 = half * 2 ** (1 / n)                     # the superellipse |x|^n + |y|^n = a^n through (half, half)

    def curve(a, m=2000):
        pts = []
        for i in range(m):
            t = 2 * math.pi * i / m + math.pi / 4
            c, s = math.cos(t), math.sin(t)
            pts.append((a * math.copysign(abs(c) ** (2 / n), c), a * math.copysign(abs(s) ** (2 / n), s)))
        return pts

    dots = []
    for k in range(rows):
        a = a0 - k * pitch * 0.866
        p = curve(a)
        seg = [math.dist(p[i], p[(i + 1) % len(p)]) for i in range(len(p))]
        per = sum(seg)
        m = 4 * round(per / pitch / 4)
        off = (0.5 if k % 2 else 0.0) * per / m
        acc, i = 0.0, 0
        for j in range(m):
            d = off + j * per / m
            while acc + seg[i] < d:
                acc += seg[i]
                i = (i + 1) % len(p)
            u = (d - acc) / seg[i]
            (x0, y0), (x1, y1) = p[i], p[(i + 1) % len(p)]
            dots.append((cx + x0 + u * (x1 - x0), y0 + u * (y1 - y0)))
    ports = [min(dots, key=lambda q: math.dist(q, mp)) for mp in mics]
    return [q for q in dots if q not in ports], ports


# Rain: the dots are blind (0.6 deep) so only the four ports go through, and each port's channel is closed at its
# foot by a hydrophobic acoustic membrane (ePTFE vent, e.g. a 6 mm adhesive disc) over the mic inlet: sound passes,
# water stops there, on a 0.6 mm lip above the board so nothing wicks onto it.
CROWN_DOTS, CROWN_PORTS = halo(XVF_X, 33.0, XVF_MICS)
DOCK_FEET = [(-16.0, 32.0), (-16.0, -32.0), (24.0, 24.0), (24.0, -24.0)]   # front pair inside the strap slots

# USB-C on the back: power in AND data, on the PD sink + 3S charger board (40 W x 30 H), screwed flat to a pad inside
# the back wall (4 x M2.5 self-tapping, blind holes). VBUS/CC go to the PD sink; the USB 2 pair goes on to the devkit.
USB_Z = 37.0
PB_X = -53.0
PB_Z0, PB_Z1, PB_Y = 32.0, 62.0, 20.0
PB_HOLES = [(sy * 17.0, z) for sy in (1, -1) for z in (35.0, 59.0)]

def cavity(w=WALL):
    """The inside of the shell, offset w along the surface NORMAL, not just sideways. A sideways offset of each slice
    is w thick on vertical walls but only w * sin(slope) on shallow ones -- 0.3-1.2 mm across the crown, which
    sliced as a ring of holes. So: intersect copies of the skin offset sideways by sqrt(w^2 - dz^2) at heights shifted
    by dz, for dz across [-w, w]; every point of the skin is then >= w (to a few %) from the cavity."""
    top = ZS[-1]
    zgrid = sorted(set([-1.0] + [z for z in ZS] + [z * 1.0 for z in range(0, int(top) + 1, 2)]))
    def shifted(dz, r):
        rows = []
        for z in zgrid:
            zz = max(z + dz, 0.0)
            if zz > top - 0.3:
                break
            p = outline_at(zz)
            xs, ys = [q[0] for q in p], [q[1] for q in p]
            if min(max(xs) - min(xs), max(ys) - min(ys)) / 2 <= r + 0.6:
                break
            rows.append((z, offset(p, r)))
        return loft(rows)
    cav = None
    for f in (-1.0, -0.7, -0.35, 0.0, 0.35, 0.7, 1.0):
        dz = f * w
        m = shifted(dz, math.sqrt(max(w * w - dz * dz, 0.0)))
        cav = m if cav is None else cav ^ m
    return cav

def export(name, m, holes=0):
    mesh = m.to_mesh()
    tm = trimesh.Trimesh(mesh.vert_properties[:, :3], mesh.tri_verts, process=True)
    parts = [b for b in tm.split(only_watertight=False) if abs(b.volume) > 1.0]   # drop zero-volume boolean slivers
    tm = trimesh.util.concatenate(parts) if len(parts) > 1 else parts[0]
    tm.export(os.path.join(OUT, name + '.stl'))
    print('%-11s watertight=%s volume=%.0f cm3 bbox=%s holes=%d' % (
        name, tm.is_watertight, tm.volume / 1000, np.round(tm.extents, 1).tolist(), holes))

def crown_dimples():
    """The halo's blind dots: 0.6 into the skin, straight down."""
    out = []
    for x, y in CROWN_DOTS:
        z = surface_z(x, y)
        out.append(mf.Manifold.cylinder(6.0, 0.7, 0.7, 12).translate((x, y, z - 0.6)))
    return out


def main():
    # desk lean: everything under the 10 deg plane (node_skull.lean_plane) is cut off the back of the foot, and the
    # floor follows the cut as a FLOOR-thick slab over it
    ls, lc, ld = sk.lean_plane(sk.LEAN, sk.FLAT_X)
    above_plane = lambda d: box(-500.0, -500.0, 0.0, 500.0, 500.0, 1000.0).rotate((0.0, sk.LEAN, 0.0)).translate((d * ls, 0.0, d * lc))
    outer = loft([(z, outline_at(z)) for z in ZS]) ^ above_plane(ld)
    skin = outer - cavity()
    front = lambda x: box(x, -100.0, -5.0, 100.0, 100.0, 250.0)
    behind = lambda x: box(-150.0, -100.0, -5.0, x, 100.0, 250.0)
    above = lambda z: box(-150.0, -100.0, z, 100.0, 100.0, 250.0)
    hs = holes()
    rods = mf.Manifold.batch_boolean(hs, mf.OpType.Add)
    xf, xr = SPLIT_X + SPLIT_GAP / 2, SPLIT_X - SPLIT_GAP / 2
    floor = outer ^ (box(-150.0, -100.0, -5.0, 100.0, 100.0, FLOOR) + (above_plane(ld) - above_plane(ld + FLOOR)))

    cavs = {}
    def cav(w):
        w = round(w, 3)
        if w not in cavs:
            cavs[w] = cavity(w)
        return cavs[w]

    # FACE MODULE --------------------------------------------------------------------------------------------------
    face = (skin + floor) ^ front(xf)
    # bottom tongue: lies on the rear shell's floor, 2 heat-set inserts for the M3s from below
    tongue = box(28.0, -18.0, FLOOR, xf + 6.0, 18.0, FLOOR + 6.0) ^ outer
    band = lambda w0, w1: cav(WALL + w0) - cav(WALL + w1)      # depths below the skin, measured from the wall's inside
    hook_zone = lambda x0, x1, grow=0.0: mf.Manifold.batch_boolean(
        [box(x0, sy * HOOK_Y - HOOK_HW - grow, 120.0, x1, sy * HOOK_Y + HOOK_HW + grow, 200.0) for sy in (1, -1)], mf.OpType.Add)
    # top hooks. Each beam runs back from a bridge on the face wall, under the lip and the keeper, with HOOK_CL
    # clearance. At its tip a hook rises HOOK_H into the space behind the keeper. Ramps are stepped in 4 bands.
    lip_in = 0.2 + LIP_T                                      # the lip's (and keeper's) inner face
    b0 = lip_in + HOOK_CL                                     # the beam's outer face
    catch = KEEPER_X - 0.15                                   # the hook's catch face, 0.15 behind the keeper
    lead, top, ret = HOOK_H / math.tan(math.radians(30)), 0.8, HOOK_H   # lead-in 30 deg, flat, return 45 deg
    tip = catch - ret - top - lead
    hooks = band(b0, b0 + HOOK_T) ^ hook_zone(tip, xf + 9.0)
    for k in range(4):
        d = b0 - HOOK_H * (k + 1) / 4                         # this step's outer face
        x0 = tip + lead * (k + 0.5) / 4
        x1 = catch - ret * (k + 0.5) / 4
        hooks = hooks + (band(d, b0 + 0.01) ^ hook_zone(x0, x1))
    # bridge: ties each beam root to the wall ahead of the lip; its back face is the x stop on the lip's front edge
    bridge = band(-0.5, b0 + HOOK_T) ^ hook_zone(xr + LIP_D, xf + 9.0)
    # side lips across the devkit's slot, where the rear has none: inside the rear wall, merged with the face wall
    side = (((band(0.2, lip_in) ^ box(xr - LIP_D, -100.0, 35.0, xf + 1.0, 100.0, 74.0))
             + (band(-0.5, lip_in) ^ box(xf, -100.0, 35.0, xf + 4.0, 100.0, 74.0))) ^ outer)
    face = face + tongue + hooks + bridge + side
    pocket = rrect_prism(FACE - PANEL_D, FACE + 10.0, 0.0, PANEL_Z, 42.2, 42.2, 1.0)
    window = rrect_prism(FACE - 20.0, FACE - PANEL_D + 0.5, 0.0, PANEL_Z, 36.0, 36.0, 2.0)
    # depth-camera windows: Ø8 through, opening to Ø11 at the face (45 deg) so the 83 deg lens isn't vignetted
    lenses = [rod((FACE, sy * STEREO_Y, STEREO_Z), (1, 0, 0), 4.0, 20.0) for sy in (1, -1)]
    lenses += [mf.Manifold.cylinder(1.5, 4.0, 5.5, 32).transform(np.array([[0, 0, 1, FACE - 1.5], [0, 1, 0, sy * STEREO_Y], [-1, 0, 0, STEREO_Z]]))
               for sy in (1, -1)]
    inserts = [mf.Manifold.cylinder(5.8, 2.0, 2.0, 24).translate((x, y, FLOOR - 0.01)) for x, y in TONGUE_SCREWS]
    dimples = crown_dimples()   # the halo crosses the split: its front dots are the face module's
    face = face - mf.Manifold.batch_boolean([pocket, window, rods ^ front(xf)] + lenses + inserts + dimples, mf.OpType.Add)

    # REAR SHELL ---------------------------------------------------------------------------------------------------
    rear = (skin + floor) ^ behind(xr)
    # lip: in front of the split it stands 0.2 inside the face module's wall; behind it, it grows out to merge with
    # the rear wall so it's one body
    # it's cut away across the devkit's height (z 35-74) so the 100 wide carrier slides in from the front
    lip = ((band(0.2, 0.2 + LIP_T) ^ box(xr - 1.0, -100.0, 12.0, xr + LIP_D, 100.0, 250.0))
           + (band(-0.5, 0.2 + LIP_T) ^ box(xr - 4.0, -100.0, 12.0, xr, 100.0, 250.0))) - box(-150.0, -100.0, 35.0, 100.0, 100.0, 74.0)
    # keepers: over each hook beam the lip's root runs back, solid with the crown, to KEEPER_X; the hooks catch there
    lip = lip + (band(-0.5, lip_in) ^ hook_zone(KEEPER_X, xr, 1.0))
    # under the jaw the skin turns flat and the lip's band ran into the face wall: keep it 0.2 clear in front
    lip = lip - ((outer - cav(WALL + 0.2)) ^ front(xr))
    usb_pad = box(-80.0, -PB_Y - 2.0, PB_Z0 - 1.0, PB_X, PB_Y + 2.0, PB_Z1 + 0.5)
    posts = [mf.Manifold.cylinder(DK_Z - FLOOR, 3.0, 3.0, 32)   # Ø6: clears the stereo board's ends
             .translate((x, y, FLOOR)) for x, y in DK_HOLES]
    top = XVF_Z + 1.2 + 0.4                                   # board top + a squeezed foam ring
    xvf = [mf.Manifold.cylinder(60.0, 3.0, 3.0, 32).translate((x, y, top)) for x, y in XVF_HOLES]     # crown bosses
    xvf += [mf.Manifold.cylinder(60.0, 2.5, 2.5, 32).translate((x, y, top + 0.6)) for x, y in CROWN_PORTS]  # mic channels
    # the front devkit posts (x 35-41) crossed the split into the face jaw and held the seam open: cut 0.2 behind it
    rear = rear + lip + ((mf.Manifold.batch_boolean(posts + [usb_pad] + xvf, mf.OpType.Add) ^ outer) - front(xr - 0.2))
    cuts = [rods ^ behind(xr)]
    cuts.append(rrect_prism(-85.0, -50.0, 0.0, USB_Z, 4.6, 1.8, 1.6))      # USB-C receptacle opening 9.2 x 3.6
    cuts.append(rrect_prism(-85.0, -56.2, 0.0, USB_Z, 6.4, 3.6, 3.0))      # plug overmold pocket 12.8 x 7.2
    cuts += [rod((PB_X - (1.6 if z < 40 else 3.0), y, z), (1, 0, 0), 1.1, 3.2 if z < 40 else 6.0) for y, z in PB_HOLES]
    cuts += [rod((x, y, DK_Z - 4.0), (0, 0, 1), 1.1, 8.5) for x, y in DK_HOLES]            # M2.5 self-tap pilots
    for x, y in TONGUE_SCREWS:
        cuts.append(mf.Manifold.cylinder(10.0, 1.7, 1.7, 24).translate((x, y, -1.0)))         # M3 clearance
        cuts.append(mf.Manifold.cylinder(1.9, 3.4, 1.5, 24).translate((x, y, -0.01)))          # 90-deg countersink
    cuts += [on_plane(box(-10.0, s_ * 35.3 - 1.0, -1.0, 10.0, s_ * 35.3 + 1.0, FLOOR + 1.0), 0.0) for s_ in (1, -1)]  # pack strap slots
    # NECK DOCK, on the lean flat: a Ø52 x 0.8 locating recess for the neck's top plate (a key notch at the front sets
    # its rotation), 4 M3 x 3 heat-set inserts on a Ø40 circle in Ø8 bosses, and a Ø14 cable port in the middle that
    # opens under the pack. On the desk the flat stands on 4 Ø8 bumpons in 0.6 recesses.
    dk = lambda m_: on_plane(m_, DOCK_X)
    rear = rear + (dk(mf.Manifold.batch_boolean([pcyl(DOCK_BCD * u, DOCK_BCD * v, 4.0, 0.0, FLOOR + 1.6)
                                                 for u, v in ((1, 0), (-1, 0), (0, 1), (0, -1))], mf.OpType.Add)) ^ outer)
    cuts.append(dk(pcyl(0.0, 0.0, 26.0, -1.0, 0.8, 96)))
    cuts.append(dk(pcyl(26.0, 0.0, 2.5, -1.0, 0.8)))
    cuts.append(dk(pcyl(0.0, 0.0, 7.0, -1.0, FLOOR + 1.0)))
    cuts += [dk(pcyl(DOCK_BCD * u, DOCK_BCD * v, 2.1, -1.0, 4.2)) for u, v in ((1, 0), (-1, 0), (0, 1), (0, -1))]
    cuts += [dk(pcyl(x, y, 4.1, -1.0, 0.6)) for x, y in DOCK_FEET]
    cuts += [mf.Manifold.cylinder(70.0, 0.7, 0.7, 16).translate((x, y, top)) for x, y in CROWN_PORTS]        # mic ports
    cuts += dimples
    cuts += [mf.Manifold.cylinder(8.0, 0.8, 0.8, 16).translate((x, y, top - 0.1)) for x, y in XVF_HOLES]  # M2 self-tap pilots
    rear = rear - mf.Manifold.batch_boolean(cuts, mf.OpType.Add)

    os.makedirs(OUT, exist_ok=True)
    export('node-face', face, len(hs))
    export('node-rear', rear)

if __name__ == '__main__':
    main()
