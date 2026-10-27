"""Printable node shell: hollow tub + floor plate, every hole cut. Run: python3 node_print.py

Builds straight from node_skull's section polygons in manifold3d (no vcad booleans: a loft minus a few hundred
holes is what kept breaking vcad), so the output is watertight by construction. Needs numpy, trimesh, manifold3d.

Writes ../fab/node/node-tub.stl and node-floor.stl in the model frame (dock origin, z up) and node-tub-print.stl
laid for the printer (crown up, open bottom on the bed... see NODE-BRIEF.md for supports).
"""
import math, os
import numpy as np
import manifold3d as mf
import trimesh
from node_skull import section, BROW, FACE, ZS, SHEET_RT, SHEET_RB

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

def rrect_prism2(x0, x1, cy, cz, hy, hz, rt, rb):
    """rrect_prism with different top (rt) and bottom (rb) corner radii."""
    pts = []
    for cyc, czc, a0, r in ((hy - rt, hz - rt, 0, rt), (-(hy - rt), hz - rt, 90, rt), (-(hy - rb), -(hz - rb), 180, rb),
                            (hy - rb, -(hz - rb), 270, rb)):
        for k in range(13):
            a = math.radians(a0 + k * 90 / 12)
            pts.append((cy + cyc + r * math.cos(a), cz + czc + r * math.sin(a)))
    m = mf.Manifold.extrude(mf.CrossSection([pts]), x1 - x0)
    return m.transform(np.array([[0, 0, 1, x0], [1, 0, 0, 0], [0, 1, 0, 0.0]]))

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
    EX, EZ, EA, EB, P = -6.0, 103.0, 6.2, 11.2, 2.6
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
    return out

def prism_yz(poly, x0, x1):
    """A y-z polygon extruded along x from x0 to x1."""
    m = mf.Manifold.extrude(mf.CrossSection([poly]), x1 - x0)
    return m.transform(np.array([[0, 0, 1, x0], [1, 0, 0, 0], [0, 1, 0, 0.0]]))

def box(x0, y0, z0, x1, y1, z1):
    return mf.Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))

# ---- the split -------------------------------------------------------------------------------------------------
# The Jetson Orin Nano devkit's carrier (P3768, 100 x 79) is wider than the foot opening, so the head splits at
# SEAM_Z into a closed-bottom BASE (battery, intake, devkit posts) and an UPPER shell that lowers over the devkit.
# The 1 mm gap between them reads as a deliberate shadow line round the jaw; a lip on the base locates the upper
# and closes the gap to light. 3 x M3 screws from under the base run up its columns into heat-set inserts in the upper.
SEAM_Z, SEAM_GAP = 30.0, 1.0
LIP_H, LIP_T = 5.0, 1.4                     # locating lip: rises above the base rim inside the upper's wall

# Devkit, from the P3768 A04 Allegro extract: board x -4..96, y -17..62 (R0.6), M2.5 holes (Ø2.75 NPTH) at
# x {0, 92} x y {0, 58} (an alternate pair at x 86); I/O edge at y -17. In the head: board x -> head y (centred,
# head_y = bx - 46), board y -> head x (I/O aft: head_x = by + DK_X0), board underside at DK_Z.
DK_X0, DK_Z = -20.0, 38.0
DK_HOLES = [(by + DK_X0, bx - 46.0) for bx in (0.0, 92.0) for by in (0.0, 58.0)]
SEAM_SCREWS = [(22.0, 48.0, 50.0), (22.0, -48.0, 50.0), (-44.0, 30.0, 50.0), (-44.0, -30.0, 50.0)]   # x, y, web: all webbed out to the side/back walls (a front-centre boss would float: the display blocks its web)

def column(x, y, z0, z1, r=4.0, web=14.0):
    """Ø2r column from z0 to z1 webbed out toward the wall (hull outward; clipped by the skin by the caller)."""
    cx = sum(q[0] for q in outline_at(z0)) / N
    d = np.array([x - cx, y]); d = d / np.linalg.norm(d) * web
    a = mf.Manifold.cylinder(z1 - z0, r, r, 32).translate((x, y, z0))
    b = mf.Manifold.cylinder(z1 - z0, r, r, 32).translate((x + d[0], y + d[1], z0))
    return mf.Manifold.batch_hull([a, b])

# USB-C on the back: power in AND data. The receptacle is on the power board itself (USB-PD sink + 3S charger,
# 40 W x 30 H), a mid-mount right-angle USB-C poking through a slot in the wall. The board screws flat to a pad inside
# the back wall of the UPPER shell (4 x M2.5 self-tapping, blind holes), just under the exhaust. VBUS/CC go to the PD
# sink; the USB 2 pair goes on to the devkit (see NODE-BRIEF.md v1.8).
PANEL_Z = 95.0                  # 4in panel centre (node.loon fz)
CAM_Y, CAM_Z = 30.0, 150.0      # stereo cameras in the brow above the panel: 60 baseline
USB_Z = 37.0                    # receptacle centre
PB_X = -77.0                    # power board's back face (the pad's flat), just inside the back wall at the port
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

def main():
    outer = loft([(z, outline_at(z)) for z in ZS])
    inner = cavity()
    skin = outer - inner
    above = lambda z: box(-100.0, -100.0, z, 100.0, 100.0, 200.0)
    below = lambda z: box(-100.0, -100.0, -5.0, 100.0, 100.0, z)
    hs = holes()
    rods = mf.Manifold.batch_boolean(hs, mf.OpType.Add)

    # UPPER: everything above the seam + gap, the face window, ears, exhaust, USB-C, seam-screw bosses
    # Face: the 3.2 mm acrylic (92 x 101.6, R8) inlays FLUSH with the face plane. It sits in a pocket on a 2.5 mm wide,
    # LAND_T thick land (0.5 VHB); the land is a frame added behind the face, since the pocket is deeper than the wall,
    # and the 4in panel bonds to its back (VHB), showing through the window. The window
    # through the land is the acrylic less 2.5 per side.
    G_T, G_HY, G_HZ, G_R, G_ZC, LAND_T = 3.2, 50.8, 76.2, 0.6, 90.9, 1.5    # a whole 4 x 6 in sheet, portrait, factory corners
    gx = FACE - G_T                                              # acrylic back face
    frame = rrect_prism2(gx - LAND_T, FACE, 0.0, G_ZC, G_HY + 3.0, G_HZ + 3.0, SHEET_RT + 3.0, SHEET_RB + 3.0) ^ outer
    pocket = rrect_prism2(gx - 0.01, FACE + 10.0, 0.0, G_ZC, G_HY + 0.2, G_HZ + 0.2, SHEET_RT + 0.2, SHEET_RB + 0.2)
    # behind the sheet: a full SENSOR PLATE (the land), with a window for the panel's 84 square glass and a Ø10 hole
    # per camera. The panel and cameras bond to its back; more sensors (ToF, light, mic) get their own holes here.
    window = rrect_prism(gx - 10.0, gx + 0.5, 0.0, PANEL_Z, 43.0, 43.0, 1.0)
    cams = [rod((gx, s_ * CAM_Y, CAM_Z), (1, 0, 0), 5.0, 12.0) for s_ in (1, -1)]
    face = pocket + window + mf.Manifold.batch_boolean(cams, mf.OpType.Add)
    zu = SEAM_Z + SEAM_GAP
    usb_pad = box(PB_X - 40.0, -PB_Y - 2.0, PB_Z0 - 1.0, PB_X, PB_Y + 2.0, PB_Z1 + 0.5)
    ubosses = [column(x, y, zu, zu + 6.5, web=w) for x, y, w in SEAM_SCREWS]   # 6.5 tall: clears the devkit's I/O band
    upper = (skin + frame + (mf.Manifold.batch_boolean(ubosses + [usb_pad], mf.OpType.Add) ^ outer)) ^ above(zu)
    cuts = [face, rods]
    cuts += [mf.Manifold.cylinder(5.8, 2.0, 2.0, 24).translate((x, y, zu - 0.01)) for x, y, _ in SEAM_SCREWS]  # M3 heat-set, 5.7 long
    cuts.append(rrect_prism(PB_X - 40.0, PB_X + 3.0, 0.0, USB_Z, 4.6, 1.8, 1.6))      # USB-C receptacle opening 9.2 x 3.6
    cuts.append(rrect_prism(PB_X - 40.0, PB_X - 3.2, 0.0, USB_Z, 6.4, 3.6, 3.0))      # plug overmold pocket 12.8 x 7.2
    cuts += [rod((PB_X - (1.6 if z < 40 else 3.0), y, z), (1, 0, 0), 1.1, 3.2 if z < 40 else 6.0) for y, z in PB_HOLES]   # M2.5 self-tap: 3.2 deep low (the pad thins there), 6 high
    upper = upper - mf.Manifold.batch_boolean(cuts, mf.OpType.Add)

    # BASE: closed bottom, skin below the seam, a locating lip, devkit posts, seam-screw columns, intake
    base = (outer ^ below(FLOOR)) + (skin ^ below(SEAM_Z))
    lip_out = loft([(z, offset(outline_at(z), WALL + 0.2)) for z in (SEAM_Z - 1.0, SEAM_Z + LIP_H)])
    lip_in = loft([(z, offset(outline_at(z), WALL + 0.2 + LIP_T)) for z in (SEAM_Z - 2.0, SEAM_Z + LIP_H + 1.0)])
    # the acrylic and the panel bonded to the upper hang below the seam (to z 14.7 / 12): no lip across the face, and
    # the same pocket + window in the base's front, so they slide down past it and the glass covers the seam there
    base = base + ((lip_out - lip_in) - box(40.0, -100.0, -5.0, 100.0, 100.0, 100.0))
    posts = [mf.Manifold.cylinder(DK_Z - FLOOR, 3.5, 3.5, 32).translate((x, y, FLOOR)) for x, y in DK_HOLES]
    cols = [column(x, y, FLOOR, SEAM_Z, web=w) for x, y, w in SEAM_SCREWS]
    base = base + (mf.Manifold.batch_boolean(posts + cols, mf.OpType.Add) ^ outer)
    bcuts = [rods ^ below(SEAM_Z), face]
    bcuts += [rod((x, y, DK_Z - 4.0), (0, 0, 1), 1.1, 8.5) for x, y in DK_HOLES]           # M2.5 self-tap pilots
    for x, y, _ in SEAM_SCREWS:
        bcuts.append(mf.Manifold.cylinder(SEAM_Z + 2.0, 1.7, 1.7, 24).translate((x, y, -1.0)))   # M3 clearance
        bcuts.append(mf.Manifold.cylinder(SEAM_Z - 8.0, 3.2, 3.2, 24).translate((x, y, -1.0)))   # counterbore: an M3 x 12 from z 22 reaches the insert
    bcuts += [box(-10.0, s * 35.3 - 1.0, -1.0, 10.0, s * 35.3 + 1.0, FLOOR + 1.0) for s in (1, -1)]   # pack strap slots
    base = base - mf.Manifold.batch_boolean(bcuts, mf.OpType.Add)

    os.makedirs(OUT, exist_ok=True)
    export('node-upper', upper, len(hs))
    export('node-base', base)

if __name__ == '__main__':
    main()
