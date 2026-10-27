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
from node_skull import section, BROW

WALL = 2.5
RAIN_DROP = 0.58                # tan 30: side holes fall 30 deg toward the outside
FLOOR = 3.0
N = 144                         # points per section
OUT = os.path.join(os.path.dirname(__file__) or '.', '..', 'fab', 'node')

ZS = [0, 1.5, 3.5, 6, 9, 13, 18, 26, 36, 48, 60, 70, 80, 88, 96, 104, 112, 116, 119, 122, 125, 128, 130.5, 132.5,
      133.6, 134.2]

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
    EX, EZ, EA, EB, P = -6.0, 66.0, 6.2, 11.2, 2.6
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
    for j in range(9):
        z = 61.0 + j * 2.2 * 0.866
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

def boss_points():
    """Four floor-screw bosses, placed by hand in the free corners of the floor plate: in front of the pack
    (x 38, y ±28) and behind the sled walls beside the pack (x -26, y ±38.6). See node_fit.py for the clearances."""
    return [(38.0, 28.0), (38.0, -28.0), (-26.0, 38.6), (-26.0, -38.6)]

def boss(x, y):
    """Ø8 boss from the floor's top face up 10 mm, webbed out to the wall (hull toward the outside, clipped by the
    skin later). Its underside is 3 mm off the bed: it needs a small tree support from the plate when printing."""
    cx = sum(q[0] for q in outline_at(0.0)) / N
    d = np.array([x - cx, y]); d = d / np.linalg.norm(d) * 20.0
    a = mf.Manifold.cylinder(10.0, 4.0, 4.0, 32).translate((x, y, FLOOR))
    b = mf.Manifold.cylinder(10.0, 4.0, 4.0, 32).translate((x + d[0], y + d[1], FLOOR))
    return mf.Manifold.batch_hull([a, b])

# USB-C on the back: a panel-mount USB-C extension (female, 2 screw ears), wired to the carrier. Low on the back
# where the surface faces down (sheltered from rain), clear of everything inside. Ears ASSUMED 2 x M3 at 18 mm
# (common panel-mount part; check yours).
USB_Z, USB_EAR = 20.0, 9.0

def main():
    outer = loft([(z, outline_at(z)) for z in ZS])
    # cavity: the outline offset by the wall, open through the bottom (the floor plate closes it)
    inner_rows = [(-1.0, offset(outline_at(0.0), WALL))] + [(z, offset(outline_at(z), WALL)) for z in ZS[1:] if z <= 131.0]
    inner = loft(inner_rows)
    face = rrect_prism(45.0, 70.0, 0.0, 65.5, 43.0, 47.5, 6.0)       # through-window behind the cover glass
    hs = holes()
    bosses = [boss(x, y) for x, y in boss_points()]
    inner_floor_clear = box(-100.0, -100.0, -5.0, 100.0, 100.0, FLOOR)   # nothing below the floor's top face
    usb_pad = box(-80.0, -14.0, USB_Z - 9.0, -56.0, 14.0, USB_Z + 9.0)        # flat inside face at x -56 for the ears
    tub = ((outer - inner) + ((mf.Manifold.batch_boolean(bosses + [usb_pad], mf.OpType.Add) ^ outer) - inner_floor_clear)) - face
    cuts = list(hs)
    cuts += [mf.Manifold.cylinder(7.0, 2.0, 2.0, 24).translate((x, y, FLOOR)) for x, y in boss_points()]   # M3 heat-set, Ø4 x 7
    cuts.append(rrect_prism(-85.0, -50.0, 0.0, USB_Z, 4.6, 1.8, 1.6))      # USB-C receptacle opening 9.2 x 3.6
    cuts += [rod((-60.0, s * USB_EAR, USB_Z), (1, 0, 0), 1.6, 30.0) for s in (1, -1)]
    tub = tub - mf.Manifold.batch_boolean(cuts, mf.OpType.Add)

    # floor plate + sled, one print, flat on the bed, no supports. The plate drops into the bottom opening (0.2
    # clearance) and screws up into the 4 bosses (M3 x 8 countersunk). Two sled walls carry the carrier on 45-degree
    # chamfered lips (drill the lips to the A603's holes); the pack sits between them on the plate, held by a hook-
    # and-loop strap through two slots.
    fp = offset(outline_at(0.0), WALL + 0.2)
    floor = mf.Manifold.extrude(mf.CrossSection([fp]), FLOOR)
    for s in (1, -1):
        wall = [(s * 40.0, FLOOR), (s * 43.0, FLOOR), (s * 43.0, 36.0), (s * 34.0, 36.0), (s * 40.0, 30.0)]
        floor += prism_yz(wall if s > 0 else wall[::-1], -18.0, 10.0)
    fcuts = []
    for x, y in boss_points():
        fcuts.append(mf.Manifold.cylinder(10.0, 1.7, 1.7, 24).translate((x, y, -1.0)))
        fcuts.append(mf.Manifold.cylinder(1.9, 3.4, 1.5, 24).translate((x, y, -0.01)))   # 90-deg countersink
    fcuts += [box(-13.0, s * 36.5 - 1.5, -1.0, 7.0, s * 36.5 + 1.5, FLOOR + 1.0) for s in (1, -1)]   # strap slots
    floor = floor - mf.Manifold.batch_boolean(fcuts, mf.OpType.Add)
    os.makedirs(OUT, exist_ok=True)
    for name, m in (('node-tub', tub), ('node-floor', floor)):
        mesh = m.to_mesh()
        tm = trimesh.Trimesh(mesh.vert_properties[:, :3], mesh.tri_verts, process=True)
        tm.export(os.path.join(OUT, name + '.stl'))
        print('%-11s watertight=%s volume=%.0f cm3 bbox=%s holes=%d' % (
            name, tm.is_watertight, tm.volume / 1000, np.round(tm.extents, 1).tolist(), len(hs) if name == 'node-tub' else 0))

if __name__ == '__main__':
    main()
