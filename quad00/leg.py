"""quad00 leg: an RS00 quadruped leg with a sheet-gear knee.

One leg, as a bench rig:

  bulkhead (torso) -- ab/ad RS00 -- arch --keyed tenons-- side plates -- hip RS00 (outboard)
                                                                    \\-- knee RS00 (inboard)
  hip RS00 -> outer thigh plate;  knee RS00 -> pinion z17 -> idler z17 -> idler z17 -> knee gear z34 (2:1)
  knee gear + shin = one laminate -> ball paw (TPU, two bolts)

The ab/ad axis runs fore-aft in the leg plane (y=0), 60 mm above the hip axis, so standing load puts
no moment on it. The arch is the only bridge between the two hip-side motors, and it sits in the
forward-up sector, the one place a quadruped thigh never goes. The thigh swings from ~horizontal
forward, down, back and over the top.

Every structural part is a flat profile in one stock (3.2 mm), laminated where it needs to be thicker.
  python3 leg.py            -> checks + out/dxf (router, per layer) + out/stl + out/3mf (P1S plates)
  python3 leg.py --check    -> gear mesh sweep, stack, ranges only
  python3 leg.py --assembly -> posed parts + out/assembly/leg.loon (vcad-render)

Leg frame: hip axis +Y through the origin, thigh hangs along -Z, X forward. Gear plane is y=0.
Needs: pip install shapely trimesh manifold3d ezdxf mapbox_earcut numpy networkx lxml
"""
import math
import os
import sys

import ezdxf
import numpy as np
import trimesh
from manifold3d import CrossSection, FillRule, Manifold
from shapely import affinity
from shapely.geometry import Point, Polygon, MultiPolygon, box
from shapely.ops import unary_union

HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ parameters (mm)
T = 3.2                    # the one stock
M = 2.5                    # gear module (router: 1.5 mm end mill fits the 1.7 mm root gap)
Z_SMALL, Z_KNEE = 17, 34   # pinion = idlers = 17, knee 34 -> 2:1
ALPHA = math.radians(20)
BACKLASH = 0.10            # per flank, mm. 0.05 for router-cut aluminium, 0.10 for print
GEAR_LAYERS = 3            # 9.6 mm face: Lewis SF 2.1 at RS00 peak x1.5 (aluminium)
PLATE_GAP = 0.4            # gear face to thigh plate

C_MESH = M * (Z_SMALL + Z_SMALL) / 2                 # 42.5 pinion-idler, idler-idler
C_KNEE = M * (Z_SMALL + Z_KNEE) / 2                  # 63.75 idler-knee
Z_IDLER1, Z_IDLER2 = -C_MESH, -2 * C_MESH            # along the thigh
Z_KNEEAX = Z_IDLER2 - C_KNEE                          # -148.75
SHIN = 150.0
PAW_D = 44.0                                          # TPU ball, centred on the shin's end bolt
PAW_BOLTS = ((-SHIN, 5.3), (-SHIN + 14, 4.3))         # M5 + M4: two bolts so the ball can't spin

# RS00 (arm/fork/rose_fork.py): body D57 x L51, output 6x M3 on D27, back 4x on D38
RS00_D, RS00_L = 57.0, 51.0
OUT_PCD, OUT_N = 27.0, 6
BACK_PCD, BACK_N = 38.0, 4
M3, M3_CSK = 3.4, 6.3
SHAFT, BUSH_OD = 8.1, 10.0          # M8 shoulder bolt / 8x10x8 plain sleeve (no flange: 0.4 mm gap)
HUB_D = 34.0                        # pinion hub: inner thigh plate rides on it (printed: direct)

# stack along Y (outboard +)
G = GEAR_LAYERS * T / 2                              # gear half-face, 4.8
Y_PLATE = (G + PLATE_GAP, G + PLATE_GAP + T)         # thigh plates at +/- this
Y_MOTOR = (Y_PLATE[1] + 1.0, Y_PLATE[1] + 1.0 + RS00_L)   # stator body (1 mm nose)
Y_SIDE = (Y_MOTOR[1], Y_MOTOR[1] + 2 * T)            # side plates: 2 layers, motor back-mounted

# the yoke: arch (YZ plane, 2 layers) ahead of the hip, ab/ad RS00 ahead of that, bulkhead = torso
ABAD_Z = 60.0                       # ab/ad axis height above the hip axis, in the leg plane (y=0)
X_ARCH = (35.0, 35.0 + 2 * T)       # clears the hip/knee motors (r28.5) by 6.5
X_ABAD = (X_ARCH[1] + 1.0, X_ARCH[1] + 1.0 + RS00_L)    # ab/ad stator; output face bears on the arch
X_BULK = (X_ABAD[1], X_ABAD[1] + 2 * T)
ARCH_W, ARCH_Z = 74.0, (-18.0, 92.0)                     # arch half-width, z extent
NOTCH = (15.0, 40.0)                # leg-plane notch in the arch: half-width, height -- thigh passes under
TENON_Z = (0.0, 66.0)               # side-plate tenons through the arch
TENON_H, TENON_OUT = 12.0, 11.0     # tenon height, how far it stands proud of the arch front
KEY_HOLE = (5.0, 3.6)               # through the tenon, starts 0.4 inside the arch front face
KEY_L, KEY_W = (34.0, (2.8, 4.8))   # tapered key from 3.2 stock: length, width tip -> head


# ------------------------------------------------------------------ 2D helpers
def circle(x, z, d, n=96):
    return Point(x, z).buffer(d / 2, n // 4)


def holes_on(pcd, n, d, phase=0.0, at=(0.0, 0.0)):
    return unary_union([circle(at[0] + pcd / 2 * math.cos(math.radians(phase + 360 * i / n)),
                               at[1] + pcd / 2 * math.sin(math.radians(phase + 360 * i / n)), d)
                        for i in range(n)])


def inv(a):
    return math.tan(a) - a


def gear(z, m=M, backlash=BACKLASH, phase=0.0, at=(0.0, 0.0), pts=24):
    """involute spur gear outline, tooth 0 centred on +X rotated by phase (deg)."""
    rp = m * z / 2
    rb = rp * math.cos(ALPHA)
    ra = rp + m
    rf = rp - 1.25 * m
    half = (math.pi * m / 2 - 2 * backlash) / (2 * rp)          # half tooth angle at pitch
    outline = []
    for k in range(z):
        c = 2 * math.pi * k / z
        flank = []
        for i in range(pts + 1):
            r = rf + (ra - rf) * i / pts
            if r < rb:
                a = half + inv(ALPHA)                               # radial below the base circle
            else:
                a = half + inv(ALPHA) - inv(math.acos(rb / r))
            flank.append((r, a))
        left = [(r * math.cos(c - a), r * math.sin(c - a)) for r, a in flank]
        right = [(r * math.cos(c + a), r * math.sin(c + a)) for r, a in reversed(flank)]
        outline += left + right
        a0 = c + flank[0][1]
        a1 = c + 2 * math.pi / z - flank[0][1]
        for j in range(1, 6):
            t = a0 + (a1 - a0) * j / 6
            outline.append((rf * math.cos(t), rf * math.sin(t)))
    g = Polygon(outline).buffer(0)
    g = affinity.rotate(g, phase, origin=(0, 0))
    return affinity.translate(g, *at)


def stadium(points_d):
    """convex hull of circles: [(x, z, d), ...]"""
    return unary_union([circle(x, z, d) for x, z, d in points_d]).convex_hull


def rounded(x0, z0, x1, z1, r):
    return box(x0 + r, z0 + r, x1 - r, z1 - r).buffer(r, 8)


# ------------------------------------------------------------------ parts: leg (XZ profiles)
def pinion_tooth_layer():
    return gear(Z_SMALL, phase=-90).difference(holes_on(OUT_PCD, OUT_N, M3))


def pinion_hub():
    return circle(0, 0, HUB_D).difference(holes_on(OUT_PCD, OUT_N, M3))


def idler(phase):
    return gear(Z_SMALL, phase=phase).difference(circle(0, 0, BUSH_OD)).difference(holes_on(24, 3, M3, phase=90))


def shin_layer():
    """knee gear and shin bar are one laminate; knee axis at origin, shin along -Z."""
    s = unary_union([gear(Z_KNEE, phase=90 + 180 / Z_KNEE), stadium([(0, 0, 24), (0, -SHIN, 24)])])
    s = s.difference(circle(0, 0, BUSH_OD))
    for ang in (90, 210, 330):                                    # lightening, clear of the bar
        s = s.difference(circle(21 * math.cos(math.radians(ang)), 21 * math.sin(math.radians(ang)), 14))
    s = s.difference(holes_on(60, 3, M3, phase=30))               # lamination bolts in the gear
    for zz in (-55, -100):
        s = s.difference(circle(0, zz, M3))
    for zz, d in PAW_BOLTS:
        s = s.difference(circle(0, zz, d))
    return s


def thigh_plate(outer):
    p = stadium([(0, 0, 60), (0, Z_IDLER1, 54), (0, Z_IDLER2, 54), (0, Z_KNEEAX, 32)])
    p = p.difference(holes_on(OUT_PCD, OUT_N, M3) if outer else circle(0, 0, HUB_D + 0.4))
    for zz in (Z_IDLER1, Z_IDLER2, Z_KNEEAX):
        p = p.difference(circle(0, zz, SHAFT))
    for zz in ((Z_IDLER1 + Z_IDLER2) / 2, (Z_IDLER2 + Z_KNEEAX) / 2):
        p = p.difference(circle(0, zz, 14))
    return p


# ------------------------------------------------------------------ parts: the yoke
def side_plate():
    """XZ. Holds a hip or knee RS00 by its back face; two keyed tenons run forward through the arch."""
    body = unary_union([circle(0, 0, RS00_D + 14), box(0, ARCH_Z[0] + 6, X_ARCH[0], TENON_Z[1] + TENON_H)]).convex_hull
    front = X_ARCH[1]
    for zt in TENON_Z:
        body = body.union(box(X_ARCH[0] - 1, zt - TENON_H / 2, front + TENON_OUT, zt + TENON_H / 2))
        body = body.difference(box(front - 0.4, zt - KEY_HOLE[1] / 2, front - 0.4 + KEY_HOLE[0], zt + KEY_HOLE[1] / 2))
    body = body.difference(holes_on(BACK_PCD, BACK_N, M3, phase=45))
    body = body.difference(circle(0, 0, 16))                                   # cable exit
    body = body.difference(circle(16, 44, 16))                                 # lightening
    return body


def arch():
    """YZ profile (u = y, v = z). The leg-plane notch lets the thigh swing to horizontal forward."""
    a = rounded(-ARCH_W, ARCH_Z[0], ARCH_W, ARCH_Z[1], 10)
    a = a.difference(rounded(-NOTCH[0], ARCH_Z[0] - 20, NOTCH[0], NOTCH[1], 6))
    for s in (1, -1):
        for zt in TENON_Z:                                                     # mortises, +0.2 fit
            y0, y1 = sorted((s * (Y_SIDE[0] - 0.2), s * (Y_SIDE[1] + 0.2)))
            a = a.difference(box(y0, zt - TENON_H / 2 - 0.2, y1, zt + TENON_H / 2 + 0.2))
        a = a.difference(circle(s * 36, 33, 26))                               # lightening / cable pass
    return a.difference(holes_on(OUT_PCD, OUT_N, M3, at=(0, ABAD_Z)))


def bulkhead():
    """YZ. Stand-in for the torso: holds the ab/ad RS00 by its back face, bolts to the bench."""
    b = rounded(-45, ABAD_Z - 45, 45, ABAD_Z + 45, 8)
    b = b.difference(holes_on(BACK_PCD, BACK_N, M3, phase=45, at=(0, ABAD_Z)))
    b = b.difference(circle(0, ABAD_Z, 16))
    for sy in (1, -1):
        for sz in (1, -1):
            b = b.difference(circle(sy * 35, ABAD_Z + sz * 35, 6.4))
    return b


def key():
    """XY profile, tapered: drives along Y through a tenon, flat side on the arch face."""
    w0, w1 = KEY_W
    return Polygon([(0, 0), (w0, 0), (w1, KEY_L), (w1 + 3, KEY_L), (w1 + 3, KEY_L + 5), (0, KEY_L + 5)])


# ------------------------------------------------------------------ the bill
# name -> dict(layers=[(profile, t)], o=first-layer offset along the plane normal, plane, csk, qty)
# plane: 'xz' extrudes along +Y; 'yz' (u=y, v=z) along +X; 'xy' along +Z.
OUT_HOLES = [(OUT_PCD / 2 * math.cos(math.radians(60 * i)), OUT_PCD / 2 * math.sin(math.radians(60 * i))) for i in range(OUT_N)]


def parts():
    P = lambda layers, o, plane='xz', csk=(), qty=1: dict(layers=layers, o=o, plane=plane, csk=list(csk), qty=qty)
    return {
        'pinion': P([(pinion_hub(), T + PLATE_GAP)] + [(pinion_tooth_layer(), T)] * GEAR_LAYERS,
                    -Y_PLATE[1], csk=[(x, z, 'top') for x, z in OUT_HOLES]),
        'idler': P([(idler(0), T)] * GEAR_LAYERS, -G, qty=2),
        'shin': P([(shin_layer(), T)] * GEAR_LAYERS, -G),
        'thigh-outer': P([(thigh_plate(True), T)], Y_PLATE[0], csk=[(x, z, 'bottom') for x, z in OUT_HOLES]),
        'thigh-inner': P([(thigh_plate(False), T)], -Y_PLATE[1]),
        'side': P([(side_plate(), T)] * 2, Y_SIDE[0], qty=2),
        'arch': P([(arch(), T)] * 2, X_ARCH[0], plane='yz'),
        'bulkhead': P([(bulkhead(), T)] * 2, X_BULK[0], plane='yz'),
        'key': P([(key(), T)], 0.0, plane='xy', qty=4),
    }


# ------------------------------------------------------------------ solids
# profile (u, v), extruded along w  ->  world
PLANE = {'xz': np.array([[1, 0, 0], [0, 0, 1], [0, 1, 0]], float),     # x=u, y=w, z=v
         'yz': np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float),     # x=w, y=u, z=v
         'xy': np.eye(3)}


def rings(poly):
    geoms = poly.geoms if isinstance(poly, MultiPolygon) else [poly]
    out = []
    for g in geoms:
        out.append(np.asarray(g.exterior.coords[:-1]))
        out += [np.asarray(r.coords[:-1]) for r in g.interiors]
    return out


def to_world(m, plane):
    return m.transform(np.hstack([PLANE[plane], np.zeros((3, 1))]))


def csk_cone(u, v, w_face, inward):
    """90 deg M3 flat-head seat, D6.3 at the face, in profile coords. inward = +1 (into +w) or -1."""
    e = 0.2
    r0 = M3_CSK / 2 + e                                   # base sits e outside the face
    r1 = M3 / 2 + 0.25                                    # stop short of the hole wall
    c = Manifold.cylinder(r0 - r1, r0, r1, 48)
    if inward < 0:
        c = c.mirror((0, 0, 1))
    return c.translate((u, v, w_face - inward * e))


def solid_manifold(spec):
    m, w = None, spec['o']
    for poly, t in spec['layers']:
        e = CrossSection(rings(poly), FillRule.EvenOdd).extrude(t).translate((0, 0, w))
        m = e if m is None else m + e
        w += t
    for u, v, f in spec['csk']:
        m = m - csk_cone(u, v, w if f == 'top' else spec['o'], -1 if f == 'top' else 1)
    return m


def tri(m):
    mesh = m.to_mesh()
    return trimesh.Trimesh(mesh.vert_properties[:, :3], mesh.tri_verts, process=False)


def solid(name, spec):
    """the part in the world (leg) frame."""
    return tri(to_world(solid_manifold(spec), spec['plane']))


def paw():
    """TPU ball on the shin's end, shin frame (knee at origin, shin along -Z): slot + two cross bolts."""
    r = PAW_D / 2
    ball = Manifold.sphere(r, 96).translate((0, 0, -SHIN))
    slot = Manifold.cube((30, 2 * G + 0.4, 40)).translate((-15, -G - 0.2, -SHIN - 14))   # down to 2 past the tip
    flat = Manifold.cube((60, 60, 10)).translate((-30, -30, -SHIN - r - 8.5))           # 1.5 mm flat to print on
    m = ball - slot - flat
    for zz, d in PAW_BOLTS:
        m = m - Manifold.cylinder(60, d / 2, d / 2, 32).rotate((90, 0, 0)).translate((0, 30, zz))
    return tri(m)


# ------------------------------------------------------------------ outputs
def to_dxf(poly, path, csks=()):
    doc = ezdxf.new('R2000')
    doc.layers.add('CUT')
    doc.layers.add('CSK_M3_90DEG')
    msp = doc.modelspace()
    for ring in rings(poly):
        msp.add_lwpolyline([tuple(p) for p in ring], close=True, dxfattribs={'layer': 'CUT'})
    for u, v, _ in csks:
        msp.add_circle((u, v), M3_CSK / 2, dxfattribs={'layer': 'CSK_M3_90DEG'})
    doc.saveas(path)


def write():
    out = os.path.join(HERE, 'out')
    for d in ('dxf', 'stl'):
        os.makedirs(os.path.join(out, d), exist_ok=True)
    for f in os.listdir(os.path.join(out, 'dxf')) + [None]:
        if f:
            os.remove(os.path.join(out, 'dxf', f))
    for f in os.listdir(os.path.join(out, 'stl')):
        os.remove(os.path.join(out, 'stl', f))
    built = {}
    for name, spec in parts().items():
        layers, qty, csks = spec['layers'], spec['qty'], spec['csk']
        uniq = []
        for i, (p, t) in enumerate(layers):
            if any(p.equals(q) for q in uniq):
                continue
            uniq.append(p)
            idx = [k for k, (q, _) in enumerate(layers) if q.equals(p)]
            face = [c for c in csks if (c[2] == 'top' and len(layers) - 1 in idx) or (c[2] == 'bottom' and 0 in idx)]
            tag = f'_csk-{qty}' if face else ''          # countersink only that many of them
            to_dxf(p, os.path.join(out, 'dxf', f'{name}-L{i}_{T}mm_x{len(idx) * qty}{tag}.dxf'), face)
        m = tri(solid_manifold(spec))                    # profile frame: thickness is already +Z, print-flat
        m.apply_translation(-m.bounds[0])
        m.export(os.path.join(out, 'stl', f'{name}_x{qty}.stl'))
        built[name] = (m, qty)
    p = paw()
    p.apply_translation(-p.bounds[0])
    p.export(os.path.join(out, 'stl', 'paw_TPU_x1.stl'))
    built['paw'] = (p, 1)
    print(f'wrote {out}/dxf + {out}/stl')
    plates(out, built)


# Bambu P1S: 256 x 256 bed, 6 mm margin, 6 mm between parts; shelf-packed, tallest rows first
BED, MARGIN, GAP = 256.0, 6.0, 6.0
PLATES = {
    'plate1_PETG_links': ['shin', 'thigh-outer', 'thigh-inner'],
    'plate2_PETG_yoke': ['arch', 'side', 'bulkhead'],
    'plate3_PETG_gears-keys': ['idler', 'pinion', 'key'],
    'plate4_TPU_paw': ['paw'],
}


def plates(out, built):
    """one 3MF per P1S plate. Built from the in-memory solids: STL drops the indexed topology."""
    d = os.path.join(out, '3mf')
    os.makedirs(d, exist_ok=True)
    for f in os.listdir(d):
        os.remove(os.path.join(d, f))
    for plate, names in PLATES.items():
        items = []
        for n in names:
            m, qty = built[n]
            for k in range(qty):
                mm = m.copy()
                if mm.extents[1] > mm.extents[0] and mm.extents[1] > 120:      # long parts run along X? no: keep Y
                    pass
                items.append((f'{n}-{k + 1}', mm))
        items.sort(key=lambda it: -it[1].extents[1])
        scene, x, y, row_h = trimesh.Scene(), MARGIN, MARGIN, 0.0
        for nm, m in items:
            w, h = m.extents[0], m.extents[1]
            if x + w > BED - MARGIN:
                x, y, row_h = MARGIN, y + row_h + GAP, 0.0
            assert y + h <= BED - MARGIN, f'{plate}: {nm} does not fit'
            scene.add_geometry(m.copy().apply_translation([x, y, 0]), node_name=nm, geom_name=nm)
            x, row_h = x + w + GAP, max(row_h, h)
        scene.export(os.path.join(d, plate + '.3mf'))
        print(f'  {plate}.3mf: {len(items)} parts, used {y + row_h:.0f} of {BED:.0f} mm')


def mass():
    """per leg, from the solids: aluminium (2.70) and as printed (PETG 1.27 at ~55% effective fill)."""
    vol = {n: solid_manifold(s).volume() * s['qty'] / 1000 for n, s in parts().items()}
    for n, v in vol.items():
        print(f'  {n:12s} {v:5.1f} cm3  alu {v * 2.70:4.0f} g')
    leg = sum(v for n, v in vol.items() if n not in ('bulkhead',))
    print(f'leg (bulkhead is torso): {leg:.0f} cm3 -> aluminium {leg * 2.70:.0f} g, printed PETG ~{leg * 1.27 * 0.55:.0f} g'
          f'  + 3x RS00 {3 * 310} g + hardware ~80 g')


# ------------------------------------------------------------------ checks
def hip_range():
    """thigh (plates + knee gear tip) swung about the hip in the leg plane against the yoke parts that
    reach the leg plane: the arch above its notch, the ab/ad motor, the bulkhead.
    Returns (forward limit, backward limit) in degrees from straight down; None if it goes all the way round."""
    leg = unary_union([stadium([(0, 0, 60), (0, Z_IDLER1, 54), (0, Z_IDLER2, 54), (0, Z_KNEEAX, 32)]),
                       circle(0, Z_KNEEAX, M * (Z_KNEE + 2))])
    walls = unary_union([box(X_ARCH[0], NOTCH[1], X_ARCH[1], ARCH_Z[1]),
                         box(X_ABAD[0], ABAD_Z - RS00_D / 2, X_ABAD[1], ABAD_Z + RS00_D / 2),
                         box(X_BULK[0], ABAD_Z - 45, X_BULK[1], ABAD_Z + 45)]).buffer(2.0)   # 2 mm clearance
    blocked = [a for a in range(360) if affinity.rotate(leg, a, origin=(0, 0)).intersects(walls)]  # CCW = forward
    if not blocked:
        return None
    return min(blocked) - 1, 360 - max(blocked) - 1


def check():
    ok = True
    pairs = [((0, 0), Z_SMALL, -90, (0, Z_IDLER1), Z_SMALL, 90 + 180 / Z_SMALL),
             ((0, Z_IDLER1), Z_SMALL, 90 + 180 / Z_SMALL, (0, Z_IDLER2), Z_SMALL, 90 + 180 / Z_SMALL + 180 / Z_SMALL + 180),
             ((0, Z_IDLER2), Z_SMALL, 0, (0, Z_KNEEAX), Z_KNEE, 0)]
    for a_at, za, pa, b_at, zb, pb in pairs:
        worst, best = 0.0, None
        ga = gear(za, phase=pa, at=a_at)
        pb = mesh_phase(ga, zb, b_at, pb)
        for s in range(13):
            da = 360 / za * s / 12
            A = gear(za, phase=pa + da, at=a_at)
            B = gear(zb, phase=pb - da * za / zb, at=b_at)
            worst = max(worst, A.intersection(B).area)
            gap = A.distance(B)
            best = gap if best is None else min(best, gap)
        ok &= worst < 0.05
        print(f'mesh z{za}->z{zb} C={math.dist(a_at, b_at):.2f}: worst overlap {worst:.3f} mm2  min gap {best:.3f} mm  '
              f'{"OK" if worst < 0.05 else "INTERFERES"}')
    print(f'stack (y): gear +/-{G:.1f}, plates {Y_PLATE[0]:.1f}..{Y_PLATE[1]:.1f}, motors {Y_MOTOR[0]:.1f}..{Y_MOTOR[1]:.1f}, '
          f'side plates {Y_SIDE[0]:.1f}..{Y_SIDE[1]:.1f}  -> {2 * Y_SIDE[1]:.0f} mm wide')
    print(f'yoke (x): arch {X_ARCH[0]:.1f}..{X_ARCH[1]:.1f}, ab/ad RS00 {X_ABAD[0]:.1f}..{X_ABAD[1]:.1f}, '
          f'bulkhead {X_BULK[0]:.1f}..{X_BULK[1]:.1f}; ab/ad axis z={ABAD_Z:.0f}, y=0 (leg plane)')
    c = X_ARCH[0] - RS00_D / 2
    ok &= c > 3
    print(f'arch to hip/knee motor bodies: {c:.1f} mm')
    idl = circle(0, Z_IDLER2 - Z_KNEEAX, M * (Z_SMALL + 2) + 1.0)
    bar = stadium([(0, 0, 24), (0, -SHIN, 24)])
    fold = min(a for a in range(0, 181) if not affinity.rotate(bar, 180 - a, origin=(0, 0)).intersects(idl))
    print(f'knee fold limit: shin bar meets idler 2 at {fold} deg interior (straight leg = 180)')
    r = hip_range()
    print('hip pitch: full circle' if r is None else
          f'hip pitch: {r[0]} deg forward .. {r[1]} deg back (over the top) from straight down  [{r[0] + r[1]} deg]')
    return ok


def mesh_phase(fixed, z, at, base):
    """the phase of a z-tooth gear at `at` that meshes with the `fixed` outline, centred in its backlash."""
    trial = [base + d * 360 / z / 120 for d in range(120)]
    free = [ph for ph in trial if fixed.intersection(gear(z, phase=ph, at=at)).area == 0]
    return max(free, key=lambda ph: fixed.distance(gear(z, phase=ph, at=at)))


def assembly(hip_height=255.0):
    """export every part posed standing (hip height above the paw centre) to out/assembly + a loon for vcad."""
    L1, L2 = -Z_KNEEAX, SHIN
    th_t = math.acos((L1 ** 2 + hip_height ** 2 - L2 ** 2) / (2 * L1 * hip_height))   # R_y(+): knee back
    kx = -L1 * math.sin(th_t)
    th_s = math.asin(kx / L2)                                     # shin points at the paw under the hip
    rel = -math.degrees(th_s - th_t)                              # shin angle in the thigh frame, CCW in XZ
    p2 = mesh_phase(gear(Z_KNEE, phase=90 + 180 / Z_KNEE + rel, at=(0, Z_KNEEAX)), Z_SMALL, (0, Z_IDLER2), 0)
    p1 = mesh_phase(gear(Z_SMALL, phase=p2, at=(0, Z_IDLER2)), Z_SMALL, (0, Z_IDLER1), 0)
    pp = mesh_phase(gear(Z_SMALL, phase=p1, at=(0, Z_IDLER1)), Z_SMALL, (0, 0), -90)
    Ry = lambda deg: trimesh.transformations.rotation_matrix(math.radians(deg), [0, 1, 0])
    thigh = Ry(math.degrees(th_t))
    P = parts()

    def place(name, spin=0.0, at=(0.0, 0.0), extra=None):
        m = solid(name, P[name])
        m.apply_transform(Ry(-spin))
        m.apply_translation([at[0], 0, at[1]])
        return m.apply_transform(extra) if extra is not None else m

    out = {
        'thigh-outer': place('thigh-outer', extra=thigh),
        'thigh-inner': place('thigh-inner', extra=thigh),
        'pinion': place('pinion', spin=pp + 90, extra=thigh),
        'idler-1': place('idler', spin=p1, at=(0, Z_IDLER1), extra=thigh),
        'idler-2': place('idler', spin=p2, at=(0, Z_IDLER2), extra=thigh),
        'shin': place('shin', spin=rel, at=(0, Z_KNEEAX), extra=thigh),
        'side-out': place('side'),
        'side-in': place('side').apply_translation([0, -(Y_SIDE[0] + Y_SIDE[1]), 0]),
        'arch': place('arch'),
        'bulkhead': place('bulkhead'),
    }
    keys = []
    for s in (1, -1):
        for zt in TENON_Z:                       # key drives along Y through the tenon, flat on the arch face
            k = solid('key', P['key'])           # XY profile: x = width, y = length, z = thickness
            k.apply_translation([X_ARCH[1], -KEY_L / 2 + s * (Y_SIDE[0] + Y_SIDE[1]) / 2, zt - T / 2])
            keys.append(k)
    out['keys'] = trimesh.util.concatenate(keys)
    cyl = lambda r, p0, p1, n=64: trimesh.creation.cylinder(radius=r, segment=[p0, p1], sections=n)
    out['rs00-hip'] = cyl(RS00_D / 2, [0, Y_MOTOR[0], 0], [0, Y_MOTOR[1], 0])
    out['rs00-knee'] = cyl(RS00_D / 2, [0, -Y_MOTOR[1], 0], [0, -Y_MOTOR[0], 0])
    out['rs00-abad'] = cyl(RS00_D / 2, [X_ABAD[0], 0, ABAD_Z], [X_ABAD[1], 0, ABAD_Z])
    shafts = [cyl(SHAFT / 2, [0, -Y_PLATE[1] - 1, zz], [0, Y_PLATE[1] + 1, zz], 24) for zz in (Z_IDLER1, Z_IDLER2, Z_KNEEAX)]
    out['shafts'] = trimesh.util.concatenate(shafts).apply_transform(thigh)
    out['paw'] = paw().apply_transform(Ry(-rel)).apply_translation([0, 0, Z_KNEEAX]).apply_transform(thigh)
    d = os.path.join(HERE, 'out', 'assembly')
    os.makedirs(d, exist_ok=True)
    for f in os.listdir(d):
        if f.endswith('.stl'):
            os.remove(os.path.join(d, f))
    mat = {'pinion': 'brass', 'idler': 'brass', 'rs00': 'black', 'shafts': 'steel', 'keys': 'steel', 'paw': 'rubber'}
    lines = [';; quad00 leg, standing at hip height %.0f -- generated by leg.py --assembly' % hip_height]
    for k, m in out.items():
        m.export(os.path.join(d, f'{k}.stl'))
        lines.append(f'[root [import-mesh "{k}.stl"] "{mat.get(k.split("-")[0], "aluminum")}"]')
    open(os.path.join(d, 'leg.loon'), 'w').write('\n'.join(lines) + '\n')
    foot = out['paw'].bounds.mean(axis=0)
    print(f'assembly: thigh {math.degrees(th_t):.1f} deg, shin rel {rel:.1f} deg, paw centre x={foot[0]:.1f} '
          f'z={foot[2]:.1f}; wrote {d}')


if __name__ == '__main__':
    if '--assembly' in sys.argv:
        assembly()
        sys.exit(0)
    if '--check' in sys.argv:
        sys.exit(0 if check() else 1)
    check()
    mass()
    write()
