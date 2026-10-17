"""quad00 leg: an RS00 quadruped leg with a sheet-gear knee.

One leg, as a bench rig: carrier -> hip RS00 (outboard) + knee RS00 (inboard) -> thigh fork
-> pinion z17 / idler z17 / idler z17 / knee gear z34 (2:1) -> shin laminate -> paw.

Every structural part is a flat profile in one stock (3.2 mm), laminated where it needs to be
thicker. The same outlines feed both outputs:
  python3 leg.py            -> out/dxf/*.dxf (router, per layer) + out/stl/*.stl (printer, laminates fused)
  python3 leg.py --check    -> gear mesh interference sweep + stack clearances, no files

Leg frame: hip axis is +Y through the origin, thigh hangs along -Z, X forward. Gear plane is y=0.
Needs: pip install shapely trimesh manifold3d ezdxf mapbox_earcut numpy
"""
import math
import os
import sys

import ezdxf
import numpy as np
import trimesh
from manifold3d import CrossSection, FillRule, Manifold
from shapely import affinity
from shapely.geometry import Point, Polygon, MultiPolygon
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

# RS00 (arm/fork/rose_fork.py): body D57 x L51, output 6x M3 on D27, back 4x on D38
RS00_D, RS00_L = 57.0, 51.0
OUT_PCD, OUT_N = 27.0, 6
BACK_PCD, BACK_N = 38.0, 4
M3, M3_CSK = 3.4, 6.3
SHAFT, BUSH_OD = 8.1, 10.0          # M8 shoulder bolt / 8x10 flanged plain bushing
HUB_D = 34.0                        # pinion hub: inner thigh plate rides on it (printed: direct)

# stack along Y (outboard +)
G = GEAR_LAYERS * T / 2                              # gear half-face, 4.8
Y_PLATE = (G + PLATE_GAP, G + PLATE_GAP + T)         # thigh plates at +/- this
Y_OUTFACE = Y_PLATE[1]                               # motor output faces bear on the plates
Y_MOTOR = (Y_OUTFACE + 1.0, Y_OUTFACE + 1.0 + RS00_L)  # stator body (1 mm nose)
Y_CARRIER = (Y_MOTOR[1], Y_MOTOR[1] + 2 * T)         # carrier: 2 layers, motor back-mounted

STANDOFFS = [(-30.0, 45.0), (0.0, 55.0), (30.0, 45.0)]   # carrier bridge, above the hip
HIP_LIMIT = 180 - math.degrees(math.asin((30 + 5) / 45.0))  # thigh hub r30 vs standoff r45, M5


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
        # root arc to the next tooth
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


# ------------------------------------------------------------------ parts (2D, leg frame XZ)
def pinion_tooth_layer():
    return gear(Z_SMALL, phase=-90).difference(holes_on(OUT_PCD, OUT_N, M3))


def pinion_hub():
    return circle(0, 0, HUB_D).difference(holes_on(OUT_PCD, OUT_N, M3))


def idler(phase):
    p = gear(Z_SMALL, phase=phase)
    p = p.difference(circle(0, 0, BUSH_OD)).difference(holes_on(24, 3, M3, phase=90))
    return p


def shin_layer():
    """knee gear and shin bar are one laminate; knee axis at origin, shin along -Z."""
    g = gear(Z_KNEE, phase=90 + 180 / Z_KNEE)
    bar = stadium([(0, 0, 24), (0, -SHIN, 24)])
    s = unary_union([g, bar])
    s = s.difference(circle(0, 0, BUSH_OD))
    for ang in (90, 210, 330):                                    # lightening, clear of the bar
        s = s.difference(circle(21 * math.cos(math.radians(ang)), 21 * math.sin(math.radians(ang)), 14))
    s = s.difference(holes_on(60, 3, M3, phase=30))               # lamination bolts in the gear
    for zz in (-55, -100):
        s = s.difference(circle(0, zz, M3))
    return s.difference(circle(0, -SHIN, 5.3))                    # paw bolt


def thigh_plate(outer):
    p = stadium([(0, 0, 60), (0, Z_IDLER1, 54), (0, Z_IDLER2, 54), (0, Z_KNEEAX, 32)])
    hip = holes_on(OUT_PCD, OUT_N, M3) if outer else circle(0, 0, HUB_D + 0.4)
    p = p.difference(hip)
    for zz in (Z_IDLER1, Z_IDLER2, Z_KNEEAX):
        p = p.difference(circle(0, zz, SHAFT))
    for zz in ((Z_IDLER1 + Z_IDLER2) / 2, (Z_IDLER2 + Z_KNEEAX) / 2):  # lightening between shafts
        p = p.difference(circle(0, zz, 14))
    return p


def carrier_plate():
    body = stadium([(0, 0, RS00_D + 14), (-40, 70, 20), (40, 70, 20)])
    body = body.difference(holes_on(BACK_PCD, BACK_N, M3, phase=45))
    body = body.difference(circle(0, 0, 16))                       # back-face cable exit
    for x, z in STANDOFFS:
        body = body.difference(circle(x, z, 5.3))
    for x in (-40, 40):
        body = body.difference(circle(x, 70, 6.4))                 # bench / torso mount
    return body


# ------------------------------------------------------------------ the bill
# name -> ([(profile, thickness), ...] from y0 outward, y0, countersinks [(x, z, 'top'|'bottom')], qty)
# The pinion hub is T + PLATE_GAP so the teeth sit centred on y=0 (aluminium: a 0.4 mm shim under it).
OUT_HOLES = [(OUT_PCD / 2 * math.cos(math.radians(60 * i)), OUT_PCD / 2 * math.sin(math.radians(60 * i))) for i in range(OUT_N)]


def parts():
    return {
        'pinion': ([(pinion_hub(), T + PLATE_GAP)] + [(pinion_tooth_layer(), T)] * GEAR_LAYERS,
                   -Y_PLATE[1], [(x, z, 'top') for x, z in OUT_HOLES], 1),
        'idler': ([(idler(0), T)] * GEAR_LAYERS, -G, [], 2),
        'shin': ([(shin_layer(), T)] * GEAR_LAYERS, -G, [], 1),
        'thigh-outer': ([(thigh_plate(True), T)], Y_PLATE[0], [(x, z, 'bottom') for x, z in OUT_HOLES], 1),
        'thigh-inner': ([(thigh_plate(False), T)], -Y_PLATE[1], [], 1),
        'carrier': ([(carrier_plate(), T)] * 2, Y_CARRIER[0], [], 2),
    }


# ------------------------------------------------------------------ outputs
def to_dxf(poly, path):
    doc = ezdxf.new('R2000')
    msp = doc.modelspace()
    geoms = poly.geoms if isinstance(poly, MultiPolygon) else [poly]
    for g in geoms:
        for ring in [g.exterior] + list(g.interiors):
            msp.add_lwpolyline(list(ring.coords), close=True)
    doc.saveas(path)


YZ = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], float)   # swap Y<->Z


def rings(poly):
    geoms = poly.geoms if isinstance(poly, MultiPolygon) else [poly]
    out = []
    for g in geoms:
        out.append(np.asarray(g.exterior.coords[:-1]))
        out += [np.asarray(r.coords[:-1]) for r in g.interiors]
    return out


def extrude(poly, y0, t):
    """manifold of an XZ profile, from y0 along +Y by t (built along Z, then axes swapped)."""
    m = CrossSection(rings(poly), FillRule.EvenOdd).extrude(t).translate((0, 0, y0))
    return m.transform(YZ[:3, :])


def csk_cone(x, z, y_face, inward):
    """90 deg M3 flat-head seat, D6.3 at the face. inward = +1 (into +Y) or -1."""
    e = 0.2
    r0 = M3_CSK / 2 + e                                   # base sits e outside the face
    r1 = M3 / 2 + 0.25                                    # stop short of the hole wall: no sliver faces
    c = Manifold.cylinder(r0 - r1, r0, r1, 48)           # 90 deg frustum, wide end at z=0
    if inward < 0:
        c = c.mirror((0, 0, 1))
    c = c.translate((x, z, y_face - inward * e))         # built in (x, z, y), swapped below
    return c.transform(YZ[:3, :])


def solid(name, spec):
    layers, y0, csks, _ = spec
    m, y = None, y0
    for poly, t in layers:
        e = extrude(poly, y, t)
        m = e if m is None else m + e
        y += t
    for x, z, f in csks:
        m = m - csk_cone(x, z, y if f == 'top' else y0, -1 if f == 'top' else 1)
    mesh = m.to_mesh()
    return trimesh.Trimesh(mesh.vert_properties[:, :3], mesh.tri_verts)


def to_dxf(poly, path):
    doc = ezdxf.new('R2000')
    msp = doc.modelspace()
    geoms = poly.geoms if isinstance(poly, MultiPolygon) else [poly]
    for g in geoms:
        for ring in [g.exterior] + list(g.interiors):
            msp.add_lwpolyline(list(ring.coords), close=True)
    doc.saveas(path)


def extrude(poly, y0, t):
    """extrude an XZ profile along +Y from y0 by t."""
    m = trimesh.creation.extrude_polygon(poly, t)      # profile in XY, extruded along +Z
    m.apply_transform(np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], float))  # Y<->Z
    if m.volume < 0:
        m.invert()
    return m.apply_translation([0, y0, 0])


def csk_cone(x, z, y_face, inward):
    """90 deg M3 flat-head seat, D6.3 at the face. inward = +1 (into +Y) or -1."""
    e = 0.2
    r0 = M3_CSK / 2 + e                                   # base sits e outside the face
    m = Manifold.cylinder(r0 - M3 / 2 + 0.1, r0, M3 / 2 - 0.1, 48).to_mesh()   # frustum, wide end at z=0
    c = trimesh.Trimesh(m.vert_properties[:, :3], m.tri_verts)
    c.apply_transform(trimesh.transformations.rotation_matrix(math.radians(-90 * inward), [1, 0, 0]))
    return c.apply_translation([x, y_face - inward * e, z])


def solid(name, spec):
    layers, y0, csks, _ = spec
    meshes, y = [], y0
    for poly, t in layers:
        meshes.append(extrude(poly, y, t))
        y += t
    m = trimesh.boolean.union(meshes, engine='manifold') if len(meshes) > 1 else meshes[0]
    if csks:
        cones = [csk_cone(x, z, y if f == 'top' else y0, -1 if f == 'top' else 1) for x, z, f in csks]
        m = trimesh.boolean.difference([m] + cones, engine='manifold')
    return m


def to_dxf(poly, path, csks=()):
    doc = ezdxf.new('R2000')
    doc.layers.add('CUT')
    doc.layers.add('CSK_M3_90DEG')
    msp = doc.modelspace()
    geoms = poly.geoms if isinstance(poly, MultiPolygon) else [poly]
    for g in geoms:
        for ring in [g.exterior] + list(g.interiors):
            msp.add_lwpolyline(list(ring.coords), close=True, dxfattribs={'layer': 'CUT'})
    for x, z, _ in csks:
        msp.add_circle((x, z), M3_CSK / 2, dxfattribs={'layer': 'CSK_M3_90DEG'})
    doc.saveas(path)


def write():
    out = os.path.join(HERE, 'out')
    for d in ('dxf', 'stl'):
        os.makedirs(os.path.join(out, d), exist_ok=True)
    built = {}
    flat = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]], float)  # thickness (Y) -> print Z
    for name, spec in parts().items():
        layers, y0, csks, qty = spec
        uniq = []
        for i, (p, t) in enumerate(layers):
            if any(p.equals(q) for q, _ in uniq):
                continue
            uniq.append((p, t))
            n = sum(1 for q, _ in layers if q.equals(p)) * qty
            idx = [k for k, (q, _) in enumerate(layers) if q.equals(p)]
            face = [c for c in csks if (c[2] == 'top' and len(layers) - 1 in idx) or (c[2] == 'bottom' and 0 in idx)]
            tag = f'_csk-{qty}' if face else ''          # countersink only that many of them
            to_dxf(p, os.path.join(out, 'dxf', f'{name}-L{i}_{T}mm_x{n}{tag}.dxf'), face)
        m = solid(name, spec).apply_transform(flat)
        m.apply_translation(-m.bounds[0])
        m.export(os.path.join(out, 'stl', f'{name}_x{qty}.stl'))
        built[f'{name}_x{qty}'] = m
    paw = trimesh.creation.cylinder(radius=16, height=28, sections=64)
    slot = trimesh.creation.box((40, 2 * G + 0.4, 20)).apply_translation([0, 0, 10])
    bolt = trimesh.creation.cylinder(radius=2.65, height=40, sections=32)
    bolt.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0])).apply_translation([0, 0, 6])
    paw = trimesh.boolean.difference([paw, slot, bolt], engine='manifold').apply_translation([0, 0, 14])
    paw.export(os.path.join(out, 'stl', 'paw_TPU_x1.stl'))
    built['paw_TPU_x1'] = paw
    print(f'wrote {out}/dxf + {out}/stl')
    plates(out, built)


# Bambu P1S: 256 x 256 bed. (stl, copies, x, y, rotate 90 about Z) -- min corners, mm from the bed corner
BED = 256.0
PLATES = {
    'plate1_PETG_links': [('shin_x1', 1, 8, 24, False), ('thigh-outer_x1', 1, 106, 30, False),
                          ('thigh-inner_x1', 1, 176, 30, False)],
    'plate2_PETG_gears-carrier': [('carrier_x2', 2, 18, 12, False), ('idler_x2', 2, 22, 150, False),
                                  ('pinion_x1', 1, 140, 150, False)],
    'plate3_TPU_paw': [('paw_TPU_x1', 1, 112, 112, False)],
}


def plates(out, built):
    """one 3MF per P1S plate, parts already flat and laid out; Bambu Studio opens each as one plate.
    Built from the in-memory solids, not the STLs: STL drops the indexed topology at the countersinks."""
    d = os.path.join(out, '3mf')
    os.makedirs(d, exist_ok=True)
    for plate, items in PLATES.items():
        scene = trimesh.Scene()
        boxes = []
        for stl, copies, x, y, rot in items:
            base = built[stl].copy()
            if rot:
                base.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 0, 1]))
            base.apply_translation(-base.bounds[0])
            w = base.extents[0]
            for k in range(copies):
                m = base.copy().apply_translation([x + k * (w + 8), y, 0])
                scene.add_geometry(m, node_name=f'{stl.rsplit("_x", 1)[0]}-{k + 1}', geom_name=f'{stl}-{k}')
                boxes.append(m.bounds)
        lo = np.min([b[0] for b in boxes], axis=0)
        hi = np.max([b[1] for b in boxes], axis=0)
        assert lo[0] >= 5 and lo[1] >= 5 and hi[0] <= BED - 5 and hi[1] <= BED - 5, f'{plate} off the bed: {lo} {hi}'
        for i in range(len(boxes)):
            for j in range(i + 1, len(boxes)):
                a, b = boxes[i], boxes[j]
                assert (a[1][:2] + 3 <= b[0][:2]).any() or (b[1][:2] + 3 <= a[0][:2]).any(), f'{plate}: parts {i},{j} overlap'
        scene.export(os.path.join(d, plate + '.3mf'))
        print(f'  {plate}.3mf: {len(boxes)} parts, x {lo[0]:.0f}..{hi[0]:.0f}  y {lo[1]:.0f}..{hi[1]:.0f}')


def mass():
    """per leg, from the solids: aluminium (2.70) and as printed (PETG 1.27 at ~55% effective fill)."""
    vol = {n: solid(n, s).volume * s[3] / 1000 for n, s in parts().items()}
    for n, v in vol.items():
        print(f'  {n:12s} {v:5.1f} cm3  alu {v * 2.70:4.0f} g')
    leg = sum(v for n, v in vol.items() if n != 'carrier')
    print(f'leg below the carrier: {leg:.0f} cm3 -> aluminium {leg * 2.70:.0f} g, printed PETG ~{leg * 1.27 * 0.55:.0f} g'
          f'  (+ 2x RS00 {2 * 310} g, hardware ~60 g); carrier is rig/torso')


# ------------------------------------------------------------------ checks
def check():
    ok = True
    # gear mesh: rotate through one tooth pitch, report worst overlap area per mesh
    pairs = [((0, 0), Z_SMALL, -90, (0, Z_IDLER1), Z_SMALL, 90 + 180 / Z_SMALL),
             ((0, Z_IDLER1), Z_SMALL, 90 + 180 / Z_SMALL, (0, Z_IDLER2), Z_SMALL, 90 + 180 / Z_SMALL + 180 / Z_SMALL + 180),
             ((0, Z_IDLER2), Z_SMALL, 0, (0, Z_KNEEAX), Z_KNEE, 0)]
    for a_at, za, pa, b_at, zb, pb in pairs:
        worst, best = 0.0, None
        # find b's phase that meshes: scan for minimum overlap at rest
        trial = [pb + d * 360 / zb / 200 for d in range(200)]
        ga = gear(za, phase=pa, at=a_at)
        pb = max((ph for ph in trial if ga.intersection(gear(zb, phase=ph, at=b_at)).area == 0), key=lambda ph: ga.distance(gear(zb, phase=ph, at=b_at)))
        for s in range(13):
            da = 360 / za * s / 12
            A = gear(za, phase=pa + da, at=a_at)
            B = gear(zb, phase=pb - da * za / zb, at=b_at)
            worst = max(worst, A.intersection(B).area)
            gap = A.distance(B)
            best = gap if best is None else min(best, gap)
        flag = 'OK' if worst < 0.05 else 'INTERFERES'
        ok &= worst < 0.05
        print(f'mesh z{za}->z{zb} C={math.dist(a_at, b_at):.2f}: worst overlap {worst:.3f} mm2  min gap {best:.3f} mm  {flag}')
    # stack
    print(f'stack (y): gear +/-{G:.1f}, plates {Y_PLATE[0]:.1f}..{Y_PLATE[1]:.1f}, motor {Y_MOTOR[0]:.1f}..{Y_MOTOR[1]:.1f}, '
          f'carrier {Y_CARRIER[0]:.1f}..{Y_CARRIER[1]:.1f}  -> {2 * Y_CARRIER[1]:.0f} mm wide')
    print(f'thigh plate covers idlers: plate r27 vs idler tip r{M * (Z_SMALL + 2) / 2:.2f}')
    # knee fold: shin bar (in the gear plane) against idler 2's tip circle
    idl = circle(0, Z_IDLER2 - Z_KNEEAX, M * (Z_SMALL + 2) + 1.0)
    bar = stadium([(0, 0, 24), (0, -SHIN, 24)])
    fold = min(a for a in range(0, 181) if not affinity.rotate(bar, 180 - a, origin=(0, 0)).intersects(idl))
    print(f'knee fold limit: shin bar meets idler 2 at {fold} deg interior (straight leg = 180)')
    print(f'knee gear tip D{M * (Z_KNEE + 2):.0f}; hip range limited by carrier bridge: +/-{HIP_LIMIT:.0f} deg from down')
    return ok


def mesh_phase(fixed, z, at, base):
    """the phase of a z-tooth gear at `at` that meshes with the `fixed` outline, centred in its backlash."""
    trial = [base + d * 360 / z / 120 for d in range(120)]
    free = [ph for ph in trial if fixed.intersection(gear(z, phase=ph, at=at)).area == 0]
    return max(free, key=lambda ph: fixed.distance(gear(z, phase=ph, at=at)))


def assembly(hip_height=255.0):
    """export every part posed standing (hip height above the paw centre) to out/assembly + a loon for vcad."""
    L1, L2 = -Z_KNEEAX, SHIN
    a = math.acos((L1 ** 2 + hip_height ** 2 - L2 ** 2) / (2 * L1 * hip_height))
    th_t = a                                                      # R_y(+th_t): knee goes back (-X)
    kx, kz = -L1 * math.sin(th_t), -L1 * math.cos(th_t)
    th_s = math.asin(-(0 - kx) / L2)                              # shin points at the paw under the hip
    rel = -math.degrees(th_s - th_t)                              # shin angle in the thigh frame, CCW in XZ
    knee_ph = 90 + 180 / Z_KNEE + rel
    p2 = mesh_phase(gear(Z_KNEE, phase=knee_ph, at=(0, Z_KNEEAX)), Z_SMALL, (0, Z_IDLER2), 0)
    p1 = mesh_phase(gear(Z_SMALL, phase=p2, at=(0, Z_IDLER2)), Z_SMALL, (0, Z_IDLER1), 0)
    pp = mesh_phase(gear(Z_SMALL, phase=p1, at=(0, Z_IDLER1)), Z_SMALL, (0, 0), -90)
    Ry = lambda deg: trimesh.transformations.rotation_matrix(math.radians(deg), [0, 1, 0])
    thigh = Ry(math.degrees(th_t))
    P = parts()
    def place(name, spin=0.0, at=(0.0, 0.0), extra=None):
        m = solid(name, P[name])
        m.apply_transform(Ry(-spin))
        m.apply_translation([at[0], 0, at[1]])
        if extra is not None:
            m.apply_transform(extra)
        return m
    knee_T = trimesh.transformations.translation_matrix([0, 0, Z_KNEEAX])
    out = {
        'thigh-outer': place('thigh-outer', extra=thigh),
        'thigh-inner': place('thigh-inner', extra=thigh),
        'pinion': place('pinion', spin=pp + 90, extra=thigh),
        'idler-1': place('idler', spin=p1, at=(0, Z_IDLER1), extra=thigh),
        'idler-2': place('idler', spin=p2, at=(0, Z_IDLER2), extra=thigh),
        'shin': place('shin', spin=rel, at=(0, Z_KNEEAX), extra=thigh),
        'carrier-out': place('carrier'),
        'carrier-in': place('carrier').apply_translation([0, -(2 * Y_CARRIER[0] + 2 * T), 0]),
    }
    cyl = lambda r, y0, y1, n=64: trimesh.creation.cylinder(radius=r, segment=[[0, y0, 0], [0, y1, 0]], sections=n)
    out['rs00-hip'] = cyl(RS00_D / 2, *Y_MOTOR)
    out['rs00-knee'] = cyl(RS00_D / 2, -Y_MOTOR[1], -Y_MOTOR[0])
    shafts = [cyl(SHAFT / 2, -Y_PLATE[1] - 1, Y_PLATE[1] + 1, 24).apply_translation([0, 0, zz]) for zz in (Z_IDLER1, Z_IDLER2, Z_KNEEAX)]
    out['shafts'] = trimesh.util.concatenate(shafts).apply_transform(thigh)
    so = [cyl(2.5, -Y_CARRIER[1], Y_CARRIER[1], 16).apply_translation([x, 0, z]) for x, z in STANDOFFS]
    out['standoffs'] = trimesh.util.concatenate(so)
    paw = trimesh.creation.cylinder(radius=16, height=28, sections=48,
                                    transform=trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
    out['paw'] = paw.apply_translation([0, 0, -SHIN]).apply_transform(Ry(-rel)).apply_translation([0, 0, Z_KNEEAX]).apply_transform(thigh)
    d = os.path.join(HERE, 'out', 'assembly')
    os.makedirs(d, exist_ok=True)
    mat = {'thigh': 'aluminum', 'shin': 'aluminum', 'carrier': 'aluminum', 'pinion': 'brass', 'idler': 'brass',
           'rs00': 'black', 'shafts': 'steel', 'standoffs': 'steel', 'paw': 'rubber'}
    lines = [';; quad00 leg, standing at hip height %.0f -- generated by leg.py --assembly' % hip_height]
    for k, m in out.items():
        m.export(os.path.join(d, f'{k}.stl'))
        lines.append(f'[root [import-mesh "{k}.stl"] "{mat[k.split("-")[0]]}"]')
    open(os.path.join(d, 'leg.loon'), 'w').write('\n'.join(lines) + '\n')
    foot = trimesh.transformations.transform_points([[0, 0, -SHIN]], Ry(-rel) @ np.eye(4))[0] + [0, 0, Z_KNEEAX]
    foot = trimesh.transformations.transform_points([foot], thigh)[0]
    print(f'assembly: thigh {math.degrees(th_t):.1f} deg, shin rel {rel:.1f} deg, paw at x={foot[0]:.1f} z={foot[2]:.1f}; wrote {d}')


if __name__ == '__main__':
    if '--assembly' in sys.argv:
        assembly()
        sys.exit(0)
    if '--check' in sys.argv:
        sys.exit(0 if check() else 1)
    check()
    mass()
    write()
