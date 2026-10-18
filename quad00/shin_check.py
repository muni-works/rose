"""shin strength from the actual geometry: cut the shin at heights along its axis, take each section's area,
centroid and second moments, and check two load cases against 5052-H32 (yield 193 MPa):
  landing  -- 3 x body weight on one leg, axial, applied on the line from the knee hub to the ankle pin,
              so every section also sees the bending of its centroid's offset from that line;
  knee     -- the knee motor's peak torque, carried as in-plane bending.
Euler buckling of the weakest stretch is reported too. Usage: python3 shin_check.py"""
import math, numpy as np, trimesh
import quad as Q
import leg as L

YIELD, E = 193.0, 69000.0                                # MPa, 5052-H32


def _ring(xy):
    """area, first and second moments (about the origin) of a closed ring, signed by orientation."""
    x, y = np.asarray(xy)[:, 0], np.asarray(xy)[:, 1]
    x1, y1 = np.roll(x, -1), np.roll(y, -1)
    cr = x * y1 - x1 * y
    A = cr.sum() / 2
    Sx = ((x + x1) * cr).sum() / 6; Sy = ((y + y1) * cr).sum() / 6
    Iyy0 = ((x * x + x * x1 + x1 * x1) * cr).sum() / 12; Ixx0 = ((y * y + y * y1 + y1 * y1) * cr).sum() / 12
    return A, Sx, Sy, Ixx0, Iyy0


def sections(parts, LS, zs):
    """section properties of the shin plates at heights z below the knee, from their flat profiles (each plate
    is profile x thickness, so a cut is the profile's x-intervals at that z times the plate's thickness)."""
    from shapely.geometry import LineString
    out = []
    for z in zs:
        rects = []
        for prof, y0, t in Q.SHIN_PROFILES:
            cut = prof.intersection(LineString([(-500, -z), (500, -z)]))
            for seg in getattr(cut, 'geoms', [cut]):
                if seg.is_empty or seg.length < 0.1: continue
                xs = [p[0] for p in seg.coords]
                rects.append((min(xs), max(xs), y0, y0 + t))
        if not rects:
            out.append(None); continue
        A = sum((x1 - x0) * (y1 - y0) for x0, x1, y0, y1 in rects)
        cx = sum((x1 - x0) * (y1 - y0) * (x0 + x1) / 2 for x0, x1, y0, y1 in rects) / A
        cy = sum((x1 - x0) * (y1 - y0) * (y0 + y1) / 2 for x0, x1, y0, y1 in rects) / A
        Ixx = sum((x1 - x0) * (y1 - y0) ** 3 / 12 + (x1 - x0) * (y1 - y0) * ((y0 + y1) / 2 - cy) ** 2 for x0, x1, y0, y1 in rects)
        Iyy = sum((y1 - y0) * (x1 - x0) ** 3 / 12 + (x1 - x0) * (y1 - y0) * ((x0 + x1) / 2 - cx) ** 2 for x0, x1, y0, y1 in rects)
        out.append(dict(z=z, A=A, cx=cx, cy=cy, Ixx=Ixx, Iyy=Iyy,
                        cy_far=max(max(r[3] for r in rects) - cy, cy - min(r[2] for r in rects)),
                        cx_far=max(max(r[1] for r in rects) - cx, cx - min(r[0] for r in rects))))
    return out


def main(mass=28.8, knee_peak=120.0, leg=(210.0, 300.0)):
    parts = Q.inline_parts(0.0, 0.0, 'ankle', leg[0], leg[1], mot='rs04', ab_mot='rs06', end_mot='rs06', dual=True)
    LS = leg[1]
    P = 3 * mass * 9.81                                  # N, one-leg landing
    y_knee = (Q.IL_SHIN[1] + Q.HY_MOT[0]) / 2            # knee hub (load in)
    G = Q.ankle2_geom(LS, 140.0, 0.8 * 88, Q.MOTORS['rs06'])
    y_ank = G['yc']                                      # ankle pin / U-joint centre (load out)
    zs = list(range(30, int(LS) - 9, 10))
    worst = (0, None)
    print(f'landing {P:.0f} N on one leg; knee peak {knee_peak:.0f} N.m; load line y {y_knee:.1f} (knee) -> {y_ank:.1f} (ankle)')
    print(' z(mm)   A(mm2)  ecc(mm)  sigma_land(MPa)  sigma_knee(MPa)')
    for s in sections(parts, LS, zs):
        if s is None: continue
        yl = y_knee + (y_ank - y_knee) * s['z'] / LS
        e = abs(s['cy'] - yl)
        sl = P / s['A'] + P * e * s['cy_far'] / s['Ixx']
        sk = knee_peak * 1000 * (1 - s['z'] / LS) * s['cx_far'] / s['Iyy'] if s['z'] < LS else 0   # knee torque reacted at the foot: falls along the shin
        flag = '  <-- over yield' if max(sl, sk) > YIELD else ('  <-- under 2x margin' if max(sl, sk) > YIELD / 2 else '')
        print(f' {s["z"]:5.0f} {s["A"]:8.0f} {e:8.1f} {sl:12.0f} {sk:16.0f}{flag}')
        worst = max(worst, (max(sl, sk), s['z']))
    Imin = min(s['Ixx'] for s in sections(parts, LS, zs) if s)
    Pcr = math.pi ** 2 * E * Imin / (LS ** 2)
    print(f'worst {worst[0]:.0f} MPa at z {worst[1]} (5052-H32 yield {YIELD:.0f}); Euler (weakest I, full length, pinned) {Pcr:.0f} N vs {P:.0f} N')


if __name__ == '__main__':
    main()
