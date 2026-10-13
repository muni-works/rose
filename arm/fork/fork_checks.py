"""Checks for the fork concept: mass per link, gravity torque, structural deflection, fold ranges.

Same conventions and limits as the original arm (arm/rose_arm_layout.py): 2.5 kg at the flange,
arm straight out, RS06 peak 36 / rated 11 N.m, RS00 14 / 5. Deflection budget 0.5 mm sag and
0.5 mm twist at the flange. Fold ranges: sweep a joint, report where two links' bodies come
within 1 mm (bodies of the two links a joint connects are allowed to nest - they sit in
different planes by design, so they only count if they actually touch).
"""
import itertools
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import rose_fork as F
import rose_arm_geom as Gm
from rose_arm_geom import add, sub, mul, norm

G = 9.81
E, Gs = 69e9, 26e9
PAYLOAD = 2.5
ACT_KG = {round(F.RS04['d'] / 2, 4): F.RS04['kg'], round(F.RS03['d'] / 2, 4): F.RS03['kg'], round(F.RS06['d'] / 2, 4): F.RS06['kg'], round(F.RS00['d'] / 2, 4): F.RS00['kg'], round(F.EDU['d'] / 2, 4): F.EDU['kg']}


_MASS_CACHE = {}


def masses():
    """(link, kg, x of centroid, zero pose) per primitive. Cached per arm length."""
    key = (F.UPPER, F.FORE, F.W)
    if key in _MASS_CACHE:
        return _MASS_CACHE[key]
    rows = []
    for p in F.build():
        link, mat, o, u, v, ol, vec = p[:7]
        if mat == 'actuator':
            r = round(abs(ol[0][1][0]), 4)
            m = ACT_KG.get(r, 0.3)
            c = add(o, mul(vec, 0.5))
        else:
            m = F.prim_mass(p) or 0.0
            pts = Gm.prim_points(p[:7], step=0.02)
            c = tuple(sum(q[i] for q in pts) / len(pts) for i in range(3))
        rows.append((link, max(m, 0.0), c[0]))
    rows.append(('harness', 0.20, 0.35))
    _MASS_CACHE[key] = rows
    return rows


def flange_x():
    return F.W + 0.040 + 0.007 + F.RS00['L'] + 0.004 + F.EDU['d'] + 0.060


MOVING = ('upper', 'fore', 'yoke', 'wrist', 'tool', 'harness')


def moment(x0, payload=PAYLOAD):
    return G * (sum(m * (x - x0) for l, m, x in masses() if l in MOVING and x > x0) + payload * (flange_x() - x0))


def section(h, W, t=F.T_PL, tc=F.T_CV, hw=None, window_frac=0.55):
    """Fork link section: two side plates (h deep, windowed over window_frac of the length),
    a top cover, a bottom cover and two 2020 bars along the bottom corners."""
    hw = hw if hw is not None else max(0.0, h - 2 * F.FLANGE)
    # bending, averaged along the length between solid and windowed stations
    I_solid = 2 * t * h ** 3 / 12
    I_win = 2 * t * (h ** 3 - hw ** 3) / 12
    I_pl = window_frac * I_win + (1 - window_frac) * I_solid
    I = I_pl + 2 * W * tc * (h / 2) ** 2 + 2 * (0.70e-8 + 1.6e-4 * (h / 2 - 0.01) ** 2)
    # torsion, Bredt: the windowed side walls carry shear only through their flanges (Vierendeel),
    # so their effective shear thickness drops with the window - keep it honest with (1 - hw/h)^3
    t_side = window_frac * t * (1 - hw / h) ** 3 + (1 - window_frac) * t
    J = 4 * (W * h) ** 2 / (2 * h / t_side + 2 * W / tc)
    return I, J


def stiffness():
    U, Fo = F.UPPER, F.FORE
    hu, Wu = 2 * F.R_LOBE_EL, 2 * F.Y_UPPER_DIST
    hf, Wf = 2 * F.R_LOBE0 + 0.016, 2 * F.Y_FORE_DIST
    Iu, Ju = section(hu, Wu)
    If, Jf = section(hf, Wf)
    xF = flange_x()
    m_e, m_w = moment(U), moment(U + Fo)
    F_e, F_w = m_e / (xF - U), m_w / (xF - U - Fo)
    th_u = (F_e * U ** 2 / 2 + m_e * U) / (E * Iu)
    d_u = (F_e * U ** 3 / 3 + m_e * U ** 2 / 2) / (E * Iu)
    th_f = (F_w * Fo ** 2 / 2 + m_w * Fo) / (E * If)
    d_f = (F_w * Fo ** 3 / 3 + m_w * Fo ** 2 / 2) / (E * If)
    sag = d_u + th_u * (xF - U) + d_f + th_f * (xF - U - Fo)
    # twist: wrist pitched 90 puts the hand's moment into the forearm as torque; elbow at 90 the same for the upper arm
    twist = m_w * Fo / (Gs * Jf) * (xF - U - Fo) + m_e * U / (Gs * Ju) * (xF - U) * 0.0 + \
        moment(U, 0) * 0 + (m_e * U / (Gs * Ju)) * (xF - U)
    return sag, twist, (Iu, Ju, If, Jf)


def posed(clouds, q):
    T = F.fk(q)
    out = []
    for link, pts in clouds:
        R, t = T[link]
        o = F.ORIGIN[link]
        P = [add(Gm.mv(R, sub(x, o)), t) for x in pts]
        out.append((link, P, tuple(min(x[i] for x in P) for i in range(3)), tuple(max(x[i] for x in P) for i in range(3))))
    return out


def pair_min(posed_, la, lb):
    best = 1.0
    for l1, P1, lo1, hi1 in posed_:
        if l1 != la:
            continue
        for l2, P2, lo2, hi2 in posed_:
            if l2 != lb:
                continue
            if any(min(hi1[k], hi2[k]) - max(lo1[k], lo2[k]) < -best for k in range(3)):
                continue
            for a in P1[::2]:
                for b in P2[::2]:
                    d = norm(sub(a, b))
                    if d < best:
                        best = d
    return best


def fold(joint, lo, hi, step=5):
    """Sweep one joint; the free range round 0. A pair of links counts as colliding when their bodies
    come within 1 mm AND closer than they sit at the zero pose - so coaxial interfaces that touch by
    design (collar halves, a plate on its sleeve) don't count, but anything new does."""
    clouds = [(p[0], Gm.prim_points(p[:7], step=0.010)) for p in F.build()]
    links = [l[0] for l in F.LINKS]
    z0 = posed(clouds, {})
    base = {(a, b): pair_min(z0, a, b) for a, b in itertools.combinations(links, 2)}
    ok = {}
    for d in range(lo, hi + 1, step):
        P = posed(clouds, {joint: math.radians(d)})
        hit = None
        for a, b in itertools.combinations(links, 2):
            m = pair_min(P, a, b)
            if m < 0.001 and m < base[(a, b)] - 0.0015:
                hit = (a, b)
                break
        ok[d] = hit
    # contiguous free range containing 0
    a = 0
    while a - step >= lo and ok.get(a - step) is None:
        a -= step
    b = 0
    while b + step <= hi and ok.get(b + step) is None:
        b += step
    first_hit = ok.get(b + step) or ok.get(a - step)
    return a, b, first_hit


def main():
    by = {}
    for l, m, x in masses():
        by[l] = by.get(l, 0.0) + m
    tot = sum(by.values())
    print('mass by link: ' + ', '.join('%s %.2f' % (k, v) for k, v in by.items()))
    print('total %.2f kg   moving (shoulder outward) %.2f kg' % (tot, sum(by[k] for k in MOVING if k in by)))
    print('flange at %.0f mm from the shoulder' % (flange_x() * 1e3))
    print('\ngravity torque, arm straight out, %.1f kg at the flange:' % PAYLOAD)
    for name, x0, peak, stall in (('J2 shoulder (RS04)', 0.0, 120, 25), ('J4 elbow (RS04)', F.UPPER, 120, 25),
                                  ('J6 wrist (RS00)', F.W, 14, 3.6)):
        m = moment(x0)
        print('  %-20s %5.1f N.m   %3.0f%% of peak   %.2fx stall-continuous' % (name, m, 100 * m / peak, m / stall))
    for name, x0, stall in (('J2', 0.0, 25), ('J4', F.UPPER, 25), ('J6', F.W, 3.6)):
        free = stall - moment(x0, 0.0)
        print('  %s holds %.2f kg straight out, continuously, no spring' % (name, free / (G * (flange_x() - x0))))

    sag, tw, secs = stiffness()
    print('\nflange deflection from structure: %.2f mm sag, %.2f mm twist   (budget 0.5 / 0.5)' % (sag * 1e3, tw * 1e3))
    print('\nfold ranges (no two links within 1 mm):')
    for j, lo, hi in (('J2_sho_pitch', -150, 120), ('J4_elbow', -170, 170), ('J6_wrist_pitch', -150, 150)):
        a, b, hit = fold(j, lo, hi)
        print('  %-15s %4d .. %-4d  limited by %s' % (j, a, b, '%s / %s' % hit if hit else 'its sweep end'))


if __name__ == '__main__':
    main()
