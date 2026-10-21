"""Clash check for the lymb exterior, across the full joint range.

  covers vs structure   every cover against every structural solid of every OTHER link, both ways
                        (cover points inside the solid, the solid's points inside the cover's envelope)
  covers vs each other  every pair of covers on different links, both ways
  covers vs own link    at the zero pose, the link's own parts must not enter a cover's material
                        (they may sit in its hollow -- hub caps, bolt heads, end-plate edges)
  covers vs harness     the cable centreline (+ its radius) against cover material, every pose

The relative pose of two links depends only on the joints between them, so each pair sweeps a grid
of just those joints (pitch joints over the arm's limits, rolls every 45 deg); others stay at zero.
A clash is > 0.3 mm deep, and counts only at a pose the bare structure reaches without colliding
(audit.interferes) -- where the plates already hit, the cover can't take range away. The joint-range table re-runs each joint alone at 1 deg to its limits and
reports the first angle a cover stops it, next to the structure's own stop (audit.py).

Run:  python3 arm/fork/exterior/clash.py [pitch step deg, default 10]
"""
import itertools
import math
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
import exterior as X
import rose_fork as F
from rose_arm_geom import add, sub, mv

DEEP = 0.0003
ORDER = [l[0] for l in F.LINKS]
JOINT = {l[0]: l[1] for l in F.LINKS}
LIM = {'J1_yaw': (-180, 180), 'J2_sho_pitch': (-180, 40), 'J4_elbow': (-115, 115), 'J5_fore_roll': (-180, 180),
       'J6_wrist_pitch': (-120, 120), 'J7_tool_roll': (-180, 180)}
ROLL = {'J1_yaw', 'J5_fore_roll', 'J7_tool_roll'}


def chain(la, lb):
    """The joints between two links. J1 drops out: every base part (foot, yaw motor, collar, drum) is turned
    about the yaw axis, so yawing the turret over the base changes nothing."""
    i, j = sorted((ORDER.index(la), ORDER.index(lb)))
    js = [JOINT[ORDER[k]] for k in range(i + 1, j + 1)]
    return [x for x in js if x != 'J1_yaw' and not (F.NO_J5 and x == 'J5_fore_roll')]


def grid(js, step):
    """Pitch joints every `step` deg (twice that when three or more pitch joints are in the chain), rolls every 45."""
    if sum(1 for j in js if j not in ROLL) >= 3:
        step = 2 * step
    axes = []
    for j in js:
        lo, hi = LIM[j]
        if j in ROLL:
            axes.append(list(range(-180, 180, 45)))
        else:
            vals = list(range(lo, hi + 1, int(step)))
            if vals[-1] != hi:
                vals.append(hi)
            axes.append(vals)
    for combo in itertools.product(*axes):
        yield {j: math.radians(d) for j, d in zip(js, combo)}


# --------------------------------------------------------------- the things being checked
_A = None


def setup():
    global _A
    if _A is None:
        import audit
        C = X.covers()
        cov = []
        for c in C:
            pts = X.surface_pts(c, 0.005)
            loc = [add(c['o'], add(X.mul(c['u'], p[0]), add(X.mul(c['v'], p[1]), X.mul(c['n'], p[2])))) for p in pts]
            cov.append(dict(c=c, pts=pts, zero=loc))
        _A = dict(audit=audit, cov=cov, solids=audit.SOLIDS)
    return _A


_FREE = {}


def struct_free(q):
    """True when the bare structure (no covers) is collision-free at q -- a pose the arm can reach today."""
    k = tuple(sorted((j, round(v, 4)) for j, v in q.items()))
    if k not in _FREE:
        _FREE[k] = setup()['audit'].interferes(q) is None
    return _FREE[k]


def aabb(P):
    return tuple(min(p[k] for p in P) for k in range(3)), tuple(max(p[k] for p in P) for k in range(3))


def overl(a, b, pad=0.0):
    return all(min(a[1][k], b[1][k]) - max(a[0][k], b[0][k]) > -pad for k in range(3))


def posed(Tk, link, P):
    R, t = Tk[link]
    o = F.ORIGIN[link]
    return [add(mv(R, sub(p, o)), t) for p in P]


def cover_vs_solid(args):
    ci, si, step = args
    A = setup()
    cv, s = A['cov'][ci], A['solids'][si]
    c = cv['c']
    worst = (0.0, None)
    for q in grid(chain(c['link'], s.link), step):
        Tk = F.fk(q)
        cp = posed(Tk, c['link'], cv['zero'])
        sp = A['audit'].posed_surf(Tk, s)
        if not overl(aabb(cp), aabb(sp), 0.001):
            continue
        d1 = max((s.depth(A['audit'].to_local(Tk, s.link, p)) for p in cp), default=0.0)
        d2 = max((X.solid_depth(c, X.local_of(Tk, c, p)) for p in sp), default=0.0)
        d = max(d1, d2)
        if d > worst[0] and (d <= DEEP or struct_free(q)):
            worst = (d, {k: round(math.degrees(v)) for k, v in q.items()})
    return ('cover-structure', c['name'], s.name, worst)


def cover_vs_cover(args):
    ai, bi, step = args
    A = setup()
    ca, cb = A['cov'][ai], A['cov'][bi]
    worst = (0.0, None)
    for q in grid(chain(ca['c']['link'], cb['c']['link']), step):
        Tk = F.fk(q)
        pa = posed(Tk, ca['c']['link'], ca['zero'])
        pb = posed(Tk, cb['c']['link'], cb['zero'])
        if not overl(aabb(pa), aabb(pb), 0.001):
            continue
        d1 = max((X.solid_depth(cb['c'], X.local_of(Tk, cb['c'], p)) for p in pa), default=0.0)
        d2 = max((X.solid_depth(ca['c'], X.local_of(Tk, ca['c'], p)) for p in pb), default=0.0)
        d = max(d1, d2)
        if d > worst[0] and (d <= DEEP or struct_free(q)):
            worst = (d, {k: round(math.degrees(v)) for k, v in q.items()})
    return ('cover-cover', ca['c']['name'], cb['c']['name'], worst)


def own_link():
    """Zero pose: own-link structure must not enter cover material; cover points must not be inside it."""
    A = setup()
    Tk = F.fk({})
    out = []
    for cv in A['cov']:
        c = cv['c']
        for s in A['solids']:
            if s.link != c['link']:
                continue
            if not overl(aabb(cv['zero']), (s.lo, s.hi), 0.001):
                continue
            d1 = max((s.depth(p) for p in cv['zero']), default=0.0)
            d2 = max((X.solid_depth(c, X.local_of(Tk, c, p), hollow=True) for p in s.surf), default=0.0)
            if max(d1, d2) > DEEP:
                out.append((c['name'], s.name, max(d1, d2)))
    for a, b in itertools.combinations(A['cov'], 2):
        if a['c']['link'] != b['c']['link']:
            continue
        d = max(max((X.solid_depth(b['c'], X.local_of(Tk, b['c'], p), hollow=True) for p in a['zero']), default=0.0),
                max((X.solid_depth(a['c'], X.local_of(Tk, a['c'], p), hollow=True) for p in b['zero']), default=0.0))
        if d > DEEP:
            out.append((a['c']['name'], b['c']['name'], d))
    return out


def bare_gap(cname, other, q):
    """At pose q: the closest the cover's own link structure comes to the other thing's link structure (mm) --
    what the plates clear by today, where the cover now touches."""
    A = setup()
    c = [cv['c'] for cv in A['cov'] if cv['c']['name'] == cname][0]
    ol = [s.link for s in A['solids'] if s.name == other] or [cv['c']['link'] for cv in A['cov'] if cv['c']['name'] == other]
    Tk = F.fk({k: math.radians(v) for k, v in q.items()})
    au = A['audit']
    pa = [p for s in A['solids'] if s.link == c['link'] for p in au.posed_surf(Tk, s)[::2]]
    pb = [p for s in A['solids'] if s.link == ol[0] for p in au.posed_surf(Tk, s)[::2]]
    cap = 0.05
    cell = {}
    for p in pb:                       # hash grid: only neighbouring cells are compared
        cell.setdefault(tuple(int(math.floor(p[k] / cap)) for k in range(3)), []).append(p)
    best = cap
    for p in pa:
        i = tuple(int(math.floor(p[k] / cap)) for k in range(3))
        for d in itertools.product((-1, 0, 1), repeat=3):
            for r in cell.get((i[0] + d[0], i[1] + d[1], i[2] + d[2]), ()):
                e = math.dist(p, r)
                if e < best:
                    best = e
    return '%.1f mm' % (best * 1e3)


def cable_pts(Tk, step=0.003):
    pts = F.cable_points(Tk)
    out = []
    for a, b in zip(pts, pts[1:]):
        n = max(1, int(X.norm(sub(b, a)) / step))
        out += [add(a, X.mul(sub(b, a), k / n)) for k in range(n)]
    return out


def harness(step):
    A = setup()
    worst = {}
    qs = [{}]
    for j, (lo, hi) in LIM.items():
        if F.NO_J5 and j == 'J5_fore_roll':
            continue
        qs += [{j: math.radians(d)} for d in range(lo, hi + 1, 5)]
    qs += [q for _, q in A['audit'].poses()]
    for q in qs:
        Tk = F.fk(q)
        P = cable_pts(Tk)
        for cv in A['cov']:
            c = cv['c']
            for p in P:
                l = X.local_of(Tk, c, p)
                # the cable is a 3 mm tube: test its centre against the material grown by its radius
                d = X.solid_depth(c, l, hollow=True)
                d = max(d, max(X.solid_depth(c, (l[0] + dx, l[1] + dy, l[2] + dz), hollow=True)
                               for dx, dy, dz in ((F.R_CABLE, 0, 0), (-F.R_CABLE, 0, 0), (0, F.R_CABLE, 0), (0, -F.R_CABLE, 0), (0, 0, F.R_CABLE), (0, 0, -F.R_CABLE))))
                if d > worst.get(c['name'], (0.0, None))[0]:
                    worst[c['name']] = (d, {k: round(math.degrees(v)) for k, v in q.items()})
    return worst


def first_stop(j, sgn, step=1):
    """First angle (deg) from zero toward sgn * limit at which a cover clashes with anything; None if never."""
    A = setup()
    lo, hi = LIM[j]
    end = hi if sgn > 0 else lo
    for d in range(0, end + sgn, sgn * step):
        Tk = F.fk({j: math.radians(d)})
        for cv in A['cov']:
            c = cv['c']
            cp = posed(Tk, c['link'], cv['zero'])
            bc = aabb(cp)
            for s in A['solids']:
                if s.link == c['link'] or j not in chain(c['link'], s.link):
                    continue
                sp = A['audit'].posed_surf(Tk, s)
                if not overl(bc, aabb(sp), 0.001):
                    continue
                if max((s.depth(A['audit'].to_local(Tk, s.link, p)) for p in cp[::2]), default=0.0) > DEEP or \
                        max((X.solid_depth(c, X.local_of(Tk, c, p)) for p in sp[::2]), default=0.0) > DEEP:
                    return d, '%s / %s' % (c['name'], s.name)
            for cv2 in A['cov']:
                c2 = cv2['c']
                if c2['link'] == c['link'] or j not in chain(c['link'], c2['link']):
                    continue
                p2 = posed(Tk, c2['link'], cv2['zero'])
                if not overl(bc, aabb(p2), 0.001):
                    continue
                if max((X.solid_depth(c2, X.local_of(Tk, c2, p)) for p in cp[::2]), default=0.0) > DEEP:
                    return d, '%s / %s' % (c['name'], c2['name'])
    return None, ''


def _stop(args):
    return args, first_stop(*args)


def main(step=10.0):
    A = setup()
    nc, ns = len(A['cov']), len(A['solids'])
    print('lymb exterior clash check, %s arm: %d covers x %d structural solids, pitch step %g deg, rolls 45 deg'
          % (F.SIZE, nc, ns, step))
    jobs_s = [(ci, si, step) for ci in range(nc) for si in range(ns) if A['solids'][si].link != A['cov'][ci]['c']['link']]
    jobs_c = [(a, b, step) for a, b in itertools.combinations(range(nc), 2) if A['cov'][a]['c']['link'] != A['cov'][b]['c']['link']]
    js = [(j, s) for j in LIM if not (F.NO_J5 and j == 'J5_fore_roll') for s in (1, -1)]
    with mp.Pool(max(1, (os.cpu_count() or 4) - 1)) as pool:
        rs = pool.map(cover_vs_solid, jobs_s, chunksize=4)
        rc = pool.map(cover_vs_cover, jobs_c, chunksize=1)
        stops = dict(pool.map(_stop, js))
    bad = [r for r in rs + rc if r[3][0] > DEEP]
    print('\n== covers vs other links\' structure, and vs each other (%d pairs, full range; only poses the bare'
          ' structure itself reaches without a collision count) ==' % (len(jobs_s) + len(jobs_c)))
    for kind, a, b, (d, q) in sorted(bad, key=lambda r: -r[3][0]):
        print('  %-15s %-22s <-> %-28s %5.1f mm at %s   (bare structure gap there %s)' % (
            kind, a, b, d * 1e3, q, bare_gap(a, b, q)))
    if not bad:
        print('  none')
    print('\n== covers vs their own link at the zero pose (material only; the hollow may hold caps and bolts) ==')
    ol = own_link()
    for a, b, d in ol:
        print('  %-22s <-> %-28s %5.1f mm' % (a, b, d * 1e3))
    if not ol:
        print('  none')
    print('\n== covers vs the harness (every joint swept alone at 5 deg + the motion video) ==')
    hw = {k: v for k, v in harness(step).items() if v[0] > DEEP}
    for k, (d, q) in hw.items():
        print('  %-22s %5.1f mm at %s' % (k, d * 1e3, q))
    if not hw:
        print('  none')
    print('\n== joint range with covers (each joint alone, 1 deg, to the limits) ==')
    for j in LIM:
        if F.NO_J5 and j == 'J5_fore_roll':
            continue
        lo, hi = LIM[j]
        (dn, why_n), (dp, why_p) = stops[(j, -1)], stops[(j, 1)]
        print('  %-15s limits %4d .. %4d   covers stop it at %s .. %s %s' % (
            j, lo, hi, 'never' if dn is None else dn, 'never' if dp is None else dp,
            ('(' + '; '.join(w for w in (why_n, why_p) if w) + ')') if (why_n or why_p) else ''))
    return bad, ol, hw, stops


if __name__ == '__main__':
    main(float(sys.argv[1]) if len(sys.argv) > 1 else 10.0)
