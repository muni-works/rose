"""Is rose pro safe to stand next to? Contact-geometry audit against ISO/TS 15066.

rose pro is meant to cobot -- to share a room and a task with a person, not to work
behind a fence. That turns a styling question into an engineering one with published
numbers, and this is the gate.

THE RULE THIS ENFORCES. ISO/TS 15066's whole biomechanical framework -- the per-body-
region force limits below -- is stated to apply to BLUNT contact geometries only, and
the standard says sharp contact "should be avoided in all cases". A 3 mm laser-cut
aluminium edge is not blunt. So an exposed square-cut plate edge is not a finding that
gets traded off against a force number; it is outside the envelope the numbers live in.
This audit therefore fails on GEOMETRY, and reports the force limits only to rank which
exposed edges matter most -- the ones at the height of a face are worse than the ones at
the height of a shin.

WHAT IT DOES NOT DO. It says nothing about whether a strike would exceed those forces:
that is a mass, speed and control question (power-and-force limiting), not a shape
question, and it belongs with the controller. Nor does it check pressure limits -- the
standard's per-region pressure table is not reproduced here. Passing this audit is
necessary, not sufficient.

Run: python3 cobot.py
"""
import math

import rose_pro_links as L

# ISO/TS 15066:2016 Annex A quasi-static contact force limits, newtons, by body
# region. Lower is more fragile. Used here for RANKING exposed edges by who they
# would meet, not as a pass/fail threshold.
#
# Heights are where that region sits on a standing adult of ~1700 mm, since the
# question this answers is "what is at the same height as this piece of metal".
REGIONS = [
    # name                    z_lo   z_hi   force N
    ('lower leg',                0,   420,   130),
    ('thigh / knee',           420,   750,   220),
    ('hand / finger',          750,   870,   140),   # arms hanging at rest
    ('pelvis',                 870,  1030,   180),
    ('abdomen',               1030,  1180,   110),   # most fragile of the torso
    ('chest',                 1180,  1370,   140),
    ('forearm / wrist',       1370,  1420,   160),
    ('back / shoulder',       1420,  1530,   210),
    ('neck',                  1530,  1580,   150),
    ('face',                  1580,  1660,    65),   # most fragile anywhere
    ('skull / forehead',      1660,  1800,   130),
]

SOLE_Z = -670.0        # trunk-origin frame: the sole plane, from the layout
SHIELD = 6.0           # an edge with structure this close is buried, not exposed
T = L.T                # 3 mm plate


def region_at(height_mm):
    for name, lo, hi, f in REGIONS:
        if lo <= height_mm < hi:
            return name, f
    return ('above head', 999) if height_mm >= 1800 else ('below floor', 999)


def edges(pl):
    """The four perimeter edges of a plate, as (mid_u, mid_v, length, label).

    In-plane only. The plate is 3 mm thick, so each perimeter edge is a 3 mm-wide
    square-cut strip -- the thing a person actually contacts.
    """
    return [
        ((pl.u0 + pl.u1) / 2, pl.v0, pl.u1 - pl.u0, 'v0'),
        ((pl.u0 + pl.u1) / 2, pl.v1, pl.u1 - pl.u0, 'v1'),
        (pl.u0, (pl.v0 + pl.v1) / 2, pl.v1 - pl.v0, 'u0'),
        (pl.u1, (pl.v0 + pl.v1) / 2, pl.v1 - pl.v0, 'u1'),
    ]


def to_world(pl, u, v):
    """Map an in-plane (u, v) on this plate back to world x, y, z."""
    i = L.AXI[pl.normal]
    ui, vi = [k for k in range(3) if k != i]
    p = [0.0, 0.0, 0.0]
    p[i] = (pl.n0 + pl.n1) / 2.0
    p[ui] = u
    p[vi] = v
    return p


def shielded(pt, me):
    """Is this point buried against other structure rather than out in the open?

    APPROXIMATION, and the one place this audit could lie: an edge is called
    shielded if any OTHER plate's box or any actuator body comes within SHIELD mm
    of it. That is a proximity test, not a visibility test, so it will call an edge
    shielded when the neighbour is merely near rather than actually covering it.
    It errs toward UNDER-reporting exposed edges, so treat the count as a floor.
    """
    for other in L.PLATES:
        if other is me:
            continue
        b = other.box()
        if all(b[k][0] - SHIELD <= pt[k] <= b[k][1] + SHIELD for k in range(3)):
            return True
    for (_n, ax, (cu, cv), r, (lo, hi)) in L.obstacles():
        i = L.AXI[ax]
        ui, vi = [k for k in range(3) if k != i]
        if lo - SHIELD <= pt[i] <= hi + SHIELD:
            if math.hypot(pt[ui] - cu, pt[vi] - cv) <= r + SHIELD:
                return True
    return False


def main():
    for pl in L.PLATES:
        pl.fit()

    tally = {}
    total_len = 0.0
    for pl in L.PLATES:
        for (u, v, ln, lab) in edges(pl):
            pt = to_world(pl, u, v)
            if shielded(pt, pl):
                continue
            height = pt[2] - SOLE_Z
            name, force = region_at(height)
            e = tally.setdefault(name, {'force': force, 'len': 0.0, 'n': 0,
                                        'parts': set()})
            e['len'] += ln
            e['n'] += 1
            e['parts'].add('%s/%s' % (pl.link, pl.tag))
            total_len += ln

    print('=' * 76)
    print('rose pro - exposed contact geometry vs ISO/TS 15066')
    print('=' * 76)
    print('  Every exposed edge below is a %.0f mm square-cut laser edge.' % T)
    print('  ISO/TS 15066 applies to BLUNT contact only, so each one sits outside')
    print('  the envelope its force limits are defined in.\n')
    print('  %-20s %7s %7s %9s   %s' % ('at the height of', 'limit', 'edges',
                                        'length', 'worst offenders'))
    order = sorted(tally.items(), key=lambda kv: kv[1]['force'])
    for name, e in order:
        parts = ', '.join(sorted(e['parts'])[:3])
        print('  %-20s %5.0f N %7d %8.0f mm   %s'
              % (name, e['force'], e['n'], e['len'], parts))

    print('\n  TOTAL exposed square-cut edge: %.0f mm across %d edges'
          % (total_len, sum(e['n'] for e in tally.values())))

    fragile = [n for n, e in order if e['force'] <= 140]
    print('\n  VERDICT: FAIL - no edge on this robot is hemmed.')
    print('  Most urgent by body region (limit <= 140 N): %s' % ', '.join(fragile))
    print("""
  THE FIX, and it is a fold not a cover. A hem -- the edge folded 180 back on
  itself -- turns a 3 mm knife into a ~6 mm doubled bead with a real radius, adds
  stiffness rather than mass, and costs one brake hit per edge. It needs the plate
  generator to emit sheet-metal bend nodes (vcad has sheet-hem) instead of solid
  boxes, which is a real rework: sheet nodes are a bend graph and cannot be
  unioned, transformed or filleted the way the current solids are.

  Corner radii are already handled the other way -- rose_pro_loon.corner_radius
  asks for 25 mm and backs off only where a bolt circle refuses, because corners
  strike first and pressure is force over area.""")


if __name__ == '__main__':
    main()
