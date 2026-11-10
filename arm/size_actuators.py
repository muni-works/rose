"""Size the fork arm's actuators so it holds its payload with NO springs, from the vendor's own
stall ratings. Holding a pose is a stall; the manuals give the stall-continuous torque.

Motors (RobStride manuals, ~/Developer/robstride_assets, 2026-10-13):
              peak  rated(rot)  stall-cont  heat sink for the rating     kg     D x L (mm)
  RS00         14      5          3.6*      90 x 85                      0.310  61 x 66
  RS06         36     11          8         130 x 155                    0.621  88 x 49
  RS03         60     20         13         215 x 220                    0.880  106 x 54
  RS04        120     40         25**       345 x 345 (35 rated on 220 x 200)  1.420  120 x 52
  * RS00 stall not tabulated here: scaled from RS06 (8/11 of rated).
  ** RS04 stall 28.5 on 345 x 345; derated by its own 35/40 for a ~220 x 200 plate, which is
     what the fork's parent plates are.

The worst holding pose is straight out at full reach. Each joint must hold its own outboard
weight plus the payload. Heavier motors raise the load on the joints behind them, so the
whole stack is evaluated together for every combination.
"""
import itertools
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import fork_checks as C
import rose_fork as F

G = 9.81
M = {  # peak, stall_cont, kg, D, L
    'RS00': (14.0, 3.6, 0.310, 0.061, 0.066),
    'RS06': (36.0, 8.0, 0.621, 0.088, 0.049),
    'RS03': (60.0, 13.0, 0.880, 0.106, 0.054),     # NOT AVAILABLE (2026-10-13) - kept for reference only
    'RS02': (17.0, 4.4, 0.380, 0.0785, 0.0455),   # stall-cont scaled from rated 6 by RS06's 8/11 - ESTIMATE
    'RS04': (120.0, 25.0, 1.420, 0.120, 0.052),
}
BASE = dict(J2='RS06', J4='RS06', J5='RS00', J6='RS00', J7='RS00')   # v3
STRUCT_K = 0.35      # extra plate mass per kg of motor upgrade (bigger lobes, wider forks)


def moments(sel, payload):
    """Holding moments straight out for a motor selection, by mass deltas against v3."""
    xF = C.flange_x()
    x = dict(J4=F.UPPER, J5=F.W - 0.070 - 0.033, J6=F.W, J7=F.W + 0.040 + 0.040)
    extra = {}
    for j in ('J4', 'J5', 'J6', 'J7'):
        dm = M[sel[j]][2] - M[BASE[j]][2]
        extra[j] = dm * (1 + STRUCT_K)
    def mom(x0):
        base = C.moment(x0, payload)
        return base + G * sum(extra[j] * (x[j] - x0) for j in extra if x[j] > x0)
    return dict(J2=mom(0.0), J4=mom(F.UPPER), J6=mom(F.W)), extra


def hold_payload(sel, joint, x0):
    """Largest payload this joint can hold continuously straight out."""
    lo, hi = 0.0, 20.0
    for _ in range(40):
        mid = (lo + hi) / 2
        if moments(sel, mid)[0][joint] <= M[sel[joint]][1]:
            lo = mid
        else:
            hi = mid
    return lo


def evaluate(sel):
    xs = dict(J2=0.0, J4=F.UPPER, J6=F.W)
    cont = {j: hold_payload(sel, j, xs[j]) for j in xs}
    peak = {}
    for j in xs:
        lo, hi = 0.0, 50.0
        for _ in range(40):
            mid = (lo + hi) / 2
            if moments(sel, mid)[0][j] <= M[sel[j]][0]:
                lo = mid
            else:
                hi = mid
        peak[j] = lo
    moving = sum(M[sel[j]][2] for j in ('J4', 'J5', 'J6', 'J7'))
    total = sum(M[sel[j]][2] for j in sel) + M['RS06'][2]          # + J1 base yaw
    return min(cont.values()), min(peak.values()), cont, peak, total, moving


def main():
    print('v3 (RS06 / RS06 / RS00 x3), no spring:')
    c, p, cont, peak, tot, mov = evaluate(BASE)
    print('  continuous %.2f kg (%s)   peak %.2f kg' % (c, ', '.join('%s %.2f' % kv for kv in cont.items()), p))
    rows = []
    for J2, J4, J6 in itertools.product(('RS06', 'RS03', 'RS04'), ('RS06', 'RS03', 'RS04'), ('RS00', 'RS06')):
        for J57 in ('RS00', 'RS06'):
            sel = dict(J2=J2, J4=J4, J5=J57, J6=J6, J7='RS00')
            c, p, cont, peak, tot, mov = evaluate(sel)
            rows.append((c, p, tot, sel, cont))
    print('\nlightest selection for each continuous-hold target (straight out, 783 mm, no springs):')
    print('  %-8s %-34s %8s %8s %8s   %s' % ('target', 'J2 / J4 / J5 / J6 / J7', 'cont', 'peak', 'motors', 'binding'))
    for target in (0.5, 1.0, 1.5, 2.0, 2.5):
        ok = [r for r in rows if r[0] >= target]
        if not ok:
            print('  %4.1f kg  none' % target)
            continue
        c, p, tot, sel, cont = min(ok, key=lambda r: (r[2], -r[0]))
        bind = min(cont, key=cont.get)
        print('  %4.1f kg  %-34s %6.2f kg %6.2f kg %6.2f kg   %s' % (
            target, ' / '.join(sel[j] for j in ('J2', 'J4', 'J5', 'J6', 'J7')), c, p, tot, bind))


AVAILABLE = ('RS00', 'RS02', 'RS04', 'RS06')     # RS01 / RS05 duplicate RS02 / RS00 at lower spec


def search():
    """Every selection from the available motors, at several reaches. Score: continuous straight-out
    hold, then fewest distinct models (simpler BOM, fewer interfaces), then lightest."""
    import itertools
    out = []
    for up, fo in ((0.300, 0.260), (0.280, 0.240), (0.260, 0.220), (0.240, 0.200)):
        F.UPPER, F.FORE, F.W = up, fo, up + fo
        reach = C.flange_x()
        for J2 in ('RS04', 'RS06'):
            for J4 in ('RS04', 'RS06', 'RS02'):
                for J6 in ('RS00', 'RS02', 'RS06'):
                    for J57 in ('RS00', 'RS02', 'RS06'):
                        for J1 in AVAILABLE:
                            sel = dict(J2=J2, J4=J4, J5=J57, J6=J6, J7=J57)
                            c, p, cont, peak, tot, mov = evaluate(sel)
                            skus = len(set(list(sel.values()) + [J1]))
                            tot = tot - M['RS06'][2] + M[J1][2]
                            out.append(dict(reach=reach, up=up, fo=fo, sel=sel, J1=J1, cont=c, peak=p, skus=skus, kg=tot,
                                            bind=min(cont, key=cont.get)))
    return out


if __name__ == '__main__':
    import sys as _s
    if len(_s.argv) > 1 and _s.argv[1] == '--available':
        rows = search()
        print('best continuous straight-out hold for each (reach, number of motor models):')
        print('  %-6s %-5s %-44s %7s %7s %7s  %s' % ('reach', 'SKUs', 'J1 base / J2 / J4 / J5 / J6 / J7', 'cont', 'peak', 'motors', 'binds'))
        for reach in sorted({r['reach'] for r in rows}, reverse=True):
            for k in (1, 2, 3):
                cand = [r for r in rows if abs(r['reach'] - reach) < 1e-6 and r['skus'] == k]
                if not cand:
                    continue
                b = max(cand, key=lambda r: (round(r['cont'], 2), -r['kg']))
                print('  %4.0f   %d    %-44s %5.2f kg %5.2f kg %5.2f kg  %s' % (
                    reach * 1e3, k, ' / '.join([b['J1']] + [b['sel'][j] for j in ('J2', 'J4', 'J5', 'J6', 'J7')]),
                    b['cont'], b['peak'], b['kg'], b['bind']))
    else:
        main()
