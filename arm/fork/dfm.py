"""DFM and efficiency audit for the fork arm.

DFM, per part:
  process     router (flat 5052 on the 4040), turned (4th axis / lathe), printed, purchased
  bed         flat parts must fit the 4040 bed with a 5 mm margin
  webs        narrowest material between a hole / window and the outline, or between two holes
              (router rule of thumb for 5052: >= 1.5 x thickness, and >= 3 mm absolute)
  tool        every cut-out must admit a 3.175 mm end mill: holes >= 3.2 mm, window radii >= 1.6 mm
  unique      identical parts (same outline, holes, thickness) counted once

Efficiency, per link:
  mass share of structure vs motors, and the stress / stiffness margins the structure actually has.
"""
import collections
import hashlib
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import rose_fork as F
import audit as Au

BED = (0.390, 0.390)
TOOL = 0.003175
PROCESS = {'sheet': 'router', 'cover': 'router', 'base': 'router', 'bar': 'purchased',
           'actuator': 'purchased', 'sleeve': 'turned', 'collar': 'turned', 'collar_turn': 'turned',
           'cap': 'spun / printed', 'printed': 'printed'}


def classify(prim):
    link, mat, o, u, v, ol, vec = prim[:7]
    t = Au.norm(vec)
    # a "cylinder" whose length exceeds a plate thickness is not flat stock
    is_circle = len(ol) == 2 and ol[0][0] == 'arc'
    proc = PROCESS.get(mat, 'router')
    if proc == 'router' and t > 0.0065:
        proc = 'turned' if is_circle else 'router (thick)'
    return proc, t, is_circle


def key(prim):
    link, mat, o, u, v, ol, vec = prim[:7]
    P = Au.poly(ol, 0.004)
    xs, ys = [p[0] for p in P], [p[1] for p in P]
    w, h = max(xs) - min(xs), max(ys) - min(ys)
    holes = sorted(round(math.hypot(*Au.poly(hh, 0.004)[0]) * 1e3) for hh in (prim[7] if len(prim) > 7 else []))
    return '%s|%.1f|%.0fx%.0f|%s' % (mat, Au.norm(vec) * 1e3, w * 1e3, h * 1e3, holes)


def min_web(prim):
    """Narrowest web: outline to each window, and window to window (sampled)."""
    link, mat, o, u, v, ol, vec = prim[:7]
    P = Au.poly(ol, 0.002)
    H = [Au.poly(h, 0.002) for h in (prim[7] if len(prim) > 7 else [])]
    best = 1e9
    for h in H:
        for (x, y) in h[::2]:
            best = min(best, Au.edge_dist(x, y, P))
    for a in range(len(H)):
        for b in range(a + 1, len(H)):
            for (x, y) in H[a][::3]:
                best = min(best, min(math.hypot(x - p[0], y - p[1]) for p in H[b][::3]))
    return best if best < 1e8 else None


def main():
    prims = F.build()
    rows = []
    for i, p in enumerate(prims):
        proc, t, circ = classify(p)
        P = Au.poly(p[5], 0.004)
        xs, ys = [q[0] for q in P], [q[1] for q in P]
        w, h = max(xs) - min(xs), max(ys) - min(ys)
        m = F.prim_mass(p)
        rows.append(dict(i=i, link=p[0], mat=p[1], proc=proc, t=t, w=w, h=h, mass=m, web=min_web(p) if proc.startswith('router') else None,
                         key=key(p), circ=circ))
    print('== parts by process ==')
    byp = collections.defaultdict(list)
    for r in rows:
        byp[r['proc']].append(r)
    for proc, rs in byp.items():
        uniq = len({r['key'] for r in rs})
        print('  %-16s %2d parts, %2d unique' % (proc, len(rs), uniq))
    print('\n== router parts: DFM rules ==')
    issues = 0
    for r in rows:
        if not r['proc'].startswith('router'):
            continue
        notes = []
        fits = (r['w'] <= BED[0] - 0.01 and r['h'] <= BED[1] - 0.01) or (r['h'] <= BED[0] - 0.01 and r['w'] <= BED[1] - 0.01)
        if not fits:
            notes.append('over the bed')
        if r['proc'] == 'router (thick)':
            notes.append('%.0f mm thick - not 3 mm stock' % (r['t'] * 1e3))
        if r['web'] is not None and r['web'] < max(0.003, 1.5 * r['t']):
            notes.append('web %.1f mm < %.1f' % (r['web'] * 1e3, max(3.0, 1.5e3 * r['t'])))
        if notes:
            issues += 1
        print('  #%-3d %-7s %-7s %4.1f mm  %4.0f x %-4.0f  %5.0f g  web %-6s %s' % (
            r['i'], r['link'], r['mat'], r['t'] * 1e3, r['w'] * 1e3, r['h'] * 1e3, (r['mass'] or 0) * 1e3,
            '%.1f' % (r['web'] * 1e3) if r['web'] else '-', ', '.join(notes) or 'ok'))
    print('  %d router parts with issues' % issues)
    print('\n== non-router parts ==')
    for r in rows:
        if r['proc'].startswith('router') or r['proc'] == 'purchased':
            continue
        print('  #%-3d %-7s %-11s %-8s %4.0f x %-4.0f  %5.0f g' % (r['i'], r['link'], r['mat'], r['proc'], r['w'] * 1e3, r['t'] * 1e3, (r['mass'] or 0) * 1e3))
    print('\n== where the structure mass is (moving links first) ==')
    tot = collections.defaultdict(float)
    for r in rows:
        if r['mass']:
            tot[(r['link'], r['mat'], r['proc'])] += r['mass']
    for (link, mat, proc), m in sorted(tot.items(), key=lambda kv: -kv[1]):
        print('  %-7s %-11s %-15s %5.0f g' % (link, mat, proc, m * 1e3))


if __name__ == '__main__':
    main()
