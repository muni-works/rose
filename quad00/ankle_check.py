"""ankle linkage checks: reach, transmission and collisions over the ankle's range, for the mini's two-rod
ankle (pitch + roll) and the tiny's single rod (pitch). Ranges: Booster K1's ankle -- 50 plantar (toes down, + here) / 20 dorsi, roll +/-20.
Usage: python3 ankle_check.py"""
import itertools, math, numpy as np
import quad as Q
from manifold3d import Manifold, Mesh

toM = lambda m: Manifold(Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32)))
PITCH, ROLL = range(-20, 51, 5), range(-20, 21, 5)


def designed(a, b, mot):
    """contacts that are joints, not collisions: a part and what it bolts to or pivots in."""
    pair = {a, b}
    if 'shin' in pair and any(x.startswith(('rs0', 'pivot')) for x in pair): return True
    if any(x.startswith('crank') for x in pair) and any(x.startswith(('rs0', 'link')) for x in pair): return True
    if 'foot' in pair and any(x.startswith(('link', 'pivot')) for x in pair): return True
    return False


def collide(parts, mot):
    P = {n: toM(g) for n, g in parts.items()}
    return [(a, b) for a, b in itertools.combinations(P, 2) if not designed(a, b, mot) and (P[a] ^ P[b]).volume() > 5]


def mini():
    e = Q.MOTORS['rs06']
    DM = max(80.0, ((Q.MOTORS['rs04']['D'] + 7) / 2 + e['D'] / 2 + 4) / math.sin(math.radians(127)))
    G = Q.ankle2_geom(300.0, DM, 0.8 * e['D'], e)
    worst, fails, al = 1.0, [], []
    for p, r in itertools.product(PITCH, ROLL):
        s = Q.ankle2_ik(G, p, r)
        if any(x is None for x in s): fails.append((p, r)); continue
        worst = min(worst, *(x[1] for x in s)); al += [x[0] for x in s]
    print(f'mini (2 x RS06, motors {G["DM1"]:.0f} / {G["DM2"]:.0f} below the knee, crank {G["CR"]:.0f}, rod spacing {G["rods"][0][1] - G["rods"][1][1]:.0f}):')
    print(f'  reach: {"all" if not fails else f"FAILS at {fails}"} of pitch -20..50 x roll +/-20; crank travel {min(al):.0f}..{max(al):.0f} deg; worst transmission {worst:.2f} (1 = ideal)')
    F = e['peak'] / (G['CR'] / 1000)   # rod force at the motor's peak, crank square
    sep = (G['rods'][0][1] - G['rods'][1][1]) / 1000
    print(f'  peak torque at neutral: pitch {2 * e["peak"]:.0f} N.m (both rods), roll {F * sep / 2 * 2:.0f} N.m')
    bad = {}
    for rel in (0, 60, 127):
        for p, r in itertools.product((-20, -10, 0, 20, 35, 50), (-20, -10, 0, 10, 20)):
            parts = Q.inline_parts(30, rel, 'ankle', 230, 300, mot='rs04', ab_mot='rs06', end_mot='rs06', dual=True, ank=(p - (rel - 30), r))
            c = collide(parts, 'rs06')
            if c: bad[(rel, p, r)] = c
    print('  collisions (knee x ankle pitch x roll):', bad or 'none')


def tiny():
    worst = min(abs(math.cos(math.radians(p))) for p in PITCH)
    print(f'tiny (1 x RS00 per ankle, parallelogram): reach all of pitch -20..50; worst transmission {worst:.2f}; no roll (by design)')
    bad = {}
    for rel in (0, 40, 80, 127):
        for p in (-20, 0, 25, 50):
            parts = Q.inline_parts(30, rel, 'ankle', 122, 158, ank=(p - (rel - 30), 0))
            c = collide(parts, 'rs00')
            if c: bad[(rel, p)] = c
    print('  collisions (knee x ankle pitch):', bad or 'none')
    bad = {}
    for rel in (0, 60, 120):
        for p in (-60, 0, 60):
            c = collide(Q.inline_parts(90, rel, 'wrist', 108, 122, ank=(p, 0)), 'rs00')
            if c: bad[(rel, p)] = c
    print('  wrist collisions (elbow x wrist):', bad or 'none')


if __name__ == '__main__':
    mini()
    tiny()
