import sys, collections, numpy as np
sys.argv = [sys.argv[0], sys.argv[1]]
exec(open(__file__.replace('sweep.py', 'rom.py')).read().split("if __name__")[0])
def sweep(kind, ths, rels, abs_):
    agg = collections.defaultdict(list)
    for th in ths:
        for rel in rels:
            for ab in abs_:
                for a, b, v in hit(limb(kind, 1, th, rel, ab), CH):
                    if (a, b) not in own: agg[(a, b)].append((th, rel, ab))
    for k, v in agg.items():
        t = sorted(set(x[0] for x in v)); r = sorted(set(x[1] for x in v)); a = sorted(set(x[2] for x in v))
        print(f'  {kind} {k}: n={len(v)} th {t} rel {r} ab {a}')
print('LEG vs chassis'); sweep('leg', range(-120, 121, 20), (0, 40, 80, 120, 150), (-30, 0, 30))
print('ARM vs chassis'); sweep('arm', range(-180, 181, 30), (0, 40, 80, 120, 150), (-30, 0, 30))
st = Q.stance(0.96 * 300)
print('LEG vs LEG (roll in)')
for ab in (-30, -15, 0, 15, 30):
    print('  ab', ab, hit(limb('leg', 1, *st, ab), limb('leg', -1, *st, 0)))
print('ARM vs LEG')
for th in range(-60, 181, 30):
    for rel in (0, 70, 140):
        h = hit(limb('arm', 1, th, rel, 0), limb('leg', 1, *st, 0))
        if h: print('  arm', th, rel, h)
