import sys, re
sys.path.insert(0, sys.argv[1])
import quad as Q
g = 9.81
def knee(H, pk, endm, n_end, leg, batt, elec):
    sc = Q.build(H); al = bk = 0
    for node in sc.graph.nodes_geometry:
        T, gg = sc.graph[node]; m = sc.geometry[gg]
        if m.extents.max() < 1: continue
        base = node.split('.')[0].split('~')[0]
        if not base.startswith(('rs0', 'int-')): al += m.volume * 2.7e-6
        if base.startswith(('shin', 'foot', 'crank', 'link', 'pivot')) and node.endswith(('.ll', '.lr')): bk += m.volume * 2.7e-6
    kg_m = float(re.search(r'motors ([\d.]+) kg', H['note']).group(1))
    bk += 2 * n_end * Q.MOTORS[endm]['kg']
    mass = kg_m + al + batt + elec
    ll = 0.97 * sum(leg) + 46 * Q.MOTORS[pk]['D'] / 57
    walk = 1.5 * (mass - bk) * g * 0.22 * ll / 1000
    P = Q.MOTORS[pk]
    return mass, al, walk, walk / (0.7 * P['peak']), 0.5 * (mass - bk) * g * 0.15 * ll / 1000 / P['cont'], round(sc.bounds[1][2] - sc.bounds[0][2])
if __name__ == '__main__':
    for s in [float(x) for x in sys.argv[2:]]:
        leg = (170 * s, 220 * s)
        H = Q.humanoid_family('t', leg=leg, arm=(150 * s, 170 * s), chest_h=86 * s)
        mass, al, walk, f, st, h = knee(H, 'rs00', 'rs00', 1, leg, 0.85 * s ** 2, 0.6)
        print(f's {s:.2f}: height {h} mm, mass {mass:.1f} kg (al {al:.1f}), walking knee {walk:.1f} N.m = {f:.0%}, standing {st:.0%}')
