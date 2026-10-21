"""mini harness check, with the loon cortex head (k1_structure.cx_parts): every cable run clears every part except its
own ends, at rest and across the head's yaw and pitch, and the two neck service loops are long enough.

    python harness_check.py      -> table + exit 1 on a clash
"""
import numpy as np, trimesh
from manifold3d import Manifold, OpType
import k1_design as KD, k1_structure as KS, rom_mini as R

TOL = 2.0                                       # mm^3 of overlap = touching, not a clash
# a run's ends: the parts it plugs into (prefixes). Everything else must clear it.
ENDS = {
    'cable-hcortex': ('cortex-cams', 'cortex-display', 'cortex-compute', 'cortex-dockboard', 'cortex-power',
                      'cortex-battery', 'cortex-speakers', 'cortex-amps', 'cortex-mics', 'cortex-sled'),
    'cable-hspine': ('dock-pogo-block',),
    'cable-nspine': (),
    'cable-tspine': ('elec-limbic', 'torso-side.r'),          # through the side plate's 10 mm pass-through
}
LOOP_SLACK = 1.3                                # a service loop: 30% over the worst straight-line gap


SKIN = ('cover-', 'trim-', 'puck-', 'accent-')   # shells are modelled solid: a run through one gets its hole cut by fab/make.py
THROUGH = {}


def clashes(cab, parts, ends):
    out = []
    for n, m in parts:
        if n == cab[0] or n.startswith('cable-') or n.startswith(ends):
            continue
        if n.startswith(SKIN):
            if (cab[1] ^ m).volume() > TOL:
                THROUGH.setdefault(cab[0], set()).add(n)
            continue
        v = (cab[1] ^ m).volume()
        if v > TOL:
            out.append((n, v))
    return out


def main():
    J, _ = KS.world()
    D = KD.design()
    P = D + KD.structure(D)
    man = [(n, R.toM(m)) for n, m in P]
    cabs = [(n, M) for n, M in man if n.startswith('cable-')]
    ok = True
    print('at rest')
    for cab in cabs:
        c = clashes(cab, man, ENDS.get(cab[0], ()))
        ok &= not c
        print(f'  {cab[0]:15s} {"clear" if not c else "CLASH " + ", ".join(f"{n} {v:.0f} mm3" for n, v in c)}')
    # the spine across the head's range: the head-side run rides head pitch (and yaw), the neck run rides yaw
    jt = {j[0]: j for j in R.joints(J)}
    body = {n: R.body_of(n) for n, _ in P}
    rot = lambda M, jn, deg: M.transform(trimesh.transformations.rotation_matrix(np.radians(deg), jt[jn][3], jt[jn][2])[:3])
    pts = {n: m for n, m in P if n in ('cable-hspine', 'cable-nspine')}
    lo, hi = R.NEED['Head_pitch'], R.NEED['Head_yaw']
    hs = dict(cabs)['cable-hspine']
    worst = {}
    print(f'spine over head pitch {lo[0]}..{lo[1]} and yaw {hi[0]}..{hi[1]}')
    for jn, rng, moving in (('Head_pitch', lo, ('hpitch',)), ('Head_yaw', hi, ('hpitch', 'hyaw'))):
        stat = [(n, M) for n, M in man if body[n] not in moving and not n.startswith('cable-')]
        for deg in np.linspace(rng[0], rng[1], 13):
            for cn in ('cable-hspine', 'cable-nspine'):
                if body[cn] not in moving:
                    continue
                c = clashes((cn, rot(dict(cabs)[cn], jn, deg)), stat, ENDS[cn])
                if c:
                    ok = False
                    print(f'  {jn} {deg:+.0f}: {cn} CLASH ' + ', '.join(f'{n} {v:.0f} mm3' for n, v in c))
        # the loop: the moving run's free end vs the static run's start
        a_end = lambda m: m.vertices[np.argmin(m.vertices[:, 2])]        # hspine's lowest point (bottom of its drop)
        b_start = lambda m: m.vertices[np.argmax(m.vertices[:, 2])]      # nspine's top
        if jn == 'Head_pitch':
            a, b = a_end(pts['cable-hspine']), b_start(pts['cable-nspine'])
        else:                                                          # the yaw loop: nspine's foot vs tspine's top
            a = pts['cable-nspine'].vertices[np.argmin(pts['cable-nspine'].vertices[:, 2])]
            b = [m for n, m in P if n == 'cable-tspine'][0]
            b = b.vertices[np.argmax(b.vertices[:, 2])]
        T = lambda deg: trimesh.transformations.rotation_matrix(np.radians(deg), jt[jn][3], jt[jn][2])
        worst[jn] = max(np.linalg.norm((T(d) @ np.append(a, 1))[:3] - b) for d in np.linspace(rng[0], rng[1], 61))
    for c, ns in THROUGH.items():
        print(f'  {c} runs inside / through {", ".join(sorted(ns))}: fab/make.py cuts the hole in the printed skin')
    for jn, d in worst.items():
        print(f'  {jn} service loop: worst gap {d:.0f} mm -> cut the loop {d * LOOP_SLACK:.0f} mm')
    print('harness:', 'PASS' if ok else 'FAIL')
    return ok


if __name__ == '__main__':
    raise SystemExit(0 if main() else 1)
