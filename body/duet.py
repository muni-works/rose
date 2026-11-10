# Run from body/: python duet.py OUTDIR '{"l_shoulder_pitch": -40, ...}'  (then vcad export OUTDIR/duet.loon, vcad-render)
"""rose mini's two arms as a bench duet: the arm chains posed by their own joint axes/pivots, on a crossbar + post over a table."""
import sys, os, json
sys.path.insert(0, os.getcwd())
import numpy as np, trimesh
import vcad_export as VE, k1_structure as KS
OUT = sys.argv[1]; POSE = json.loads(sys.argv[2])
KS.use('mini'); KD = VE._patch(); J, M = KS.world(); D = KD.design(); C = KD.structure(D)
P = VE.pivots(J); JL = {ch: (jn, par, np.asarray(a, float)) for jn, par, ch, a, lo, hi in VE.joints(J)}
def chain(b):
    out = []
    while b in JL:
        jn, par, a = JL[b]; out.append((jn, b, a)); b = par
    return out, b
def pose_T(b):
    T = np.eye(4)
    for jn, ch, a in chain(b)[0]:             # leaf first: rotate about each joint's rest pivot, then its parent's
        T = trimesh.transformations.rotation_matrix(np.radians(POSE.get(jn, 0.0)), a, P[ch]) @ T
    return T
mats = {'cover': 'satin_black', 'trim': 'satin_black', 'plate': 'anodized', 'motor': 'motor_dark'}
L = ['[material "satin_black" 0.09 0.094 0.106 0.1 0.55]', '[material "anodized" 0.06 0.06 0.065 0.5 0.4]',
     '[material "motor_dark" 0.08 0.085 0.09 0.6 0.4]', '[material "bench" 0.62 0.6 0.56 0.0 0.65]', '[material "wood" 0.55 0.42 0.3 0.0 0.6]']
os.makedirs(OUT, exist_ok=True)
sz = []
for n, m in D + C:
    b = VE.body_of(n)
    if b is None or not any(b.startswith(k) for k in ('sp.', 'sr.', 'ep.', 'ey.')):
        continue
    mm = m.copy(); mm.apply_transform(pose_T(b)); f = n.replace('.', '_') + '.stl'; mm.export(os.path.join(OUT, f)); sz.append(mm.bounds)
    L.append('[root [import-mesh-scaled 0.001 0.001 0.001 "%s"] "%s"]' % (f, mats.get(VE.mat_of(n), 'satin_black')))
lo = np.min([b[0] for b in sz], 0); hi = np.max([b[1] for b in sz], 0)
sl, sr = P['sp.l'], P['sp.r']; zs = sl[2]; yb = abs(sl[1]) - 40.0
# the mount: a satin crossbar between the shoulder discs (the chest's job, without the chest), a post, a table
L.append('[root [translate %.1f %.1f %.1f [cube 70.0 %.1f 80.0]] "satin_black"]' % (sl[0] - 45.0, -yb, zs - 40.0, 2 * yb))
L.append('[root [translate %.1f 0.0 %.1f [cylinder 22.0 %.1f]] "satin_black"]' % (sl[0] - 10.0, zs - 300.0, 260.0))
zt = zs - 300.0                              # the work surface 300 below the shoulders (ALOHA-like)
L.append('[root [translate %.1f %.1f %.1f [cube 700.0 900.0 30.0]] "wood"]' % (sl[0] - 260.0, -450.0, zt - 30.0))
L.append('[root [translate %.1f %.1f %.1f [cube 120.0 160.0 10.0]] "satin_black"]' % (sl[0] - 70.0, -80.0, zt))
open(os.path.join(OUT, 'duet.loon'), 'w').write('\n'.join(L) + '\n')
print('arms span', lo.round(0), hi.round(0), 'table z', round(zt))
