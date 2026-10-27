"""Mini + full-size nodes side by side on a bare desk with a banana for scale -> renders/node-full/. Run: python3 node_pair.py

No puck: each head rests on the back of its foot round, leaned back LEAN deg so the screen meets a seated eye line.
The lift that puts the lowest shell point on the desk is computed from the sections."""
import math, os, subprocess, tempfile, shutil
import node_skull as m, node_full as f

LEAN = 10.0
V = os.environ.get('VCAD', '/tmp/vt/release/vcad')
R = os.environ.get('VCAD_RENDER', '/tmp/vt/release/vcad-render')

def rest(secs):                                       # rotation about y that tips the face up: x' z' as vcad does it
    a = math.radians(LEAN)
    return -min(-x * math.sin(-a) + z * math.cos(-a) for z, pts in secs for x, _ in pts)

def grille(face, cz, half, y_lim):
    d = ['[translate %.2f %.2f %.2f [rotate 0.0 90.0 0.0 [cylinder 0.7 0.3]]]' % (face - 0.2, 2.2 * (i + 0.5 * (j % 2)), cz + 1.9 * j)
         for j in range(-2, 3) for i in range(-half, half + 1) if abs(2.2 * (i + 0.5 * (j % 2))) <= y_lim]
    u = d[0]
    for p in d[1:]:
        u = '[union %s %s]' % (p, u)
    return u

def main():
    lm = rest([(z, m.section(z)) for z in m.ZS])
    lf = rest([(z, f.section(z)) for z in [f.lerp_map(z / m.VS, f.ZMAP) for z in m.ZS]])
    pose = lambda s, y, lift: '[translate 0.0 %.1f %.2f [rotate 0.0 %.1f 0.0 %s]]' % (y, lift, -LEAN, s)
    L = ['[use node [node-head node-glass node-face node-lenses]]',
         '[use node-full [full-shell full-glass full-face full-lenses full-grille full-ears]]',
         '[material "satin_black" 0.035 0.037 0.042 0.05 0.6] [material "glass" 0.01 0.012 0.014 0.4 0.04] '
         '[material "face" 0.72 0.95 0.76 0.0 0.5] [material "lens" 0.035 0.04 0.045 0.3 0.08] [material "dots" 0.0 0.0 0.0 0.0 0.9] '
         '[material "banana" 0.95 0.78 0.15 0.0 0.55]']
    mg = grille(56.0, 28.5, 12, 30.0 - 9.5)           # node_print.py's chin grille
    for s, mat in [('node-head', 'satin_black'), ('node-glass', 'glass'), ('node-face', 'face'), ('node-lenses', 'lens'), (mg, 'dots')]:
        L.append('[root %s "%s"]' % (pose(s, 105.0, lm), mat))
    for s, mat in [('full-shell', 'satin_black'), ('full-glass', 'glass'), ('full-face', 'face'), ('full-lenses', 'lens'),
                   ('full-grille', 'dots'), ('full-ears', 'dots')]:
        L.append('[root %s "%s"]' % (pose(s, -95.0, lf), mat))
    parts = []
    for i in range(29):                               # banana: ~190 along a 120 radius arc
        t = i / 28
        a = math.radians(-50 + 100 * t)
        r = 18.0 * math.sin(math.pi * min(max(t, 0.06), 0.94)) ** 0.6
        parts.append('[translate %.1f %.1f %.1f [sphere %.2f]]' % (130 + 120 * (1 - math.cos(a)), 120 * math.sin(a) + 5, r, r))
    u = parts[0]
    for p in parts[1:]:
        u = '[union %s %s]' % (p, u)
    L.append('[root %s "banana"]' % u)
    tmp = tempfile.mkdtemp(dir='.')
    try:
        for n in ('node.loon', 'node-full.loon'):
            shutil.copy(n, tmp)
        open(os.path.join(tmp, 'pair.loon'), 'w').write('\n'.join(L) + '\n')
        subprocess.run([V, 'export', 'pair.loon', 'pair.vcad'], cwd=tmp, check=True, capture_output=True)
        os.makedirs('renders/node-full', exist_ok=True)
        subprocess.run([R, os.path.join(tmp, 'pair.vcad'), '--photoreal', '--azimuth=22', '--elevation=12', '--size', '1600',
                        '--fill', '0.85', '--auto-aspect', '-o', 'renders/node-full/mini-vs-full-desk.png'], check=True, capture_output=True)
        subprocess.run([R, os.path.join(tmp, 'pair.vcad'), '--photoreal', '--azimuth=90', '--elevation=2', '--size', '1600',
                        '--fill', '0.85', '--auto-aspect', '-o', 'renders/node-full/mini-vs-full-side.png'], check=True, capture_output=True)
    finally:
        shutil.rmtree(tmp)
    print('lift mini %.2f full %.2f at %.0f deg' % (lm, lf, LEAN))

if __name__ == '__main__':
    main()
