"""Generate the skull sections for node.loon (between the BEGIN/END skull markers). Run: python3 node_skull.py

Each section is an egg in plan: a superellipse whose half width runs from `wf` at the face to `wb` at the back,
and which squares off toward the face so the 84 mm glass sits on a flat. Heights: face plane chin 18 to brow 104,
cranium dome to 134 (crown peak behind the face), occiput swell at z 80, neck recess under the back."""
import math, re

def se(u, n):
    u = min(max(u, 0.0), 0.998)
    return (1 - u ** n) ** (1 / n)

def outline(z):
    fx = 56.0 if 18 <= z <= 104 else (44 + 12 * se((18 - z) / 18, 2.2) if z < 18 else -4 + 60 * se((z - 104) / 30, 2.2))
    bx = -(42 + 24 * se((80 - z) / 80, 1.6)) if z <= 80 else -4 - 62 * se((z - 80) / 54, 2.3)
    wb = 58 + 6 * se((80 - z) / 80, 1.8) if z <= 80 else 64 * se((z - 80) / 54, 2.4)   # back (parietal) half width
    wf = min(wb, 55.0) * (1 if z <= 104 else se((z - 104) / 30, 2.2))   # face half width
    wf = min(wf, wb)
    if z < 12:
        k = 12 * (1 - se((12 - z) / 12, 2.2))
        wb -= k; wf -= k; bx += k
    return fx, bx, wf, wb

def section(z, n=48):
    fx, bx, wf, wb = outline(z)
    xc, a = (fx + bx) / 2, (fx - bx) / 2
    pts = []
    for i in range(n):
        t = 2 * math.pi * i / n
        c, s = math.cos(t), math.sin(t)
        e = 12.0 if c > 0 else 2.6                      # squarer toward the face
        x = xc + a * math.copysign(abs(c) ** (2 / e), c)
        u = min(max((x - bx) / (fx - bx), 0.0), 1.0)    # 0 back .. 1 face
        w = wb + (wf - wb) * u ** 1.6
        y = w * math.copysign(abs(s) ** (2 / e), s)
        pts.append((x, y))
    return pts

zs = [0, 1.5, 3.5, 6, 9, 13, 18, 26, 36, 48, 60, 70, 80, 88, 96, 103, 109, 114, 118, 122, 125, 128, 130.5, 132.5, 133.6, 134.2]
lines = []
for z in zs:
    p = section(z)
    segs = ' '.join('[line %.2f %.2f %.2f %.2f]' % (*p[i], *p[(i + 1) % len(p)]) for i in range(len(p)))
    lines.append('  [sketch 0.0 0.0 %.2f 1.0 0.0 0.0 0.0 1.0 0.0 #[%s]]' % (z, segs))
block = '[let node-shell [loft #[\n' + '\n'.join(lines) + ']]]\n'
src = open('node.loon').read()
src = re.sub(r'; BEGIN skull\n.*?; END skull\n', '; BEGIN skull\n' + block + '; END skull\n', src, flags=re.S)
open('node.loon', 'w').write(src)
