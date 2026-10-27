"""Generate the skull sections for node.loon (between the BEGIN/END skull markers). Run: python3 node_skull.py

Each section is an egg in plan: a superellipse whose half width runs from `wf` at the face to `wb` at the back,
and which squares off toward the face so the 84 mm glass sits on a flat. Heights: face plane chin 18 to brow 116 (the glass covers the panel and the cameras),
cranium dome to 134 (crown peak behind the face), occiput swell at z 80, neck recess under the back."""
import math, re

BROW = 116.0                                         # top of the flat face plane

def se(u, n):
    u = min(max(u, 0.0), 0.998)
    return (1 - u ** n) ** (1 / n)

def outline(z):
    fx = 56.0 if 18 <= z <= BROW else (44 + 12 * se((18 - z) / 18, 2.2) if z < 18 else -4 + 60 * se((z - BROW) / (134 - BROW), 2.2))
    bx = -(42 + 24 * se((80 - z) / 80, 1.6)) if z <= 80 else -4 - 62 * se((z - 80) / 54, 2.3)
    wb = 58 + 6 * se((80 - z) / 80, 1.8) if z <= 80 else 64 * se((z - 80) / 54, 2.4)   # back (parietal) half width
    wf = min(wb, 55.0) * (1 if z <= BROW else se((z - BROW) / (134 - BROW), 2.2))   # face half width
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

def sk(z, pts):
    segs = ' '.join('[line %.2f %.2f %.2f %.2f]' % (*pts[i], *pts[(i + 1) % len(pts)]) for i in range(len(pts)))
    return '[sketch 0.0 0.0 %.2f 1.0 0.0 0.0 0.0 1.0 0.0 #[%s]]' % (z, segs)

def scaled(pts, k, cx=None):
    cx = sum(x for x, _ in pts) / len(pts) if cx is None else cx
    return [(cx + (x - cx) * k, y * k) for x, y in pts]

def circle(r, cx=0.0, n=48):
    return [(cx + r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n)) for i in range(n)]

def loft(name, rows):
    return '[let %s [loft #[\n%s]]]\n' % (name, '\n'.join('  ' + sk(z, p) for z, p in rows))

zs = [0, 1.5, 3.5, 6, 9, 13, 18, 26, 36, 48, 60, 70, 80, 88, 96, 104, 112, 116, 119, 122, 125, 128, 130.5, 132.5, 133.6, 134.2]
block = loft('node-shell', [(z, section(z)) for z in zs])

# Neck join: the neck's top trim is the foot's own outline 1 mm smaller (the reveal), 1 mm under it, then it necks
# down to a round column. The head rests on it; nothing reads as bolted.
foot = section(0)
block += loft('node-neck', [(-1.0, scaled(foot, 0.97)), (-6.0, scaled(foot, 0.86)), (-16.0, circle(30, 2.0)),
                            (-30.0, circle(25, 4.0)), (-58.0, circle(27, 4.0))])

# Jaw slot: a 2.4 mm slot along each side, under the cheeks, rising 8 deg toward the back like a jaw
# line, from the face corner to under the ear (speakers, mics and air intake behind it). The back stays clean.
JZ, JH = 24.0, 2.4
ring = lambda k: [(JZ, scaled(section(JZ), k)), (JZ + JH, scaled(section(JZ), k))]
block += loft('jaw-outer', ring(1.08)) + loft('jaw-inner', ring(0.955))
block += loft('pipe-inner', [(JZ + 0.6, scaled(section(JZ), 0.95)), (JZ + JH - 0.6, scaled(section(JZ), 0.95))])
block += loft('pipe-outer', [(JZ + 0.6, scaled(section(JZ), 0.962)), (JZ + JH - 0.6, scaled(section(JZ), 0.962))])
block += '''[let jaw-tilt [fn [s] [translate 40.0 0.0 JZ [rotate 0.0 8.0 0.0 [translate -40.0 0.0 [- 0.0 JZ] s]]]]]
[let behind-face [translate -42.0 -100.0 -50.0 [cube 78.0 200.0 200.0]]]
[let node-jaw [jaw-tilt [intersection behind-face [difference jaw-inner jaw-outer]]]]
[let node-pipe [jaw-tilt [intersection behind-face [difference pipe-inner pipe-outer]]]]
[let node-head [difference node-jaw node-shell]]
'''.replace('JZ', '%.1f' % JZ)

# Ear grilles: a 19-dot hex patch (Ø1.2 on 2.6) on each side where a human ear sits, behind mid-depth and just above
# the jaw slot's back end. The mic port is the centre dot. Massing only: the dots are drawn as dark discs on the
# surface (each at the local surface y); print prep cuts them as real holes.
EAR_X, EAR_Z, P = -14.0, 50.0, 2.6

def surface_y(x, z):
    pts = [q for q in section(z, 96) if q[1] > 0]
    pts.sort()
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        if x0 <= x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return max(y for _, y in pts)

dots = [(EAR_X + P * (i + 0.5 * j), EAR_Z + P * 0.866 * j) for j in range(-2, 3) for i in range(-2, 3) if abs(i + 0.5 * j) + abs(j) * 0.5 <= 2.01 and abs(j) <= 2]
discs = []
for x, z in dots:
    y = surface_y(x, z)
    for s in (1, -1):
        discs.append('[translate %.2f %.2f %.2f [rotate %d 0.0 0.0 [cylinder-n 0.6 0.6 12]]]' % (x, s * (y - 0.35), z, -90 if s > 0 else 90))
u = discs[0]
for d in discs[1:]:
    u = '[union %s %s]' % (d, u)
block += '[let node-ears %s]\n' % u
src = open('node.loon').read()
src = re.sub(r'; BEGIN skull\n.*?; END skull\n', '; BEGIN skull\n' + block + '; END skull\n', src, flags=re.S)
open('node.loon', 'w').write(src)
