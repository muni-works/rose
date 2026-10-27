"""Generate the skull sections for node.loon (between the BEGIN/END skull markers). Run: python3 node_skull.py

Each section is an egg in plan: a superellipse whose half width runs from `wf` at the face to `wb` at the back,
and which squares off toward the face so the 152 x 102 acrylic sits on a flat. Heights: face plane chin 18 to brow 116 (the glass covers the panel and the cameras),
cranium dome to 134 (crown peak behind the face), occiput swell at z 80, neck recess under the back."""
import math, re

CHIN = 11.0                                          # bottom of the flat face plane (the sheet's bottom 14.7 - its frame)
BROW = 119.5                                         # top of the flat face plane (the sheet's top 116.3 + its frame)
FACE = 59.5                                          # the face plane (front of the inlaid acrylic)
WIDE = 1.36                                          # width scale for the landscape 4 x 6 in face (152.4 wide)
FACE_HW = 86.0                                       # face half width: the sheet's 76.2 + the frame

def se(u, n):
    u = min(max(u, 0.0), 0.998)
    return (1 - u ** n) ** (1 / n)

def outline(z):
    fx = FACE if CHIN <= z <= BROW else (FACE - 12 + 12 * se((CHIN - z) / CHIN, 2.2) if z < CHIN else -4 + (FACE + 4) * se((z - BROW) / (134 - BROW), 2.2))
    bx = -(42 + 24 * se((80 - z) / 80, 1.6)) if z <= 80 else -4 - 62 * se((z - 80) / 54, 2.3)
    wb = WIDE * (58 + 6 * se((80 - z) / 80, 1.8) if z <= 80 else 64 * se((z - 80) / 54, 4.0))   # back (parietal) half width; squarer crown (n 4) holds the width up to the sheet's top corners
    wf = min(wb, FACE_HW) * (1 if z <= BROW else se((z - BROW) / (134 - BROW), 2.2))   # face half width
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
        e = 28.0 if c > 0 else 2.6                      # square toward the face: the flat sheet runs nearly to the sides
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

def main():
    zs = [0, 1.5, 3.5, 6, 9, 13, 18, 26, 36, 48, 60, 70, 80, 88, 96, 104, 112, 116, 119, 122, 125, 128, 130.5, 132.5, 133.6, 134.2]
    block = loft('node-shell', [(z, section(z)) for z in zs])

    # Neck join: the neck's top trim is the foot's flat (the dock face) 1 mm under it, then it necks down to a round
    # column. It stops inside the foot's bottom round, so the intake band on that round stays open on the neck too.
    foot = section(0)
    block += loft('node-neck', [(-1.0, scaled(foot, 0.8)), (-6.0, scaled(foot, 0.72)), (-16.0, circle(30, 2.0)),
                                (-30.0, circle(25, 4.0)), (-58.0, circle(27, 4.0))])

    block += '[let node-head node-shell]\n'

    # Intake: a perforated band on the foot's bottom round (z 1.5-7), facing down and out. On the neck and on the puck it
    # sits in the shadow under the head, so it's hidden from any normal view. Ø1.4 on 2.2 hex, 3 rows round the whole
    # perimeter: ~3 x 150 = 450 holes, ~690 mm² (SPEC wants ~650 at 25 W). Drawn as a dark band; print prep cuts holes.
    block += loft('intake-out', [(1.5, scaled(section(1.5), 1.01)), (7.0, scaled(section(7.0), 1.006))])
    block += loft('intake-in', [(1.5, scaled(section(1.5), 0.985)), (7.0, scaled(section(7.0), 0.99))])
    block += '[let node-intake [difference intake-in intake-out]]\n'

    # Ear grilles: where a human ear sits -- at eye height, just behind the middle of the head's depth -- a vertical
    # oval patch of dots (Ø1.2 on a 2.6 hex, ~12 x 22) on each side, like the ear's own outline. The mic ports and the
    # speaker sit behind it. Massing only: the dots are drawn as dark discs on the surface (each at the local surface y);
    # print prep cuts them as real holes.
    EAR_X, EAR_Z, EAR_A, EAR_B, P = -6.0, 66.0, 6.2, 11.2, 2.6

    def surface_y(x, z):
        pts = [q for q in section(z, 96) if q[1] > 0]
        pts.sort()
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            if x0 <= x <= x1:
                return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
        return max(y for _, y in pts)

    dots = [(EAR_X + P * (i + 0.5 * (j % 2)), EAR_Z + P * 0.866 * j) for j in range(-6, 7) for i in range(-4, 4)]
    dots = [(x, z) for x, z in dots if ((x - EAR_X) / EAR_A) ** 2 + ((z - EAR_Z) / EAR_B) ** 2 <= 1.0]
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


if __name__ == '__main__':
    main()
