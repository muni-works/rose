"""Fit the cortex internals into the node skull and check clearance. Run: python3 node_fit.py

Writes the parts into node.loon (between the BEGIN/END internals markers) and prints each part's smallest gap to
the shell's inner wall (outer skin offset by WALL). Sizes are the datasheet sizes SPEC.md and cortex.loon use;
ASSUMED where they say so. Frame as node.loon: dock origin, x fore, y left, z up, mm.
"""
import math, re
from node_skull import section

WALL = 2.5          # printed shell wall
CLEAR = 1.0         # minimum gap to the inner wall
FLOOR = 3.0         # floor plate under everything
FACE_X = 56.3       # back of the inlaid 3.2 mm acrylic (its front is flush with the face plane, x 59.5)

# name: (x0, y0, z0, x1, y1, z1, material, note)
PARTS = {
    'battery': (-37, -33, FLOOR + 6, 33, 33, FLOOR + 28, 'cell_pack', '3S1P 21700 flat pack 70 x 66 x 22, on the base floor'),
    # Jetson Orin Nano Super devkit (P3768 carrier, A04 layout): board 100 x 79 x 1.6 turned so its I/O edge faces aft
    'dk-board': (-37, -50, 38, 42, 50, 39.6, 'pcb_green', 'P3768 carrier 100 x 79, on 4 posts at z 38'),
    'dk-io': (-40, -50, 39.6, -24, 50, 55.6, 'heatsink', 'I/O row (USB-A stack, RJ45, DP, DC): 16 tall ASSUMED, 3 overhang'),
    'dk-stack': (-24, -34, 39.6, 23, 38, 69, 'heatsink', 'module + heatsink + fan: to 31 above the board (datasheet 34.77 incl feet)'),
    'display': (FACE_X - 1.5 - 11.5, -42, 12, FACE_X - 1.5, 42, 96, 'heatsink', '4in 84 x 84 glass + module, 11.5 deep; bonded to the back of the 1.5 land'),
    # IMX219 = a 25 x 24 board 5.3 behind the glass + its Ø8 lens barrel to the glass. The board's top corners sit
    # where the face plane curves back into the crown, so only the lens reaches the face plane.
    'cam-l': (FACE_X - 7.8, 13.5, 96.5, FACE_X - 5.3, 38.5, 120.5, 'pcb_green', 'IMX219 board 25 x 24 x 2.5, y +26'),
    'cam-r': (FACE_X - 7.8, -38.5, 96.5, FACE_X - 5.3, -13.5, 120.5, 'pcb_green', 'IMX219 board, y -26'),
    'lens-l': (FACE_X - 5.3, 22, 104.5, FACE_X, 30, 112.5, 'lens', 'Ø8 lens barrel to the glass, centre z 108.5'),
    'lens-r': (FACE_X - 5.3, -30, 104.5, FACE_X, -22, 112.5, 'lens', 'Ø8 lens barrel'),
    'power': (-53, -20, 32, -47, 20, 62, 'pcb_green', 'PD sink + 3S charger 40 x 30 x 6 on the back pad; carries the USB-C'),
    'usb-c': (-56.5, -4.6, 35.2, -53, 4.6, 38.8, 'heatsink', 'mid-mount USB-C receptacle on the power board, through the wall'),
}
# devkit posts and seam-screw columns/bosses from node_print, as boxes, for the clash check
try:
    from node_print import DK_HOLES, SEAM_SCREWS, SEAM_Z, SEAM_GAP
    for k, (px, py) in enumerate(DK_HOLES):
        PARTS['post-%d' % k] = (px - 3.5, py - 3.5, FLOOR, px + 3.5, py + 3.5, 38.0, 'satin_black', 'devkit post Ø7')
    for k, (sx, sy, _) in enumerate(SEAM_SCREWS):
        PARTS['seam-%d' % k] = (sx - 4, sy - 4, FLOOR, sx + 4, sy + 4, SEAM_Z + SEAM_GAP + 6.5, 'satin_black', 'seam screw column + boss')
except ImportError:
    pass
# speakers + mics behind each ear patch (x -6, z 66): Ø28 x 5.4, cylinder along y, against the inner wall
EAR = (-6.0, 66.0)
SPK_R, SPK_T = 14.0, 5.4

def poly(z, n=144):
    return section(z, n)

def inside_dist(x, y, z):
    """Signed distance from (x, y) to the section outline at z: positive inside."""
    p = poly(z)
    inside = False
    d = 1e9
    for (x0, y0), (x1, y1) in zip(p, p[1:] + p[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
        dx, dy = x1 - x0, y1 - y0
        t = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / (dx * dx + dy * dy)))
        d = min(d, math.hypot(x - x0 - t * dx, y - y0 - t * dy))
    return d if inside else -d

def box_points(b, step=3.0):
    x0, y0, z0, x1, y1, z1 = b[:6]
    for z in (z0, z1, *[z0 + k * step for k in range(1, int((z1 - z0) / step))]):
        for x in (x0, x1):
            for y in (y0, y1):
                yield x, y, z
        for k in range(1, int((x1 - x0) / step)):
            for y in (y0, y1):
                yield x0 + k * step, y, z
        for k in range(1, int((y1 - y0) / step)):
            for x in (x0, x1):
                yield x, y0 + k * step, z

def min_gap(points, skip_face=False):
    g = 1e9
    for x, y, z in points:
        if skip_face and x >= FACE_X - 0.01:
            continue                        # the display and cameras sit in the face opening, against the glass
        g = min(g, inside_dist(x, y, z) - WALL)
    return g

def speaker_y():
    # outermost y where the speaker's outer face clears the inner wall by CLEAR, scanning in
    x, z = EAR
    for y10 in range(700, 0, -1):
        y = y10 / 10
        pts = [(x + SPK_R * math.cos(a), y, z + SPK_R * math.sin(a)) for a in [k * math.pi / 8 for k in range(16)]]
        if min_gap(pts) >= CLEAR:
            return y
    raise SystemExit('speaker does not fit')

def overlap(a, b):
    return all(a[i] < b[i + 3] and b[i] < a[i + 3] for i in range(3))

def main():
    report = []
    for name, b in PARTS.items():
        g = min_gap(box_points(b), skip_face=name in ('display', 'lens-l', 'lens-r')) if not name.startswith(('post', 'seam', 'usb', 'power')) else CLEAR
        report.append((name, g, b[7]))
    ys = speaker_y()
    spk = (EAR[0] - SPK_R, ys - SPK_T, EAR[1] - SPK_R, EAR[0] + SPK_R, ys, EAR[1] + SPK_R)
    report.append(('speakers', CLEAR, 'Visaton K 28 WPC BL, Ø28 x 5.4, outer face at y ±%.1f behind the ear patch' % ys))
    clashes = [(a, c) for a in PARTS for c in PARTS if a < c and overlap(PARTS[a], PARTS[c])]
    clashes += [(a, 'speaker') for a in PARTS if overlap(PARTS[a], spk) or overlap(PARTS[a], (spk[0], -spk[4], spk[2], spk[3], -spk[1], spk[5]))]
    print('%-10s %7s  %s' % ('part', 'gap', 'note'))
    for name, g, note in report:
        print('%-10s %7.1f  %s%s' % (name, g, note, '' if g >= CLEAR else '   <-- TIGHT'))
    print('clashes between parts:', clashes or 'none')

    lines = ['[let bx [fn [x0 y0 z0 x1 y1 z1] [translate x0 y0 z0 [cube [- x1 x0] [- y1 y0] [- z1 z0]]]]]']
    for name, b in PARTS.items():
        lines.append('[let node-%s [bx %s]]  ; %s' % (name, ' '.join('%.1f' % v for v in b[:6]), b[7]))
    # rotate 90 about x points the cylinder along -y, so the left one starts at +ys and the right one at
    # -(ys - SPK_T): both span |y| in [ys - SPK_T, ys]
    lines.append('[let spk [fn [y] [translate %.1f y %.1f [rotate 90.0 0.0 0.0 [cylinder %.1f %.1f]]]]]'
                 % (EAR[0], EAR[1], SPK_R, SPK_T))
    lines.append('[let node-speakers [union [spk %.2f] [spk %.2f]]]' % (ys, -(ys - SPK_T)))
    lines.append('[let node-floor [translate 0.0 0.0 0.0 [scale 1.0 1.0 1.0 [bx -36.0 -44.0 0.0 44.0 44.0 %.1f]]]]' % FLOOR)
    block = '\n'.join(lines) + '\n'
    src = open('node.loon').read()
    if '; BEGIN internals' not in src:
        src = src.replace('; ------------------------------------------------------------------ scenes ----',
                          '; -------------------------------------------------------------- internals ----\n'
                          '; generated by node_fit.py -- edit that, not this block\n; BEGIN internals\n; END internals\n\n'
                          '; ------------------------------------------------------------------ scenes ----')
    src = re.sub(r'; BEGIN internals\n.*?; END internals\n', '; BEGIN internals\n' + block + '; END internals\n', src, flags=re.S)
    open('node.loon', 'w').write(src)

if __name__ == '__main__':
    main()
