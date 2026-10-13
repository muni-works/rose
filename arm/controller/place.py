"""Placement for the arm controller board -> the `;; ---- placement` block of arm_controller.loon.

Anchors (connectors, MCU, PHY, transceivers, power FETs) are placed by hand below:
connectors on their board edge, mating face outward. Every other part is packed
next to the anchor it shares the most signal nets with, at the free spot that
keeps its air-wires to that anchor shortest, without courtyard overlap.

Run after editing ANCHORS or the circuit; it rewrites only the placement block.
The result is plain [EcadPlace ref x y rot] lines - hand-tune them freely and
stop running this script.
"""
import collections, math, re, sys

LOON = 'arm_controller.loon'
FP = '/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints'
W, H = 150.0, 40.0          # board, KiCad coordinates (y down)
MARGIN = 0.6                # courtyard to board edge
GAP = 0.25                  # courtyard to courtyard
BEGIN, END = ';; ---- placement (place.py)', ';; ---- end placement'

# ref: (x, y, rot) for parts, or (edge, along) for edge connectors
ANCHORS = {
    # power entry, hot-swap, bus, brake (left end)
    'J1': ('L', 9.0),         # XT60 48 V in
    'Q1': (27.0, 8.0, 0),     # hot-swap FET
    'C7': (9.0, 24.0, 0),     # bulk 220u
    'J3': ('B', 29.0),        # brake resistor terminal
    'Q2': (45.0, 31.0, 0),    # brake FET
    # motor buses (top edge)
    'J5': ('T', 52.0),        # XT30 RS04 power
    'J8': ('T', 63.0),        # GH CAN A
    'J9': ('T', 70.0),        # XT30(2+2) wrist bus (placeholder header)
    'U11': (63.0, 11.0, 0),   # CAN A
    'U12': (77.0, 12.0, 90),  # CAN B
    # MCU
    'U10': (91.0, 20.0, 0),
    'J2': ('B', 76.0),        # e-stop
    # uplink, aux, comms (right end)
    'J10': ('T', 112.0),      # CAN-FD uplink
    'U13': (112.0, 10.0, 0),
    'J11': ('B', 112.0),      # aux
    'J13': ('R', 12.5),       # RJ45
    'U15': (124.0, 15.0, 0),  # PHY
    'J12': ('R', 33.0),       # USB-C
}


def parse_loon(src):
    lets = dict(re.findall(r'\[let (\w+) "([^"]*)"\]', src))
    parts = {}
    q = r'"([^"]*)"'
    for m in re.finditer(r'\[(r|c) %s %s %s %s\]' % (q, q, q, q), src):
        parts[m.group(2)] = (lets['R0402' if m.group(1) == 'r' else 'C0402'], {'1': m.group(4), '2': m.group(5)})
    for m in re.finditer(r'\[cfp %s %s (\w+) %s %s\]' % (q, q, q, q), src):
        parts[m.group(1)] = (lets[m.group(3)], {'1': m.group(4), '2': m.group(5)})
    for m in re.finditer(r'\[part %s %s %s (?:"([^"]*)"|(\w+))\s*\n\s*#\[([^\]]*)\]\]' % (q, q, q), src):
        fp = m.group(4) if m.group(4) is not None else lets[m.group(5)]
        toks = re.findall(q, m.group(6))
        if fp:
            parts[m.group(1)] = (fp, dict(zip(toks[::2], toks[1::2])))
    return parts


def footprint(fp):
    lib, name = fp.split(':')
    t = open('%s/%s.pretty/%s.kicad_mod' % (FP, lib, name)).read()
    xs, ys = [], []
    for blk in re.findall(r'\((?:fp_line|fp_rect|fp_poly|fp_circle)(.*?)\(layer "F\.CrtYd"\)', t, re.S):
        for x, y in re.findall(r'\((?:start|end|xy|center) ([\-\d.]+) ([\-\d.]+)\)', blk):
            xs.append(float(x)); ys.append(float(y))
    pads = {}
    for num, x, y in re.findall(r'\(pad "([^"]*)" \w+ \w+\s*\(at ([\-\d.]+) ([\-\d.]+)', t):
        pads.setdefault(num, (float(x), float(y)))
    if not xs:
        px = [p[0] for p in pads.values()]; py = [p[1] for p in pads.values()]
        xs, ys = [min(px) - 1, max(px) + 1], [min(py) - 1, max(py) + 1]
    return (min(xs), min(ys), max(xs), max(ys)), pads


def rot(p, a):
    """KiCad footprint rotation (degrees, CCW on screen, y down)."""
    r = math.radians(a)
    return (p[0] * math.cos(r) + p[1] * math.sin(r), -p[0] * math.sin(r) + p[1] * math.cos(r))


def box(cy, x, y, a):
    pts = [rot(p, a) for p in ((cy[0], cy[1]), (cy[2], cy[1]), (cy[2], cy[3]), (cy[0], cy[3]))]
    return (x + min(p[0] for p in pts), y + min(p[1] for p in pts), x + max(p[0] for p in pts), y + max(p[1] for p in pts))


def overlap(a, b, g=GAP):
    return not (a[2] + g <= b[0] or b[2] + g <= a[0] or a[3] + g <= b[1] or b[3] + g <= a[1])


def inside(b):
    e = 1e-6
    return b[0] >= MARGIN - e and b[1] >= MARGIN - e and b[2] <= W - MARGIN + e and b[3] <= H - MARGIN + e


def edge_place(cy, pads, edge, along):
    """Rotation and origin putting the courtyard on `edge` with the mating face outward."""
    pc = (sum(p[0] for p in pads.values()) / len(pads), sum(p[1] for p in pads.values()) / len(pads))
    cc = ((cy[0] + cy[2]) / 2, (cy[1] + cy[3]) / 2)
    out = {'L': (-1, 0), 'R': (1, 0), 'T': (0, -1), 'B': (0, 1)}[edge]
    best = None
    for a in (0, 90, 180, 270):
        d = rot((cc[0] - pc[0], cc[1] - pc[1]), a)
        score = d[0] * out[0] + d[1] * out[1]
        if best is None or score > best[0]:
            best = (score, a)
    a = best[1]
    b = box(cy, 0, 0, a)
    # `along` is the courtyard centre along the edge
    if edge == 'L':
        x, y = MARGIN - b[0], along - (b[1] + b[3]) / 2
    elif edge == 'R':
        x, y = W - MARGIN - b[2], along - (b[1] + b[3]) / 2
    elif edge == 'T':
        x, y = along - (b[0] + b[2]) / 2, MARGIN - b[1]
    else:
        x, y = along - (b[0] + b[2]) / 2, H - MARGIN - b[3]
    return round(x, 2), round(y, 2), a


def main():
    src = open(LOON).read()
    parts = parse_loon(src)
    fps = {r: footprint(fp) for r, (fp, _) in parts.items()}
    count = collections.Counter(n for _, pins in parts.values() for n in pins.values())
    rail = {n for n, c in count.items() if c > 8}
    placed, boxes = {}, {}
    for ref, spec in ANCHORS.items():
        cy, pads = fps[ref]
        x, y, a = edge_place(cy, pads, *spec) if isinstance(spec[0], str) else spec
        placed[ref] = (x, y, a)
        boxes[ref] = box(cy, x, y, a)
        if not inside(boxes[ref]):
            print('warning: anchor %s outside the board %s' % (ref, boxes[ref]), file=sys.stderr)
    for a in list(placed):
        for b in list(placed):
            if a < b and overlap(boxes[a], boxes[b], 0):
                print('warning: anchors %s and %s overlap' % (a, b), file=sys.stderr)

    def pin_world(ref, pin, x, y, a):
        p = fps[ref][1].get(pin)
        if p is None:
            return (x, y)
        q = rot(p, a)
        return (x + q[0], y + q[1])

    def cost(ref, x, y, a):
        c = 0.0
        for pin, net in parts[ref][1].items():
            if net in rail:
                continue
            here = pin_world(ref, pin, x, y, a)
            others = [pin_world(o, p2, *placed[o]) for o in placed for p2, n2 in parts[o][1].items() if n2 == net]
            if others:
                c += min(math.dist(here, o) for o in others)
        return c

    todo = [r for r in parts if r not in placed]
    # most-connected first, then biggest
    def deg(r):
        return sum(1 for n in parts[r][1].values() if n not in rail)
    todo.sort(key=lambda r: (-deg(r), -(fps[r][0][2] - fps[r][0][0]) * (fps[r][0][3] - fps[r][0][1])))
    step = 0.5
    for ref in todo:
        # the placed parts it connects to (signal nets), else anything on a shared rail
        mates = [o for o in placed if any(n2 in parts[ref][1].values() and n2 not in rail for n2 in parts[o][1].values())]
        if not mates:
            mates = [o for o in placed if set(parts[o][1].values()) & set(parts[ref][1].values())] or list(placed)
        cx = sum(placed[o][0] for o in mates) / len(mates)
        cy0 = sum(placed[o][1] for o in mates) / len(mates)
        best = None
        for ring in range(0, 120):
            r = ring * step
            n = max(1, int(2 * math.pi * r / step))
            for k in range(n):
                t = 2 * math.pi * k / n
                x, y = round(cx + r * math.cos(t), 2), round(cy0 + r * math.sin(t), 2)
                for a in (0, 90):
                    b = box(fps[ref][0], x, y, a)
                    if not inside(b) or any(overlap(b, bb) for bb in boxes.values()):
                        continue
                    c = cost(ref, x, y, a) + 0.3 * r
                    if best is None or c < best[0]:
                        best = (c, x, y, a, b)
            if best is not None and r > 6 * step + (best[0] if best[0] < 30 else 30) * 0.3:
                break
        if best is None:
            print('could not place', ref, file=sys.stderr)
            continue
        _, x, y, a, b = best
        placed[ref] = (x, y, a)
        boxes[ref] = b
    lines = [BEGIN, ';; board: 150 x 40 mm, 4 layers. Anchors by hand, the rest packed near the part they connect to.',
             '[EcadBoard 0.0 0.0 150.0 40.0 2.0 1.6 4.0]',
             '[EcadRules 0.15 0.15 0.45 0.25 0.3 0.15]',
             '[EcadPour "GND" "In1.Cu"]',
             '[EcadPour "+3V3" "In2.Cu"]']
    for ref in sorted(placed, key=lambda r: (re.sub(r'\d', '', r), int(re.sub(r'\D', '', r) or 0))):
        x, y, a = placed[ref]
        lines.append('[EcadPlace "%s" %.2f %.2f %.1f]' % (ref, x, y, float(a)))
    lines.append(END)
    block = '\n'.join(lines) + '\n'
    if BEGIN in src:
        src = src[:src.index(BEGIN)] + block + src[src.index(END) + len(END) + 1:]
    else:
        src = src.rstrip('\n') + '\n\n' + block
    open(LOON, 'w').write(src)
    area = sum((b[2] - b[0]) * (b[3] - b[1]) for b in boxes.values())
    print('placed %d / %d parts, courtyard fill %.0f%%' % (len(placed), len(parts), 100 * area / (W * H)))


if __name__ == '__main__':
    main()
