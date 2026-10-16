"""Brain placement: anchors by hand, the rest packed next to what it connects to.

    python3 place.py        (rewrites the placement block at the end of brain.loon)

The SoC sits where its ball groups point (sheet 6 of ROCK 5C does the same): DDR CH0
leaves its left edge -> DRAM0 on the left, CH1 its top edge -> DRAM1 above, eMMC and
PMIC control leave the right edge -> eMMC to the right. The 0201 decoupling of the
SoC and DRAM rails goes on the back, under the ball field it feeds. Connectors sit on
the edges they serve. Everything else is packed by a spiral search from the
centroid of the pins it connects to (rails excluded: they are planes).
"""
import math, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
BOARD = os.environ.get('BOARD', 'brain')            # brain | module | carrier
LOON = os.path.join(HERE, BOARD + '.loon')
KI = '/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints'
LIBS = {'brain': os.path.join(HERE, 'brain.pretty'), 'art': os.path.join(HERE, '..', 'arm', 'controller', 'art.pretty')}
W, H = {'brain': (110.0, 84.0), 'module': (50.0, 54.0), 'carrier': (110.0, 84.0)}[BOARD]
MARGIN, GAP = 0.6, 0.2
BEGIN, END = ';; ---- placement (place.py)', ';; ---- end placement'

# 8-layer HDI: F sig / In1 GND / In2 SoC power regions / In3 sig (BGA escape) / In4 VDD2 + board power / In5 GND / In6 VDDQ + sig / B sig + caps.
# GND and power drop into planes next to the surface; only signals reach the signal layers.
STACK = ['F.Cu', 'In1.Cu', 'In2.Cu', 'In3.Cu', 'In4.Cu', 'In5.Cu', 'In6.Cu', 'B.Cu']

BRAIN_ANCHORS = {   # ref: (x, y, rot, side)  side 'F' / 'B'
    'U_SOC': (40.0, 36.0, 0, 'F'),
    'U_DRAM0': (24.0, 35.0, 0, 'F'),
    'U_DRAM1': (40.5, 19.5, 90, 'F'),
    'U_EMMC': (56.5, 37.0, 0, 'F'),
    'U_PMIC': (38.0, 55.0, 0, 'F'),
    'U_RT': (80.0, 34.0, 0, 'F'),
    'J1': (12.0, 78.5, 0, 'F'),      # 12 V + BMS CAN from the spine
    'J2': (25.0, 80.5, 0, 'F'),      # e-stop / pack status
    'J3': (4.5, 20.0, 270, 'F'),     # USB-C
    'J5': (58.0, 3.0, 180, 'F'),     # camera 0
    'J6': (75.0, 3.0, 180, 'F'),     # camera 1
    'J7': (8.0, 50.0, 270, 'F'),     # SoC console
    'J8': (98.0, 22.0, 0, 'F'),      # H7 SWD
    'J10': (107.0, 10.0, 90, 'F'), 'J11': (107.0, 22.0, 90, 'F'), 'J12': (107.0, 34.0, 90, 'F'),
    'J13': (107.0, 46.0, 90, 'F'), 'J14': (107.0, 58.0, 90, 'F'),
    'H1': (3.5, 3.5, 0, 'F'), 'H2': (106.5, 80.5, 0, 'F'), 'H3': (3.5, 80.5, 0, 'F'), 'H4': (98.0, 80.5, 0, 'F'),
    'SW1': (14.0, 58.0, 0, 'F'), 'SW2': (14.0, 50.0, 0, 'F'),
}
# the module: the brain's SoC block moved to the origin (same relative
# placement, so the DDR geometry carries over), B2Bs on the back along the
# bottom edge
MODULE_ANCHORS = {
    'U_SOC': (24.0, 24.0, 0, 'F'),
    'U_DRAM0': (8.0, 23.0, 0, 'F'),
    'U_DRAM1': (24.5, 7.5, 90, 'F'),
    'U_EMMC': (40.5, 25.0, 0, 'F'),
    'U_PMIC': (22.0, 43.0, 0, 'F'),
    'J_B2B1': (13.0, 50.0, 0, 'B'),
    'J_B2B2': (37.0, 50.0, 0, 'B'),
}
# the carrier: the module sits where the brain's SoC block was (its origin at
# MODULE_AT), its B2B headers landing on the carrier's sockets; nothing else
# may sit under it (the module's back carries caps and the headers)
MODULE_AT = (16.0, 12.0)
MODULE_WH = (50.0, 54.0)
CARRIER_ANCHORS = {k: v for k, v in BRAIN_ANCHORS.items() if not k.startswith(('U_SOC', 'U_DRAM', 'U_EMMC', 'U_PMIC'))}
for j in ('J_B2B1', 'J_B2B2'):
    x_, y_, a_, _ = MODULE_ANCHORS[j]
    CARRIER_ANCHORS[j] = (MODULE_AT[0] + x_, MODULE_AT[1] + y_, a_, 'F')
KEEPOUTS = {'carrier': [(MODULE_AT[0], MODULE_AT[1], MODULE_AT[0] + MODULE_WH[0], MODULE_AT[1] + MODULE_WH[1])]}.get(BOARD, [])
ANCHORS = {'brain': BRAIN_ANCHORS, 'module': MODULE_ANCHORS, 'carrier': CARRIER_ANCHORS}[BOARD]
THT = re.compile(r'^(J\d|H|SW)')     # through-hole: blocks both sides
SOC_RAILS = re.compile(r'^(VDD_|VCC|VDDA|VDDQ|AVDD|VCCA|VCCIO|PMIC_VCCIO|EMMC_VDDI)')
PLANE = {'GND'}


def parse_loon():
    src = open(LOON).read()
    parts = {}
    for ref, lib, val, fp, pins in re.findall(r'\[part "([^"]+)" "([^"]+)" "([^"]*)" "([^"]*)"\n  #\[([^\]]*)\]\]', src):
        toks = re.findall(r'"([^"]*)"', pins)
        if fp:
            parts[ref] = (fp, dict(zip(toks[::2], toks[1::2])))
    return src, parts


def fp_file(fp):
    lib, name = fp.split(':')
    d = LIBS.get(lib, os.path.join(KI, lib + '.pretty'))
    return os.path.join(d, name + '.kicad_mod')


_cache = {}


def footprint(fp):
    """(courtyard box (x0,y0,x1,y1) at rot 0, {pad: (x, y)})"""
    if fp in _cache:
        return _cache[fp]
    t = open(fp_file(fp)).read()
    pads = {}
    for num, x, y in re.findall(r'\(pad "([^"]*)" \w+ \w+\s*\(at ([\-\d\.]+) ([\-\d\.]+)', t):
        pads.setdefault(num, (float(x), float(y)))
    xs, ys = [], []
    for blk in re.findall(r'\((?:fp_line|fp_rect|fp_poly|fp_circle)\b.*?\(layer "F\.CrtYd"\)', t, re.S):
        for x, y in re.findall(r'\((?:start|end|xy|center) ([\-\d\.]+) ([\-\d\.]+)\)', blk):
            xs.append(float(x))
            ys.append(float(y))
    if not xs:
        xs = [p[0] for p in pads.values()] or [0]
        ys = [p[1] for p in pads.values()] or [0]
        xs += [min(xs) - 0.5, max(xs) + 0.5]
        ys += [min(ys) - 0.5, max(ys) + 0.5]
    _cache[fp] = ((min(xs), min(ys), max(xs), max(ys)), pads)
    return _cache[fp]


def rot(p, a):
    a = math.radians(a)
    c, s = math.cos(a), math.sin(a)
    return (p[0] * c - p[1] * s, p[0] * s + p[1] * c)


def world(fp, x, y, a, side, local):
    lx, ly = local
    if side == 'B':
        ly = -ly
    q = rot((lx, ly), a)
    return (x + q[0], y + q[1])


def box(fp, x, y, a, side):
    b = footprint(fp)[0]
    pts = [world(fp, x, y, a, side, c) for c in ((b[0], b[1]), (b[2], b[1]), (b[2], b[3]), (b[0], b[3]))]
    return (min(p[0] for p in pts), min(p[1] for p in pts), max(p[0] for p in pts), max(p[1] for p in pts))


def overlap(a, b, g=GAP):
    return not (a[2] + g <= b[0] or b[2] + g <= a[0] or a[3] + g <= b[1] or b[3] + g <= a[1])


def inside(b):
    return b[0] >= MARGIN and b[1] >= MARGIN and b[2] <= W - MARGIN and b[3] <= H - MARGIN


def main():
    src, parts = parse_loon()
    placed, boxes = {}, {'F': [], 'B': []}

    def put(ref, x, y, a, side):
        fp = parts[ref][0]
        b = box(fp, x, y, a, side)
        placed[ref] = (x, y, a, side)
        for s in (('F', 'B') if THT.match(ref) else (side,)):
            boxes[s].append((b, ref))
        return b

    for ref, (x, y, a, side) in ANCHORS.items():
        if ref in parts:
            put(ref, x, y, a, side)
    # pins of anchored / placed parts, by net
    def net_points():
        pts = {}
        for ref, (x, y, a, side) in placed.items():
            fp, pins = parts[ref]
            pads = footprint(fp)[1]
            for pin, net in pins.items():
                if pin in pads:
                    pts.setdefault(net, []).append(world(fp, x, y, a, side, pads[pin]))
        return pts

    for k in KEEPOUTS:                  # the module's footprint, front side
        boxes['F'].append((k, 'module'))
    soc_box = box(parts['U_SOC'][0], *placed['U_SOC']) if 'U_SOC' in placed else (0, 0, W, H)
    dram_boxes = {r: box(parts[r][0], *placed[r]) for r in ('U_DRAM0', 'U_DRAM1') if r in placed}
    # order: ICs and inductors first, then the passives by how many signal nets tie them down
    rest = [r for r in parts if r not in placed]
    weight = lambda r: (0 if re.match(r'^(U|Y|L|D|F|Q)', r) else 1, -len(parts[r][1]))
    rest.sort(key=weight)
    for ref in rest:
        fp, pins = parts[ref]
        pts = net_points()
        sig = [n for n in pins.values() if n not in PLANE]
        side = 'F'
        target = None
        # the 0201 decoupling of the SoC and DRAM rails: on the back, under the part it feeds
        if fp.endswith('C_0201_0603Metric') and any(SOC_RAILS.match(n) for n in sig):
            side = 'B'
            rail = next(n for n in sig if SOC_RAILS.match(n))
            home = soc_box
            for r_, b_ in dram_boxes.items():
                if any(n.startswith(('VDD2_DDR', 'VCC1V8_PMU_DDR')) for n in sig) and 'DDR' in rail and r_ == 'U_DRAM0':
                    pass
            own = [p for p in pts.get(rail, []) if home[0] <= p[0] <= home[2] and home[1] <= p[1] <= home[3]]
            target = (sum(p[0] for p in own) / len(own), sum(p[1] for p in own) / len(own)) if own else ((home[0] + home[2]) / 2, (home[1] + home[3]) / 2)
        if target is None:
            cand = [p for n in sig for p in pts.get(n, []) if not SOC_RAILS.match(n) or len(sig) == 1]
            if not cand:
                cand = [p for n in sig for p in pts.get(n, [])]
            target = (sum(p[0] for p in cand) / len(cand), sum(p[1] for p in cand) / len(cand)) if cand else (W / 2, H / 2)
        best = None
        for rr in [i * 0.25 for i in range(0, 160)]:
            steps = max(1, int(2 * math.pi * rr / 0.25))
            for k in range(steps):
                ang = 2 * math.pi * k / steps
                x, y = round(target[0] + rr * math.cos(ang), 2), round(target[1] + rr * math.sin(ang), 2)
                for a in (0, 90):
                    b = box(fp, x, y, a, side)
                    if not inside(b):
                        continue
                    if any(overlap(b, ob) for ob, _ in boxes[side]):
                        continue
                    if THT.match(ref) and any(overlap(b, ob) for ob, _ in boxes['B' if side == 'F' else 'F']):
                        continue
                    best = (x, y, a)
                    break
                if best:
                    break
            if best:
                break
        if not best:
            sys.exit(f'no room for {ref}')
        put(ref, best[0], best[1], best[2], side)

    lines = [BEGIN, (f';; board {W:g} x {H:g} mm, 6-layer: F sig / In1 GND / In2 3V3_RT / In3 5V_RT / In4 sig / B sig' if BOARD == 'carrier'
                     else f';; board {W:g} x {H:g} mm, 8-layer HDI: ' + ' / '.join(STACK)),
             f'[EcadBoard 0.0 0.0 {W:.1f} {H:.1f} 1.5 1.6 {6.0 if BOARD == "carrier" else 8.0}]',
             '[EcadRules 0.09 0.09 0.40 0.20 0.3 0.15]',
             '[EcadPour "GND" "In1.Cu"]'] + (['[EcadPour "3V3_RT" "In2.Cu"]', '[EcadPour "5V_RT" "In3.Cu"]'] if BOARD == 'carrier' else ['[EcadPour "GND" "In5.Cu"]'])
    for ref in sorted(placed, key=lambda r: (re.sub(r'\d+$', '', r), int(re.sub(r'\D', '', r) or 0))):
        x, y, a, side = placed[ref]
        form = 'EcadPlaceBack' if side == 'B' else 'EcadPlace'
        lines.append(f'[{form} "{ref}" {x:.2f} {y:.2f} {float(a):.1f}]')
    lines.append(END)
    block = '\n'.join(lines) + '\n'
    if BEGIN in src:
        src = src[:src.index(BEGIN)] + block + src[src.index(END) + len(END) + 1:]
    else:
        src = src.rstrip('\n') + '\n\n' + block
    open(LOON, 'w').write(src)
    fill = {s: sum((b[2] - b[0]) * (b[3] - b[1]) for b, _ in boxes[s]) / (W * H) for s in 'FB'}
    print(f'placed {len(placed)} parts; courtyard fill front {100 * fill["F"]:.0f}%, back {100 * fill["B"]:.0f}%')


if __name__ == '__main__':
    main()
