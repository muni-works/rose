"""BGA fanout: a laser via in every used ball of the SoC, the DRAMs and the eMMC,
and an In2 power region per SoC rail. Run after place.py; vcad routes from these.

    python3 fanout.py

Via-in-pad, the HDI way (the 0.4 mm SoC leaves no room between balls):
  GND ball    -> microvia F.Cu-In1.Cu into the GND plane
  power ball  -> stacked microvias F.Cu-In2.Cu into its rail's In2 region
  signal ball -> stacked microvias F.Cu-In3.Cu, routed out on In3 and below
                 (vcad's via-in-pad); outer-ring balls can also leave on F.Cu
So the planes take the GND and power vias, and the signal layers see only the
signal vias: an escape channel between balls is not walled off by ground.
Laser vias 0.25 / 0.10 mm, filled and capped (VIPPO) at the fab.
"""
import math, os, re
from collections import defaultdict

import place

VIA = (0.25, 0.10)
BGAS = ('U_SOC', 'U_DRAM0', 'U_DRAM1', 'U_EMMC')
BEGIN, END = ';; ---- fanout (fanout.py)', ';; ---- end fanout'
PLANE_GND = 'In1.Cu'
POWER_LAYER = 'In2.Cu'
SIGNAL_LAYER = 'In3.Cu'
POWER_LAYERS = ('In2.Cu', 'In4.Cu', 'In6.Cu')
# DRAM balls interleave VDD2 / VDDQ / VDD1, so one layer can't hold all three
# as regions: VDD2 takes In4, VDDQ In6, and VDD1 (1.8 V, low current) is routed
RAIL_LAYER = {'VDD2_DDR_S3': 'In4.Cu', 'VDDQ_DDR_S0': 'In6.Cu'}
TRACE_RAILS = {'VCC1V8_PMU_DDR_S3'}
BOARD_RAILS = {'12V', '12V_RT', '5V_RT', '3V3_RT', 'VCC_SYSIN', 'VDDA_RT'}
assign = {}
layer_of = lambda net: assign.get(net, POWER_LAYER)


# Only the high-current rails get a plane region; the rest (analog, IO, PLL
# supplies: tens of mA, pads scattered over the board) are routed like signals -
# as regions they overlapped every other rail on In2 and fell into islands.
PLANE_RAILS = re.compile(r'^(VDD_CPU_(BIG0|BIG1|LIT)(_MEM)?_S0|VDD_(GPU|NPU|VDENC)(_MEM)?_S0|VDD_LOGIC_S0|VDD_DDR_S0|VDD2_DDR_S3|VDDQ_DDR_S0)$')


def is_power(net):
    return bool(PLANE_RAILS.match(net))


def main():
    src, parts = place.parse_loon()
    placed = {r: (float(x), float(y), float(a), 'B' if f.endswith('Back') else 'F')
              for f, r, x, y, a in re.findall(r'\[(EcadPlace(?:Back)?) "([^"]+)" ([\-\d\.]+) ([\-\d\.]+) ([\-\d\.]+)\]', src)}
    lines = [BEGIN]
    rail_pts = defaultdict(list)
    n = defaultdict(int)
    for ref in BGAS:
        fp, pins = parts[ref]
        pads = place.footprint(fp)[1]
        x, y, a, side = placed[ref]
        for pin, net in sorted(pins.items()):
            if pin not in pads:
                continue
            px, py = place.world(fp, x, y, a, side, pads[pin])
            if net == 'GND':
                lines.append(f'[EcadViaSpan {px:.4f} {py:.4f} {VIA[0]} {VIA[1]} "GND" "F.Cu" "{PLANE_GND}"]')
                n['gnd'] += 1
            elif is_power(net):
                lines.append(('POWERVIA', px, py, net))
                rail_pts[net].append((px, py))
                n['power'] += 1
            else:
                lines.append(f'[EcadViaSpan {px:.4f} {py:.4f} {VIA[0]} {VIA[1]} "{net}" "F.Cu" "{SIGNAL_LAYER}"]')
                n['signal'] += 1
    # Power regions: every rail gets one region over ALL its pads (BGA balls,
    # decoupling, the regulator that drives it) on In2 or In4. Largest rails
    # choose first, each taking the layer where it overlaps the least, so a rail
    # is not carved into islands by smaller ones. Within a layer the smaller
    # region fills first.
    rails = defaultdict(list)
    for ref, (fp, pins) in parts.items():
        if ref not in placed:
            continue
        pads = place.footprint(fp)[1]
        x, y, a, side = placed[ref]
        for pin, net in pins.items():
            if pin in pads and (is_power(net) or net in BOARD_RAILS):
                rails[net].append(place.world(fp, x, y, a, side, pads[pin]))
    boxes = []
    for net, pts in rails.items():
        if len(pts) < 2:
            continue
        b_ = (min(p[0] for p in pts) - 0.8, min(p[1] for p in pts) - 0.8,
              max(p[0] for p in pts) + 0.8, max(p[1] for p in pts) + 0.8)
        boxes.append(((b_[2] - b_[0]) * (b_[3] - b_[1]), net, b_))
    boxes.sort(reverse=True)
    def ov(a_, b_):
        w = min(a_[2], b_[2]) - max(a_[0], b_[0])
        h = min(a_[3], b_[3]) - max(a_[1], b_[1])
        return w * h if w > 0 and h > 0 else 0.0
    # fixed split: the DDR rails span both DRAMs and half the SoC, and the board
    # rails sit in the H7 / power corners - they share In4; the SoC's own rails,
    # whose vias are short and local, take In2 (spreading every rail over both
    # layers sent power vias through the In3 signal layer: 3x the unconnects)
    assign.clear()
    for area, net, b_ in boxes:
        assign[net] = RAIL_LAYER.get(net) or ('In4.Cu' if net in BOARD_RAILS else 'In2.Cu')
    # Regions per layer as a nearest-pad partition (Voronoi on a 0.4 mm grid,
    # one cell per ball pitch): every cell goes to the rail whose pad is
    # nearest, so rails meet along the lines between their ball groups instead
    # of as overlapping boxes - where the bigger box lost its middle to the
    # smaller one's priority and fell into islands (523 unconnects, most on
    # the core rails). Cells merge into row runs, then runs into rectangles;
    # each rect overhangs its cell by 0.05 mm so same-rail neighbours overlap
    # (one fill) while a foreign overlap is settled by the per-rail priority.
    STEP, OVER = 0.4, 0.05
    by_layer = defaultdict(list)
    for _, net, b_ in boxes:
        by_layer[assign[net]].append((net, b_))
    prio = {net: len(boxes) - k for k, (_, net, _) in enumerate(sorted(boxes))}
    nrect = 0
    for layer, members in sorted(by_layer.items()):
        pads_ = [(px, py, net) for net, _ in members for px, py in rails[net]]
        x0 = min(b_[0] for _, b_ in members); y0 = min(b_[1] for _, b_ in members)
        x1 = max(b_[2] for _, b_ in members); y1 = max(b_[3] for _, b_ in members)
        nx, ny = int((x1 - x0) / STEP) + 1, int((y1 - y0) / STEP) + 1
        boxof = dict(members)
        grid = []
        for j in range(ny):
            row = []
            cy = y0 + (j + 0.5) * STEP
            for i in range(nx):
                cx = x0 + (i + 0.5) * STEP
                best, bn = 1e18, None
                for px, py, net in pads_:
                    b_ = boxof[net]
                    if not (b_[0] <= cx <= b_[2] and b_[1] <= cy <= b_[3]):
                        continue      # never outside the rail's own box
                    d = (px - cx) ** 2 + (py - cy) ** 2
                    if d < best:
                        best, bn = d, net
                row.append(bn)
            grid.append(row)
        # row runs -> rects (merge a run with the identical run in the next row)
        open_ = {}
        def emit(net, i0, i1, j0, j1):
            nonlocal nrect
            lines.append(f'[EcadPourRect "{net}" "{layer}" {x0 + i0 * STEP - OVER:.2f} {y0 + j0 * STEP - OVER:.2f} '
                         f'{x0 + (i1 + 1) * STEP + OVER:.2f} {y0 + (j1 + 1) * STEP + OVER:.2f} {prio[net]}.0]')
            nrect += 1
        for j in range(ny + 1):
            runs = {}
            if j < ny:
                i = 0
                while i < nx:
                    net = grid[j][i]
                    k = i
                    while k + 1 < nx and grid[j][k + 1] == net:
                        k += 1
                    if net:
                        runs[(i, k, net)] = True
                    i = k + 1
            nxt = {}
            for key, j0 in open_.items():
                if key in runs:
                    nxt[key] = j0
                else:
                    emit(key[2], key[0], key[1], j0, j - 1)
            for key in runs:
                nxt.setdefault(key, j)
            open_ = nxt
    print(f'fanout: {nrect} region rects')
    rects = boxes
    lines = [f'[EcadViaSpan {l[1]:.4f} {l[2]:.4f} {VIA[0]} {VIA[1]} "{l[3]}" "F.Cu" "{layer_of(l[3])}"]'
             if isinstance(l, tuple) else l for l in lines]
    lines.append(END)
    block = '\n'.join(lines) + '\n'
    if BEGIN in src:
        src = src[:src.index(BEGIN)] + block + src[src.index(END) + len(END) + 1:]
    else:
        src = src.rstrip('\n') + '\n\n' + block
    open(place.LOON, 'w').write(src)
    split = {l: sum(1 for v in assign.values() if v == l) for l in POWER_LAYERS}
    print(f"fanout: {n['signal']} signal microvias, {n['gnd']} GND, {n['power']} power; rail regions {split}")


if __name__ == '__main__':
    main()
