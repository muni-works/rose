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
PLANE_RAILS = re.compile(r'^(VDD_CPU_(BIG0|BIG1|LIT)_S0|VDD_(GPU|NPU|VDENC)_S0|VDD_LOGIC_S0|VDD_DDR_S0|VDD2_DDR_S3|VDDQ_DDR_S0)$')


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
    # Ball-field webbing: at 0.4 mm pitch a via sits in every ball, and the
    # plane's clearance to the foreign ones leaves no copper between them - the
    # plane under the SoC breaks into islands and its ball vias land on nothing
    # (most of the remaining unconnects). Join each net's ball vias to their
    # orthogonal / diagonal same-net neighbours on the plane layer instead: a
    # spanning tree of 0.1 mm links, which the plane outside the field then
    # picks up. Diagonals clear the two foreign vias they pass by 0.11 mm.
    web = defaultdict(list)
    for l in lines:
        if isinstance(l, tuple):
            web[l[3]].append((l[1], l[2]))
    for l in lines:
        if isinstance(l, str) and l.startswith('[EcadViaSpan') and '"GND" "F.Cu" "In1.Cu"' in l:
            x_, y_ = map(float, l.split()[1:3])
            web['GND'].append((x_, y_))
    webs = []
    for net, pts in web.items():
        par = list(range(len(pts)))
        def find(i):
            while par[i] != i:
                par[i] = par[par[i]]
                i = par[i]
            return i
        cell = {(round(px / 0.4), round(py / 0.4)): k for k, (px, py) in enumerate(pts)}
        for (cx, cy), k in sorted(cell.items()):
            for dx, dy in ((1, 0), (0, 1), (1, 1), (1, -1)):
                m = cell.get((cx + dx, cy + dy))
                if m is None or find(k) == find(m):
                    continue
                par[find(k)] = find(m)
                webs.append((net, pts[k], pts[m]))
    n['web'] = len(webs)
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
        # Join each rail's patches: nearest-pad cells of one rail can come out
        # as separate patches with other rails between them (each patch then a
        # separate pour island). Cut a one-cell corridor from every stray patch
        # to the rail's biggest one along the path that crosses the fewest
        # foreign cells (0-1 BFS), smallest rails first so big ones route last
        # around them. A corridor never cuts the last cell joining another rail.
        import heapq
        def comps(net):
            seen, out = set(), []
            for j in range(ny):
                for i in range(nx):
                    if grid[j][i] != net or (i, j) in seen:
                        continue
                    st, c = [(i, j)], []
                    seen.add((i, j))
                    while st:
                        a_, b_ = st.pop()
                        c.append((a_, b_))
                        for da, db in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                            q = (a_ + da, b_ + db)
                            if 0 <= q[0] < nx and 0 <= q[1] < ny and q not in seen and grid[q[1]][q[0]] == net:
                                seen.add(q)
                                st.append(q)
                    out.append(c)
            return sorted(out, key=len, reverse=True)
        joined = 0
        for net, _ in [m for _ in range(4) for m in sorted(members, key=lambda m: -len(rails[m[0]]))]:
            cs = comps(net)
            for stray in cs[1:]:
                main = set(comps(net)[0])
                if stray[0] in main:
                    continue
                if set(stray) & main:
                    continue
                dist = {c: 0 for c in stray}
                prev = {}
                pq = [(0, c) for c in stray]
                heapq.heapify(pq)
                end = None
                while pq:
                    d, c = heapq.heappop(pq)
                    if c in main:
                        end = c
                        break
                    if d > dist.get(c, 1e9):
                        continue
                    for da, db in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        q = (c[0] + da, c[1] + db)
                        if not (0 <= q[0] < nx and 0 <= q[1] < ny):
                            continue
                        g = grid[q[1]][q[0]]
                        w = 0 if g == net else (1 if g is None else 4)
                        if d + w < dist.get(q, 1e9):
                            dist[q] = d + w
                            prev[q] = c
                            heapq.heappush(pq, (d + w, q))
                c = end
                while c is not None and c in prev:
                    grid[c[1]][c[0]] = net
                    c = prev[c]
                joined += 1
        print(f'fanout: {layer}: joined {joined} stray rail patches; still split: ' +
              str({net: len(comps(net)) for net, _ in members if len(comps(net)) > 1}))
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
    for net, a_, b_ in webs:
        lay = PLANE_GND if net == 'GND' else layer_of(net)
        lines.append(f'[EcadTrace {a_[0]:.4f} {a_[1]:.4f} {b_[0]:.4f} {b_[1]:.4f} 0.1 "{lay}" "{net}"]')
    lines.append(END)
    block = '\n'.join(lines) + '\n'
    if BEGIN in src:
        src = src[:src.index(BEGIN)] + block + src[src.index(END) + len(END) + 1:]
    else:
        src = src.rstrip('\n') + '\n\n' + block
    open(place.LOON, 'w').write(src)
    split = {l: sum(1 for v in assign.values() if v == l) for l in POWER_LAYERS}
    print(f"fanout: {n['signal']} signal microvias, {n['gnd']} GND, {n['power']} power, {n['web']} ball-field links; rail regions {split}")


if __name__ == '__main__':
    main()
