"""BGA fanout: a laser via in every used ball of the SoC, the DRAMs and the eMMC,
and an In4 power region per SoC rail. Run after place.py; vcad routes from these.

    python3 fanout.py

Via-in-pad, the HDI way (the 0.4 mm SoC leaves no room between balls):
  signal ball -> microvia F.Cu-In1.Cu, routed out on In1 (vcad's via-in-pad)
  GND ball    -> blind F.Cu-In2.Cu into the GND plane
  power ball  -> stacked microvias F.Cu-In4.Cu into its rail's In4 region
Laser vias 0.25 / 0.10 mm, filled and capped (VIPPO) at the fab.
"""
import math, os, re
from collections import defaultdict

import place

VIA = (0.25, 0.10)
BGAS = ('U_SOC', 'U_DRAM0', 'U_DRAM1', 'U_EMMC')
BEGIN, END = ';; ---- fanout (fanout.py)', ';; ---- end fanout'
PLANE_GND = 'In2.Cu'
POWER_LAYER = 'In4.Cu'


def is_power(net):
    return bool(re.match(r'^(VDD|VCC|AVDD|VDDQ)', net))


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
                lines.append(f'[EcadViaSpan {px:.4f} {py:.4f} {VIA[0]} {VIA[1]} "{net}" "F.Cu" "{POWER_LAYER}"]')
                rail_pts[net].append((px, py))
                n['power'] += 1
            else:
                lines.append(f'[EcadViaSpan {px:.4f} {py:.4f} {VIA[0]} {VIA[1]} "{net}" "F.Cu" "In1.Cu"]')
                n['signal'] += 1
    # one In4 region per rail over its balls (plus a margin); small regions fill first
    rects = []
    for net, pts in rail_pts.items():
        if len(pts) < 2:
            continue
        x0, y0 = min(p[0] for p in pts) - 0.6, min(p[1] for p in pts) - 0.6
        x1, y1 = max(p[0] for p in pts) + 0.6, max(p[1] for p in pts) + 0.6
        rects.append(((x1 - x0) * (y1 - y0), net, x0, y0, x1, y1))
    rects.sort()
    for k, (_, net, x0, y0, x1, y1) in enumerate(rects):
        lines.append(f'[EcadPourRect "{net}" "{POWER_LAYER}" {x0:.2f} {y0:.2f} {x1:.2f} {y1:.2f} {len(rects) - k}.0]')
    lines.append(END)
    block = '\n'.join(lines) + '\n'
    if BEGIN in src:
        src = src[:src.index(BEGIN)] + block + src[src.index(END) + len(END) + 1:]
    else:
        src = src.rstrip('\n') + '\n\n' + block
    open(place.LOON, 'w').write(src)
    print(f"fanout: {n['signal']} signal microvias, {n['gnd']} GND, {n['power']} power; {len(rects)} In4 rail regions")


if __name__ == '__main__':
    main()
