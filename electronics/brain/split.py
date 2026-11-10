"""Split the brain into a compute module and a carrier.

    python3 split.py            (report: which parts go where, what crosses)

The module is the hard part - the RK3588S2, its LPDDR4X, eMMC and the PMIC
with its core regulators - on a small HDI board; the carrier is everything
else on a cheap 4-layer board. They meet at two Hirose DF40C 100-pin 0.4 mm
board-to-board connectors (the CM4 family): DP header on the module, DS socket
on the carrier.

Partition: the seed chips are the module; a passive joins the side whose chips
its signal nets reach (ground doesn't vote), ties going to the module so a
regulator's input caps sit at the regulator. Every net with pins on both sides
crosses the connector.
"""
import re, sys
from collections import defaultdict

import place

SEEDS = {'U_SOC', 'U_DRAM0', 'U_DRAM1', 'U_EMMC', 'U_PMIC', 'U4', 'U5', 'U6', 'U7', 'U8', 'Y1'}
PASSIVE = re.compile(r'^(R|C|L|D|FB)\d')
GND = {'GND'}
B2B_PINS = 200


def partition(parts):
    side = {r: ('M' if r in SEEDS else None) for r in parts}
    for r in parts:
        if side[r] is None and not PASSIVE.match(r):
            side[r] = 'C'
    net_parts = defaultdict(set)
    for r, (_, pins) in parts.items():
        for n in pins.values():
            net_parts[n].add(r)
    # passives settle by vote, repeated until stable (a cap on a regulator's
    # switch node follows the regulator, then the filter cap after it, ...)
    for _ in range(20):
        changed = False
        for r in parts:
            if not PASSIVE.match(r):
                continue
            votes = {'M': 0, 'C': 0}
            for n in set(parts[r][1].values()) - GND:
                chips = [o for o in net_parts[n] if not PASSIVE.match(o)]
                if len(chips) == 1 and side[chips[0]]:
                    # a chip's private net (switch node, feedback divider):
                    # its passives belong where the chip is
                    votes[side[chips[0]]] += 100
                    continue
                for o in net_parts[n] - {r}:
                    if side[o] in votes:
                        votes[side[o]] += 1 if PASSIVE.match(o) else 4
            s = 'M' if votes['M'] >= votes['C'] and votes['M'] else ('C' if votes['C'] else side[r])
            if s and s != side[r]:
                side[r] = s
                changed = True
        if not changed:
            break
    for r in parts:
        side[r] = side[r] or 'C'
    crossing = sorted(n for n, rs in net_parts.items()
                      if n not in GND and not n.startswith('~nc~') and {side[r] for r in rs} == {'M', 'C'})
    return side, crossing


def pinmap(crossing):
    """Two 100-pin maps (pin -> net). Connector 1 carries the fast pairs -
    CSI clocks / lanes and USB - each pair on adjacent pins with ground pins
    between pairs; connector 2 carries power (spread so the current shares
    pins: ~0.3 A each on DF40) and the slow signals, ground in between."""
    pair = lambda n: re.match(r'^(CAM\d_(?:C|D\d)|USB_OTG)(P|N|_DP|_DM)$', n)
    fast = sorted(n for n in crossing if (n.startswith('CAM') and pair(n)) or n in ('USB_OTG_DP', 'USB_OTG_DM'))
    pairs = defaultdict(list)
    for n in fast:
        m = pair(n)
        pairs[m.group(1).rstrip('_')].append(n)
    c1, k = {}, 1
    for base in sorted(pairs):
        p_ = sorted(pairs[base], key=lambda n: (not n.endswith(('P', '_DP')), n))
        c1[k], c1[k + 1] = 'GND', 'GND'              # odd/even rows: ground beside the pair
        c1[k + 2], c1[k + 4] = p_[0], p_[1]          # pair on adjacent pins of one row
        c1[k + 3], c1[k + 5] = 'GND', 'GND'
        k += 6
    power = {'VCC_SYSIN': 20, 'VCC_3V3_S0': 4, 'USB_VBUS': 2}
    slow = sorted(n for n in crossing if n not in fast and n not in power)
    c2, k = {}, 1
    for n, cnt in power.items():
        if n in crossing:
            for _ in range(cnt):
                c2[k] = n
                k += 1
    k += 2                                            # a ground gap after power
    for n in slow:
        c2[k] = n
        k += 2                                        # every other pin: ground between
    for c in (c1, c2):
        for i in range(1, 101):
            c.setdefault(i, 'GND')
        if max(c) > 100:
            sys.exit(f'pin map overflow: {max(c)} pins')
    return c1, c2


def write(src, parts, side):
    """module.loon / carrier.loon: the original header, each side's parts and
    comment lines in order, plus its half of the B2B pair."""
    _, crossing = partition(parts)
    c1, c2 = pinmap(crossing)
    head = src[:src.index(';; ---- power in')]
    body = src[src.index(';; ---- power in'):src.index(';; ---- placement (place.py)')]
    blocks = re.split(r'(?m)^(?=\[part |;;)', body)
    for s_, name, fp, what in (('M', 'module', 'Connector_Hirose_DF40:Hirose_DF40C-100DP-0.4V_2x50-1MP_P0.4mm', 'DF40C-100DP header'),
                              ('C', 'carrier', 'Connector_Hirose_DF40:Hirose_DF40C-100DS-0.4V_2x50_P0.4mm', 'DF40C-100DS socket')):
        out = [head.replace('rose brain v0', f'rose brain {name} v0 (split.py from brain.loon)')]
        for b in blocks:
            m = re.match(r'\[part "([^"]+)"', b)
            if m and side.get(m.group(1), 'C') != s_:
                continue
            if m is None and not b.strip():
                continue
            out.append(b)
        out.append(f';; ---- board-to-board: 2x Hirose {what}, pin map in b2b_pinmap.csv\n')
        # power that crosses the connector is driven from the other board as
        # far as ERC can see: flag it on both halves (and the module's ground)
        flagged = set(re.findall(r'"power:PWR_FLAG" "PWR_FLAG" ""\n  #\["1" "([^"]+)"', ''.join(out)))
        for n in ['GND'] + [n for n in crossing if n.startswith(('VCC', 'USB_VBUS'))]:
            if n not in flagged:
                out.append(f'[part "PF_{name[0].upper()}_{n}" "power:PWR_FLAG" "PWR_FLAG" ""\n  #["1" "{n}"]]\n')
        for j, cmap in (('J_B2B1', c1), ('J_B2B2', c2)):
            pins = ' '.join(f'"{i}" "{cmap[i]}"' for i in range(1, 101))
            out.append(f'[part "{j}" "brain:DF40_100" "{what}" "{fp}"\n  #[{pins}]]\n')
        open(f'{name}.loon', 'w').write(''.join(out))
    with open('b2b_pinmap.csv', 'w') as f:
        f.write('connector,pin,net\n')
        for j, cmap in (('J_B2B1', c1), ('J_B2B2', c2)):
            for i in range(1, 101):
                f.write(f'{j},{i},{cmap[i]}\n')


def main():
    src, parts = place.parse_loon()
    side, crossing = partition(parts)
    m = sorted(r for r in parts if side[r] == 'M')
    c = sorted(r for r in parts if side[r] == 'C')
    print(f'module {len(m)} parts, carrier {len(c)} parts, {len(crossing)} nets cross')
    groups = defaultdict(list)
    for n in crossing:
        groups[re.sub(r'\d+', '#', n).split('_')[0]].append(n)
    for k, v in sorted(groups.items(), key=lambda kv: -len(kv[1])):
        print(f'  {len(v):3d} {k}: {", ".join(v[:8])}{" ..." if len(v) > 8 else ""}')
    gnd = B2B_PINS - len(crossing)
    print(f'B2B: {len(crossing)} signal/power pins + {gnd} ground of {B2B_PINS}')
    if gnd < 50:
        sys.exit('too few ground pins: the fast pairs need a ground between them')
    if '--write' in sys.argv:
        all_parts = {r: (f, p) for r, (f, p) in parts.items()}
        # parts without a footprint (power flags) follow the net they flag
        for r in re.findall(r'\[part "([^"]+)" "power:PWR_FLAG"', src):
            n = re.search(r'\[part "' + re.escape(r) + r'"[^\n]*\n  #\["1" "([^"]+)"', src).group(1)
            side[r] = 'M' if n in ('VCC_SYSIN',) or all(side.get(o) == 'M' for o, (_, p) in parts.items() if n in p.values()) else 'C'
        write(src, all_parts, side)
        c1, c2 = pinmap(crossing)
        print(f'wrote module.loon, carrier.loon, b2b_pinmap.csv '
              f'(J_B2B1 {sum(v != "GND" for v in c1.values())} signals, J_B2B2 {sum(v != "GND" for v in c2.values())} power/signals)')


if __name__ == '__main__':
    main()
