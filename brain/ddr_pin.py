"""Route the DDR buses alone, then pin them: their copper goes into brain.loon as
EcadTrace / EcadViaSpan (Manual copper, never ripped) and the full route works
around it. Under a 0.4 mm SoC the DDR lanes lose to whatever got there first
when everything routes together; alone, they get the board to themselves.

    python3 ddr_pin.py            (then SKIP_PLACE=1 ./route.sh)
    EFFORT=2 python3 ddr_pin.py

Rerun after placement or fanout changes; the block is replaced each time.
"""
import os, re, subprocess, sys

import place

BEGIN, END = ';; ---- ddr pinned (ddr_pin.py)', ';; ---- end ddr pinned'
DDR = re.compile(r'^D[01][A-D]_')
OUT = 'ddr_only.kicad_pcb'


def strip(src):
    if BEGIN in src:
        src = src[:src.index(BEGIN)] + src[src.index(END) + len(END) + 1:]
    return src


def main():
    src = strip(open(place.LOON).read())
    open(place.LOON, 'w').write(src)
    nets = sorted(set(n for n in re.findall(r'"(D[01][A-D]_[A-Za-z0-9_]+)"', src)))
    env = dict(os.environ, VCAD_KICAD_SYMBOLS=os.path.join(place.HERE, 'lib'),
               VCAD_KICAD_FOOTPRINTS=f"{place.HERE}:{os.path.join(place.HERE, '..', 'arm', 'controller')}")
    cmd = [os.environ.get('VCAD', 'vcad'), 'route', '--microvia', '0.25,0.10', '--effort', os.environ.get('EFFORT', '2')]
    for n in nets:
        cmd += ['--only', n]
    cmd += [place.LOON, OUT]
    r = subprocess.run(cmd, cwd=place.HERE, env=env, capture_output=True, text=True)
    print('\n'.join(l for l in r.stdout.splitlines() if l.startswith('Routed')))
    if r.returncode:
        sys.exit(r.stderr[-2000:])
    pcb = open(os.path.join(place.HERE, OUT)).read()
    names = dict(re.findall(r'\(net (\d+) "([^"]*)"\)', pcb))
    have = set(re.findall(r'\[EcadViaSpan ([\-\d\.]+) ([\-\d\.]+) [\d\.]+ [\d\.]+ "([^"]+)"', src))
    lines = [BEGIN]
    nt = nv = 0
    for blk in re.findall(r'\(segment\s+(.*?)\n\t\)', pcb, re.S):
        net = names.get(re.search(r'\(net (\d+)\)', blk).group(1), '')
        if not DDR.match(net):
            continue
        (x1, y1), (x2, y2) = re.findall(r'\((?:start|end) ([\-\d\.]+) ([\-\d\.]+)\)', blk)
        w = re.search(r'\(width ([\d\.]+)\)', blk).group(1)
        layer = re.search(r'\(layer "([^"]+)"\)', blk).group(1)
        lines.append(f'[EcadTrace {x1} {y1} {x2} {y2} {w} "{layer}" "{net}"]')
        nt += 1
    for blk in re.findall(r'\(via(?: blind| micro)?\s+(.*?)\n\t\)', pcb, re.S):
        net = names.get(re.search(r'\(net (\d+)\)', blk).group(1), '')
        if not DDR.match(net):
            continue
        x, y = re.search(r'\(at ([\-\d\.]+) ([\-\d\.]+)\)', blk).groups()
        if any(abs(float(x) - float(a)) < 1e-3 and abs(float(y) - float(b)) < 1e-3 and n == net for a, b, n in have):
            continue        # the fanout's own ball via
        d = re.search(r'\(size ([\d\.]+)\)', blk).group(1)
        dr = re.search(r'\(drill ([\d\.]+)\)', blk).group(1)
        la, lb = re.search(r'\(layers "([^"]+)" "([^"]+)"\)', blk).groups()
        lines.append(f'[EcadViaSpan {x} {y} {d} {dr} "{net}" "{la}" "{lb}"]')
        nv += 1
    lines.append(END)
    open(place.LOON, 'w').write(src.rstrip('\n') + '\n\n' + '\n'.join(lines) + '\n')
    print(f'ddr_pin: pinned {nt} traces, {nv} vias on {len(nets)} DDR nets')


if __name__ == '__main__':
    main()
