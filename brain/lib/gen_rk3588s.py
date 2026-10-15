"""RK3588S2 footprint and symbol from lib/rk3588s_pins.csv (the datasheet's ball list).

    python3 lib/gen_rk3588s.py

Writes:
  brain.pretty/Rockchip_FCCSP-1253_17x17mm_P0.4mm.kicad_mod
  lib/brain.kicad_sym        (symbol "RK3588S2"; loon refers to it as "brain:RK3588S2")

Package (datasheet Fig.2-5): body 17 x 17, 42 x 42 grid at e = 0.4, D1 = E1 = 16.4,
ball b = 0.26 nom. Pads are NSMD 0.23 mm with a 0.05 mm mask opening each side.
A1 is top-left seen from the top; rows A..BB go down, columns 1..42 go right.

The symbol is one unit. Ground and supply balls that share a rail name
(VSS_1..VSS_n, VDD_LOGIC_1..) stack on one visible pin and the rest are hidden at
the same point, so the schematic stays readable and every ball is still a pin.
"""
import csv, os, re
from collections import OrderedDict

HERE = os.path.dirname(os.path.abspath(__file__))
BRAIN = os.path.dirname(HERE)
L = 'ABCDEFGHJKLMNPRTUVWY'
ROWS = list(L) + ['A' + r for r in L] + ['B' + r for r in L]
PITCH, N, PAD, MASK = 0.4, 42, 0.23, 0.05
FP_NAME = 'Rockchip_FCCSP-1253_17x17mm_P0.4mm'

balls = [(r['ball'], r['name']) for r in csv.DictReader(open(os.path.join(HERE, 'rk3588s_pins.csv')))]
assert len(balls) == 1253


def pos(ball):
    m = re.match(r'([A-Z]+)(\d+)$', ball)
    row, col = ROWS.index(m.group(1)), int(m.group(2)) - 1
    return round((col - (N - 1) / 2) * PITCH, 3), round((row - (N - 1) / 2) * PITCH, 3)


def footprint():
    out = [f'(footprint "{FP_NAME}"', '\t(version 20241229)', '\t(generator "gen_rk3588s.py")', '\t(layer "F.Cu")',
           '\t(descr "Rockchip RK3588S/RK3588S2 FCCSP1253L, 17x17 mm, 42x42 @ 0.4 mm, NSMD 0.23 mm pads (datasheet Rev 1.6 Fig.2-5)")',
           '\t(attr smd)',
           '\t(property "Reference" "REF**" (at 0 -9.6 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))',
           f'\t(property "Value" "{FP_NAME}" (at 0 9.6 0) (layer "F.Fab") (effects (font (size 1 1) (thickness 0.15))))',
           '\t(fp_rect (start -8.5 -8.5) (end 8.5 8.5) (stroke (width 0.1) (type solid)) (layer "F.Fab") (fill no))',
           '\t(fp_rect (start -8.75 -8.75) (end 8.75 8.75) (stroke (width 0.05) (type solid)) (layer "F.CrtYd") (fill no))',
           '\t(fp_poly (pts (xy -8.62 -7.6) (xy -8.62 -8.62) (xy -7.6 -8.62)) (stroke (width 0.12) (type solid)) (layer "F.SilkS") (fill no))',
           '\t(fp_circle (center -9.0 -9.0) (end -8.85 -9.0) (stroke (width 0.3) (type solid)) (layer "F.SilkS") (fill solid))']
    for ball, _ in balls:
        x, y = pos(ball)
        out.append(f'\t(pad "{ball}" smd circle (at {x} {y}) (size {PAD} {PAD}) (layers "F.Cu" "F.Paste" "F.Mask") '
                   f'(solder_mask_margin {MASK}))')
    out.append(')')
    d = os.path.join(BRAIN, 'brain.pretty')
    os.makedirs(d, exist_ok=True)
    open(os.path.join(d, FP_NAME + '.kicad_mod'), 'w').write('\n'.join(out) + '\n')


def rail(name):
    """VSS_12 -> VSS; VDD_CPU_BIG0_3 -> VDD_CPU_BIG0. Signal names stay whole."""
    m = re.match(r'^((?:A?VSS|VDD|AVDD|VCC|DVDD|AVCC|PMUIO\d|VCCIO\d)[A-Z0-9_]*?)_(\d+)$', name)
    return m.group(1) if m else None


def symbol():
    groups = OrderedDict()                   # rail -> [balls]
    sigs = []
    for ball, name in balls:
        r = rail(name)
        if r:
            groups.setdefault(r, []).append(ball)
        else:
            sigs.append((name, ball))
    sigs.sort(key=lambda s: s[0])
    # left column: signals A..M (DDR first), right column: the rest; power on top, ground at the bottom
    pwr = [(k, v) for k, v in groups.items() if 'VSS' not in k]
    gnd = [(k, v) for k, v in groups.items() if 'VSS' in k]
    half = (len(sigs) + 1) // 2
    left, right = sigs[:half], sigs[half:]
    rows = max(len(left), len(right) + len(pwr) + len(gnd) + 2)
    step = 2.54
    h = (rows + 2) * step
    w = 60 * step
    pins = []

    def pin(num, name, x, y, ang, etype, hide=False):
        hide_s = ' (hide yes)' if hide else ''
        pins.append(f'\t\t\t(pin {etype} line (at {x:.2f} {y:.2f} {ang}) (length 5.08){hide_s} '
                    f'(name "{name}" (effects (font (size 1.27 1.27)))) (number "{num}" (effects (font (size 1.27 1.27)))))')

    top = h / 2 - step
    for i, (name, ball) in enumerate(left):
        pin(ball, name, -w / 2 - 5.08, top - i * step, 0, 'bidirectional')
    y = top
    for name, ball in right:
        pin(ball, name, w / 2 + 5.08, y, 180, 'bidirectional')
        y -= step
    y -= step
    for k, bs in pwr + gnd:
        et = 'power_in'
        for j, b in enumerate(bs):
            pin(b, k, w / 2 + 5.08, y, 180, et, hide=j > 0)
        y -= step
    sym = ['\t(symbol "RK3588S2"', '\t\t(exclude_from_sim no) (in_bom yes) (on_board yes)',
           f'\t\t(property "Reference" "U" (at 0 {h / 2 + 2.54:.2f} 0) (effects (font (size 1.27 1.27))))',
           f'\t\t(property "Value" "RK3588S2" (at 0 {-h / 2 - 2.54:.2f} 0) (effects (font (size 1.27 1.27))))',
           f'\t\t(property "Footprint" "brain:{FP_NAME}" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))',
           '\t\t(property "Datasheet" "brain/reference/Rockchip_RK3588S_Datasheet_V1.6-20240821.pdf" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))',
           '\t\t(symbol "RK3588S2_0_1"',
           f'\t\t\t(rectangle (start {-w / 2:.2f} {h / 2:.2f}) (end {w / 2:.2f} {-h / 2:.2f}) (stroke (width 0.254) (type default)) (fill (type background)))',
           '\t\t)', '\t\t(symbol "RK3588S2_1_1"'] + pins + ['\t\t)', '\t)']
    return sym, len(groups), len(sigs)


if __name__ == '__main__':
    footprint()
    sym, nrails, nsig = symbol()
    lib = ['(kicad_symbol_lib', '\t(version 20241209)', '\t(generator "gen_rk3588s.py")'] + sym + [')']
    open(os.path.join(HERE, 'brain.kicad_sym'), 'w').write('\n'.join(lib) + '\n')
    print(f'footprint: {len(balls)} pads; symbol: {nsig} signal pins, {nrails} rails')
