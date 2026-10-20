"""The brain's own symbol and footprint library, generated from pin tables.

    python3 lib/gen_lib.py

Pin tables (lib/*.csv) come from the vendors' datasheets by script or by a
checked transcription; nothing here is typed per pin. Writes:
  lib/brain.kicad_sym                 symbols, used as "brain:<name>" in brain.loon
  brain.pretty/*.kicad_mod            footprints KiCad doesn't ship

| symbol      | from                                        | footprint                              |
|-------------|---------------------------------------------|----------------------------------------|
| RK3588S2    | rk3588s_pins.csv (datasheet Table 2-1)      | brain:Rockchip_FCCSP-1253 (generated)  |
| LPDDR4X_200 | lpddr4x_200_pins.csv (JEDEC 200-ball, x32)  | Package_BGA:BGA-200_10x14.5mm...       |
| RK806-1     | rk806_pins.csv (datasheet pin table)        | Package_DFN_QFN:ArtInChip_QFN-68...    |
| RK860       | RK860 below (datasheet Fig.1-3)             | brain:Rockchip_WLCSP-20 (generated)    |
| SY8089      | ROCK 5C sheet 23 (U2303)                    | Package_TO_SOT_SMD:SOT-23-5            |
| SGM2576     | ROCK 5C sheet 23 (U2304)                    | Package_TO_SOT_SMD:SOT-23-5            |
| eMMC_153    | emmc_153_pins.csv (JEDEC eMMC 5.1)          | Package_BGA:LFBGA-153_11.5x13mm...     |

Repeated pin names (VSS, VDD2, ...) stack: one visible pin, the rest hidden at
the same point, so every ball is still a pin with its own number.
"""
import csv, os, re
from collections import OrderedDict

HERE = os.path.dirname(os.path.abspath(__file__))
BRAIN = os.path.dirname(HERE)
PRETTY = os.path.join(BRAIN, 'brain.pretty')
JEDEC = 'ABCDEFGHJKLMNPRTUVWY'
ROWS = list(JEDEC) + ['A' + r for r in JEDEC] + ['B' + r for r in JEDEC]


def table(name, key='ball'):
    return [(r[key], r['name']) for r in csv.DictReader(open(os.path.join(HERE, name)))]


RK860 = [('A1', 'VSEL'), ('A2', 'EN'), ('A3', 'SCL'), ('A4', 'VOUT'), ('B1', 'SDA'), ('B2', 'GND'), ('B3', 'GND'),
         ('B4', 'AGND'), ('C1', 'GND'), ('C2', 'GND'), ('C3', 'GND'), ('C4', 'GND'), ('D1', 'VIN'), ('D2', 'VIN'),
         ('D3', 'LX'), ('D4', 'LX'), ('E1', 'VIN'), ('E2', 'VIN'), ('E3', 'LX'), ('E4', 'LX')]


SY8089 = [('1', 'EN'), ('2', 'GND'), ('3', 'LX'), ('4', 'VIN'), ('5', 'FB')]       # ROCK 5C sheet 23, U2303
SGM2576 = [('1', 'VOUT'), ('2', 'GND'), ('3', 'OCB'), ('4', 'EN'), ('5', 'IN')]    # ROCK 5C sheet 23, U2304


def stem(name):
    """Stacking key: VSS_12 -> VSS, VDD_CPU_BIG0_3 -> VDD_CPU_BIG0; a unique signal stays itself."""
    m = re.match(r'^((?:A?VSS|VDD|AVDD|VCC|PMUIO\d|VCCIO\d)[A-Z0-9_]*?)_(\d+)$', name)
    return m.group(1) if m else name


def ptype(name):
    if re.match(r'^(PLDO\d|NLDO\d)$', name):
        return 'power_out'
    if re.match(r'^(VOUT\d*|FB\d*|SW\d*|LX|VDC|VCCIO|OCB)$', name):
        return 'passive'
    if 'VSS' in name:
        return 'power_in'
    if name == 'RESERVED':
        return 'passive'
    if re.match(r'^(A?VSS|GND|AGND|EPAD|VSSQ)', name) or re.match(r'^(VDD|VCC|AVDD|PMUIO|VIN$|IN$)', name):
        return 'power_in'
    if 'GPIO' not in name and re.search(r'(VDD|VCC|AVCC|PMUIO|EMMCIO_1V8|OSC_1V8|PMU_0V75)', name):
        return 'power_in'
    if name in ('DNU', 'NC') or name.startswith('NC_'):
        return 'no_connect'
    return 'bidirectional'


def symbol(sym, pins, fp, value=None, ref='U'):
    """One-unit box: signals down the left (then right), stacked rails on the right."""
    groups = OrderedDict()
    for num, name in pins:
        key = stem(name) if ptype(name) != 'no_connect' else f'{name}_{num}'    # no-connect balls stay apart
        groups.setdefault(key, []).append((num, name))
    sigs = sorted((k for k, v in groups.items() if ptype(v[0][1]) != 'power_in'), key=natural)
    rails = sorted((k for k, v in groups.items() if ptype(v[0][1]) == 'power_in'), key=lambda k: ('SS' in k or 'GND' in k, k))
    half = (len(sigs) + 1) // 2 if len(sigs) > 24 else len(sigs)
    left, right = sigs[:half], sigs[half:] + [None] + rails
    step, plen = 2.54, 5.08
    rows = max(len(left), len(right))
    longest = max(len(k) for k in groups)
    w = max(20.32, round((longest * 1.27 * 2 + 10) / 2.54) * 2.54)
    h = (rows + 1) * step
    out = []

    def pin(num, name, x, y, ang, et, hide):
        h_ = ' (hide yes)' if hide else ''
        out.append(f'\t\t\t(pin {et} line (at {x:.2f} {y:.2f} {ang}) (length {plen}){h_} '
                   f'(name "{name}" (effects (font (size 1.27 1.27)))) (number "{num}" (effects (font (size 1.27 1.27)))))')

    top = h / 2 - step
    for side, col, x, ang in ((0, left, -w / 2 - plen, 0), (1, right, w / 2 + plen, 180)):
        for i, k in enumerate(col):
            if k is None:
                continue
            for j, (num, name) in enumerate(groups[k]):
                pin(num, k if len(groups[k]) > 1 else name, x, top - i * step, ang, ptype(name), j > 0)
    value = value or sym
    head = [f'\t(symbol "{sym}"', '\t\t(exclude_from_sim no) (in_bom yes) (on_board yes)',
            f'\t\t(property "Reference" "{ref}" (at 0 {h / 2 + 1.27:.2f} 0) (effects (font (size 1.27 1.27))))',
            f'\t\t(property "Value" "{value}" (at 0 {-h / 2 - 1.27:.2f} 0) (effects (font (size 1.27 1.27))))',
            f'\t\t(property "Footprint" "{fp}" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))',
            f'\t\t(symbol "{sym}_0_1"',
            f'\t\t\t(rectangle (start {-w / 2:.2f} {h / 2:.2f}) (end {w / 2:.2f} {-h / 2:.2f}) (stroke (width 0.254) (type default)) (fill (type background)))',
            '\t\t)', f'\t\t(symbol "{sym}_1_1"']
    return head + out + ['\t\t)', '\t)']


def natural(s):
    return [int(t) if t.isdigit() else t for t in re.split(r'(\d+)', s)]


def bga_footprint(name, descr, balls, rows, pitch_x, pitch_y, ncols, nrows, pad, body_w, body_h, mask=0.05):
    def pos(ball):
        m = re.match(r'([A-Z]+)(\d+)$', ball)
        r, c = rows.index(m.group(1)), int(m.group(2)) - 1
        return round((c - (ncols - 1) / 2) * pitch_x, 4), round((r - (nrows - 1) / 2) * pitch_y, 4)
    bw, bh = body_w / 2, body_h / 2
    out = [f'(footprint "{name}"', '\t(version 20241229)', '\t(generator "gen_lib.py")', '\t(layer "F.Cu")',
           f'\t(descr "{descr}")', '\t(attr smd)',
           f'\t(property "Reference" "REF**" (at 0 {-bh - 1.2:.2f} 0) (layer "F.SilkS") (effects (font (size 0.8 0.8) (thickness 0.12))))',
           f'\t(property "Value" "{name}" (at 0 {bh + 1.2:.2f} 0) (layer "F.Fab") (effects (font (size 0.8 0.8) (thickness 0.12))))',
           f'\t(fp_rect (start {-bw} {-bh}) (end {bw} {bh}) (stroke (width 0.1) (type solid)) (layer "F.Fab") (fill no))',
           f'\t(fp_rect (start {-bw - 0.25} {-bh - 0.25}) (end {bw + 0.25} {bh + 0.25}) (stroke (width 0.05) (type solid)) (layer "F.CrtYd") (fill no))',
           f'\t(fp_circle (center {-bw - 0.45} {-bh - 0.45}) (end {-bw - 0.33} {-bh - 0.45}) (stroke (width 0.25) (type solid)) (layer "F.SilkS") (fill solid))']
    for b in balls:
        x, y = pos(b)
        out.append(f'\t(pad "{b}" smd circle (at {x} {y}) (size {pad} {pad}) (layers "F.Cu" "F.Paste" "F.Mask") (solder_mask_margin {mask}))')
    out.append(')')
    os.makedirs(PRETTY, exist_ok=True)
    open(os.path.join(PRETTY, name + '.kicad_mod'), 'w').write('\n'.join(out) + '\n')
    return f'brain:{name}'


def main():
    syms = []
    soc = table('rk3588s_pins.csv')
    assert len(soc) == 1253
    fp = bga_footprint('Rockchip_FCCSP-1253_17x17mm_P0.4mm',
                       'Rockchip RK3588S/RK3588S2 FCCSP1253L, 17x17 mm, 42x42 @ 0.4 mm, NSMD 0.23 mm (datasheet Rev 1.6 Fig.2-5)',
                       [b for b, _ in soc], ROWS, 0.4, 0.4, 42, 42, 0.23, 17.0, 17.0)
    syms += symbol('RK3588S2', soc, fp)
    dram = table('lpddr4x_200_pins.csv')
    assert len(dram) == 200
    syms += symbol('LPDDR4X_200', dram, 'Package_BGA:BGA-200_10x14.5mm_Layout12x22_P0.8x0.65mm', ref='U')
    pmic = table('rk806_pins.csv', key='pin')
    assert len(pmic) == 69
    syms += symbol('RK806-1', pmic, 'Package_DFN_QFN:ArtInChip_QFN-68-1EP_7x7mm_P0.35mm_EP5.49x5.49mm_ThermalVias')
    fp = bga_footprint('Rockchip_WLCSP-20_1.65x2.05mm_P0.4mm', 'Rockchip RK860 WLCSP20, 4x5 @ 0.4 mm (datasheet Rev 1.6 Fig.6-1)',
                       [b for b, _ in RK860], list('ABCDE'), 0.4, 0.4, 4, 5, 0.2, 1.65, 2.05, mask=0.04)
    syms += symbol('RK860', RK860, fp)
    syms += symbol('SY8089', SY8089, 'Package_TO_SOT_SMD:SOT-23-5')
    syms += symbol('SGM2576', SGM2576, 'Package_TO_SOT_SMD:SOT-23-5')
    if os.path.exists(os.path.join(HERE, 'emmc_153_pins.csv')):
        emmc = table('emmc_153_pins.csv')
        syms += symbol('eMMC_153', emmc, 'Package_BGA:LFBGA-153_11.5x13mm_Layout14x14_P0.5mm')
    # board-to-board (module <-> carrier): Hirose DF40C 100-pin, 0.4 mm
    syms += symbol('DF40_100', [(str(i), f'P{i}') for i in range(1, 101)],
                   'Connector_Hirose_DF40:Hirose_DF40C-100DS-0.4V_2x50_P0.4mm', ref='J')
    lib = ['(kicad_symbol_lib', '\t(version 20241209)', '\t(generator "gen_lib.py")'] + syms + [')']
    open(os.path.join(HERE, 'brain.kicad_sym'), 'w').write('\n'.join(lib) + '\n')
    print('ok:', ', '.join(re.findall(r'^\t\(symbol "([^"]+)"$', '\n'.join(lib), re.M)))


if __name__ == '__main__':
    main()
