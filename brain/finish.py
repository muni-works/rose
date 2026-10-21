"""Board finish: matte black solder mask, white silk, ENIG - written into the
board's stackup so KiCad's 3D view, renders and fab outputs all carry it.
Run after vcad writes the board (route.sh does).

    python3 finish.py brain.kicad_pcb

The mask is "matte black" at the fab (order note); KiCad's stackup only knows
"Black", which is what renders.
"""
import re, sys

MASK, SILK, FINISH = 'Black', 'White', 'ENIG'
DIELECTRIC = 0.17       # per gap, prepreg / core alternating (8 layers: 1.6 mm)


def main(path):
    src = open(path).read()
    src = re.sub(r'\n\t\t\(stackup\n.*?\n\t\t\)', '', src, flags=re.S)     # rerun-safe
    cu = re.findall(r'\(\d+ "((?:F|B|In\d+)\.Cu)" signal\)', src)
    rows = [f'(layer "F.SilkS" (type "Top Silk Screen") (color "{SILK}"))',
            '(layer "F.Paste" (type "Top Solder Paste"))',
            f'(layer "F.Mask" (type "Top Solder Mask") (color "{MASK}") (thickness 0.01))']
    for i, c in enumerate(cu):
        rows.append(f'(layer "{c}" (type "copper") (thickness 0.035))')
        if i < len(cu) - 1:
            kind = 'core' if i % 2 else 'prepreg'
            rows.append(f'(layer "dielectric {i + 1}" (type "{kind}") (thickness {DIELECTRIC}) '
                        '(material "FR4") (epsilon_r 4.5) (loss_tangent 0.02))')
    rows += [f'(layer "B.Mask" (type "Bottom Solder Mask") (color "{MASK}") (thickness 0.01))',
             '(layer "B.Paste" (type "Bottom Solder Paste"))',
             f'(layer "B.SilkS" (type "Bottom Silk Screen") (color "{SILK}"))',
             f'(copper_finish "{FINISH}")', '(dielectric_constraints no)']
    block = '\n\t\t(stackup\n' + '\n'.join('\t\t\t' + r for r in rows) + '\n\t\t)'
    src = re.sub(r'\(setup', '(setup' + block, src, count=1)
    open(path, 'w').write(src)
    print(f'finish: {len(cu)}-layer stackup, {MASK.lower()} mask, {SILK.lower()} silk, {FINISH}')


if __name__ == '__main__':
    main(sys.argv[1])
