"""Board finish in the stackup: matte black mask, muni orange silk, ENIG.

    set_finish.py board.kicad_pcb        (plain Python; route.sh runs it)

Writes a (stackup ...) into the board's (setup ...). The fab reads the finish
from it; KiCad's 3D view and `pcb render` draw with its colours.

Silk is muni orange (#ff6600, brand/muni-logo.svg). Orange silk isn't every
fab's stock option: confirm with the fab, or set SILK=White.
"""
import os, re, sys

SILK = os.environ.get('SILK', '#FF6600FF')

p = sys.argv[1]
s = open(p).read()
copper = re.findall(r'\(\d+ "((?:F|B|In\d+)\.Cu)" signal', s)
order = ['F.Cu'] + sorted([c for c in copper if c.startswith('In')], key=lambda c: int(c[2:-3])) + ['B.Cu']
L = ['\t\t(stackup',
     '\t\t\t(layer "F.SilkS" (type "Top Silk Screen") (color "%s"))' % SILK,
     '\t\t\t(layer "F.Paste" (type "Top Solder Paste"))',
     '\t\t\t(layer "F.Mask" (type "Top Solder Mask") (color "Black") (thickness 0.01))']
diel = (1.6 - 0.035 * len(order) - 0.02) / (len(order) - 1)
for i, c in enumerate(order):
    L.append('\t\t\t(layer "%s" (type "copper") (thickness 0.035))' % c)
    if i + 1 < len(order):
        kind = 'core' if i % 2 == 1 else 'prepreg'
        L.append('\t\t\t(layer "dielectric %d" (type "%s") (thickness %.4f) (material "FR4") (epsilon_r 4.5) (loss_tangent 0.02))' % (i + 1, kind, diel))
L += ['\t\t\t(layer "B.Mask" (type "Bottom Solder Mask") (color "Black") (thickness 0.01))',
      '\t\t\t(layer "B.Paste" (type "Bottom Solder Paste"))',
      '\t\t\t(layer "B.SilkS" (type "Bottom Silk Screen") (color "%s"))' % SILK,
      '\t\t\t(copper_finish "ENIG")',
      '\t\t\t(dielectric_constraints no)',
      '\t\t)']
s = re.sub(r'\n\t\t\(stackup\b.*?\n\t\t\)', '', s, flags=re.S)       # replace any earlier one
s = s.replace('\t(setup\n', '\t(setup\n' + '\n'.join(L) + '\n', 1)
open(p, 'w').write(s)
print('finish: black mask, %s silk, ENIG (%d copper layers)' % (SILK, len(order)))
