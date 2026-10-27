"""CNC file for the inlaid face acrylic -> ../fab/node/node-acrylic.dxf (R12, mm). Run: python3 node_acrylic.py

Seen from the FRONT (as the head looks at you), origin at the sheet centre, X right (the head's -y), Y up (z).
Layers:
  CUT          101.6 x 152.4 portrait outline, top corners R24 / bottom R10: round a stock 4 x 6 in sheet's corners
  MASK_SCREEN  the 4in panel's active area (71.8 square), left CLEAR when the back is masked black
  MASK_CAMS    the two Ø8 camera windows, left CLEAR
The mask goes on the BACK: to engrave it as a paint guide with the sheet face-down, mirror X in CAM.
"""
import math, os

W, H, R, RB = 101.6, 152.4, 24.0, 10.0   # a whole 4 x 6 in sheet, portrait; top corners R24, bottom R10 (node_skull.SHEET_RT/RB)
ZC = 90.9                           # sheet centre height in the head
PANEL_Z, ACTIVE = 95.0, 71.8        # panel centre (node.loon fz), active area of the 720x720 4in IPS
CAM_Y, CAM_Z, CAM_D = 30.0, 150.0, 8.0

def rrect(cx, cy, w, h, r, layer, rb=None):
    rb = r if rb is None else rb
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2
    e = []
    if r > 0:
        e += [('LINE', layer, (x0 + rb, y0, x1 - rb, y0)), ('LINE', layer, (x1, y0 + rb, x1, y1 - r)),
              ('LINE', layer, (x1 - r, y1, x0 + r, y1)), ('LINE', layer, (x0, y1 - r, x0, y0 + rb))]
        for (ax, ay, a0, rr) in ((x1 - rb, y0 + rb, 270, rb), (x1 - r, y1 - r, 0, r), (x0 + r, y1 - r, 90, r), (x0 + rb, y0 + rb, 180, rb)):
            e.append(('ARC', layer, (ax, ay, rr, a0, a0 + 90)))
    else:
        e += [('LINE', layer, p) for p in ((x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0))]
    return e

ents = rrect(0, 0, W, H, R, 'CUT', RB)
ents += rrect(0, PANEL_Z - ZC, ACTIVE, ACTIVE, 0, 'MASK_SCREEN')
ents += [('CIRCLE', 'MASK_CAMS', (sx * CAM_Y, CAM_Z - ZC, CAM_D / 2)) for sx in (1, -1)]

out = ['0', 'SECTION', '2', 'HEADER', '9', '$INSUNITS', '70', '4', '0', 'ENDSEC', '0', 'SECTION', '2', 'ENTITIES']
for kind, layer, v in ents:
    out += ['0', kind, '8', layer]
    if kind == 'LINE':
        out += ['10', '%.4f' % v[0], '20', '%.4f' % v[1], '11', '%.4f' % v[2], '21', '%.4f' % v[3]]
    elif kind == 'ARC':
        out += ['10', '%.4f' % v[0], '20', '%.4f' % v[1], '40', '%.4f' % v[2], '50', '%.4f' % v[3], '51', '%.4f' % v[4]]
    else:
        out += ['10', '%.4f' % v[0], '20', '%.4f' % v[1], '40', '%.4f' % v[2]]
out += ['0', 'ENDSEC', '0', 'EOF']
path = os.path.join(os.path.dirname(__file__) or '.', '..', 'fab', 'node', 'node-acrylic.dxf')
open(path, 'w').write('\n'.join(out) + '\n')
print('node-acrylic.dxf: %d entities; sheet %g x %g R%g/R%g; screen window %g sq at y %+.1f; cams Ø%g at (±%g, %+.1f)'
      % (len(ents), W, H, R, RB, ACTIVE, PANEL_Z - ZC, CAM_D, CAM_Y, CAM_Z - ZC))
