"""Placeholder 3D models for footprints whose model file KiCad doesn't ship.

    make_models.py board.kicad_pcb        (KiCad's Python; route.sh runs it)

KiCad renders a footprint with a missing model as an empty outline, which is
what the XT60PW, the XT30PWs and the Hanrun RJ45 looked like. For each such
footprint this writes a coloured box (VRML, in 3d/) and points the footprint
at it. Size and colour come from BODIES (real body dimensions where known);
anything else gets a grey box over its courtyard.
"""
import glob, os, sys
import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '3d')
KICAD_3D = sorted(glob.glob('/Applications/KiCad/KiCad.app/Contents/SharedSupport/3dmodels'))

NYLON = (0.93, 0.76, 0.18)        # XT connectors: yellow PA
STEEL = (0.75, 0.76, 0.78)        # RJ45 shield
# footprint name -> (body x, y, z mm, centre offset x, y mm, colour); x/y in footprint axes
BODIES = {
    'AMASS_XT60PW-M_1x02_P7.20mm_Horizontal': (15.8, 15.5, 8.2, 0.0, -2.0, NYLON),
    'AMASS_XT30PW-M_1x02_P2.50mm_Horizontal': (10.2, 11.0, 5.4, 0.0, -1.0, NYLON),
    'RJ45_Hanrun_HR911105A_Horizontal': (16.0, 21.3, 13.5, 0.0, 0.0, STEEL),
}


def vrml_box(path, sx, sy, sz, cx, cy, rgb):
    # KiCad reads VRML in 0.1 in units: 1 unit = 2.54 mm
    k = 1 / 2.54
    hx, hy = sx / 2 * k, sy / 2 * k
    x0, x1, y0, y1, z0, z1 = (cx * k - hx), (cx * k + hx), (-cy * k - hy), (-cy * k + hy), 0.0, sz * k
    pts = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
           (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = '0,3,2,1,-1, 4,5,6,7,-1, 0,1,5,4,-1, 1,2,6,5,-1, 2,3,7,6,-1, 3,0,4,7,-1'
    open(path, 'w').write('''#VRML V2.0 utf8
Shape {
  appearance Appearance { material Material { diffuseColor %g %g %g specularColor 0.3 0.3 0.3 shininess 0.4 } }
  geometry IndexedFaceSet { coord Coordinate { point [ %s ] } coordIndex [ %s ] }
}
''' % (rgb + (', '.join('%.4f %.4f %.4f' % p for p in pts), faces)))


def resolve(fn):
    for base in KICAD_3D:
        fn = fn.replace('${KICAD9_3DMODEL_DIR}', base).replace('${KICAD8_3DMODEL_DIR}', base)
    return fn.replace('${KIPRJMOD}', HERE)


board = pcbnew.LoadBoard(sys.argv[1])
os.makedirs(OUT, exist_ok=True)
for name, (sx, sy, sz, cx, cy, rgb) in BODIES.items():      # always current with the table
    vrml_box(os.path.join(OUT, name + '.wrl'), sx, sy, sz, cx, cy, rgb)
made = set()
for fp in board.GetFootprints():
    name = fp.GetFPID().GetLibItemName().wx_str()
    models = fp.Models()
    have = [m for m in models if os.path.exists(resolve(m.m_Filename)) or
            os.path.exists(os.path.splitext(resolve(m.m_Filename))[0] + '.wrl')]
    if have or name not in BODIES:
        continue
    made.add(name)
    models.clear()
    m = pcbnew.FP_3DMODEL()
    m.m_Filename = '${KIPRJMOD}/3d/%s.wrl' % name
    models.append(m)
board.Save(sys.argv[1])
print('placeholder models: %s' % (', '.join(sorted(made)) or 'none needed'))
