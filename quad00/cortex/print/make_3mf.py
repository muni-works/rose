"""Bambu Studio 3MF plates for the cortex head print (P1S, PETG). One 3MF per plate: parts pre-oriented and placed on
the 256 mm bed, per-object settings in Metadata/model_settings.config (layer height, walls, infill, supports). Pick the
"Bambu Lab P1S 0.4 nozzle" printer and a PETG filament preset when it opens. Run from this folder after make_print.sh."""
import zipfile, numpy as np, trimesh
from trimesh.transformations import rotation_matrix as R

BED = 256.0

def rot(m, deg, axis):
    m = m.copy(); m.apply_transform(R(np.radians(deg), axis)); return m

def place(parts):
    """lay each part on z = 0 and pack them along x on the bed, centred."""
    out, x = [], 0.0
    for name, m, cfg in parts:
        m = m.copy(); m.apply_translation(-m.bounds[0]); out.append([name, m, cfg, x]); x += m.extents[0] + 8.0
    w = x - 8.0; dep = max(p[1].extents[1] for p in out)
    for p in out:
        p[1].apply_translation([BED / 2 - w / 2 + p[3], BED / 2 - p[1].extents[1] / 2, 0.0])
        assert p[1].bounds[1][0] <= BED and p[1].bounds[1][1] <= BED, f'{p[0]} off the bed'
    return [(n, m, c) for n, m, c, _ in out]

def write(path, parts):
    objs, items, cfgs = [], [], []
    for i, (name, m, cfg) in enumerate(parts, 1):
        v = '\n'.join(f'<vertex x="{a:.4f}" y="{b:.4f}" z="{c:.4f}"/>' for a, b, c in m.vertices)
        t = '\n'.join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in m.faces)
        objs.append(f'<object id="{i}" type="model" name="{name}"><mesh><vertices>\n{v}\n</vertices><triangles>\n{t}\n</triangles></mesh></object>')
        items.append(f'<item objectid="{i}" printable="1"/>')
        md = ''.join(f'<metadata key="{k}" value="{val}"/>' for k, val in [('name', name), ('extruder', '1')] + list(cfg.items()))
        cfgs.append(f'<object id="{i}">{md}<part id="{i}" subtype="normal_part"><metadata key="name" value="{name}"/></part></object>')
    model = ('<?xml version="1.0" encoding="UTF-8"?>\n<model unit="millimeter" xml:lang="en-US" '
             'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
             '<metadata name="Application">rose cortex (vcad)</metadata><resources>' + ''.join(objs) + '</resources><build>' + ''.join(items) + '</build></model>')
    inst = ''.join(f'<model_instance><metadata key="object_id" value="{i}"/><metadata key="instance_id" value="0"/></model_instance>' for i in range(1, len(parts) + 1))
    settings = ('<?xml version="1.0" encoding="UTF-8"?>\n<config>' + ''.join(cfgs) +
                '<plate><metadata key="plater_id" value="1"/><metadata key="plater_name" value=""/><metadata key="locked" value="false"/>' + inst + '</plate></config>')
    ctypes = ('<?xml version="1.0" encoding="UTF-8"?>\n<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
              '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
              '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>'
              '<Default Extension="config" ContentType="text/xml"/></Types>')
    rels = ('<?xml version="1.0" encoding="UTF-8"?>\n<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
    with zipfile.ZipFile(path, 'w', zipfile.ZIP_DEFLATED) as z:
        z.writestr('[Content_Types].xml', ctypes); z.writestr('_rels/.rels', rels)
        z.writestr('3D/3dmodel.model', model); z.writestr('Metadata/model_settings.config', settings)
    print(path, [n for n, _, _ in parts])

PETG = {'layer_height': '0.2', 'wall_loops': '3', 'sparse_infill_density': '15%', 'sparse_infill_pattern': 'gyroid'}
FINE = {'layer_height': '0.12', 'wall_loops': '2', 'sparse_infill_density': '20%', 'enable_support': '0'}
L = lambda f: trimesh.load(f, force='mesh')

# 1. the coupon plate: flat, no supports
write('3mf/cortex-coupons.3mf', place([('coupons', L('coupons-plate.stl'), dict(PETG, enable_support='0', sparse_infill_density='20%'))]))

# 2. full head, one plate per job
tag = 'full'
write(f'3mf/cortex-{tag}-1-tub.3mf', place([('tub', L(f'stl/cortex-{tag}-tub.stl'),           # upright on its open bottom
    dict(PETG, enable_support='1', support_type='tree(auto)', support_on_build_plate_only='1', support_threshold_angle='40'))]))
write(f'3mf/cortex-{tag}-2-floor.3mf', place([('floor', L(f'stl/cortex-{tag}-floor.stl'),      # dock face down
    dict(PETG, wall_loops='4', sparse_infill_density='30%', enable_support='0'))]))
write(f'3mf/cortex-{tag}-3-sled.3mf', place([('sled', L(f'stl/cortex-{tag}-sled.stl'),             # pillars up (too big to share the floor's plate)
    dict(PETG, wall_loops='4', sparse_infill_density='40%', enable_support='0'))]))
E = L(f'stl/cortex-{tag}-ear-grilles.stl')                          # both ears in one STL; split by side, not connectivity
side = lambda sgn: E.submesh([np.where(np.sign(E.triangles_center[:, 1]) == sgn)[0]], append=True)
ears = [side(-1), side(1)]                                          # right (-y) then left (+y)
write(f'3mf/cortex-{tag}-4-grilles.3mf', place([
    ('ear-grille-r', rot(ears[0], -90, [1, 0, 0]), FINE),             # inner face down
    ('ear-grille-l', rot(ears[-1], 90, [1, 0, 0]), FINE),
    ('exhaust-grille', rot(L(f'stl/cortex-{tag}-exhaust-grille.stl'), 90, [0, 1, 0]), FINE)]))
def face_down(m):
    """lay the largest outward (+x) planar face on the bed: the glass face has an 8 deg rake."""
    m = m.copy(); n = m.face_normals; a = m.area_faces; k = n[:, 0] > 0.5
    v = (n[k] * a[k, None]).sum(0); v /= np.linalg.norm(v)
    m.apply_transform(trimesh.geometry.align_vectors(v, [0, 0, -1])); return m
write(f'3mf/cortex-{tag}-5-glass.3mf', place([('glass', face_down(L(f'stl/cortex-{tag}-glass.stl')),   # face down; smoke PETG
    dict(PETG, layer_height='0.12', wall_loops='2', sparse_infill_density='100%', enable_support='0'))]))
