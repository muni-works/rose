"""mini v1 fab package: the frozen mini humanoid (k1_design + k1_structure) -> files a shop can cut or print.

    cd body && python fab/make.py        (needs numpy trimesh manifold3d shapely ezdxf; ~2 min)

  plates/parts/<part>.dxf     one per distinct plate, 1/8" (3.2 mm) 5052, layer CUT, mm, true size
  plates/sheet-N.dxf          the parts nested on 390 x 390 sheets (Anolex 4040 bed), layer CUT; labels on NOTE
  plates/cutlist.csv          part, qty, size, sheet
  shells/<part>[-a|-b].stl    printed skins: the solid design forms hollowed to the 2.0 mm wall the plates are clipped
                              to, cleared round everything inside (+0.4 mm) and round other bodies' shells (+0.5 mm),
                              split in two where they wrap the frame (each half slides off clear of the frame)
  shells/shells.csv           part, body, material, halves, split axis, print size, fits the P1S
  cortex/glass.dxf            the face glass, flat
The cortex head's parts come from cortex/export.sh (cortex.loon -> fab/cortex/*.step + *.stl).
"""
import os, sys, csv
import numpy as np, trimesh, ezdxf
from manifold3d import Manifold, Mesh
from shapely.geometry import Polygon
from shapely import affinity
from shapely.ops import unary_union

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import k1_design as KD, k1_structure as KS, rom_mini as R

SHEET, MARGIN, GAP = 390.0, 6.0, 8.0          # Anolex 4040 usable bed; edge margin; 1/8" end mill + web between parts
WALL, CLEAR, RUN = 2.0, 0.4, 0.5              # printed skin; to the frame inside; to a shell on another body
BED = 250.0                                   # Bambu P1S 256 mm cube, 3 mm margin each side
SKIN = ('cover-', 'trim-', 'accent-', 'puck-')
NOT_PLATE = ('rs', 'elec', 'node', 'cable', 'cortex', 'dock', 'motor') + SKIN + ('sole',)


def toM(m):
    return Manifold(Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32)))


def toT(M):
    g = M.to_mesh()
    return trimesh.Trimesh(np.asarray(g.vert_properties)[:, :3], np.asarray(g.tri_verts), process=False)


# ------------------------------------------------------------------------------------------------ plates
def outline(m):
    """a plate mesh -> its cut profile (shapely, mm) in the plate's own plane, and its thickness."""
    lo, hi = m.bounds
    ax = int(np.argmin(np.abs(hi - lo - KS.T)))               # the axis that is plate-thick
    u, v = [i for i in range(3) if i != ax]
    perm = np.eye(3)[[u, v, ax]]
    sec = toM(m).transform(np.hstack([perm, np.zeros((3, 1))])).slice((lo[ax] + hi[ax]) / 2)
    polys = [np.asarray(p) for p in sec.to_polygons() if len(p) >= 3]
    area = lambda p: 0.5 * np.sum(p[:, 0] * np.roll(p[:, 1], -1) - np.roll(p[:, 0], -1) * p[:, 1])
    outer = [Polygon(p) for p in polys if area(p) > 0]
    holes = [Polygon(p) for p in polys if area(p) <= 0]
    prof = unary_union(outer).difference(unary_union(holes)) if holes else unary_union(outer)
    return prof.simplify(0.02), hi[ax] - lo[ax]


def flat(prof):
    """lay a profile flat: long side of its min-area rectangle along x, at the origin."""
    r = prof.minimum_rotated_rectangle
    c = np.asarray(r.exterior.coords)[:4]
    e = c[1] - c[0] if np.linalg.norm(c[1] - c[0]) >= np.linalg.norm(c[2] - c[1]) else c[2] - c[1]
    p = affinity.rotate(prof, -np.degrees(np.arctan2(e[1], e[0])), origin='centroid')
    x0, y0, _, _ = p.bounds
    return affinity.translate(p, -x0, -y0)


def key(p):
    """congruence key: a part and its mirror image cut the same (through-cut, no countersinks) -- flip it over."""
    n = len(p.geoms) if p.geom_type == 'MultiPolygon' else 1
    holes = sum(len(g.interiors) for g in (p.geoms if n > 1 else [p]))
    w, h = p.bounds[2] - p.bounds[0], p.bounds[3] - p.bounds[1]
    return (round(p.area / 4) * 4, round(p.length / 2) * 2, holes, round(max(w, h)), round(min(w, h)))


def rings(p):
    for g in (p.geoms if p.geom_type == 'MultiPolygon' else [p]):
        yield list(g.exterior.coords)
        for i in g.interiors:
            yield list(i.coords)


def dxf(path, items):
    doc = ezdxf.new('R2000', setup=True)
    doc.units = ezdxf.units.MM
    doc.layers.add('CUT', color=1)
    doc.layers.add('NOTE', color=3)
    msp = doc.modelspace()
    for p, label, (dx, dy) in items:
        for r in rings(affinity.translate(p, dx, dy)):
            msp.add_lwpolyline(r, close=True, dxfattribs={'layer': 'CUT'})
        if label:
            c = affinity.translate(p, dx, dy).representative_point()
            msp.add_text(label, height=3.0, dxfattribs={'layer': 'NOTE'}).set_placement((c.x, c.y))
    doc.saveas(path)


def nest(parts):
    """bounding boxes (+ GAP) max-rects packed onto SHEET squares inside the margin; parts may turn 90 deg."""
    import rectpack
    q = 10.0                                                    # rectpack wants integers: 0.1 mm units
    pk = rectpack.newPacker(rotation=True, pack_algo=rectpack.MaxRectsBssf, sort_algo=rectpack.SORT_AREA)
    for i, (_, p) in enumerate(parts):
        pk.add_rect(int((p.bounds[2] + GAP) * q) + 1, int((p.bounds[3] + GAP) * q) + 1, rid=i)
    side = int((SHEET - 2 * MARGIN + GAP) * q)
    for _ in parts:
        pk.add_bin(side, side)
    pk.pack()
    sheets = {}
    for b, x, y, w, h, i in pk.rect_list():
        name, p = parts[i]
        if (w > h) != (p.bounds[2] + GAP > p.bounds[3] + GAP):  # packed turned
            p = flat(affinity.rotate(p, 90))
            if (p.bounds[2] > p.bounds[3]) != (w > h):
                p = affinity.translate(affinity.rotate(p, 90, origin=(0, 0)), p.bounds[3], 0)
        sheets.setdefault(b, []).append((name, p, (MARGIN + x / q, MARGIN + y / q)))
    assert sum(len(v) for v in sheets.values()) == len(parts), 'a part did not fit a sheet'
    return [sheets[b] for b in sorted(sheets)]


def plates(parts, raw, out):
    os.makedirs(os.path.join(out, 'parts'), exist_ok=True)
    groups = {}
    for n, m in parts:
        if n.startswith(NOT_PLATE):
            continue
        if not any(2.5 < e < 4.0 for e in m.extents):
            continue
        prof, t = outline(m)
        if n in raw and (prof.is_empty or np.sort(m.extents)[0] < 2.5):   # the shell clip ate it: cut the designed plate
            print(f'  !! {n}: clipped to {np.sort(m.extents)[0]:.1f} mm by its shell; cut from the unclipped profile')
            prof, t = outline(raw[n])
        if prof.is_empty:
            continue
        p = flat(prof)
        k = key(p)
        groups.setdefault(k, []).append((n, p, t))
    rows, pieces = [], []
    for k, g in sorted(groups.items(), key=lambda kv: kv[1][0][0]):
        name = g[0][0].replace('.', '_')
        p = g[0][1]
        w, h = p.bounds[2], p.bounds[3]
        dxf(os.path.join(out, 'parts', f'{name}.dxf'), [(p, None, (0, 0))])
        rows.append([name, len(g), f'{w:.1f} x {h:.1f}', ' '.join(n for n, _, _ in g), round(g[0][2], 2)])
        pieces += [(f'{name}' + (f' ({i + 1}/{len(g)})' if len(g) > 1 else ''), p) for i in range(len(g))]
        if max(w, h) > SHEET - 2 * MARGIN:
            print(f'  !! {name} is {max(w, h):.0f} mm: over the 4040 bed')
    sheets = nest(pieces)
    where = {}
    for i, s in enumerate(sheets):
        dxf(os.path.join(out, f'sheet-{i + 1}.dxf'), [(p, n, xy) for n, p, xy in s])
        for n, _, _ in s:
            where.setdefault(n.split(' (')[0], set()).add(i + 1)
    with open(os.path.join(out, 'cutlist.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['part', 'qty', 'size mm', 'is (model names; a mirror pair is one part flipped)', 'thickness mm', 'sheet'])
        for r in rows:
            w.writerow(r + [' '.join(str(s) for s in sorted(where[r[0]]))])
    area = sum(p.area for _, p in pieces)
    print(f'plates: {len(rows)} distinct, {len(pieces)} pieces, {len(sheets)} sheets of {SHEET:.0f} x {SHEET:.0f} '
          f'({area / (len(sheets) * SHEET ** 2) * 100:.0f}% used)')
    return rows, len(sheets)


# ------------------------------------------------------------------------------------------------ shells
def sweep(M, d, L):
    """everything a convex hull of M passes through moving along d by L (the path a half clears as it slides off)."""
    return Manifold.batch_hull([M, M.translate(list(np.asarray(d) * L))])


def shells(parts, out):
    os.makedirs(out, exist_ok=True)
    body = {n: R.body_of(n) for n, _ in parts}
    M = {n: toM(m) for n, m in parts}
    bb = {n: m.bounds for n, m in parts}
    hit = lambda a, b: np.all(bb[a][0] <= bb[b][1]) and np.all(bb[b][0] <= bb[a][1])
    cube = Manifold.cube([2 * CLEAR] * 3, True)
    rows = []
    for n, m in parts:
        if not n.startswith(SKIN):
            continue
        mat = 'PETG satin black' if n.startswith('cover-') else 'PETG gunmetal grey'
        inside = [k for k, _ in parts if k != n and not k.startswith(SKIN + ('sole',)) and hit(n, k)]
        others = [k for k, _ in parts if k != n and k.startswith(SKIN + ('sole',)) and body[k] != body[n] and hit(n, k)]
        S = M[n]
        thick = np.sort(m.extents)[0] > 4 * WALL + 4
        if thick:                                               # hollow: the wall the plates were clipped inside of
            S = S - S.minkowski_difference(Manifold.sphere(WALL, 12))
        for k in inside:                                        # clear the frame, motors, boards, runs inside it
            S = S - Manifold.batch_hull([M[k]]).minkowski_sum(cube)
        for k in others:                                        # a running gap to the next body's shell
            S = S - M[k].minkowski_sum(Manifold.cube([2 * RUN] * 3, True))
        best = None
        if thick and inside:                                    # split in two so it goes on over the frame
            c = m.bounds.mean(0)
            L = float(np.max(m.extents)) + 10
            for ax in range(3):
                d = np.eye(3)[ax]
                a, b = S.split_by_plane(list(d), float(c[ax]))
                for k in inside:                                # each half slides off its own way clear of the frame
                    H = Manifold.batch_hull([M[k]]).minkowski_sum(cube)
                    a, b = a - sweep(H, -d, L), b - sweep(H, d, L)
                lost = S.volume() - a.volume() - b.volume()
                if best is None or lost < best[0]:
                    best = (lost, ax, a, b)
        halves = [('', S)] if best is None else [('-a', best[2]), ('-b', best[3])]
        if halves and max(float(np.max(toT(H).extents)) for _, H in halves if not H.is_empty()) > BED:
            ax = int(np.argmax(m.extents))                      # over the P1S (the chest): halve again across its widest axis
            c = float(m.bounds.mean(0)[ax])
            halves = [(t + s_, P) for t, H in halves for s_, P in zip('lr', H.split_by_plane(list(np.eye(3)[ax]), c))]
        sizes = []
        for tag, H in halves:
            if H.is_empty():
                continue
            t = toT(H)
            t.export(os.path.join(out, f'{n.replace(".", "_")}{tag}.stl'))
            sizes.append(np.sort(t.extents)[::-1])
        big = max((s[0] for s in sizes), default=0.0)
        rows.append([n.replace('.', '_'), body[n], mat, len(sizes), 'xyz'[best[1]] if best else '-',
                     ' / '.join(' x '.join(f'{v:.0f}' for v in s) for s in sizes),
                     'EMPTY: the motor fills the form -- needs a designed cover' if not sizes else 'yes' if big <= BED else 'NO'])
        print(f'  {n:22s} {len(sizes)} part(s) {rows[-1][5]}{"" if big <= BED else "  OVER THE P1S"}', flush=True)
    for n, m in parts:                                          # the soles: TPU, printed solid
        if n.startswith('sole'):
            m.export(os.path.join(out, f'{n.replace(".", "_")}.stl'))
            s = np.sort(m.extents)[::-1]
            rows.append([n.replace('.', '_'), body[n], 'TPU 95A', 1, '-', ' x '.join(f'{v:.0f}' for v in s), 'yes' if s[0] <= BED else 'NO'])
    with open(os.path.join(out, 'shells.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['part', 'body', 'material', 'pieces', 'split axis', 'piece sizes mm', 'fits P1S'])
        w.writerows(rows)
    return rows


def glass(out):
    """the cortex face glass, flat (v1: laser-cut 3 mm smoked acrylic). cortex.loon glass-sec at d = 0: half sizes
    sw = hw - rc - 3, sh = 0.33 hh, corner 0.55 min(sw, sh)."""
    from shapely.geometry import box
    hw, hd, hh = KS.CX_SIZE['hw'], (KS.CX_SIZE['x_face'] - KS.CX_SIZE['x_back']) / 2, KS.CX_SIZE['hh']
    sw, sh = hw - 0.30 * hd - 3.0, 0.33 * hh
    r = 0.55 * min(sw, sh)
    p = box(-sw + r, -sh + r, sw - r, sh - r).buffer(r, 32)
    dxf(os.path.join(out, 'glass.dxf'), [(affinity.translate(p, sw, sh), None, (0, 0))])
    print(f'glass: {2 * sw:.1f} x {2 * sh:.1f} mm, R{r:.1f}')


def main(what=('plates', 'shells', 'glass')):
    KS.use('mini')
    D = KD.design()
    P = D + KD.structure(D)
    if 'plates' in what:
        plates(P, dict(KS.structure()), os.path.join(HERE, 'plates'))
    if 'shells' in what:
        shells(P, os.path.join(HERE, 'shells'))
    if 'glass' in what:
        glass(os.path.join(HERE, '..', '..', 'head', 'fab', 'cortex'))


if __name__ == '__main__':
    main(tuple(sys.argv[1:]) or ('plates', 'shells', 'glass'))
