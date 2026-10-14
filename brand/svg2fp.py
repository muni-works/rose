"""Stroked SVG artwork -> KiCad silkscreen footprints.

    python3 brand/svg2fp.py <in.svg> <out.pretty> <name> [width_mm ...]

Line art drawn with a pen (stroked paths with round caps - like the muni logo)
maps one-to-one onto silkscreen: each path becomes fp_line segments at the
SVG stroke width (scaled), and filled circles become a filled fp_circle. The
footprint is centred on its origin and has no pads, so it can sit anywhere.
One footprint per width and per side: <name>_<w>mm (F.SilkS) and
<name>_<w>mm_back (B.SilkS, mirrored so it reads from the back).

Handles M/L/H/V/C/Z (absolute and relative) and <circle>. Other SVG features
(fills, text, transforms) are ignored - outline them first.

KiCad: add the .pretty folder as a footprint library. vcad: put its parent
directory on VCAD_KICAD_FOOTPRINTS and place with [EcadArt ref "muni:muni_logo_12mm" x y rot].
"""
import math, os, re, sys
import xml.etree.ElementTree as ET

MIN_LINE = 0.15  # mm, fab minimum silkscreen line


def flatten(d, steps=14):
    toks = re.findall(r'[MmLlHhVvCcZz]|-?\d*\.?\d+(?:e-?\d+)?', d)
    polys, cur, pos, start, cmd, i = [], [], (0.0, 0.0), (0.0, 0.0), None, 0
    def num():
        nonlocal i
        v = float(toks[i]); i += 1
        return v
    while i < len(toks):
        if re.match(r'[A-Za-z]', toks[i]):
            cmd = toks[i]; i += 1
        rel = cmd.islower()
        c = cmd.upper()
        ox, oy = pos if rel else (0.0, 0.0)
        if c == 'M':
            if cur:
                polys.append(cur)
            pos = (ox + num(), oy + num()); start = pos; cur = [pos]
            cmd = 'l' if rel else 'L'
        elif c == 'L':
            pos = (ox + num(), oy + num()); cur.append(pos)
        elif c == 'H':
            pos = ((pos[0] if rel else 0.0) + num(), pos[1]); cur.append(pos)
        elif c == 'V':
            pos = (pos[0], (pos[1] if rel else 0.0) + num()); cur.append(pos)
        elif c == 'C':
            p1 = (ox + num(), oy + num()); p2 = (ox + num(), oy + num()); p3 = (ox + num(), oy + num())
            p0 = pos
            for k in range(1, steps + 1):
                t = k / steps; u = 1 - t
                cur.append((u**3 * p0[0] + 3 * u * u * t * p1[0] + 3 * u * t * t * p2[0] + t**3 * p3[0],
                            u**3 * p0[1] + 3 * u * u * t * p1[1] + 3 * u * t * t * p2[1] + t**3 * p3[1]))
            pos = p3
        elif c == 'Z':
            cur.append(start); pos = start
        else:
            raise ValueError('unsupported path command %s' % cmd)
    if cur:
        polys.append(cur)
    return polys


def load(svg):
    root = ET.parse(svg).getroot()
    strokes, dots = [], []
    def walk(el, width):
        w = float(el.get('stroke-width', width or 1))
        tag = el.tag.split('}')[-1]
        if tag == 'path':
            for poly in flatten(el.get('d')):
                strokes.append((poly, w))
        elif tag == 'circle':
            dots.append(((float(el.get('cx')), float(el.get('cy'))), float(el.get('r'))))
        for ch in el:
            walk(ch, w)
    walk(root, None)
    return strokes, dots


def footprint(name, strokes, dots, width_mm, back):
    xs = [p[0] for poly, w in strokes for p in poly] + [c[0] for c, r in dots]
    ys = [p[1] for poly, w in strokes for p in poly] + [c[1] for c, r in dots]
    pad = max([w for _, w in strokes] + [0]) / 2
    x0, x1, y0, y1 = min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad
    s = width_mm / (x1 - x0)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    mx = -1 if back else 1
    layer = 'B.SilkS' if back else 'F.SilkS'
    T = lambda p: (round(mx * (p[0] - cx) * s, 4), round((p[1] - cy) * s, 4))
    out = ['(footprint "%s"' % name, '  (version 20240108)', '  (generator "svg2fp")', '  (layer "F.Cu")',
           '  (descr "%s, %.1f mm wide, silkscreen art")' % (name, width_mm), '  (attr exclude_from_pos_files exclude_from_bom allow_missing_courtyard)',
           '  (fp_text reference "REF**" (at 0 0) (layer "F.Fab") hide (effects (font (size 0.5 0.5) (thickness 0.08))))',
           '  (fp_text value "%s" (at 0 0) (layer "F.Fab") hide (effects (font (size 0.5 0.5) (thickness 0.08))))' % name]
    for poly, w in strokes:
        sw = max(w * s, MIN_LINE)
        for a, b in zip(poly, poly[1:]):
            if a != b:
                pa, pb = T(a), T(b)
                out.append('  (fp_line (start %g %g) (end %g %g) (stroke (width %g) (type solid)) (layer "%s"))' % (pa[0], pa[1], pb[0], pb[1], sw, layer))
    for c, r in dots:
        pc = T(c)
        out.append('  (fp_circle (center %g %g) (end %g %g) (stroke (width %g) (type solid)) (fill solid) (layer "%s"))' % (pc[0], pc[1], pc[0] + r * s / 2, pc[1], r * s, layer))
    out.append(')')
    return '\n'.join(out) + '\n', (x1 - x0) * s, (y1 - y0) * s


def main():
    svg, pretty, name = sys.argv[1:4]
    widths = [float(w) for w in sys.argv[4:]] or [8.0, 12.0, 20.0]
    strokes, dots = load(svg)
    os.makedirs(pretty, exist_ok=True)
    for w in widths:
        for back in (False, True):
            fp = '%s_%gmm%s' % (name, w, '_back' if back else '')
            text, ww, hh = footprint(fp, strokes, dots, w, back)
            open(os.path.join(pretty, fp + '.kicad_mod'), 'w').write(text)
            if not back:
                print('%s: %.1f x %.1f mm' % (fp, ww, hh))


if __name__ == '__main__':
    main()
