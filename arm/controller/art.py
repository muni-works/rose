"""Board art for the arm controller -> art.pretty (silkscreen-only footprints).

    rose_egg_mcu    front, inside the MCU's pad ring: only visible on a bare board
    rose_back       back: board name, the fork arm it drives (drawn from its own dimensions), notes
"""
import math, os

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'art.pretty')


def fp(name, body):
    return '\n'.join(['(footprint "%s"' % name, '  (version 20240108)', '  (generator "art.py")', '  (layer "F.Cu")',
                      '  (attr exclude_from_pos_files exclude_from_bom allow_missing_courtyard)',
                      '  (fp_text reference "REF**" (at 0 0) (layer "F.Fab") hide (effects (font (size 0.5 0.5) (thickness 0.08))))',
                      '  (fp_text value "%s" (at 0 0) (layer "F.Fab") hide (effects (font (size 0.5 0.5) (thickness 0.08))))' % name]
                     + body + [')']) + '\n'


def text(s, x, y, size, layer, th=None):
    th = th or max(0.15, size * 0.15)
    mirror = ' (justify mirror)' if layer.startswith('B.') else ''
    return '  (fp_text user "%s" (at %g %g) (layer "%s") (effects (font (size %g %g) (thickness %g))%s))' % (s, x, y, layer, size, size, th, mirror)


def line(a, b, w, layer):
    return '  (fp_line (start %g %g) (end %g %g) (stroke (width %g) (type solid)) (layer "%s"))' % (a[0], a[1], b[0], b[1], w, layer)


def circle(c, r, w, layer):
    return '  (fp_circle (center %g %g) (end %g %g) (stroke (width %g) (type solid)) (fill none) (layer "%s"))' % (c[0], c[1], c[0] + r, c[1], w, layer)


def arm(layer, s=0.04, ox=-25.0, oy=2.0):
    """The fork arm in its render pose, side view, from the v9 dimensions (mm, scaled by s).
    Left of the footprint origin; the text sits to the right."""
    mx = 1
    P = lambda x, z: (round(mx * (ox + x * s), 3), round(oy - z * s, 3))
    body = []
    # foot and turret
    body += [line(P(-115, -120), P(115, -120), 0.3, layer), line(P(-60, -120), P(-60, -50), 0.25, layer), line(P(60, -120), P(60, -50), 0.25, layer)]
    # links: shoulder -> elbow -> wrist -> jaw tips (upper 260, fore 220, wrist+tool 251)
    a1, a2, a3 = math.radians(55), math.radians(55 - 95), math.radians(55 - 95 - 40)
    j = [(0.0, 0.0)]
    for L, a in ((260, a1), (220, a2), (251, a3)):
        x, z = j[-1]
        j.append((x + L * math.cos(a), z + L * math.sin(a)))
    for k, (r_lobe, w) in enumerate(((66, 0.5), (66, 0.5), (45, 0.4))):
        a, b = j[k], j[k + 1]
        # the plate as a tapered stadium: two tangent lines plus the joint lobe
        ang = math.atan2(b[1] - a[1], b[0] - a[0])
        nx, nz = -math.sin(ang), math.cos(ang)
        r2 = 0.6 * r_lobe if k < 2 else 18
        body.append(line(P(a[0] + nx * r_lobe, a[1] + nz * r_lobe), P(b[0] + nx * r2, b[1] + nz * r2), 0.25, layer))
        body.append(line(P(a[0] - nx * r_lobe, a[1] - nz * r_lobe), P(b[0] - nx * r2, b[1] - nz * r2), 0.25, layer))
        c = P(*a)
        body.append(circle(c, r_lobe * s, 0.25, layer))
        body.append(circle(c, 18 * s, 0.4, layer))   # the hub cap
    # jaws
    t = j[-1]
    ang = a3
    for side in (-1, 1):
        nx, nz = -math.sin(ang) * side, math.cos(ang) * side
        body.append(line(P(t[0] + nx * 12, t[1] + nz * 12), P(t[0] + 70 * math.cos(ang) + nx * 6, t[1] + 70 * math.sin(ang) + nz * 6), 0.3, layer))
    return body


def main():
    os.makedirs(OUT, exist_ok=True)
    F, B = 'F.SilkS', 'B.SilkS'
    open(os.path.join(OUT, 'rose_egg_mcu.kicad_mod'), 'w').write(fp('rose_egg_mcu', [
        text('you found the', 0, -1.6, 0.8, F),
        text('secret middle', 0, -0.4, 0.8, F),
        text('hi from cam', 0, 0.8, 0.8, F),
        text('+ claude', 0, 2.0, 0.8, F),
    ]))
    open(os.path.join(OUT, 'rose_back.kicad_mod'), 'w').write(fp('rose_back', arm(B) + [
        text('rose arm controller', 14, -13, 1.6, B, 0.25),
        text('v0.1  2026-10  48 V  4 layers', 14, -11, 0.9, B),
        text('rose fork arm v9', 14, -8.4, 1.2, B, 0.2),
        text('this board drives', 14, -6.4, 0.9, B),
        text('the arm drawn here', 14, -5.0, 0.9, B),
        text('loon -> vcad -> kicad', 14, -2.4, 0.9, B),
        text('48 V: the brake eats regen', 14, 0, 0.9, B),
        text('if found, return to', 14, 2.6, 0.9, B),
        text('the nearest robot', 14, 4.0, 0.9, B),
    ]))
    print('wrote', ', '.join(sorted(os.listdir(OUT))))


if __name__ == '__main__':
    main()
