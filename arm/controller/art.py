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


HERE = os.path.dirname(os.path.abspath(__file__))
SHIELD_W, SHIELD_H = 24.0, 20.0     # mm, the MCU can's frame (outside)
STRIP, SEG, GAP = 0.8, 4.0, 1.2     # frame strip width, pad length, gap for traces


def shield_frame(name, w, h):
    """A shield-can frame: segmented GND solder strips round a w x h box, gaps for
    traces to leave on F.Cu, the can lid as its 3D model. Pads are all pad 1."""
    body = []
    def strip(x0, y0, x1, y1, horizontal):
        length = (x1 - x0) if horizontal else (y1 - y0)
        n = max(1, int((length + GAP) // (SEG + GAP)))
        seg = (length - (n - 1) * GAP) / n
        for k in range(n):
            a = k * (seg + GAP)
            if horizontal:
                cx, cy, sx, sy = x0 + a + seg / 2, (y0 + y1) / 2, seg, STRIP
            else:
                cx, cy, sx, sy = (x0 + x1) / 2, y0 + a + seg / 2, STRIP, seg
            body.append('  (pad "1" smd rect (at %.3f %.3f) (size %.3f %.3f) (layers "F.Cu" "F.Paste" "F.Mask"))' % (cx, cy, sx, sy))
    hw, hh = w / 2, h / 2
    strip(-hw, -hh, hw, -hh + STRIP, True)
    strip(-hw, hh - STRIP, hw, hh, True)
    strip(-hw, -hh + STRIP + GAP, -hw + STRIP, hh - STRIP - GAP, False)
    strip(hw - STRIP, -hh + STRIP + GAP, hw, hh - STRIP - GAP, False)
    # no courtyard: KiCad reads a ring as malformed and a solid one collides with
    # everything inside; place.py blocks the strips itself
    body.append('  (fp_rect (start %g %g) (end %g %g) (stroke (width 0.1) (type solid)) (fill none) (layer "F.Fab"))' % (-hw, -hh, hw, hh))
    body.append('  (model "${KIPRJMOD}/3d/%s.wrl" (offset (xyz 0 0 0)) (scale (xyz 1 1 1)) (rotate (xyz 0 0 0)))' % name)
    return '\n'.join(['(footprint "%s"' % name, '  (version 20240108)', '  (generator "art.py")', '  (layer "F.Cu")',
                      '  (descr "shield-can frame %g x %g mm: GND strips with trace gaps")' % (w, h), '  (attr smd allow_missing_courtyard)',
                      '  (fp_text reference "REF**" (at 0 %g) (layer "F.Fab") (effects (font (size 0.8 0.8) (thickness 0.12))))' % (-hh - 1),
                      '  (fp_text value "%s" (at 0 0) (layer "F.Fab") hide (effects (font (size 0.8 0.8) (thickness 0.12))))' % name]
                     + body + [')']) + '\n'


def vrml_lid(path, w, h, z):
    """The can: a brushed-steel lid w x h at height z, with walls (VRML, 0.1 in units)."""
    k = 1 / 2.54
    hx, hy, zz, t = w / 2 * k, h / 2 * k, z * k, 0.15 * k
    def box(x0, y0, z0, x1, y1, z1):
        pts = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
        return ('Shape { appearance Appearance { material Material { diffuseColor 0.78 0.79 0.80 specularColor 0.9 0.9 0.9 shininess 0.7 } }\n'
                ' geometry IndexedFaceSet { coord Coordinate { point [ %s ] } coordIndex [ 0,3,2,1,-1, 4,5,6,7,-1, 0,1,5,4,-1, 1,2,6,5,-1, 2,3,7,6,-1, 3,0,4,7,-1 ] } }\n'
                % ', '.join('%.4f %.4f %.4f' % p for p in pts))
    shapes = [box(-hx, -hy, zz - t, hx, hy, zz),
              box(-hx, -hy, 0, hx, -hy + t, zz), box(-hx, hy - t, 0, hx, hy, zz),
              box(-hx, -hy, 0, -hx + t, hy, zz), box(hx - t, -hy, 0, hx, hy, zz)]
    os.makedirs(os.path.dirname(path), exist_ok=True)
    open(path, 'w').write('#VRML V2.0 utf8\n' + ''.join(shapes))


def edge_ring(name, w, h, r, inset=0.35, width=1.0):
    """A gold rim: mask opened in a band along the board edge over the GND flood
    (front and back), and a keepout so no signal trace runs under it. Placed with
    its origin at the board's top-left corner."""
    def outline(d):
        # rounded rectangle inset by d, as points
        pts, rr = [], max(r - d, 0.05)
        corners = ((w - d - rr, d + rr, -90), (w - d - rr, h - d - rr, 0), (d + rr, h - d - rr, 90), (d + rr, d + rr, 180))
        for cx, cy, a0 in corners:
            for k in range(7):
                a = math.radians(a0 + 15 * k)
                pts.append((cx + rr * math.cos(a), cy + rr * math.sin(a)))
        return pts
    mid = outline(inset + width / 2)
    body = []
    for layer in ('F.Mask', 'B.Mask'):
        for a, b in zip(mid, mid[1:] + mid[:1]):
            body.append('  (fp_line (start %.3f %.3f) (end %.3f %.3f) (stroke (width %g) (type solid)) (layer "%s"))' % (a[0], a[1], b[0], b[1], width, layer))
    band = inset + width + 0.25
    for layer in ('F.Cu', 'B.Cu'):
        for (x0, y0, x1, y1) in ((0, 0, w, band), (0, h - band, w, h), (0, 0, band, h), (w - band, 0, w, h)):
            body.append('  (zone (net 0) (net_name "") (layers "%s") (hatch edge 0.5) (connect_pads (clearance 0)) (min_thickness 0.25)'
                        ' (keepout (tracks not_allowed) (vias allowed) (pads allowed) (copperpour allowed) (footprints allowed))'
                        ' (fill (thermal_gap 0.5) (thermal_bridge_width 0.5))'
                        ' (polygon (pts (xy %g %g) (xy %g %g) (xy %g %g) (xy %g %g))))' % (layer, x0, y0, x1, y0, x1, y1, x0, y1))
    return fp(name, body)


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
        text('rose limb controller', 14, -13, 1.6, B, 0.25),
        text('v0.2  2026-10  48 V  6 layers', 14, -11, 0.9, B),
        text('rose fork arm v9', 14, -8.4, 1.2, B, 0.2),
        text('arm, leg, torso, head:', 14, -6.4, 0.9, B),
        text('one board, any limb', 14, -5.0, 0.9, B),
        text('loon -> vcad -> kicad', 14, -2.4, 0.9, B),
        text('48 V: the brake eats regen', 14, 0, 0.9, B),
        text('if found, return to', 14, 2.6, 0.9, B),
        text('the nearest robot', 14, 4.0, 0.9, B),
        text('designed by muni', 14, 6.8, 1.2, B, 0.2),
        text('6L  1.6 mm  ENIG  black / muni orange', 14, 8.6, 0.7, B, 0.12),
    ]))
    open(os.path.join(OUT, 'shield_mcu.kicad_mod'), 'w').write(shield_frame('shield_mcu', SHIELD_W, SHIELD_H))
    vrml_lid(os.path.join(HERE, '3d', 'shield_mcu.wrl'), SHIELD_W, SHIELD_H, 2.0)
    print('wrote', ', '.join(sorted(os.listdir(OUT))))


if __name__ == '__main__':
    main()
