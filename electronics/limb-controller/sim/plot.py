"""power_path.csv -> power_path.svg (no plotting deps)."""
import csv
rows = [tuple(map(float, r.values())) for r in csv.DictReader(open('power_path.csv'))]
W, H, L, R, T, B = 900, 420, 60, 20, 30, 50
ph = (H - T - B - 20) / 2
def xs(t): return L + t / 1.0 * (W - L - R)
def panel(y0, lo, hi, col, key, label, ticks):
    sy = lambda v: y0 + ph - (v - lo) / (hi - lo) * ph
    out = ['<rect x="%d" y="%.1f" width="%d" height="%.1f" fill="none" stroke="#ccc"/>' % (L, y0, W - L - R, ph)]
    for tv in ticks:
        out.append('<line x1="%d" x2="%d" y1="%.1f" y2="%.1f" stroke="#eee"/><text x="%d" y="%.1f" font-size="11" text-anchor="end" fill="#666">%g</text>' % (L, W - R, sy(tv), sy(tv), L - 5, sy(tv) + 4, tv))
    pts = ' '.join('%.1f,%.1f' % (xs(r[0]), sy(min(max(r[key], lo), hi))) for r in rows)
    out.append('<polyline points="%s" fill="none" stroke="%s" stroke-width="1.4"/>' % (pts, col))
    out.append('<text x="%d" y="%.1f" font-size="12" fill="%s">%s</text>' % (L + 6, y0 + 14, col, label))
    return out, sy
svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" font-family="Helvetica"><rect width="100%%" height="100%%" fill="#fff"/>' % (W, H)]
svg.append('<text x="%d" y="18" font-size="14" font-weight="bold">Arm controller power path, fixed values: 3.3 ohm brake, 10 A regen 500-600 ms, e-stop at 800 ms</text>' % L)
a, sy = panel(T, 0, 60, '#c0392b', 1, 'VBUS (V)', [0, 20, 40, 48, 54])
svg += a
# brake-on shading
on = None
for r in rows + [(1.0, 0, 0, 0)]:
    if r[3] > 0.5 and on is None: on = r[0]
    if r[3] < 0.5 and on is not None:
        svg.append('<rect x="%.1f" y="%d" width="%.1f" height="%.1f" fill="#c0392b" opacity="0.12"/>' % (xs(on), T, max(xs(r[0]) - xs(on), 0.6), ph)); on = None
b, _ = panel(T + ph + 20, 0, 3.5, '#2c6fbb', 2, 'LM5060 EN (V); turn-off at 2.0 V (assumed)', [0, 2.0, 3.3])
svg += b
for t in [0, 0.2, 0.4, 0.5, 0.6, 0.8, 1.0]:
    svg.append('<text x="%.1f" y="%d" font-size="11" text-anchor="middle" fill="#666">%g s</text>' % (xs(t), H - B + 32, t))
svg.append('</svg>')
open('power_path.svg', 'w').write('\n'.join(svg))
print('ok')
