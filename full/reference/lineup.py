"""Compose vcad renders into one side-by-side at TRUE relative scale.

vcad-render --auto-aspect --fill 0.92 draws each robot's bounding box at 92 % of
the image height, so a render's pixel/mm is (0.92 * image_h) / (robot height).
This rescales every panel to a common pixel/mm, stands them on one floor, and
labels them. Heights are sole-to-crown from each model's own geometry.

  python3 lineup.py out.png  img1.png:HEIGHT_MM:label  img2.png:HEIGHT_MM:label ...
"""
import sys
from PIL import Image, ImageDraw, ImageFont

PPMM = 0.55        # output pixels per mm
GAP = 60
FLOOR = 70         # px below the tallest robot's feet for labels
TOP = 40


def main():
    out = sys.argv[1]
    panels = []
    for spec in sys.argv[2:]:
        path, h, label = spec.split(':', 2)
        im = Image.open(path).convert('RGBA')
        h = float(h)
        bbox = im.getbbox()                       # drawn region (transparent bg)
        crop = im.crop(bbox)
        # the drawn region IS the robot's bounding box, its height = h mm
        scale = PPMM * h / crop.height
        crop = crop.resize((max(1, int(crop.width * scale)), max(1, int(crop.height * scale))),
                           Image.LANCZOS)
        panels.append((crop, label, h))
    W = sum(p[0].width for p in panels) + GAP * (len(panels) + 1)
    H = TOP + int(max(p[2] for p in panels) * PPMM) + FLOOR
    canvas = Image.new('RGBA', (W, H), (255, 255, 255, 255))
    d = ImageDraw.Draw(canvas)
    try:
        font = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', 22)
    except Exception:
        font = ImageFont.load_default()
    x = GAP
    floor_y = H - FLOOR
    d.line([(0, floor_y), (W, floor_y)], fill=(120, 120, 120, 255), width=2)
    for crop, label, h in panels:
        canvas.alpha_composite(crop, (x, floor_y - crop.height))
        d.text((x, floor_y + 12), '%s  %.0f mm' % (label, h), fill=(30, 30, 30, 255), font=font)
        x += crop.width + GAP
    canvas.convert('RGB').save(out)
    print('wrote', out, canvas.size)


if __name__ == '__main__':
    main()
