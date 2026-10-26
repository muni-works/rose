"""The cortex face UI mockup: the eyes are drawn on the screen (SPEC.md). Writes ui-face-<w>x<h>.png.

    python3 ui_face.py [1024 600]     # the 7" (Waveshare 7inch HDMI LCD (C)) native resolution by default

Minimal: black field, two soft rounded-square eyes with a gentle glow and a highlight, a thin status line at the
bottom (frog state, battery, time). Black/white with one orange accent (the design direction: no decoration).
Needs pillow.
"""
import sys
from PIL import Image, ImageDraw, ImageFilter, ImageFont

W, H = (int(sys.argv[1]), int(sys.argv[2])) if len(sys.argv) > 2 else (1024, 600)
S = 4                                                     # supersample, then downscale
w, h = W * S, H * S
img = Image.new("RGB", (w, h), (4, 5, 7))

# eyes: rounded squares, a little taller than wide, set slightly above centre, looking a touch to the right
ew, eh, gap = int(0.17 * w), int(0.40 * h), int(0.13 * w)
cy = int(0.45 * h)
look = int(0.012 * w)
eyes = [((w - gap) // 2 - ew + look, cy - eh // 2), ((w + gap) // 2 + look, cy - eh // 2)]
r = int(0.32 * ew)

glow = Image.new("RGB", (w, h), (0, 0, 0))
gd = ImageDraw.Draw(glow)
for x, y in eyes:
    gd.rounded_rectangle([x - 6 * S, y - 6 * S, x + ew + 6 * S, y + eh + 6 * S], r + 6 * S, fill=(150, 200, 255))
glow = glow.filter(ImageFilter.GaussianBlur(28 * S))
img = Image.blend(img, glow, 0.35)

d = ImageDraw.Draw(img)
for x, y in eyes:
    d.rounded_rectangle([x, y, x + ew, y + eh], r, fill=(236, 244, 255))
    hx, hy = x + int(0.62 * ew), y + int(0.16 * eh)            # a small catch-light, upper right
    d.ellipse([hx, hy, hx + int(0.16 * ew), hy + int(0.16 * ew)], fill=(255, 255, 255))

# status line
def font(size):
    for f in ("/System/Library/Fonts/SFNSMono.ttf", "/System/Library/Fonts/Menlo.ttc"):
        try:
            return ImageFont.truetype(f, size)
        except OSError:
            pass
    return ImageFont.load_default()

fs = int(0.032 * h)
f = font(fs)
y0 = h - int(0.085 * h)
dot = int(0.36 * fs)
d.ellipse([int(0.05 * w), y0 + fs // 2 - dot, int(0.05 * w) + 2 * dot, y0 + fs // 2 + dot], fill=(255, 107, 18))   # orange: listening
d.text((int(0.05 * w) + 3 * dot, y0), "frog  listening", font=f, fill=(120, 126, 136))
right = "93%   10:42"
tw = d.textlength(right, font=f)
d.text((w - int(0.05 * w) - tw, y0), right, font=f, fill=(120, 126, 136))

img = img.resize((W, H), Image.LANCZOS)
out = f"ui-face-{W}x{H}.png"
img.save(out)
print(out)
