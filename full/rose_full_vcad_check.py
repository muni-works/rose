"""Volume-verify vcad's output against the independent Python plate model.

Cam's rule from the K1 cuff work: verify CAD by VOLUME against an independent
model, never by render. Every failure in that bug hunt was silent and looked
plausible in a screenshot.

The Python side (rose_full_links.py) computes plate area x thickness analytically
with no CSG at all, so agreement between it and vcad's tessellated solid is a
genuine cross-check of the boolean pipeline, not a tautology.

Expect vcad to come in slightly UNDER the analytic figure: the analytic model uses
square plate outlines, while the loon plates have 8 mm rounded corners.
"""
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rose_full_links as L

STL = sys.argv[1] if len(sys.argv) > 1 else '/tmp/rose_full_struct.stl'
DENSITY = 2.68e-3       # g/mm^3, 5052


def stl_volume(path):
    """Signed volume by the divergence theorem over origin tetrahedra."""
    with open(path, 'rb') as f:
        f.read(80)
        n = struct.unpack('<I', f.read(4))[0]
        vol = 0.0
        lo = [1e18] * 3
        hi = [-1e18] * 3
        for _ in range(n):
            d = f.read(50)
            v = struct.unpack('<12f', d[:48])
            a, b, c = v[3:6], v[6:9], v[9:12]
            vol += (a[0] * (b[1] * c[2] - b[2] * c[1])
                    - a[1] * (b[0] * c[2] - b[2] * c[0])
                    + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
            for p in (a, b, c):
                for k in range(3):
                    lo[k] = min(lo[k], p[k])
                    hi[k] = max(hi[k], p[k])
    return abs(vol), n, lo, hi


def main():
    if not os.path.exists(STL):
        print('missing %s - run the vcad export first' % STL)
        return 1
    vol, tris, lo, hi = stl_volume(STL)

    # analytic: every plate, both sides, square outlines
    no_mirror = ('neck', 'trunkC', 'head')
    analytic = 0.0
    for p in L.PLATES:
        n = 1 if p.link in no_mirror else 2
        analytic += p.area() * L.T * n
        for (_, _, d) in p.holes:
            pass   # holes are in the loon too; the analytic area ignores them both

    print('vcad STL      %10.1f cm3   %8.0f g   (%d triangles)'
          % (vol / 1000.0, vol * DENSITY, tris))
    print('analytic      %10.1f cm3   %8.0f g   (square outlines, holes ignored)'
          % (analytic / 1000.0, analytic * DENSITY))
    print('ratio         %10.3f   (expect < 1: rounded corners + drilled holes)'
          % (vol / analytic))
    print('\nbbox mm   x %8.1f .. %-8.1f  y %8.1f .. %-8.1f  z %8.1f .. %-8.1f'
          % (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))
    print('expected  z %.1f .. %.1f (sole .. crown, from the layout)' % (L.ZS, L.ZCROWN))

    ok = True
    if not (0.60 < vol / analytic < 1.0):
        print('\nFAIL: volume ratio outside the plausible band - suspect a boolean fault')
        ok = False
    if abs(lo[2] - L.ZS) > 2.0:
        print('\nFAIL: sole is not at z = %.0f' % L.ZS)
        ok = False
    print('\n%s' % ('volume cross-check PASSED' if ok else 'CROSS-CHECK FAILED'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
