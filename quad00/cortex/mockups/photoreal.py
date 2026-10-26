"""Photoreal renders of the full-head mockups, with the face UI lit on the 7-inch screen.

    python3 photoreal.py [spp]          # -> <option>-photo.png for each full-*.loon here, plus mini-photo.png

Each option builds from the one cortex.loon (its full-*.loon copied in as size.loon). The screen is a thin slab on the
display glass over the active area, with an emissive material whose image (ui-face-1024x600.png) is planar-mapped
onto the active area. The map's plane is computed here from the same formulas cortex.loon uses (zf, xfz, the 8 deg
tilt), so keep them in step. Needs vcad-render with emissive maps (VCAD_RENDER to override).
"""
import math, os, re, shutil, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(HERE)
T = os.path.join(os.environ.get("CARGO_TARGET_DIR", os.path.expanduser("~/Developer/vcad/target")), "release")
R = os.environ.get("VCAD_RENDER", os.path.join(T, "vcad-render"))
SPP = int(sys.argv[1]) if len(sys.argv) > 1 else 128
UI = "ui-face-1024x600.png"
RAKE, WALL, TILT = 0.14, 2.5, math.radians(-8.0)


def size_of(path):
    v = dict(re.findall(r"\[pub let ([a-z0-9-]+) (-?[0-9.]+)\]", open(path).read()))
    return {k: float(x) for k, x in v.items()}


def screen_plane(s):
    """world-space map origin (image top-left), u (width), v (height down) of the 7-inch active area."""
    hd = (s["x-face"] - s["x-back"]) / 2
    rb, rf = 0.24 * hd, 0.42 * hd
    zf = (rb + s["hh"] - rf) / 2
    xfz = s["x-face"] - RAKE * zf
    zc = zf + s["d7-dz"]
    xs = xfz - WALL + 0.3                        # the slab's front face, just proud of the glass
    aw, ah = s["d7-act-w"] / 2, s["d7-act-h"] / 2

    def tilt(p):                                 # cortex.loon tilt: rotate about y by -8 deg about (xfz, zf)
        x, y, z = p[0] - xfz, p[1], p[2] - zf
        c, n = math.cos(TILT), math.sin(TILT)
        return (xfz + c * x + n * z, y, zf - n * x + c * z)

    o = tilt((xs, -aw, zc + ah))                 # viewer's top-left: looking down -x with z up, the viewer's left is -y
    pu = tilt((xs, aw, zc + ah))
    pv = tilt((xs, -aw, zc - ah))
    u = tuple(b - a for a, b in zip(o, pu))
    v = tuple(b - a for a, b in zip(o, pv))
    return o, u, v, xs, zc


def f3(t):
    return " ".join(f"{x:.4f}" for x in t)


def meshes(size_file, d):
    """the watertight parts at this size (print_build's manifold booleans), plus the plain proxies, as STLs in d."""
    sys.path.insert(0, SRC)
    import print_build as PB
    parts, _ = PB.build_size(size_file)
    names = {}
    for k in ("cortex-tub-full", "cortex-floor", "cortex-ear-grilles", "cortex-exhaust-grille", "cortex-back-grille"):
        parts[k].export(os.path.join(d, k + ".stl")); names[k] = k + ".stl"
    for k in ("d7-glass", "d7-cams"):                # simple boxes: vcad tessellates them watertight
        with open(os.path.join(d, k + ".loon"), "w") as f:
            f.write(f'[use cortex [{k}]]\n[root {k} "x"]\n')
        subprocess.run([PB.VCAD, "export", k + ".loon", k + ".stl"], cwd=d, check=True, capture_output=True)
        names[k] = k + ".stl"
    return names


def scene(opt, s, d, names):
    o, u, v, xs, zc = screen_plane(s)
    aw, ah = s["d7-act-w"] / 2, s["d7-act-h"] / 2
    mats = f"""[material "satin_black" 0.035 0.037 0.042 0.0 0.42]
[material "glass_black" 0.01 0.01 0.012 0.0 0.05]
[material "lens" 0.0 0.0 0.0 0.0 0.08]
[material-emissive "screen_ui" 0.01 0.01 0.012 0.0 0.15 1.0 1.0 1.0 3.0 "{UI}" {f3(o)} {f3(u)} {f3(v)}]"""
    mat = {"cortex-tub-full": "satin_black", "cortex-floor": "satin_black", "cortex-ear-grilles": "satin_black",
           "cortex-exhaust-grille": "satin_black", "cortex-back-grille": "satin_black", "d7-glass": "glass_black",
           "d7-cams": "lens"}
    roots = "\n".join(f'[root [import-mesh-scaled 0.001 0.001 0.001 "{f}"] "{mat[k]}"]' for k, f in names.items())
    return f"""[use cortex [tilt]]
{mats}
{roots}
[root [tilt [translate {xs - 0.3:.4f} {-aw:.4f} {zc - ah:.4f} [cube 0.3 {2 * aw:.4f} {2 * ah:.4f}]]] "screen_ui"]
"""


def render(opt, size_file):
    d = tempfile.mkdtemp(prefix=f"cortex-photo-{opt}-")
    shutil.copy(os.path.join(SRC, "cortex.loon"), d)
    shutil.copy(size_file, os.path.join(d, "size.loon"))
    shutil.copy(os.path.join(HERE, UI), d)
    s = size_of(size_file)
    names = meshes(os.path.join(d, "size.loon"), d)
    with open(os.path.join(d, "photo.loon"), "w") as f:
        f.write(scene(opt, s, d, names))
    for view, az, el in (("34", 28, 14), ("front", 0, 6)):
        out = os.path.join(HERE, f"{opt}-photo-{view}.png")
        cmd = [R, os.path.join(d, "photo.loon"), "--photoreal", "--spp", str(SPP), "--size", "1600x1100",
               "--azimuth", str(az), "--elevation", str(el), "--fov", "24", "--aperture", "0.012",
               "--backdrop", "studio", "--env", "studio", "--fill", "0.8", "-o", out]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode:
            raise SystemExit(f"{opt}: {r.stderr[-800:]}")
    print(opt)
    shutil.rmtree(d)


if __name__ == "__main__":
    only = sys.argv[2:] or ["above", "beside", "corner"]
    for opt in only:
        render(opt, os.path.join(HERE, f"full-{opt}.loon"))
