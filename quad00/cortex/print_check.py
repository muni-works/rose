"""Print-readiness checks for the cortex head's printed parts (FDM/SLA).

    python3 cortex/print_check.py [stl ...]      # default: the MAKE parts in fab/cortex/

Per part, per candidate build direction (+-X, +-Y, +-Z up):
  - bed contact area (faces within 0.05 mm of the lowest point, facing down)
  - overhang area: down-facing faces steeper than 45 deg from vertical, not on the bed
  - wall thickness: rays cast inward from sampled face centroids (min, 1st and 5th percentile)
Needs: numpy trimesh rtree.  Frame: the cortex dock frame, mm.
"""
import sys, glob, os
import numpy as np
import trimesh

FDM_MIN_WALL = 1.2      # 3 perimeters at 0.4 nozzle
OVERHANG_DEG = 45.0
UPS = {"+X": [1, 0, 0], "-X": [-1, 0, 0], "+Y": [0, 1, 0], "-Y": [0, -1, 0], "+Z": [0, 0, 1], "-Z": [0, 0, -1]}


def thickness(m, n=4000, seed=0):
    rng = np.random.default_rng(seed)
    p = m.area_faces / m.area_faces.sum()
    idx = rng.choice(len(m.faces), size=n, replace=True, p=p)
    o = m.triangles_center[idx] - m.face_normals[idx] * 1e-3
    d = -m.face_normals[idx]
    loc, ri, _ = m.ray.intersects_location(o, d, multiple_hits=False)
    t = np.full(len(idx), np.inf)
    t[ri] = np.linalg.norm(loc - o[ri], axis=1)
    ok = np.isfinite(t)
    return t[ok], m.triangles_center[idx][ok]


def orient(m, up):
    up = np.asarray(up, float)
    h = m.vertices @ up
    z0 = h.min()
    dn = m.face_normals @ up
    fh = m.triangles_center @ up
    on_bed = (dn < -0.99) & (fh - z0 < 0.05)
    steep = dn < -np.cos(np.radians(90 - OVERHANG_DEG))   # facing down more than 45 deg past vertical
    over = steep & ~on_bed
    return dict(bed=m.area_faces[on_bed].sum(), over=m.area_faces[over].sum(), height=h.max() - z0)


def report(path):
    m = trimesh.load(path, force="mesh")
    name = os.path.basename(path)
    ext = m.bounds[1] - m.bounds[0]
    print(f"\n== {name}: {len(m.faces)} tris, watertight={m.is_watertight}, vol={m.volume/1000:.1f} cm3, "
          f"bbox {ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f}")
    print(f"   bounds min {np.round(m.bounds[0], 2).tolist()} max {np.round(m.bounds[1], 2).tolist()}")
    t, at = thickness(m)
    if len(t):
        thin = (t < FDM_MIN_WALL).mean() * 100
        print(f"   wall: min {t.min():.2f}  p1 {np.percentile(t, 1):.2f}  p5 {np.percentile(t, 5):.2f}  "
              f"median {np.median(t):.2f} mm;  {thin:.1f}% of sampled surface < {FDM_MIN_WALL} mm")
        th = at[t < FDM_MIN_WALL]
        if len(th):
            print(f"   thin (<{FDM_MIN_WALL}) samples span {np.round(th.min(0), 1).tolist()} .. {np.round(th.max(0), 1).tolist()}")
    rows = sorted(((k, orient(m, u)) for k, u in UPS.items()), key=lambda r: (r[1]["over"], -r[1]["bed"]))
    for k, r in rows:
        print(f"   up {k}: bed {r['bed']:7.0f} mm2  overhang>45 {r['over']:7.0f} mm2  height {r['height']:6.1f} mm")


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    paths = sys.argv[1:] or sorted(glob.glob(os.path.join(here, "..", "fab", "cortex", "*.stl")))
    for p in paths:
        report(p)
