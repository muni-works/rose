"""cortex.loon -> watertight print STLs, laid in print orientation.

    sh export.sh                  # first: vcad exports (stl/, ../fab/cortex/)
    python3 print_build.py        # then: rebuild the parts vcad tessellates with holes, write ../fab/cortex/print/

Why: vcad's boolean tessellation leaves open edges on the tub (~240), the floor (~1100) and the exhaust grille (377
fragments, zero volume), so a slicer can't trust them. Every leaf solid those parts are built from IS watertight out
of vcad, so this exports the leaves from cortex.loon and evaluates the same top-level boolean tree in manifold3d.
The shapes all still come from cortex.loon; only the last booleans run here. Keep TREES in step with cortex.loon.

Needs: numpy trimesh manifold3d.  VCAD=path/to/vcad to override.
"""
import os, subprocess, tempfile, shutil
import numpy as np
import trimesh
import manifold3d as mf

HERE = os.path.dirname(os.path.abspath(__file__))
FAB = os.path.join(HERE, "..", "fab", "cortex")
PRINT = os.path.join(FAB, "print")
VCAD = os.environ.get("VCAD") or os.path.join(
    os.environ.get("CARGO_TARGET_DIR", os.path.expanduser("~/Developer/vcad/target")), "release", "vcad")

LEAVES = ["shell-solid", "shell-inner", "below-floor", "ear-windows", "exhaust-window", "face-opening", "io-cut",
          "back-holes", "fix-blocks", "glass-band", "insert-holes", "floor-holes", "dock-cuts", "cone-bosses",
          "exhaust-holes", "d4-open", "d4-cam-holes", "d7-open", "d7-cam-holes", "ear-holes"]


def export_leaves(names, size_file=None):
    """export leaf solids from cortex.loon at a size (default: size.loon, the mini)."""
    tmp = tempfile.mkdtemp(prefix="cortex-leaves-")
    shutil.copy(os.path.join(HERE, "cortex.loon"), tmp)
    shutil.copy(size_file or os.path.join(HERE, "size.loon"), os.path.join(tmp, "size.loon"))
    procs = []
    for n in names:
        with open(os.path.join(tmp, n + ".loon"), "w") as f:
            f.write(f'[use cortex [{n}]]\n[root {n} "x"]\n')
        procs.append(subprocess.Popen([VCAD, "export", n + ".loon", n + ".stl"], cwd=tmp,
                                      stdout=subprocess.DEVNULL, stderr=subprocess.PIPE))
    for n, p in zip(names, procs):
        if p.wait():
            raise SystemExit(f"vcad export {n} failed: {p.stderr.read().decode()}")
    out = {}
    for n in names:
        m = trimesh.load(os.path.join(tmp, n + ".stl"), force="mesh")
        assert m.is_watertight, f"leaf {n} is not watertight"
        out[n] = to_mf(m)
    shutil.rmtree(tmp)
    return out


def to_mf(m):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(m.vertices, np.float32),
                               tri_verts=np.asarray(m.faces, np.uint32)))


def to_tm(s):
    g = s.to_mesh()
    return trimesh.Trimesh(np.asarray(g.vert_properties)[:, :3], np.asarray(g.tri_verts), process=True)


def U(*xs):
    return mf.Manifold.batch_boolean(list(xs), mf.OpType.Add)


# the top-level trees, as in cortex.loon ([difference A B] there is B - A)
def build(L):
    band = L["shell-solid"] - L["shell-inner"]
    cavity = L["shell-inner"] - L["fix-blocks"] - L["glass-band"]
    tub = L["shell-solid"] - U(cavity, L["below-floor"], L["ear-windows"], L["exhaust-window"], L["face-opening"],
                               L["io-cut"], L["insert-holes"])
    floor = U(L["below-floor"] ^ L["shell-solid"], L["cone-bosses"]) - U(L["dock-cuts"], L["floor-holes"])
    exhaust = (L["exhaust-window"] ^ band) - L["exhaust-holes"]
    back = (L["io-cut"] ^ band) - L["back-holes"]
    ears = (L["ear-windows"] ^ band) - L["ear-holes"]
    cavity_bare = L["shell-inner"] - L["fix-blocks"]
    tub_full = L["shell-solid"] - U(cavity_bare, L["below-floor"], L["ear-windows"], L["exhaust-window"], L["d7-open"],
                                    L["d7-cam-holes"], L["io-cut"], L["insert-holes"])
    tub_d4 = L["shell-solid"] - U(cavity, L["below-floor"], L["ear-windows"], L["exhaust-window"], L["d4-open"],
                                  L["d4-cam-holes"], L["io-cut"], L["insert-holes"])
    return {"cortex-tub": tub, "cortex-floor": floor, "cortex-exhaust-grille": exhaust,
            "cortex-back-grille": back, "cortex-tub-d4": tub_d4, "cortex-tub-full": tub_full, "cortex-ear-grilles": ears}


def rot(axis, deg):
    return trimesh.transformations.rotation_matrix(np.radians(deg), axis)


# print orientation per printed body: (output name, source part, body selector, rotation)
PRINTS = [
    ("cortex-sled-print-l", "cortex-sled-print", lambda c: c[1] > 0, rot([0, 1, 0], -90)),    # +X up, profile on the bed
    ("cortex-sled-print-r", "cortex-sled-print", lambda c: c[1] < 0, rot([0, 1, 0], -90)),
    ("cortex-floor", "cortex-floor", None, np.eye(4)),                                         # desk face down
    ("cortex-tub", "cortex-tub", None, np.eye(4)),                                             # as assembled, open side down
    ("cortex-ear-grille-l", "cortex-ear-grilles", lambda c: c[1] > 0, rot([1, 0, 0], -90)),   # outer face down
    ("cortex-ear-grille-r", "cortex-ear-grilles", lambda c: c[1] < 0, rot([1, 0, 0], 90)),
    ("cortex-exhaust-grille", "cortex-exhaust-grille", None, np.eye(4)),                      # standing on its long edge
    ("cortex-back-grille", "cortex-back-grille", None, rot([0, 1, 0], 90)),                  # outer face down, flat
    ("cortex-tub-d4", "cortex-tub-d4", None, np.eye(4)),                                      # as assembled, open side down
]


def lay(m, R):
    m = m.copy()
    m.apply_transform(R)
    lo, hi = m.bounds
    m.apply_translation([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])
    return m


MINI_ONLY = {"cortex-tub", "cortex-tub-d4"}          # the 3.5" glass face and the 4" face are mini-size parts
FULL_ONLY = {"cortex-tub-full"}                      # the 7" face


def build_size(size_file=None):
    """watertight parts at a size: {name: trimesh}. size_file=None is the mini (size.loon)."""
    full = size_file is not None and "true" in dict(
        __import__("re").findall(r"\[pub let (size-full) (true|false)\]", open(size_file).read())).get("size-full", "")
    skip = MINI_ONLY if full else FULL_ONLY
    return {k: to_tm(v) for k, v in build(export_leaves(LEAVES, size_file)).items() if k not in skip}, full


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--size", help="a size file (e.g. size-full.loon); default the mini")
    a = ap.parse_args()
    parts, full = build_size(a.size)
    fab, stl, prn = (FAB, os.path.join(HERE, "stl"), PRINT) if not full else \
        (FAB + "-full", os.path.join(HERE, "stl-full"), os.path.join(FAB + "-full", "print"))
    for d in (fab, stl, prn):
        os.makedirs(d, exist_ok=True)
    for k, m in parts.items():
        assert m.is_watertight, k
        m.export(os.path.join(fab, k + ".stl"))
        m.export(os.path.join(stl, k + ".stl"))
        print(f"{k}: watertight, {m.volume / 1000:.1f} cm3, {len(m.split())} bodies")
    for out, src, pick, R in PRINTS + ([("cortex-tub-full", "cortex-tub-full", None, np.eye(4))] if full else []):
        if src not in parts and not os.path.exists(os.path.join(fab, src + ".stl")):
            continue
        m = parts.get(src) or trimesh.load(os.path.join(fab, src + ".stl"), force="mesh")
        if pick:
            bodies = [b for b in m.split(only_watertight=False) if pick(b.bounds.mean(0))]
            m = trimesh.util.concatenate(bodies)
        m = lay(m, R)
        assert m.is_watertight, out
        m.export(os.path.join(prn, out + ".stl"))
        e = m.extents
        print(f"print/{out}.stl: {e[0]:.1f} x {e[1]:.1f} x {e[2]:.1f} mm, {m.volume / 1000:.1f} cm3")


if __name__ == "__main__":
    main()
