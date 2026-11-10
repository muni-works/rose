"""Numbers + swept-envelope mesh for TORSO-INTERFACE.md (lymb v10.3 on the quad00 full humanoid).

Frame: quad00 k1_structure's body frame for 'full' (x fore, y left, z up, mm), the frame rose's torso
pass uses (shoulder-pitch axis at z 434.3; the vcad export lifts everything by z0 = 999.9 so the soles
stand on z = 0). Left side; the right mirrors in y.

    ROSE_ARM=v10 ROSE_ARM_PEDESTAL=0 ROSE_ARM_PED_H=0.04105 uv run -q --with numpy --with manifold3d \\
        --with trimesh --with scipy python arm/torso_interface.py
  -> arm/interface/j1j2_envelope_l.stl  (+ prints the table)
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np
import trimesh
import rose_fork as F
import skin_v10 as S

SP = (0.0, 434.3)            # quad00 full shoulder-pitch axis (x, z), pre-lift - J1's axis
SIDE_PLATE = 87.0            # torso side plate outer face |y|


def main():
    ped = F.PED_H * 1e3
    y0 = SIDE_PLATE + ped                    # arm z = 0 (the foot's bottom face) at this |y|
    zs, zo = F.Z_SH * 1e3, (F.T_FOOT + F.J1M['L']) * 1e3
    print('chest adapter %.2f mm: foot |y| %.1f..%.1f, J1 can %.1f..%.1f, J1 output face (flank) %.1f, J2 axis %.1f'
          % (ped, y0, y0 + F.T_FOOT * 1e3, y0 + F.T_FOOT * 1e3, y0 + zo, y0 + zo, y0 + zs))
    parts = {nm: (l, m) for nm, l, m, _ in S.skin_meshes()}
    o = lambda l: np.array(F.ORIGIN[l]) * 1e3
    fixed = np.concatenate([parts[n][1].vertices + o(parts[n][0]) for n in ('turret', 'puck-J2', 'cap-J2-1', 'cap-J2+1')])
    upper = parts['upper'][1].vertices + o('upper')
    upper = np.concatenate([upper, parts['upperd'][1].vertices + o('upperd')])
    upper = upper[np.hypot(upper[:, 0], upper[:, 2] - zs) < 160.0]     # the sleeve's top: within 160 mm of J2
    clouds = [fixed]
    for a in np.radians(np.arange(-180, 11, 2)):          # J2 over its soft range
        c, s = math.cos(a), math.sin(a)
        P = upper.copy()
        x, z = P[:, 0], P[:, 2] - zs
        P[:, 0], P[:, 2] = c * x + s * z, -s * x + c * z + zs
        clouds.append(P)
    A = np.concatenate(clouds)
    r = np.hypot(A[:, 0], A[:, 1])                          # radius about J1 (arm z)
    zb = np.arange(math.floor(A[:, 2].min()), math.ceil(A[:, 2].max()) + 1, 1.0)
    prof = []
    for z0 in zb:
        m = np.abs(A[:, 2] - z0) < 1.0
        if m.any():
            prof.append((z0, r[m].max()))
    prof = np.array(prof)
    print('swept envelope (J1 360 x J2 -180..+10): |y| %.1f .. %.1f, max radius about J1 %.1f mm'
          % (y0 + prof[:, 0].min(), y0 + prof[:, 0].max(), prof[:, 1].max()))
    for z0, rr in prof[::10]:
        print('   |y| %6.1f   r %6.1f' % (y0 + z0, rr))
    # revolve the profile about J1 and place it in the body frame (left)
    pts = [(0.0, prof[0, 0])] + [(rr + 0.5, z0) for z0, rr in prof] + [(0.0, prof[-1, 0])]
    env = trimesh.creation.revolve(np.array(pts), sections=96)          # about its z
    M = np.array([[1.0, 0, 0], [0, 0, 1.0], [0, -1.0, 0]])              # revolve axis (its z) -> body +y
    V = env.vertices @ M.T
    V[:, 1] += y0
    V[:, 0] += SP[0]
    V[:, 2] += SP[1]
    env = trimesh.Trimesh(V, env.faces)
    os.makedirs(os.path.join(HERE, 'interface'), exist_ok=True)
    fn = os.path.join(HERE, 'interface', 'j1j2_envelope_l.stl')
    env.export(fn)
    print('wrote', fn, 'bounds', env.bounds.round(1).tolist())


if __name__ == '__main__':
    main()
