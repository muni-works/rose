"""cortex -- the rose head as its own product.

The cortex is the brain module: compute (Jetson), its own battery, face screen, stereo eyes, ears, and a back panel.
It sits on any rose body's neck dock and runs detached on a desk too. One USB-C link through the dock carries power
(USB-PD 20 V / 5 A) and data (USB 3). The limbic board stays in the torso and is shared by every body.

Frame: the product is built about its DOCK ORIGIN, the centre of the dock interface on the head's flat floor
(x fore, y left, z up, mm). frame() places it in a robot (or on a desk), and everything here derives from that frame.
Sizes come from datasheets where found; ASSUMED marks numbers to confirm against a drawing.
"""
import math
import numpy as np
import trimesh
from manifold3d import Manifold, OpType

# ---- the bill of materials (mini-scale head; R scales the shell, not the parts)
PARTS = dict(
    compute=dict(part='NVIDIA Jetson Orin NX 16GB on Seeed A603 carrier (87 x 52 mm) + ATS 63 x 40 x 8 fan heatsink',
                 size=(52.0, 87.0, 36.0), kg=0.40),          # stack height 36 ASSUMED (board + SO-DIMM + module + 8 mm heatsink)
    display=dict(part='Waveshare 3.5inch HDMI LCD (E), IPS 640x480, 76.6 x 63.6 mm, active 70.68 x 53.36',
                 size=(11.0, 76.6, 63.6), kg=0.06),          # thickness 11 ASSUMED (panel + driver board)
    camera=dict(part='2x Arducam IMX219 (B0287-class) 25 x 24 mm, 22-pin 0.5 mm FFC to the A603 CSI ports',
                size=(13.0, 25.0, 24.0), baseline=106.0),    # depth 13 ASSUMED; 106 mm baseline: ~1.7x the depth resolution of 63
    speaker=dict(part='2x Visaton K 28 WPC BL, 8 ohm 1 W, 28 x 5.4 mm', size=(28.0, 5.4, 28.0)),
    amp=dict(part='2x Adafruit 3006 MAX98357A I2S amp, 19.4 x 17.8 x 3.0 mm', size=(19.4, 3.0, 17.8)),
    mic=dict(part='2x Adafruit 3421 SPH0645LM4H I2S MEMS mic, 16.7 x 12.7 x 1.8 mm', size=(16.7, 1.8, 12.7)),
    battery=dict(part='3S1P 21700 (Molicel P45B class, 3 x 4.5 Ah, 11.1 V 50 Wh) + 3S BMS: the A603 wants 9-20 V',
                 size=(22.0, 66.0, 74.0), kg=0.23),
    power=dict(part='USB-PD sink (TPS25750 class) + 3S charger / power-path (BQ25713 class), ASSUMED 40 x 30 x 6',
               size=(30.0, 40.0, 6.0)),
)

# ---- the dock: ONE USB-C link. The head's receptacle is in its floor at the dock origin; the body's dock carries a
# floating USB-C plug, two alignment pins (they take the side loads, not the connector) and two magnets (seat + hold).
# On a desk the same receptacle takes any USB-C PD charger / a laptop.
DOCK = dict(
    link='USB-C: USB-PD 3.0 20 V / 5 A (100 W) + USB 3.2 Gen 1 (5 Gbps) data; the limbic board is the PD source + USB device',
    receptacle=dict(part='vertical-mount USB-C receptacle on a 14 x 20 board (GCT USB4105 class), opening down', size=(14.0, 20.0, 10.0),
                    slot=(9.6, 3.9)),                        # floor opening for the plug shell (8.94 x 3.26 + float)
    plug=dict(part='floating USB-C plug on a +-0.8 mm float mount, tongue 6.5 mm proud of the dock face', size=(16.0, 26.0, 12.0),
              tongue=(8.3, 2.6, 6.5)),
    pins=dict(d=4.0, h=6.0, y=18.0),                          # alignment pins, at +-y, on the dock plate; holes in the head floor
    magnets=dict(d=8.0, h=3.0, y=30.0),                       # N52 discs, flush in both faces
)


def frame(origin, hh, hw, R=1.0):
    """the head's box about its dock origin: face/back planes, flat floor, size and corner rounds."""
    ox, oy, oz = origin
    x_face, x_back = ox + 46.0 * R, ox - 58.0 * R
    hd = (x_face - x_back) / 2
    return dict(ox=ox, oy=oy, zb=oz, hh=hh, hw=hw, hd=hd, x_face=x_face, x_back=x_back, R=R,
                rc_p=0.30 * min(hw, hd), rf=0.42 * min(hw, hd), rb=0.24 * min(hw, hd))


def eye_z(HF):
    """the cameras' height: up in the face's flat band (eye level above the screen centre), clear of the crown round."""
    return HF['zb'] + HF['hh'] - HF['rf'] - PARTS['camera']['size'][2] / 2 - 4.0 * HF['R']


def face_z(HF):
    """the centre of the face's flat band (the display sits on it)."""
    return (HF['zb'] + HF['rb'] + HF['zb'] + HF['hh'] - HF['rf']) / 2


def shell(HF):
    """the product's skin: shell (with vents + dock openings), face glass, eyes, ears, back panel."""
    import k1_design as KD
    from design import M_, tm
    rrect, hull_rings, cut_shell, cyl = KD.rrect, KD.hull_rings, KD.cut_shell, KD.cyl
    hh, hw, hd, x_face, x_back, zb, R = HF['hh'], HF['hw'], HF['hd'], HF['x_face'], HF['x_back'], HF['zb'], HF['R']
    rc_p, rf, rb_ = HF['rc_p'], HF['rf'], HF['rb']
    hc = np.array([x_face - hd, 0.0, zb + hh / 2])
    out = []
    add = lambda n, m: out.append((n, m))
    zs_ = sorted([(zb + rb_ * (1 - math.sin(t)), rb_ * (1 - math.cos(t))) for t in np.linspace(0, np.pi / 2, 5)] +
                 [(hc[2] + hh / 2 - rf * (1 - math.sin(t)), rf * (1 - math.cos(t))) for t in np.linspace(np.pi / 2, 0, 7)])
    rings = [rrect(np.array([x_face - hd, 0.0, z_]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), hw - d_, hd - d_, max(rc_p - d_, 2.0), k=14)
             for z_, d_ in zs_]
    zm = zb + hh * 0.42                                          # vents: side slots near the back (in left, out right)
    cuts = [('box', [x_back + rc_p + 4.0 + 7.0 * i, (hw - 4.0) if s_ > 0 else -(hw + 2.0), zm - 12.0 * R], [3.5, 6.0, 24.0 * R])
            for s_ in (1, -1) for i in range(5)]
    sx, sy_ = DOCK['receptacle']['slot']                        # the dock: receptacle slot + pin holes + magnet seats in the floor
    cuts.append(('box', [HF['ox'] - sy_ / 2, -sx / 2, zb - 2.0], [sy_, sx, 6.0]))   # (the plug's tongue runs along y)
    p = DOCK['pins']
    cuts += [('box', [HF['ox'] - (p['d'] + 0.4) / 2, s_ * p['y'] - (p['d'] + 0.4) / 2, zb - 2.0], [p['d'] + 0.4, p['d'] + 0.4, p['h'] + 3.0]) for s_ in (1, -1)]
    add('cover-helmet', cut_shell(hull_rings(rings), cuts))
    # the face glass: display + both eyes behind it
    sw, sh = hw - rc_p - 3.0, 0.33 * hh
    zfc = face_z(HF)
    add('visor-face', hull_rings([rrect(np.array([x_, 0.0, zfc]), np.array([0, 0, 1.0]), np.array([0, 1.0, 0]), sh - d_, sw - d_, 0.55 * min(sw, sh), k=12)
                                  for x_, d_ in ((x_face - 3.0, 0.0), (x_face + 0.4, 0.0), (x_face + 0.8, 0.4))]))
    for s_ in (1, -1):                                          # eyes: on the IMX219 lenses, gunmetal bezels
        t = 'l' if s_ > 0 else 'r'
        e = np.array([x_face + 0.8, s_ * PARTS['camera']['baseline'] / 2 * R, eye_z(HF)])
        add(f'trim-eyering.{t}', cyl(e + np.array([0.3, 0, 0]), [1.0, 0, 0], 5.2 * R, 1.0))
        add(f'visor-eye.{t}', cyl(e + np.array([0.9, 0, 0]), [1.0, 0, 0], 3.6 * R, 0.8))
        ec = np.array([x_face - 0.27 * (x_face - x_back), s_ * (hw + 0.4), zfc])      # ears: a mic / speaker grille
        ear = M_(cyl(ec, [0, 1.0, 0], 11.0 * R, 1.4))
        for k in range(7):
            a_ = k * np.pi / 3
            hp_ = ec + (np.array([5.5 * R * math.cos(a_), 0, 5.5 * R * math.sin(a_)]) if k else np.zeros(3))
            ear = ear - M_(cyl(hp_, [0, 1.0, 0], 1.1 * R, 4.0))
        add(f'trim-ear.{t}', tm(ear))
    # the back panel: power button + status LED, USB-C, Ethernet
    zpm, pw, ph = panel_z(HF), hw * 0.62, hh * 0.16
    add('trim-headpanel', hull_rings([rrect(np.array([x_back + dx, 0.0, zpm]), np.array([0, 0, 1.0]), np.array([0, 1.0, 0]), ph - d_, pw - d_, 5.0, k=10)
                                      for dx, d_ in ((1.0, 0.0), (-0.8, 0.0), (-1.4, 0.5))]))
    xo = x_back - 1.4
    add('cover-headbutton', cyl(np.array([xo - 1.0, 0.68 * pw, zpm]), [1.0, 0, 0], 5.0, 2.4))
    add('visor-headled', cyl(np.array([xo - 0.4, 0.40 * pw, zpm]), [1.0, 0, 0], 1.6, 1.2))
    add('visor-headport', hull_rings([rrect(np.array([xo + dx, -0.10 * pw, zpm]), np.array([0, 0, 1.0]), np.array([0, 1.0, 0]), 2.0, 5.0, 1.9, k=8) for dx in (0.4, -0.2)]))
    add('visor-headeth', hull_rings([rrect(np.array([xo + dx, -0.58 * pw, zpm]), np.array([0, 0, 1.0]), np.array([0, 1.0, 0]), 6.0, 8.0, 1.0, k=6) for dx in (0.4, -0.2)]))
    return out


def panel_z(HF):
    return HF['zb'] + HF['hh'] * 0.38


def insides(HF):
    """the parts inside, laid out in the head frame."""
    C = PARTS
    xf, xb, zb, hw = HF['x_face'], HF['x_back'], HF['zb'], HF['hw']
    box = lambda n, lo, hi: (n, trimesh.creation.box(bounds=[lo, hi]))
    zc = face_z(HF)
    out = []
    dx, dy, dz = C['display']['size']
    out.append(box('elec-cxdisplay', [xf - 1.5 - dx, -dy / 2, zc - dz / 2], [xf - 1.5, dy / 2, zc + dz / 2]))
    cx_, cy_, cz_ = C['camera']['size']
    ze = eye_z(HF)
    for sy in (1, -1):
        yc = sy * C['camera']['baseline'] / 2
        out.append(box(f'elec-cxcam.{"l" if sy > 0 else "r"}', [xf - 1.5 - cx_, yc - cy_ / 2, ze - cz_ / 2], [xf - 1.5, yc + cy_ / 2, ze + cz_ / 2]))
    rx, ry, rz = DOCK['receptacle']['size']                       # the USB-C receptacle board, at the dock origin, opening down
    out.append(box('elec-cxusbc', [HF['ox'] - rx / 2, -ry / 2, zb + 1.5], [HF['ox'] + rx / 2, ry / 2, zb + 1.5 + rz]))
    jx, jy, jz = C['compute']['size']
    x1 = xf - 1.5 - dx - 4.0                                       # behind the display (4 mm for the HDMI FPC), up over the receptacle
    z_j = zb + 1.5 + rz + 1.5
    out.append(box('elec-jetson', [x1 - jx, -jy / 2, z_j], [x1, jy / 2, z_j + jz]))
    bx, by, bz = C['battery']['size']
    out.append(box('elec-cbat', [xb + 4.5, 0.0, zb + 4.0], [xb + 4.5 + bx, by, zb + 4.0 + bz]))
    px, py, pz = C['power']['size']
    out.append(box('elec-cxpower', [xb + 3.0, -6.0, zb + 4.0 + bz + 2.0], [xb + 3.0 + px, -6.0 + py, zb + 4.0 + bz + 2.0 + pz]))
    ec_x = xf - 0.27 * (xf - xb)                                   # the ears: speaker behind the grille, mic above, amp below
    for sy in (1, -1):
        t = 'l' if sy > 0 else 'r'
        sd, st, _ = C['speaker']['size']
        yw = sy * (hw - 2.5)
        out.append(('elec-cxspk.' + t, trimesh.creation.cylinder(radius=sd / 2, segment=[[ec_x, yw, zc], [ec_x, yw - sy * st, zc]], sections=48)))
        mx, my, mz = C['mic']['size']
        out.append(box('elec-cxmic.' + t, [ec_x - mx / 2, min(yw, yw - sy * my), zc + sd / 2 + 2], [ec_x + mx / 2, max(yw, yw - sy * my), zc + sd / 2 + 2 + mz]))
        ax_, ay_, az_ = C['amp']['size']
        out.append(box('elec-cxamp.' + t, [ec_x - ax_ / 2, min(yw, yw - sy * ay_), zc - sd / 2 - 2 - az_], [ec_x + ax_ / 2, max(yw, yw - sy * ay_), zc - sd / 2 - 2]))
    return out


def wiring(HF):
    """the cortex's internal cables (each clears every part but its two ends)."""
    from k1_structure import cable
    P = dict(insides(HF))
    c = lambda n: P[n].bounds
    xb, zb, hh, hw = HF['x_back'], HF['zb'], HF['hh'], HF['hw']
    jet, cam_l, disp, pw_, usb = c('elec-jetson'), c('elec-cxcam.l'), c('elec-cxdisplay'), c('elec-cxpower'), c('elec-cxusbc')
    xg = (P['elec-cbat'].bounds[1][0] + jet[0][0]) / 2              # the gap between the pack and the Jetson
    jt, jb, jz0 = jet[1][2], jet[0][0], jet[0][2]
    jc = (jet[0][0] + jet[1][0]) / 2
    zpm = panel_z(HF)
    out = []
    for sy, t in ((1, 'l'), (-1, 'r')):
        cy = sy * (cam_l[0][1] + cam_l[1][1]) / 2
        out.append(cable(f'cable-hcsi.{t}', [[cam_l[0][0] - 0.8, cy, (cam_l[0][2] + cam_l[1][2]) / 2], [cam_l[0][0] - 3, sy * 46, jt + 4], [jet[1][0] - 10, sy * 20, jt + 1.5]], flat=16.0))
        a = c(f'elec-cxamp.{t}'); m_ = c(f'elec-cxmic.{t}')
        out.append(cable(f'cable-haudio.{t}', [[(a[0][0] + a[1][0]) / 2, sy * (abs(a[0][1]) + abs(a[1][1])) / 2 - sy * 2, (a[0][2] + a[1][2]) / 2], [jc - 1.5, sy * 52, jt + 4], [jc - 13.5, sy * 36, jt + 2.5]], r=1.5))
        out.append(cable(f'cable-hmic.{t}', [[(m_[0][0] + m_[1][0]) / 2, sy * (abs(m_[0][1]) + abs(m_[1][1])) / 2 - sy * 2, (m_[0][2] + m_[1][2]) / 2], [jc - 3.5, sy * 50, jt + 12], [jc - 17.5, sy * 34, jt + 2.5]], r=1.5))
    out.append(cable('cable-hhdmi', [[disp[0][0] - 1.2, 0, disp[1][2] - 12], [disp[0][0] - 1.2, -10, jz0 + 14], [jet[1][0] + 0.6, -10, jz0 + 10]], flat=14.0))
    out.append(cable('cable-hpower', [[pw_[1][0] - 2, 14, pw_[0][2] - 1], [xg, 14, pw_[0][2] - 5], [xg, 25, jz0 + 15], [jb - 0.6, 25, jz0 + 13]], r=2.5))
    ux = (usb[0][0] + usb[1][0]) / 2                                # USB-C receptacle -> PD sink/charger (VBUS + CC) and -> the A603 (USB 3)
    out.append(cable('cable-hdockpd', [[usb[0][0] - 0.5, 6, usb[0][2] + 5], [xg, 6, usb[0][2] + 5], [xg, 5, pw_[0][2] - 5], [pw_[1][0] - 2, 5, pw_[0][2] - 1]], r=2.5))
    out.append(cable('cable-hdockusb', [[usb[0][0] - 0.5, -6, usb[0][2] + 5], [xg, -6, usb[0][2] + 5], [xg, -14, jz0 + 8], [jb - 0.6, -14, jz0 + 8]], r=2.5))
    out.append(cable('cable-heth', [[xb + 3.5, -0.58 * hw * 0.62, zpm], [xb + 18, -29, zpm - 10], [jb - 0.6, -29, jz0 + 14]], r=3.0))
    out.append(cable('cable-husbc', [[xb + 3.5, -0.10 * hw * 0.62 - 1.0, zpm], [xb + 18, -5, zpm - 12], [jb - 0.6, -5, jz0 + 22]], r=2.5))
    return out


def dock_body(origin, plate_t, R=1.0):
    """the BODY side of the dock (on a robot's neck): floating plug, alignment pins. origin = the dock face (plate top)."""
    from k1_design import cyl
    ox, oy, oz = origin
    px, py, pz = DOCK['plug']['size']
    tx, ty, tz = DOCK['plug']['tongue']
    p = DOCK['pins']
    out = [('elec-dockplug', trimesh.creation.box(bounds=[[ox - px / 2, oy - py / 2, oz - plate_t - pz], [ox + px / 2, oy + py / 2, oz - plate_t]])),
           ('trim-dockplug', trimesh.creation.box(bounds=[[ox - ty / 2, oy - tx / 2, oz], [ox + ty / 2, oy + tx / 2, oz + tz]]))]
    for s_ in (1, -1):
        out.append((f'trim-dockpin.{"l" if s_ > 0 else "r"}', cyl(np.array([ox, oy + s_ * p['y'], oz + p['h'] / 2]), [0, 0, 1.0], p['d'] / 2, p['h'])))
    return out


def dock_cutout(origin, R=1.0):
    """the body's dock plate opening for the plug (x0, y0, x1, y1)."""
    ox, oy, _ = origin
    tx, ty, _ = DOCK['plug']['tongue']
    return (ox - ty / 2 - 0.6, oy - tx / 2 - 0.6, ox + ty / 2 + 0.6, oy + tx / 2 + 0.6)


def standalone(hh=120.6, hw=81.8, R=1.0):
    """the product on its own: the head about its dock origin at the world origin (as it sits on a desk)."""
    HF = frame((0.0, 0.0, 0.0), hh, hw, R)
    return HF, shell(HF) + insides(HF) + wiring(HF)
