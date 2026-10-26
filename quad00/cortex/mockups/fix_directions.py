import subprocess, ast

s = subprocess.run(['git', 'show', 'HEAD:quad00/cortex/mockups/directions.py'], capture_output=True, text=True).stdout

s = s.replace('W, D, H = 128.0, 150.0, 172.0        # deeper than wide, taller than wide: a head', '''# two sizes, one head language. A human head is ~155 W x 195 D x 230 H; the 7" (166 wide) forces the full to ~1.3x.
SIZES = {
    # 4" square (Waveshare 4inch HDMI LCD (C), 720x720, active ~72 x 72)
    "mini": dict(W=128.0, D=150.0, H=172.0, ACT_W=72.0, ACT_H=72.0, UI="ui-eyes-720x720.png"),
    # 7" (Waveshare 7inch HDMI LCD (C), 1024x600, active 154.21 x 85.92; outline 164.9 x 107)
    "full": dict(W=200.0, D=215.0, H=235.0, ACT_W=154.21, ACT_H=85.92, UI="ui-eyes-1024x600.png"),
}


def use_size(name):
    # rebind the envelope the option builders read (they read these module globals at call time).
    globals().update(SIZES[name])


use_size("mini")''')

s = s.replace('    ex, ez, er = -10.0, 0.50 * H, 23.0',
              '    ex, ez, er = -10.0 * W / 128.0, 0.50 * H, 23.0 * W / 128.0')
s = s.replace('    lo, hi = 0.28 * H, 0.74 * H',
              '    lo, hi = (0.28 * H, 0.74 * H) if H < 200 else (0.25 * H, 0.76 * H)')
s = s.replace('    Rf, xf = 260.0, a[0] - 6.0',
              '    Rf, xf = 260.0 * W / 128.0, a[0] - 6.0             # the same bow at every size')
s = s.replace('''        for i in range(-10, 11):
            for k in range(-11, 12):''', '''        ni, nk = int(er / 2.2) + 1, int(er / 1.905) + 1
        for i in range(-ni, ni + 1):
            for k in range(-nk, nk + 1):''')
s = s.replace('''    a = ACT / 2
    mname = f"face_{tag}"''', '''    aw, ah = ACT_W / 2, ACT_H / 2
    mname = f"face_{tag}"''')
s = s.replace('''        mat = (f'[material-emissive "{mname}" 0.006 0.006 0.008 0.0 0.04 1.0 1.0 1.0 3.0 "{UI}" '
               f'{hi_x:.3f} {dy - a:.3f} {zc + a:.3f} 0.0 {2 * a:.3f} 0.0 0.0 0.0 {-2 * a:.3f}]')''', '''        mat = (f'[material-emissive "{mname}" 0.006 0.006 0.008 0.0 0.04 1.0 1.0 1.0 3.0 "{UI}" '
               f'{hi_x:.3f} {dy - aw:.3f} {zc + ah:.3f} 0.0 {2 * aw:.3f} 0.0 0.0 0.0 {-2 * ah:.3f}]')''')

# D: also scale its ear geometry with W (it reads the mini globals at call time)
s = s.replace('    ex, ez, er = -22.0 * W / 128.0, 0.50 * H, 22.0 * W / 128.0',
              '    ex, ez, er = -22.0 * W / 128.0, 0.50 * H, 22.0 * W / 128.0')

open('directions.py', 'w').write(s)
ast.parse(s)
print('base restored, lines', s.count('\n'))
