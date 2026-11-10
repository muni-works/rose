"""Motion video of the fork arm: keyframed poses -> vcad-render drafting frames -> mp4.

Run:  python3 arm/animate.py OUT_DIR [workers]      (frames in OUT_DIR, video in render/)
Frames already in OUT_DIR are skipped, so an interrupted run resumes.
"""
import math, os, shutil, subprocess, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, '..'))
import rose_fork as F

K = [  # (s, deg): J1 yaw, J2 shoulder, J4 elbow, J5 forearm roll, J6 wrist pitch, J7 tool roll
    (0.0, dict(J1_yaw=0, J2_sho_pitch=-80, J4_elbow=100, J5_fore_roll=0, J6_wrist_pitch=-60, J7_tool_roll=0)),
    (1.6, dict(J1_yaw=0, J2_sho_pitch=-25, J4_elbow=60, J5_fore_roll=0, J6_wrist_pitch=55, J7_tool_roll=0)),
    (3.0, dict(J1_yaw=-60, J2_sho_pitch=-10, J4_elbow=45, J5_fore_roll=0, J6_wrist_pitch=55, J7_tool_roll=90)),
    (4.2, dict(J1_yaw=-60, J2_sho_pitch=-10, J4_elbow=45, J5_fore_roll=80, J6_wrist_pitch=30, J7_tool_roll=-90)),
    (5.6, dict(J1_yaw=50, J2_sho_pitch=-40, J4_elbow=80, J5_fore_roll=-60, J6_wrist_pitch=40, J7_tool_roll=0)),
    (7.0, dict(J1_yaw=50, J2_sho_pitch=0, J4_elbow=0, J5_fore_roll=0, J6_wrist_pitch=0, J7_tool_roll=0)),
    (8.6, dict(J1_yaw=0, J2_sho_pitch=-80, J4_elbow=100, J5_fore_roll=0, J6_wrist_pitch=-60, J7_tool_roll=0)),
]
FPS = 15
R = os.path.expanduser('~/Developer/vcad/target/release/vcad-render')


def pose(t):
    for (t0, a), (t1, b) in zip(K, K[1:]):
        if t0 <= t <= t1:
            s = (t - t0) / (t1 - t0)
            s = s * s * (3 - 2 * s)
            return {j: math.radians(a[j] + (b[j] - a[j]) * s) for j in a}
    return {j: math.radians(v) for j, v in K[-1][1].items()}


def frames(out, w, k):
    n = int(K[-1][0] * FPS) + 1
    for i in range(w, n, k):
        png = os.path.join(out, 'f%04d.png' % i)
        if os.path.exists(png):
            continue
        src = F.world_loon(pose(i / FPS))
        # fixed framing: two tiny anchors at opposite corners of the whole reach
        # envelope (the tool swings below the base and behind it), so no pose
        # grows the bounding box and the camera never rescales between frames
        src += '[let a0 [translate -230 -620 -230 [sphere 0.3]]]\n[let a1 [translate 700 680 660 [sphere 0.3]]]\n[root a0 "base"]\n[root a1 "base"]\n'
        lf = os.path.join(out, 'f%d.loon' % w)
        open(lf, 'w').write(src)
        try:
            subprocess.run([R, lf, '--azimuth=-45', '--elevation=18', '--size', '1280x800', '--fill', '0.95', '-o', png],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=600)
        except subprocess.TimeoutExpired:       # a pose that hangs the renderer: skipped, filled below
            pass


if __name__ == '__main__':
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    if len(sys.argv) > 3:                       # worker
        frames(out, int(sys.argv[2]), int(sys.argv[3]))
        sys.exit()
    k = int(sys.argv[2]) if len(sys.argv) > 2 else 8
    ps = [subprocess.Popen([sys.executable, __file__, out, str(w), str(k)]) for w in range(k)]
    for p in ps:
        p.wait()
    # any frame the renderer gave up on repeats the frame before it
    n = int(K[-1][0] * FPS) + 1
    for i in range(1, n):
        a, b = os.path.join(out, 'f%04d.png' % (i - 1)), os.path.join(out, 'f%04d.png' % i)
        if not os.path.exists(b) and os.path.exists(a):
            shutil.copy(a, b)
            print('frame %d: renderer timed out, held frame %d' % (i, i - 1))
    mp4 = os.path.join(HERE, 'render', 'fork-v9-motion.mp4')
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', str(FPS), '-i', os.path.join(out, 'f%04d.png'),
                    '-vf', 'scale=1280:-2,format=yuv420p', '-c:v', 'libx264', '-crf', '20', mp4], check=True)
    print(mp4)
