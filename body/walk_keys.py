# python walk_keys.py TRACE.booster.tsv OUT.json T0 DUR FPS [ARM_SWING]  -> vcad-render vcad/mini/mini.vcad --animate OUT.json
"""K1 hardware walk TSV (ipse-booster-tsv v2, q_* columns, rad, 50 Hz) -> a vcad-render --animate timeline for rose mini."""
import sys, json, math
src, out, t0, dur, fps = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), int(sys.argv[5])
lines = [l.rstrip('\n') for l in open(src)]
cols = lines[1].lstrip('# ').split('\t')
rows = [l.split('\t') for l in lines[2:] if l and not l.startswith('#')]
T = [float(r[cols.index('t_policy_s')]) for r in rows]
MAP = {'AAHead_yaw': 'head_yaw', 'Head_pitch': 'head_pitch'}
for side, t in (('Left', 'l'), ('Right', 'r')):
    MAP[f'A{side}_Shoulder_Pitch'] = f'{t}_shoulder_pitch'; MAP[f'{side}_Shoulder_Roll'] = f'{t}_shoulder_roll'
    MAP[f'{side}_Elbow_Pitch'] = f'{t}_upperarm_rot'; MAP[f'{side}_Elbow_Yaw'] = f'{t}_elbow'
    for j in ('Hip_Pitch', 'Hip_Roll', 'Hip_Yaw', 'Knee_Pitch', 'Ankle_Pitch', 'Ankle_Roll'):
        MAP[f'{side}_{j}'] = f'{t}_{j.lower()}'
# the mini is modelled arms-down: its zero is the K1 T-pose with this shoulder roll baked in (k1.arms_down())
REST = {'Left_Shoulder_Roll': -77.35, 'Right_Shoulder_Roll': 77.35}
tracks = []
for k1, ours in MAP.items():
    c = cols.index('q_' + k1)
    keys = [{'t': round(T[i] - T[0] - t0, 4), 'value': round(math.degrees(float(rows[i][c])) - REST.get(k1, 0.0), 3)} for i in range(len(rows)) if t0 <= T[i] - T[0] <= t0 + dur]
    tracks.append({'joint': ours, 'keys': keys})
# arms: this run drifted them out (--arm-drift 1.2). Show them relaxed at the sides instead, swinging against the
# opposite leg: shoulder pitch follows the other hip's pitch (about its mean) x SWING, a slight elbow bend.
SWING = float(sys.argv[6]) if len(sys.argv) > 6 else 0.6
if SWING:
    by = {t_['joint']: t_ for t_ in tracks}
    for t, o in (('l', 'r'), ('r', 'l')):
        hp = by[f'{o}_hip_pitch']['keys']; m = sum(k['value'] for k in hp) / len(hp)
        by[f'{t}_shoulder_pitch']['keys'] = [{'t': k['t'], 'value': round(SWING * (k['value'] - m), 3)} for k in hp]
        for j, v in ((f'{t}_shoulder_roll', 0.0), (f'{t}_upperarm_rot', 0.0), (f'{t}_elbow', -18.0 if t == 'l' else 18.0)):
            by[j]['keys'] = [{'t': k['t'], 'value': v} for k in hp]
json.dump({'durationS': dur, 'fps': fps, 'tracks': tracks}, open(out, 'w'))
print(len(tracks), 'tracks,', len(tracks[0]['keys']), 'keys each, t', t0, '..', t0 + dur)
