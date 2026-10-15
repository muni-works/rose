"""Unconnected items per net from kicad-cli's JSON DRC (its list caps near 500).

    python3 drcnets.py brain.kicad_pcb [N]
"""
import collections, json, re, subprocess, sys
subprocess.run(['/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli', 'pcb', 'drc', '--format', 'json', '-o', '/tmp/drc.json', sys.argv[1]],
               capture_output=True)
d = json.load(open('/tmp/drc.json'))
c = collections.Counter()
for v in d.get('unconnected_items', []):
    m = re.search(r'\[([^\]]+)\]', v['items'][0]['description'])
    c[m.group(1) if m else '?'] += 1
for k, n in c.most_common(int(sys.argv[2]) if len(sys.argv) > 2 else 15):
    print(f'{n:4d} {k}')
