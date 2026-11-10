"""Check the exported schematic against the loon circuit: every pin on the net loon put it on,
and no two loon nets merged. Usage: python3 netcheck.py arm_controller.loon arm_controller.kicad_sch"""
import collections, re, subprocess, sys, tempfile, xml.etree.ElementTree as ET

KICAD_CLI = '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
loon, sch = sys.argv[1], sys.argv[2]
src = open(loon).read()
want = {}
q = r'"([^"]*)"'
for m in re.findall(r'\[(?:r|c) %s %s %s %s\]' % (q, q, q, q), src):
    want[(m[0], '1')], want[(m[0], '2')] = m[2], m[3]
for ref, a, b in [(m[0], m[2], m[3]) for m in re.findall(r'\[cfp %s %s \S+ %s %s\]' % (q, q, q, q), src)]:
    want[(ref, '1')], want[(ref, '2')] = a, b
for ref, net in re.findall(r'\[flag %s %s\]' % (q, q), src):
    want[(ref, '1')] = net
for ref, body in re.findall(r'\[part %s [^\n]*\n\s*#\[([^\]]*)\]\]' % q, src):
    toks = re.findall(q, body)
    for pin, net in zip(toks[::2], toks[1::2]):
        want[(ref, pin)] = net
with tempfile.NamedTemporaryFile(suffix='.xml') as t:
    subprocess.run([KICAD_CLI, 'sch', 'export', 'netlist', '--format', 'kicadxml', '-o', t.name, sch],
                   check=True, capture_output=True)
    got = {(n.get('ref'), n.get('pin')): net.get('name')
           for net in ET.parse(t.name).getroot().iter('net') for n in net.iter('node')}
groups = collections.defaultdict(set)
for k, n in want.items():
    groups[got.get(k)].add(n)
merged = {g: v for g, v in groups.items() if len(v) > 1}
missing = [k for k in want if k not in got]
print('%d pins, %d loon nets -> %d KiCad nets' % (len(want), len(set(want.values())), len(set(got.values()))))
for g, v in merged.items():
    print('MERGED in KiCad:', sorted(v))
if missing:
    print('pins missing in KiCad:', missing)
sys.exit(1 if merged or missing else 0)
