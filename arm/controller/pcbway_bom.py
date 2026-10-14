"""kicad bom + bom_mpn.csv -> pcbway assembly bom. resistors go by value (1%, 0402, yageo rc).
dnp: footprint-only or undecided parts, never sent to the assembler."""
import csv, sys
DNP = {'H', 'PF', 'J4', 'R68', 'SH1', 'J9'}   # holes, flags, tag-connect pads, id strap (arm), TBD parts
mpn = {}

def yageo(v):  # 10.2k -> 10K2, 49.9 -> 49R9, 10k -> 10K, 1M -> 1M
    unit = v[-1].upper() if v[-1] in 'kM' else 'R'
    num = v[:-1] if unit != 'R' else v
    return num.replace('.', unit) if '.' in num else num + unit
for r in csv.DictReader(open('bom_mpn.csv')):
    mpn[r['value']] = (r['manufacturer'], r['mpn'])
out = csv.writer(open(sys.argv[2], 'w', newline=''))
out.writerow(['Item', 'Designator', 'Qty', 'Manufacturer', 'Mfg Part #', 'Description / Value', 'Package/Footprint', 'Type'])
missing, n = [], 0
for r in csv.DictReader(open(sys.argv[1])):
    refs = r['Reference']
    first = refs.split(',')[0].split('-')[0]
    if first in DNP or first.rstrip('0123456789') in DNP:
        continue
    v, fp = r['Value'], r['Footprint'].split(':')[-1]
    m = mpn.get(v + '@' + fp) or mpn.get(v)
    if m is None and first.startswith('R'):
        size = '0603' if '0603' in fp else '0402'
        m = ('Yageo', 'RC%sFR-07%sL' % (size, yageo(v.split()[0])) if v != '0' else 'RC0402JR-070RL')
    if m is None:
        missing.append(v); continue
    n += 1
    kind = 'THT' if any(s in fp for s in ('THT', 'Radial', 'XT60', 'XT30', 'RJ45')) else 'SMD'
    out.writerow([n, refs, r['QUANTITY'], m[0], m[1], v, fp, kind])
print('%d lines' % n, '; missing:', missing or 'none')
