"""RK3588S ball list from the datasheet (Table 2-1) -> lib/rk3588s_pins.csv.

    python3 lib/rk3588s_pins.py > lib/rk3588s_pins.csv

Reads the table cells with pdfplumber (names wrap across lines; cells don't).
Checks: 1253 unique balls (the package's count), every ball a JEDEC row
(A..Y, AA..AY, BA.., no I O Q S X Z) and a column 1..42, no ball named twice.
"""
import re, sys
import pdfplumber

PDF = 'reference/Rockchip_RK3588S_Datasheet_V1.6-20240821.pdf'
L = 'ABCDEFGHJKLMNPRTUVWY'
ROWS = list(L) + ['A' + r for r in L] + ['B' + r for r in L]
BALL = re.compile(r'^([A-Z]{1,2})(\d{1,2})$')

out = {}
with pdfplumber.open(PDF) as pdf:
    for p in pdf.pages:
        t = p.extract_text() or ''
        if 'Pin Name' not in t or 'Table 2-1' not in t and not out:
            continue
        if 'Chapter 3' in t and out:
            break
        for tab in p.extract_tables():
            for row in tab:
                cells = [c for c in row if c]
                for name, ball in zip(cells[0::2], cells[1::2]):
                    name, ball = name.replace('\n', ''), ball.strip()
                    if name == 'Pin Name':
                        continue
                    m = BALL.match(ball)
                    if not m or m.group(1) not in ROWS or not 1 <= int(m.group(2)) <= 42:
                        sys.exit(f'bad ball {ball!r} for {name!r}')
                    if ball in out:
                        sys.exit(f'ball {ball} twice: {out[ball]} / {name}')
                    out[ball] = name
if len(out) != 1253:
    sys.exit(f'{len(out)} balls, want 1253')
key = lambda k: (ROWS.index(BALL.match(k).group(1)), int(BALL.match(k).group(2)))
print('ball,name')
for k in sorted(out, key=key):
    print(f'{k},{out[k]}')
