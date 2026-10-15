#!/bin/sh
# brain: tables -> loon -> KiCad schematic, checked and plotted.
set -e
cd "$(dirname "$0")"
VCAD=${VCAD:-vcad}
KICAD_CLI=${KICAD_CLI:-/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli}
export VCAD_KICAD_SYMBOLS="$(pwd)/lib"
export VCAD_KICAD_FOOTPRINTS="$(pwd):$(cd ../arm/controller && pwd)${VCAD_KICAD_FOOTPRINTS:+:$VCAD_KICAD_FOOTPRINTS}"
python3 lib/gen_lib.py
python3 gen_brain.py
$VCAD export brain.loon brain.kicad_sch
python3 ../arm/controller/netcheck.py brain.loon brain.kicad_sch
$KICAD_CLI sch erc --severity-error -o erc.rpt brain.kicad_sch >/dev/null 2>&1 || true
grep -E "^\[" erc.rpt | sed 's/:.*//' | sort | uniq -c
$KICAD_CLI sch export pdf -o brain.pdf brain.kicad_sch >/dev/null 2>&1
echo ok
