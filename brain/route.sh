#!/bin/sh
# place, fan out, route (vcad, HDI), then check in KiCad. Run build.sh first.
set -e
cd "$(dirname "$0")"
B=${BOARD:-brain}
VCAD=${VCAD:-vcad}
KICAD_CLI=${KICAD_CLI:-/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli}
export VCAD_KICAD_SYMBOLS="$(pwd)/lib"
export VCAD_KICAD_FOOTPRINTS="$(pwd):$(cd ../arm/controller && pwd)"
[ "${SKIP_PLACE:-0}" = 1 ] || python3 place.py
python3 fanout.py
date +%T
PRIO=$(grep -oE '"D[01][A-D]_[A-Za-z0-9_]+"' $B.loon | tr -d '"' | sort -u | sed 's/^/--priority /' | tr '\n' ' ')
RUST_LOG=${RUST_LOG:-info} $VCAD route --microvia 0.25,0.10 ${FAST:+--fast} ${ESCAPE:+--escape $ESCAPE} --effort ${EFFORT:-1} $PRIO $B.loon $B.kicad_pcb 2>&1 | tee route.vcad.log | grep -v "^\s" | tail -25
date +%T
python3 finish.py $B.kicad_pcb
KICAD_PY=${KICAD_PY:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}
"$KICAD_PY" ../arm/controller/fill_zones.py $B.kicad_pcb >/dev/null 2>&1 || true
$KICAD_CLI pcb drc -o drc.rpt $B.kicad_pcb >/dev/null 2>&1 || true
grep -E "Found" drc.rpt
"$KICAD_PY" unconnected.py $B.kicad_pcb 2>/dev/null | head -1
grep -E "^\[" drc.rpt | sed 's/:.*//' | sort | uniq -c | sort -rn
