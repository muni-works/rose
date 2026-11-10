#!/bin/sh
# loon circuit -> KiCad schematic, checked and rendered.
# Needs vcad with loon circuits (EcadSymbol / EcadConn, .kicad_sch export) and KiCad 9.
set -e
cd "$(dirname "$0")"
VCAD=${VCAD:-vcad}
# board art: brand/muni.pretty (shared) and art.pretty (this board)
export VCAD_KICAD_FOOTPRINTS="$(cd ../brand && pwd):$(pwd)${VCAD_KICAD_FOOTPRINTS:+:$VCAD_KICAD_FOOTPRINTS}"
KICAD_CLI=${KICAD_CLI:-/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli}
$VCAD export arm_controller.loon arm_controller.kicad_sch
python3 netcheck.py arm_controller.loon arm_controller.kicad_sch      # every pin on its loon net, nothing merged
$KICAD_CLI sch erc --severity-error --exit-code-violations -o erc.rpt arm_controller.kicad_sch 2>/dev/null
$KICAD_CLI sch export pdf -o arm_controller.pdf arm_controller.kicad_sch 2>/dev/null
echo ok
