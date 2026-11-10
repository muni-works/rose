#!/bin/zsh
# fab outputs for pcbway: gerbers + drill (zip), pick-and-place, bom
set -e
cd "${0:A:h}"
K=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
B=arm_controller.kicad_pcb
rm -rf fab && mkdir -p fab/gerbers
$K pcb export gerbers --no-protel-ext -o fab/gerbers/ $B >/dev/null
$K pcb export drill --format excellon --excellon-separate-th --generate-map --map-format gerberx2 -o fab/gerbers/ $B >/dev/null
(cd fab/gerbers && zip -qr ../arm_controller_gerbers.zip .)
$K pcb export pos --format csv --units mm --side both --exclude-dnp -o fab/arm_controller_pos.csv $B >/dev/null
$K sch export bom --fields 'Reference,Value,Footprint,${QUANTITY},MPN,Manufacturer,LCSC,${DNP}' \
  --group-by Value,Footprint,MPN --exclude-dnp -o fab/arm_controller_bom.csv arm_controller.kicad_sch >/dev/null
python3 pcbway_bom.py fab/arm_controller_bom.csv fab/arm_controller_bom_pcbway.csv
ls fab
