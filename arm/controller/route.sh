#!/bin/sh
# Route the board with vcad (~30 min) and check it in KiCad. Run build.sh first.
set -e
cd "$(dirname "$0")"
VCAD=${VCAD:-vcad}
KICAD_CLI=${KICAD_CLI:-/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli}
export VCAD_KICAD_FOOTPRINTS="$(cd ../../brand && pwd):$(pwd)${VCAD_KICAD_FOOTPRINTS:+:$VCAD_KICAD_FOOTPRINTS}"
# continuous currents: the 48 V path and ground carry the motors, the brake path its resistor
$VCAD route arm_controller.loon arm_controller.kicad_pcb \
  --current VIN=40 --current VBUS_SW=40 --current VBUS=40 --current GND=40 \
  --current BRK_R=16 --current +12V=1 --current +5V=1.5
$KICAD_CLI pcb drc --format report -o drc.rpt arm_controller.kicad_pcb 2>/dev/null | grep -E "violations|unconnected" || true
