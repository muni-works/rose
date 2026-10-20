#!/bin/zsh
# The router's parallel search makes each run a little different: route N
# times (default 3), keep the board with the fewest DRC items.
cd "$(dirname "$0")"
N=${1:-3}
best=999999
for i in $(seq 1 $N); do
  RENDER=0 ./route.sh > route.log 2>&1
  n=$(grep -E "^\*\* Found [0-9]+ (DRC violations|unconnected pads)" drc.rpt | grep -oE "[0-9]+" | paste -sd+ - | bc)
  echo "run $i: $n DRC items"
  if [ "$n" -lt "$best" ]; then
    best=$n
    cp arm_controller.kicad_pcb .best.kicad_pcb; cp drc.rpt .best.rpt; cp route.log .best.log
  fi
  [ "$best" = 0 ] && break
done
mv .best.kicad_pcb arm_controller.kicad_pcb; mv .best.rpt drc.rpt; mv .best.log route.log
echo "kept: $best DRC items"
RENDER_ONLY=1 ./route.sh >/dev/null 2>&1 || true
