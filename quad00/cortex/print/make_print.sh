#!/bin/zsh
# export each [root] of head-print.loon to its own STL for Bambu Studio
V=${VCAD:-vcad}; cd "${0:a:h}"
SRC=${1:-head-print.loon}; TAG=${2:-mini}
names=(tub floor sled ear-grilles exhaust-grille glass)
src=$(sed -n '1,/^\[root /p' $SRC | sed '$d')
i=1; grep '^\[root ' $SRC | while read -r line; do
  print -r -- "$src
$line" > .print-$i.loon
  $V export .print-$i.loon "stl/cortex-${TAG}-${names[$i]}.stl" && i=$((i+1))
done
rm -f .print-*.loon
