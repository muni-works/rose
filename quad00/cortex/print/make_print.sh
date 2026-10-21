#!/bin/zsh
# export each [root] of head-print.loon to its own STL for Bambu Studio
V=${VCAD:-vcad}; cd "${0:a:h}"
names=(tub floor sled ear-grilles exhaust-grille glass)
src=$(sed -n '1,/^\[root /p' head-print.loon | sed '$d')
i=1; grep '^\[root ' head-print.loon | while read -r line; do
  print -r -- "$src
$line" > .print-$i.loon
  $V export .print-$i.loon "stl/cortex-mini-${names[$i]}.stl" && i=$((i+1))
done
rm -f .print-*.loon
