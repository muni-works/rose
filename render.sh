#!/bin/sh
# Render each variant from the geometry that actually exists in this repo.
#
# vcad-render takes .loon directly (an older note said it needed a loon2vcad
# conversion first -- that is no longer true). Build the binary with:
#   cargo build --release -p vcad-render     # in ~/Developer/vcad
#
# pro takes ~2.5 min: 40 bent-sheet plates, each with a bolt circle cut out of
# it, is a lot of boolean work. mini is ~5 s, max ~9 s.
set -e
R="${VCAD_RENDER:-$HOME/Developer/vcad/target/release/vcad-render}"
OPTS="--view iso --size 1400 --fill 0.92 --auto-aspect"

$R mini/cad/k1.loon        $OPTS -o mini/render/rose-mini-iso.png
$R max/cad/rose1-v6.loon   $OPTS -o max/render/rose-max-iso.png
$R pro/cad/rose-pro.loon   $OPTS -o pro/render/rose-pro-iso.png
