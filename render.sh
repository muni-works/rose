#!/bin/sh
# Render each variant from the geometry that actually exists in this repo.
#
# vcad-render takes .loon directly (an older note said it needed a loon2vcad
# conversion first -- that is no longer true). Build the binary with:
#   cargo build --release -p vcad-render     # in ~/Developer/vcad
#
# pro takes ~2 min now (66 plates / 115 roots, but one window instead of a
# field of bores on each torso face - it was ~6 min with the hole fields).
# mini is ~5 s, max ~9 s.
#
# pro/cad is MODULES: rose-pro.loon is the roots-only assembly; links/<link>.loon
# is one module per link (its own roots render standalone, and an importer does
# not see them); actuators.loon the envelopes and pushrods; plates.loon the plate
# vocabulary (vcad repo, hardware/rose-pro, on VCAD_LOON_PATH below). vcad's
# root-mesh cache is keyed per root, so a one-link edit re-cuts that link only:
#   $R pro/cad/links/thigh.loon --azimuth 30 --elevation 15 --size 800 --fill 0.9 --auto-aspect -o /tmp/thigh.png
# is 0.3 s warm. A cold cache (new kernel build) is ~5 min for the whole body.
#
# zsh does NOT word-split an unquoted $VAR, so the OPTS below only works because
# this file is #!/bin/sh. Typing the same line at a zsh prompt hands vcad-render
# one argument and a usage error.
#
# Two vcad boolean pathologies cost real time finding, recorded so they are not
# found twice: (1) plate corner radius is NOT monotonic in cost - r=25 was ~10x
# slower than r=20 on the same plate (see rose_pro_loon.corner_radius);
# (2) unioning a link's plates into one solid, where lapped/stacked plates touch
# face to face, sent R_shank past 400 s while its mirror L_shank took 10 s. The
# generator now emits one root per plate and never unions plates.
set -e
R="${VCAD_RENDER:-$HOME/Developer/vcad/target/release/vcad-render}"
# pro/cad/rose-pro.loon is roots only; its plate vocabulary is the loon module
# hardware/rose-pro/plates.loon in vcad. [use plates] finds it on the lib path.
export VCAD_LOON_PATH="${VCAD_LOON_PATH:-$HOME/Developer/vcad/hardware/rose-pro}"
# Evaluated roots are cached under ~/.cache/vcad (keyed on expression + kernel
# build), so a re-render after a one-plate edit is seconds, not minutes.
OPTS="--view iso --size 1400 --fill 0.92 --auto-aspect"

$R mini/cad/k1.loon        $OPTS -o mini/render/rose-mini-iso.png
$R max/cad/rose1-v6.loon   $OPTS -o max/render/rose-max-iso.png
$R pro/cad/rose-pro.loon   $OPTS -o pro/render/rose-pro-iso.png
# profile and front, explicit camera (--view side is not a reliable profile)
$R pro/cad/rose-pro.loon   --azimuth 90 --elevation 0 --size 1400 --fill 0.92 --auto-aspect -o pro/render/rose-pro-side.png
$R pro/cad/rose-pro.loon   --azimuth 0  --elevation 0 --size 1400 --fill 0.92 --auto-aspect -o pro/render/rose-pro-front.png
