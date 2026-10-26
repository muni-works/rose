#!/bin/sh
# cortex.loon -> one STL per part (stl/, the head proxies the Python checks load) and STEP + STL for the parts
# someone makes (../fab/cortex/). Run from this folder:  sh export.sh   (VCAD=path/to/vcad to override)
set -e
VCAD=${VCAD:-${CARGO_TARGET_DIR:-$HOME/Developer/vcad/target}/release/vcad}
MAKE="cortex-tub cortex-floor cortex-ear-grilles cortex-exhaust-grille cortex-sled cortex-sled-print cortex-back-grille cortex-tub-d4 d4-glass d4-body cortex-keeper dock-pawl dock-release dock-face"
BUY="cortex-glass cortex-padboard cortex-pads cortex-strikes cortex-display cortex-cams cortex-dockboard cortex-power
     cortex-battery cortex-compute cortex-heatsink cortex-speakers cortex-mics cortex-amps cortex-cables
     dock-pogo dock-pogo-block dock-cones dock-magnets"
mkdir -p stl ../fab/cortex
tmp=$(mktemp -d .exportXXXX)
trap 'rm -rf "$tmp"' EXIT
for p in $MAKE $BUY; do
  printf '[use cortex [%s]]\n[root %s "x"]\n' "$p" "$p" > "$tmp/$p.loon"
  cp cortex.loon "$tmp/"
  "$VCAD" export "$tmp/$p.loon" "stl/$p.stl" >/dev/null
  case " $MAKE " in *" $p "*)
    cp "stl/$p.stl" "../fab/cortex/$p.stl"
    "$VCAD" export "$tmp/$p.loon" "../fab/cortex/$p.step" >/dev/null ;;
  esac
  echo "$p"
done
