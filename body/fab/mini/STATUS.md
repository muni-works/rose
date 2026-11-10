# mini v1 fab -- STATUS (cut 2026-10-20, Cam: "Cut mini fab")

Stopped. Nothing ordered, no PR. Package docs: ../README.md.

**Done:** the mini and cortex geometry are frozen. The changes are the dock windows and the spine moving to the pogo block. `cortex.py` is deleted, and the head proxies now come from cortex.loon (`cortex/export.sh` -> `cortex/stl`). The plate DXFs are nested on 4 x 390 sheets and the nest is validated. Cortex STEP/STL exported. Glass DXF. BOM.csv. MEASURE.md replaces the SPEC.md TBDs. ROM 12/12 ok. harness_check.py passes (46 mm service loops).

**Partial:** the printed skins are generated (2 mm hollow, auto-split), not designed: no bosses or fasteners. BOM prices: RS06, Jetson, cells and most head parts are unpriced.

**Left (needs Cam / shape work):** the neck magnets overhang the dock plate. chassis-rollmount is clipped by its shell, so its DXF is cut from the unclipped profile. Three covers are empty (shoulder, shoulderyoke, neck). The RS06 bolt patterns are ASSUMED. The limbic-package parts are not included.
