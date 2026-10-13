# Elbow rig — turned part

**v9: the rig uses the arm's RS04 sleeve exactly - see `../fab/SLEEVES.md` and
`../fab/sleeve_RS04.svg`.** The rig is back-mounted like the arm: parent B bolts to the RS04
back face (10x M4 on Ø106) with the sleeve flange under the same screws; parent A is a Ø102
clearance plate round the Ø98 nose. The drawing below is the v6 sleeve, kept for the record.


## Sleeve boss (1 off, 6061)

Bolts through parent plate B to the RS04 back face (v5: the elbow is an RS04). Child plate B rides a
flanged plain bushing on its spigot. The cable enters through the side slot and
leaves along the axis, so it only twists across the joint.

```
           flange                 spigot (bushing seat)        retaining groove
   |<---- 4 ---->|<------------------ 30 ------------------->|
   +-------------+                                            
   |             +--------------------------------------+--+-+
   |  D 116      |   D 16 g6                            |  | |  DIN 471-16 groove:
   |             |                                      |  | |  D 15.2 x 1.1 wide, 1.5 from the end
   |   ======== bore D 9 through (cable) =================== |
   |             |                                      |  | |
   |             +--------------------------------------+--+-+
   +-------------+
```

| feature | size |
|---|---|
| flange | Ø116 × 4, 10× Ø4.5 on Ø106 (match parent plate B and the back face) |
| spigot | Ø16 g6 × 30 — running fit for the igus GFM-1618-06 bushing (ID 16); spans the 18 mm cable gap (v6) |
| bore | Ø9 through, edges broken R1 (cable) |
| cable slot | 9 wide × 12 long through the spigot wall into the bore, inside the cable gap |
| groove | DIN 471-16 retaining ring, 1.5 from the spigot end |
| finish | spigot Ra 0.8 (bushing running surface), rest as machined |

Stack along the spigot (v6): parent plate B (3) is under the flange; then the 18 mm
cable gap, where each cable leg lies on its own plate; child plate B (3) on the
bushing; bushing flange (1.5) outboard; ring; then the hub cap (Ø40 × 12) over
the end, where the cable turns. The cable slot is in the spigot wall, inside the gap.

## Spacer disc (cut, not turned)

Ø48 × 3 from the same 3 mm sheet (in the nest). 8× Ø5.5 on Ø36. Bolts with
8× M5 × 12 through child plate A and the disc into the RS04 output (thread depth 6.5).
