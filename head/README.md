# head

cortex, rose's head: a detachable module that holds the compute, battery, cameras,
microphones and a screen face. It docks on the neck or sits on a desk.

- [`SPEC.md`](SPEC.md): the specification, with the open numbers marked.
- [`NODE-BRIEF.md`](NODE-BRIEF.md): the "node" redesign, locked at v1.21 for a first print.
- [`CORTEX.md`](CORTEX.md), [`FULL-HEAD.md`](FULL-HEAD.md), [`USB-HEAD.md`](USB-HEAD.md),
  [`CMF.md`](CMF.md), [`NOTES.md`](NOTES.md): the stack, the full-size head, the USB
  variant, colours and finishes, and design notes.
- [`MEASURE.md`](MEASURE.md): bench measurements still to take. Each fills a TBD in SPEC.md.

## Sources and outputs

`cortex.loon` (mini) and `cortex-full.loon` are the design sources; `node.loon` and
`node-full.loon` are the node. Run the scripts from this folder:

```bash
sh export.sh              # cortex.loon -> fab/cortex/*.step|stl, stl/ (proxies the body checks load)
python3 print_build.py    # print-ready parts -> fab/cortex/print/
python3 node_print.py     # node tub + floor -> fab/node/
python3 node_3mf.py       # node print plates (3MF) -> fab/node/
sh node_render.sh         # renders/node/
```

[`fab/`](fab) holds the files to make (`cortex/`, `node/`), [`print/`](print) the
slicer setups, [`face/`](face) the face UI, and [`renders/`](renders) the renders
(`release/` has the launch set).
