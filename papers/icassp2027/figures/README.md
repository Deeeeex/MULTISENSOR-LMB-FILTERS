# Mainline method schematic

`method_mainline_lark.svg` is the editable, self-contained source for the
method-design illustration in the canonical Lark document. The three
formations are a structural example, not an M24/X36 sensor layout.

The figure separates the implemented V242 sparse routing backbone from the
V278 missing-packet receiver rule whose X36 single-seed comparison is complete.
Its set-error/consistency gains trade off against conditional RMSE; it failed
the predefined follow-up gate and is not a cross-scale replacement. Planned
connectivity is not a guarantee of delivered connectivity or tracking gain.

The weight example has self / main-neighbor / weak-neighbor weights of
0.25 / 0.70 / 0.05. When the main packet is absent, renormalization gives
0.8333 / 0.1667 to the surviving inputs; self fallback gives 0.95 / 0.05.
These are two processing rules, not a claim that the candidate is uniformly better.

The local SVG/PNG contain the completed V278 status. The retained online preview
is still the earlier version: the minimal raw-node update was rejected, and an
SVG overwrite awaits user confirmation. Do not treat that preview as synchronized.

To regenerate the local PNG on macOS from this directory:

```sh
swift render_method_mainline_lark.swift method_mainline_lark.svg method_mainline_lark.png
```

The local PNG is a browser rendering. Any separately retained online-preview
image is the exported Lark whiteboard rendering and may differ in font layout.
