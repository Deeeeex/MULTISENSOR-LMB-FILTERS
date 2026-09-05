# Mainline method schematic

## Reproducible paper figures

The paper uses two separate Python figures: `method_paper` explains the
scheduled graph and missing-input weights; `routing_tradeoff` shows attempted
payload versus E-OSPA for every executed main-table arm, alongside the
sparse arms' remaining fixed-count localization-only reduction ceilings.
The ceiling bars are analytic post-hoc bounds, not achieved improvements.
Both figures have vector PDF,
editable-text SVG and 600-dpi PNG exports. `FIGURE_CONTRACT.md` defines the
claims and layout before rendering; `paper_figures_manifest.json` records
units, sample size and source commits.

From the repository root:

```sh
octave --no-gui --quiet --eval "addpath('papers/icassp2027/figures'); exportPaperFigureData();"
/Users/dex/miniconda3/bin/python3 papers/icassp2027/figures/plot_paper_figures.py
```

The exporter reads the already completed MAT results, not rounded paper
tables. `routing_tradeoff_source.csv` has seven episode rows;
`count_budget_source.csv` has six arm-level diagnostic rows. Replotting needs
only these committed CSV files and Python with Matplotlib and NumPy. The
Octave re-export additionally requires the locally retained source MAT files
in the worktrees named in the CSV. Neither command runs a filter.

## Canonical Lark schematic

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

The approved SVG overwrite synchronized the existing Lark whiteboard
`Yv1nwom4Bh15o5batldjLRr6prc` on 2026-09-06 (returned node `o1:103`).
`method_mainline_lark-online-preview.jpg` was exported from that same board and
visually checked for the `V278 已测试` label and the `相对原稀疏骨干` X36 footnote.
The content is synchronized; Lark changes text colors/font layout and exports a
square canvas, so the online preview is not pixel-identical to the local PNG.
The SVG overwrite does not preserve native node identities or connector bindings,
as approved by the user.

To regenerate the local PNG on macOS from this directory:

```sh
swift render_method_mainline_lark.swift method_mainline_lark.svg method_mainline_lark.png
```

The local PNG is a browser rendering. Any separately retained online-preview
image is the exported Lark whiteboard rendering and may differ in font layout.
