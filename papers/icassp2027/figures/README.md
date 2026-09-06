# Mainline paper figures and Lark schematic

## Reproducible paper figures

The paper uses three Python figures with distinct roles:

| Figure | Source stem | Role |
|:--|:--|:--|
| 1 | `method_paper` | Motivation and architecture: retain, repair, then distinguish scheduled links from delivered packets. |
| 2 | `packet_fusion_mechanism` | Mechanism: a missing packet changes surviving weights, spatial overlap and Bernoulli existence. |
| 3 | `routing_tradeoff` | Results: attempted payload versus E-OSPA, plus fixed-count localization-only improvement ceilings. |

The mechanism is an exact illustrative Gaussian calculation, not a tracking
experiment or a ranking against truth. The ceiling bars are post-hoc bounds,
not achieved improvements. All figures have vector PDF, editable-text SVG
and 600-dpi PNG exports. `FIGURE_CONTRACT.md` defines the claims and layout.
`paper_figures_manifest.json` records the introductory/result sources and
sample boundaries; `packet_fusion_mechanism_manifest.json` records the
separate analytic assumptions and exports.

From the repository root:

```sh
octave --no-gui --quiet --eval "addpath('papers/icassp2027/figures'); exportPaperFigureData();"
/Users/dex/miniconda3/bin/python3 papers/icassp2027/figures/plot_paper_figures.py
/Users/dex/miniconda3/bin/python3 papers/icassp2027/figures/plot_packet_fusion_mechanism.py
octave --no-gui --quiet --eval "addpath('papers/icassp2027/figures'); exportPaperGuardrailData();"
```

The exporter reads the already completed MAT results, not rounded paper
tables. `routing_tradeoff_source.csv` has seven episode rows;
`count_budget_source.csv` has six arm-level diagnostic rows. Replotting needs
only these committed CSV files and Python with Matplotlib and NumPy. The
Octave re-export additionally requires the locally retained source MAT files
in the worktrees named in the CSV. Neither command runs a filter.
Use `--method-only` to revise the introductory schematic without redrawing the
result figure. The illustrative graphs contain 13 planned messages before
and after repair and 12 delivered messages after one gateway packet is lost.
The independent mechanism renderer needs only NumPy and Matplotlib; it writes
the input/weight CSV and every plotted ordinate. `paired_guardrail_source.csv`
feeds Table 2: common-finite RMSE support/gains and the worst formation's gain.
The latter uses each arm's original finite cells, not a common-cell restriction.
No plot or table exporter runs a filter or treats sensor-time cells as
independent statistical replicates.

The final manuscript render has the three figures on pages 2, 3 and 4,
respectively, with two result tables on page 4. Main text and conclusion fit
within four pages; page 5 contains declarations and 15 references. The
producing agent inspected all rendered pages and the analytic PNG. No new
filter experiment or independent scientific verification was performed for
this presentation update.

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
