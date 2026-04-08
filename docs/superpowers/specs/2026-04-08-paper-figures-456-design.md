# Paper Figures 4-6 Design

Date: 2026-04-08
Branch: `paper`

## Goal

Produce the first batch of paper-ready figures for the current GA-LMB paper narrative:

1. Figure 4: main GA consensus result under tiered heterogeneous packet loss
2. Figure 5: factor ablation under tiered packet loss
3. Figure 6: ideal-communication supporting comparison

The figures should be suitable for direct inclusion in the paper body, generated primarily with Python, and exported as vector PDF files.

## Success Criteria

- A Python-based figure pipeline exists for Figure 4, Figure 5, and Figure 6.
- The default export format for all three figures is `PDF`.
- The figures are aligned with the current paper storyline in [README.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/README.md) and [09_figures_tables_and_gaps.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/09_figures_tables_and_gaps.md).
- Figure 4 emphasizes time-varying consensus improvement.
- Figure 5 emphasizes the five-arm factor-by-factor progression.
- Figure 6 emphasizes that the ideal-communication result remains positive on both consensus and aggregated local metrics.
- Existing standard-ideal track imagery is not promoted into the main figure sequence; it remains appendix-only or qualitative support.

## Scope

This design covers only the first batch of paper figures:

- Figure 4
- Figure 5
- Figure 6

It does not yet cover:

- Figure 1 system overview schematic
- Figure 2 adaptive-weight factorization schematic
- Figure 3 existence-confidence mapping curve
- appendix-only robustness plots

## Narrative Role

The three figures should form one short evidence chain in the paper body:

1. Figure 4 shows that the adaptive method improves consensus over time, not only in final scalar averages.
2. Figure 5 shows that the improvement is not a one-shot gain but a staged progression from `fixed` to `+structure-aware decoupled KLA`.
3. Figure 6 shows that the final refinement remains useful even when packet loss is removed, so the method is not merely compensating for damaged links.

This ordering matches the current main text:

- [06_experimental_setup.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/06_experimental_setup.md)
- [07_results_and_ablation.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/07_results_and_ablation.md)
- [09_figures_tables_and_gaps.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/09_figures_tables_and_gaps.md)

## Output Files

The generated figure artifacts should live under:

- `/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/figures/figure4_main_ga_consensus.pdf`
- `/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/figures/figure5_factor_ablation.pdf`
- `/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/figures/figure6_ideal_support.pdf`

Optional preview exports may also be produced as PNG for quick inspection, but PDF is the primary artifact.

## Rendering Stack

The figures should be rendered in Python, with `matplotlib` as the primary plotting library.

Recommended rendering rules:

- use vector PDF export via `savefig(..., format="pdf")`
- use one shared style definition for fonts, line widths, colors, and subplot spacing
- use paper-safe default typography, preferably serif or a neutral publication style
- keep backgrounds white and avoid presentation-style decoration
- use a consistent color mapping across all three figures

Recommended semantic colors:

- fixed or ordinary GA: neutral gray-blue
- current best adaptive or structure-aware decoupled KLA: deep red
- intermediate ablation arms: light blue to orange progression

## Data Strategy

### Figure 4

Figure 4 requires time-series data, not only final means. The current tracked reports already provide headline scalar results, but the paper figure should show temporal behavior.

The required data are:

- consensus OSPA over time
- consensus RMSE over time
- consensus cardinality disagreement over time

Comparison:

- fixed weights
- current best adaptive method

Primary source logic:

- the main tiered-drop GA scenario used in the current paper headline
- the same comparison summarized in [GA_TIERED_LINK_ABLATION_20260326_182435.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_20260326_182435.md)

Implementation implication:

- the Octave runner will likely need a clean export path for per-time-step series, for example as CSV, MAT, or JSON-like text
- Python should consume the exported time-series data rather than scraping plots from Octave

### Figure 5

Figure 5 can be generated from already stabilized scalar results. The required five arms are:

- `fixed`
- `+covariance`
- `+link quality`
- `+existence confidence`
- `+structure-aware decoupled KLA`

Metrics:

- consensus OSPA
- consensus RMSE
- consensus cardinality disagreement

Primary numeric sources:

- [GA_TIERED_LINK_ABLATION_20260322_001613.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_20260322_001613.md)
- [GA_TIERED_LINK_ABLATION_20260326_182435.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_20260326_182435.md)

The figure should be buildable immediately from structured values extracted from those reports or from a small hand-defined data table in Python that is traceable to those reports.

### Figure 6

Figure 6 should be built from the already stabilized ideal-communication comparison.

Consensus metrics:

- OSPA
- RMSE
- cardinality disagreement

Aggregated local metrics:

- E-OSPA
- RMSE

Comparison:

- ordinary GA
- structure-aware decoupled KLA

Primary source:

- [GA_IDEAL_COMM_COMPARE_20260326_184508.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_IDEAL_COMM_COMPARE_20260326_184508.md)

## Per-Figure Layout

### Figure 4 Layout

Recommended layout:

- one full-width figure
- three horizontally arranged panels
- panel order: consensus OSPA, consensus RMSE, consensus cardinality disagreement

Visual rules:

- same x-axis scale across all panels
- direct legend for `fixed` and `adaptive`
- adaptive curve should be visually emphasized
- optional inset or caption note with headline means:
  - OSPA: `2.624 -> 1.862`
  - RMSE: `2.703 -> 1.750`
  - Card: `0.879 -> 0.244`

This figure should read as the main body result, not as an appendix plot.

### Figure 5 Layout

Recommended layout:

- one full-width figure
- three horizontally arranged panels
- panel order: consensus OSPA, consensus RMSE, consensus cardinality disagreement
- each panel uses the same five arm labels

Preferred chart type:

- compact bar chart

Rationale:

- bar charts make the monotonic or near-monotonic progression easier to read than a decorative line chart
- this figure is about contribution decomposition, not about time

### Figure 6 Layout

Recommended layout:

- one medium-width figure
- left block: consensus metrics
- right block: aggregated local metrics

Preferred chart type:

- grouped compact bars

Panel grouping:

- left: OSPA, RMSE, Card
- right: local E-OSPA, local RMSE

Optional note:

- local H-OSPA may remain in the table rather than in the figure because it is nearly unchanged and not visually informative

## Implementation Structure

Recommended file structure:

- `/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/figures/`
- `/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/figures/render_paper_figures.py`
- `/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/figures/paper_figure_data.py`
- `/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/figures/README.md`

Recommended responsibilities:

- `paper_figure_data.py`: central place for stable scalar values and file references
- `render_paper_figures.py`: generate one or all figures with a simple CLI
- `README.md`: record the source reports, commands, and output filenames

If Figure 4 requires exported time-series data, an additional exporter on the Octave side may be added later, but the Python renderer should remain the final plotting authority.

## Non-Goals

The following are explicitly out of scope for this batch:

- polishing the standard ideal fixed trajectory image into a main paper figure
- mixing appendix-style qualitative trajectory figures into Figure 4, Figure 5, or Figure 6
- adding decorative backgrounds, gradients, or presentation-style embellishment
- broadening the figure batch to cover unrelated appendix experiments

## Risks And Mitigations

Risk:

- Figure 4 currently lacks a stable tracked time-series export artifact.

Mitigation:

- treat Figure 5 and Figure 6 as immediately buildable
- add a minimal export path for Figure 4 time-series before final rendering

Risk:

- report values may drift if the source experiments are rerun later.

Mitigation:

- record exact source report filenames in the figure README
- avoid hidden manual edits to numeric values

Risk:

- mixed plot styles may make the paper feel inconsistent.

Mitigation:

- keep one shared Python style sheet across all three figures

## Acceptance Checklist

- Figure 4, Figure 5, and Figure 6 each have a clear narrative job
- Python is the final renderer for all three figures
- PDF is the default exported format
- Figure 4 uses time-series data
- Figure 5 uses the five-arm ablation progression
- Figure 6 uses the ideal-communication comparison with consensus and local support
- the standard ideal fixed trajectory figure remains appendix-only or supporting material
