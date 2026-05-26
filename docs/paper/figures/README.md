# Paper Figure Assets

This directory stores Python-rendered paper-ready figure artifacts for the current `paper` branch.

Current target outputs:

- `figure3_existence_confidence_curve.pdf`
- `figure_simulation_scenario.pdf`
- `figure4_main_ga_consensus.pdf`
- `figure5_factor_ablation.pdf`

`figure5_factor_ablation.pdf` and the earlier `figure6_ideal_support.pdf` are retained as reproducible plotting artifacts, but the current manuscript presents those scalar comparisons as Tables 3 and 4.

Prompt-driven figure specifications:

- `figure1_system_overview_prompt.md`
- `figure2_weight_factorization_prompt.md`

Current stable scalar sources:

- `RUN/GA/GA_TIERED_LINK_ABLATION_20260322_001613.md`
- `RUN/GA/GA_TIERED_LINK_ABLATION_MAIN20_20260507.md`
- `RUN/GA/GA_IDEAL_COMM_MAIN20_PAIRED_20260507.md`

Render command:

```bash
/opt/homebrew/anaconda3/bin/python3 docs/paper/figures/render_paper_figures.py --output-dir docs/paper/figures
```

Figure 4 additionally requires a consensus time-series CSV named `figure4_consensus_series.csv` in the same output directory. The CSV contains the fixed baseline, Balanced mode, and Cardinality-critical mode.

The simulation-scenario schematic is rendered with MATLAB/Octave:

```bash
octave --no-gui --quiet --eval "addpath('docs/paper/figures'); render_simulation_scenario_figure"
```
