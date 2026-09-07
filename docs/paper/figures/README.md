# Paper Figure Assets

This directory stores Python-rendered paper-ready figure artifacts for the current manuscript.

Current target outputs:

- `figure3_existence_confidence_curve.pdf`
- `figure1_v2.pdf`
- `figure_simulation_scenario.pdf`
- `figure4_main_ga_consensus.pdf`
- `figure5_factor_ablation.pdf`
- `comm_level_fixed_balanced_n50.pdf`

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
/opt/homebrew/anaconda3/bin/python3 docs/paper/figures/render_paper_figures.py --output-dir docs/paper/figures --include-figure4
```

Figure 4 additionally requires a consensus time-series CSV named `figure4_consensus_series.csv` in the same output directory. The CSV contains the fixed baseline, Balanced mode, and Cardinality-critical mode. The current CSV is exported from the saved 50-trial run `RUN/GA/mc50_20260527_172137/01_tiered_main_fidfia_n50_seed1.mat`, so the plotted curves are trial-mean trajectories rather than a representative single run.

The communication-level sensitivity figure uses `comm_level_fixed_balanced_n50.csv`. Its four levels are transcribed from the coherent Fixed/Balanced paired reports `GA_TIERED_LINK_ABLATION_N50_SEED1_20260528_185729.md`, `..._213357.md`, `..._235647.md`, and `..._20260529_015909.md`; level 2 is the same deterministic seed set used for the corresponding Fixed and Balanced rows in the main table. Render it with:

```bash
python3 docs/paper/figures/plot_comm_level_method_compare.py \
  --input-csv docs/paper/figures/comm_level_fixed_balanced_n50.csv \
  --output-base docs/paper/figures/comm_level_fixed_balanced_n50
```

The simulation-scenario schematic is rendered with Matplotlib:

```bash
python3 docs/paper/figures/render_simulation_scenario_figure.py
```

The older Octave entry point `render_simulation_scenario_figure.m` remains as a
compatibility wrapper around the Python renderer.
