# Paper Figure Assets

This directory stores Python-rendered paper-ready figure artifacts for the current `paper` branch.

Current target outputs:

- `figure3_existence_confidence_curve.pdf`
- `figure4_main_ga_consensus.pdf`
- `figure5_factor_ablation.pdf`
- `figure6_ideal_support.pdf`

Prompt-driven figure specifications:

- `figure1_system_overview_prompt.md`
- `figure2_weight_factorization_prompt.md`

Current stable scalar sources:

- `RUN/GA/GA_TIERED_LINK_ABLATION_20260322_001613.md`
- `RUN/GA/GA_TIERED_LINK_ABLATION_20260326_182435.md`
- `RUN/GA/GA_IDEAL_COMM_COMPARE_20260326_184508.md`

Render command:

```bash
/opt/homebrew/anaconda3/bin/python3 docs/paper/figures/render_paper_figures.py --output-dir docs/paper/figures
```

Figure 4 additionally requires a consensus time-series CSV named `figure4_consensus_series.csv` in the same output directory.
