# Figures, Tables, And Gaps

## Figure Plan

Figure 1:

- system overview
- distributed local update plus adaptive KLA fusion pipeline

Figure 2:

- adaptive weight factorization diagram
- show `mask`, `covScore`, `linkQuality`, `existenceConfidence`

Figure 3:

- existence-confidence mapping curve
- illustrate high confidence near `r=0/1` and low confidence near `r=0.5`

Figure 4:

- main GA scenario consensus curves over time
- base versus adaptive

Figure 5:

- factor ablation bar chart or line chart
- `fixed`, `+covariance`, `+link quality`, `+existence confidence`

Figure 6:

- communication level robustness plot

## Table Plan

Table 1:

- scenario and parameter configuration

Table 2:

- main GA result on local and consensus metrics

Table 3:

- factor ablation under tiered packet loss

Table 4:

- secondary `robust NIS` ablation

Table 5:

- communication-level results

Table 6:

- negative ablations for `freshness`, `history`, `association ambiguity`, and `cardinality consensus`

## Current Evidence Gaps

Highest priority:

- convert headline single-run results into Monte Carlo statistics
- add variance or confidence intervals
- add stronger baselines beyond fixed versus adaptive
- report runtime overhead
- verify whether existence confidence remains beneficial beyond the current 5-trial evidence

Medium priority:

- verify communication-level result tables cleanly from saved outputs
- tighten wording around AA generalization

Low priority:

- expand ambiguity-aware weighting only if it starts showing consistent gains

## Practical Next Steps

1. Reproduce the GA main result with Monte Carlo statistics.
2. Consolidate all tiered-GA result tables into one consistent format.
3. Draft the method section directly from the implementation files.
4. Draft the abstract only after the result tables are stabilized.
