# Fusion-Sufficient Moment Exchange Evidence

- Evidence schema: `fusion-sufficient-moment-exchange-v2`
- Git commit: `7974f10179a8973875bec9f301b8a5f84477d860`
- Immutable batch identity: `confirmatory-primary-seeds82-131-v2`
- Wire schema version: `1`
- Execution provenance (runtime only; not a paper contribution): `deterministic-seed-subprocess-v2`, Octave `11.1.0`, fixed `maxWorkers=6`.
- Frozen config SHA-256: `b21c92e99beb8e23371037e3ba5d690a2b447292036109534bfc45850fd07e6c`
- Required-source manifest SHA-256: `479bb72fbc0282692521a99b4bb569e912572215660fb2c22eeb1ae5a121de79`
- Trials and seeds: N=50, `82:131`
- Arms: `Periodic full GM fusion message` vs. `Periodic moment message on static topology`
- Changed variable: sender-side canonical moment projection before encoding.
- Byte boundary: Encoded application-layer bytes only; excludes MAC/PHY framing, network/transport headers, fragmentation, and retransmission. The loss model is payload-size independent.
- MAT artifact: `RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.mat`
- MAT SHA-256: `8ad63429d72a75e028bd7b9a1745bce7a047e4f9558443ea1bf547cfd69ab81f`
- CSV artifact: `RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.csv`
- CSV SHA-256: `1c1c4717ae68009417ff74d5e27c94e8e30fb5f8972b596328b3e12dc29db9a2`

## Pre-registered primary result

The per-seed attempted-byte reduction averages **58.277264%** (min/median/max 55.921689/58.045087/61.383973%). The paired percentile-bootstrap 95% interval is [57.923222, 58.636095]% (seed 20270710, 10000 resamples).

Attempted bytes (full/moment): 1254185200 / 522888880. Delivered bytes (full/moment): 1004548968 / 418898448.

## Equivalence audit

Compared labels: 1119037; compared snapshots: 40000; label-set mismatches: 0; missing labels: 0; missing snapshots: 0.

Maximum residuals `|delta r|`, `|delta mu|`, `|delta Sigma|`: `0`, `0`, `0`.

All exact: `1`; attempted masks equal: `1`; delivered masks equal: `1`.

## Per-seed audit preview

| Seed | Attempted reduction (%) | Max residual | Exact |
|---:|---:|---:|:---:|
| 82 | 57.592201 | 0 | 1 |
| 83 | 58.989939 | 0 | 1 |
| 84 | 58.796283 | 0 | 1 |
| 85 | 57.223943 | 0 | 1 |
| 86 | 59.833010 | 0 | 1 |
| 87 | 60.252744 | 0 | 1 |
| 88 | 56.450719 | 0 | 1 |
| 89 | 57.335941 | 0 | 1 |
| 90 | 59.674031 | 0 | 1 |
| 91 | 57.203967 | 0 | 1 |
| 92 | 59.348767 | 0 | 1 |
| 93 | 57.540940 | 0 | 1 |
| 94 | 57.273333 | 0 | 1 |
| 95 | 58.510389 | 0 | 1 |
| 96 | 60.197415 | 0 | 1 |
| 97 | 58.001874 | 0 | 1 |
| 98 | 57.773986 | 0 | 1 |
| 99 | 60.159636 | 0 | 1 |
| 100 | 58.552621 | 0 | 1 |
| 101 | 55.921689 | 0 | 1 |
| 102 | 59.394786 | 0 | 1 |
| 103 | 56.913292 | 0 | 1 |
| 104 | 57.733054 | 0 | 1 |
| 105 | 58.088301 | 0 | 1 |
| 106 | 59.378725 | 0 | 1 |
| 107 | 61.383973 | 0 | 1 |
| 108 | 56.729543 | 0 | 1 |
| 109 | 58.322842 | 0 | 1 |
| 110 | 59.911275 | 0 | 1 |
| 111 | 56.617089 | 0 | 1 |
| 112 | 59.582251 | 0 | 1 |
| 113 | 57.492208 | 0 | 1 |
| 114 | 57.331996 | 0 | 1 |
| 115 | 56.439796 | 0 | 1 |
| 116 | 60.302118 | 0 | 1 |
| 117 | 58.592653 | 0 | 1 |
| 118 | 59.034089 | 0 | 1 |
| 119 | 57.797751 | 0 | 1 |
| 120 | 57.844730 | 0 | 1 |
| 121 | 57.554969 | 0 | 1 |
| 122 | 57.039873 | 0 | 1 |
| 123 | 56.093343 | 0 | 1 |
| 124 | 57.276397 | 0 | 1 |
| 125 | 58.150949 | 0 | 1 |
| 126 | 57.116346 | 0 | 1 |
| 127 | 59.655310 | 0 | 1 |
| 128 | 58.749762 | 0 | 1 |
| 129 | 57.262353 | 0 | 1 |
| 130 | 59.287976 | 0 | 1 |
| 131 | 60.152031 | 0 | 1 |

## Regeneration

```bash
octave-cli --quiet --eval "setPath; addpath('RUN/GA'); [r,c,m]=runFusionSufficientMomentExchangeConfirmatory(true,50,81); disp(r); disp(c); disp(m);"
```

<!-- FUSION_SUFFICIENT_EVIDENCE_MACHINE
evidence_schema=fusion-sufficient-moment-exchange-v2
test_only=0
git_commit=7974f10179a8973875bec9f301b8a5f84477d860
batch_identity=confirmatory-primary-seeds82-131-v2
artifact_stem=GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131
wire_schema_version=1
execution_protocol=deterministic-seed-subprocess-v2
octave_version=11.1.0
max_workers=6
config_sha256=b21c92e99beb8e23371037e3ba5d690a2b447292036109534bfc45850fd07e6c
required_sources_sha256=479bb72fbc0282692521a99b4bb569e912572215660fb2c22eeb1ae5a121de79
number_of_trials=50
base_seed=81
first_seed=82
last_seed=131
simulation_length=100
bootstrap_seed=20270710
bootstrap_resamples=10000
summary_sha256=8ad63429d72a75e028bd7b9a1745bce7a047e4f9558443ea1bf547cfd69ab81f
csv_sha256=1c1c4717ae68009417ff74d5e27c94e8e30fb5f8972b596328b3e12dc29db9a2
aggregate_all_attempted_masks_equal=1
aggregate_all_delivered_masks_equal=1
aggregate_all_exact_match=1
aggregate_bootstrap_ci_high_percent=58.63609499846882
aggregate_bootstrap_ci_low_percent=57.923221880702556
aggregate_max_abs_consensus_cardinality_delta=0
aggregate_max_abs_consensus_ospa_delta=0
aggregate_max_abs_consensus_position_delta=0
aggregate_max_abs_local_eospa_delta=0
aggregate_max_abs_mu=0
aggregate_max_abs_r=0
aggregate_max_abs_sigma=0
aggregate_max_attempted_reduction_percent=61.383972629778278
aggregate_mean_attempted_reduction_percent=58.2772642282589
aggregate_mean_delivered_reduction_percent=58.267212081993868
aggregate_median_attempted_reduction_percent=58.045087372144543
aggregate_min_attempted_reduction_percent=55.921689078069072
aggregate_total_comparison_count=1119037
aggregate_total_full_attempted_bytes=1254185200
aggregate_total_full_delivered_bytes=1004548968
aggregate_total_label_set_mismatch_count=0
aggregate_total_missing_label_count=0
aggregate_total_missing_snapshot_count=0
aggregate_total_moment_attempted_bytes=522888880
aggregate_total_moment_delivered_bytes=418898448
aggregate_total_snapshot_count=40000
-->
