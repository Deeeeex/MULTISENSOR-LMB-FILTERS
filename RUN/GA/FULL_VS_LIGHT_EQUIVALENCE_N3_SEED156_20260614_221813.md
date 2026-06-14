# Full-vs-light equivalence validation

- Trials: 3
- Seeds: `[157 158 159]`
- Simulation length: 50
- Comparison report: `/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_DUAL_THRESHOLD_EVENT_TRIGGER_N3_SEED156_20260614_221813.md`

## Payload bytes

| Arm | Mean payload scalars | Mean payload bytes |
|:--|--:|--:|
| Periodic full posterior | 1523927 | 12191413 |
| Periodic light posterior on static topology | 618323 | 4946581 |
| Light reduction | 59.43% | 59.43% |

## Moment equivalence diagnostics

| Stage | Comparisons | Objects | Missing labels | Max existence diff | Max mean norm diff | Max covariance Fro diff |
|:--|--:|--:|--:|--:|--:|--:|
| Payload: full moment-matched vs light payload | 4800 | 75292 | 0 | 0.000e+00 | 0.000e+00 | 0.000e+00 |
| Fused posterior: full-input KLA vs light-input KLA | 1200 | 39845 | 0 | 0.000e+00 | 0.000e+00 | 0.000e+00 |

## End-to-end metric deltas

| Metric | Max absolute difference |
|:--|--:|
| Trial/sensor local E-OSPA | 0.000e+00 |
| Trial consensus OSPA | 0.000e+00 |
| Trial position disagreement | 0.000e+00 |
| Trial cardinality dispersion | 0.000e+00 |

## Interpretation

In the current single-round label-wise KLA implementation, neighbor full GM-LMB payloads are moment-matched before fusion. The light payload sends those same label-wise moments directly. Therefore the full and light inputs produce numerically identical KLA fused posteriors up to floating-point tolerance, while the light payload reduces transmitted scalars and bytes.
