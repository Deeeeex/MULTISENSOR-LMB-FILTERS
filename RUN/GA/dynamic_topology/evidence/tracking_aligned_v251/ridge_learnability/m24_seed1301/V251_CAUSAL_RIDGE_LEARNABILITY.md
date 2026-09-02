# V251 causal ridge learnability

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`
- Oracle source commit: `3248df0edeba8e8e24d897821a4922522cdec5a8`
- Analysis source commit: `390ba873c27d2e2c7eb673ec6ad6905944c9f56a`
- Anchors: `[70 84 151]`
- Frozen ridge lambda: `1e-06`
- Next decision: `pairwise-representation-fits-but-more-independent-anchors-are-required`

## Method decision

A formation-level observable risk proxy is not used as a deployment target because confident spatial bias need not appear as large covariance or within-formation disagreement. V251 instead predicts eight candidate outcome heads from current directed-edge features. The deterministic selector applies the V250 core, byte and formation-regression constraints to the predictions, then maximizes the predicted RMSE gain of the offline target formation. The latter is a teacher target, never a feature.

## Why formation-level risk is not the selector

| Anchor | Truth-worst | Bayes-risk worst | Uncertainty worst | Within-formation disagreement worst |
|--:|--:|--:|--:|--:|
| 70 | F4 | F2 | F2 | F3 |
| 84 | F2 | F2 | F4 | F2 |
| 151 | F1 | F3 | F2 | F3 |

Match counts are Bayes risk `1/3`, uncertainty `0/3`, and within-formation spatial disagreement `1/3`. A confident but biased formation can therefore look internally certain and coherent; candidate-level cross-formation complementarity is required.

| Feature set | Features | In-sample recall / aligned / pass | LOAO recall / aligned / pass | Oracle-safe rank recall in / LOAO |
|:--|--:|:--|:--|:--|
| link-only | 11 | 0/3 / 0/3 / 0 | 0/3 / 0/3 / 0 | 3/3 / 3/3 |
| posterior-rich | 47 | 0/3 / 0/3 / 0 | 0/3 / 0/3 / 0 | 3/3 / 3/3 |

Pairwise ridge is the actual ranking baseline; the table above keeps the multi-head outcome predictor as a diagnostic. Pairwise results follow.

| Feature set | In-sample top-1 / top-3 / aligned / pass | LOAO top-1 / top-3 / aligned / pass |
|:--|:--|:--|
| link-only | 0/3 / 3/3 / 0/3 / 0 | 0/3 / 0/3 / 0/3 / 0 |
| posterior-rich | 3/3 / 3/3 / 3/3 / 1 | 0/3 / 0/3 / 0/3 / 0 |

## link-only

### in-sample

| Anchor | Teacher | Selected | Recalled | Tail-aligned | Admissible | Rank-only / safe-rank | Realized E / R / C | Byte saving | Target E / R | Minimum formation E / R |
|--:|--:|--:|:--:|:--:|--:|:--|:--|--:|:--|:--|
| 70 | 11 | 1 | 0 | 0 | 0 | 20 / 11 | +0.000% / +0.000% / +0.000% | +0.000% | +0.000% / +0.000% | +0.000% / +0.000% |
| 84 | 20 | 1 | 0 | 0 | 0 | 3 / 20 | +0.000% / +0.000% / +0.000% | +0.000% | +0.000% / +0.000% | +0.000% / +0.000% |
| 151 | 13 | 1 | 0 | 0 | 0 | 3 / 13 | +0.000% / +0.000% / +0.000% | +0.000% | +0.000% / +0.000% | +0.000% / +0.000% |

Aggregate selected actions: E `+0.000%`, RMSE `+0.000%`, consistency `+0.000%`, attempted-byte saving `+0.000%`, weakest formation E/R `+0.000% / +0.000%`; gate `0`.
### leave-one-anchor-out

| Anchor | Teacher | Selected | Recalled | Tail-aligned | Admissible | Rank-only / safe-rank | Realized E / R / C | Byte saving | Target E / R | Minimum formation E / R |
|--:|--:|--:|:--:|:--:|--:|:--|:--|--:|:--|:--|
| 70 | 11 | 1 | 0 | 0 | 0 | 3 / 11 | +0.000% / +0.000% / +0.000% | +0.000% | +0.000% / +0.000% | +0.000% / +0.000% |
| 84 | 20 | 1 | 0 | 0 | 0 | 3 / 20 | +0.000% / +0.000% / +0.000% | +0.000% | +0.000% / +0.000% | +0.000% / +0.000% |
| 151 | 13 | 10 | 0 | 0 | 1 | 14 / 13 | -0.173% / -0.085% / -0.016% | -1.126% | +0.000% / +0.000% | -0.825% / -0.632% |

Aggregate selected actions: E `-0.056%`, RMSE `-0.033%`, consistency `-0.005%`, attempted-byte saving `-0.354%`, weakest formation E/R `-0.228% / -0.265%`; gate `0`.
### pairwise-in-sample

| Anchor | Teacher | Selected | Teacher rank | Top-3 | Tail-aligned | Score margin | Realized E / R / C | Byte saving | Target E / R | Minimum formation E / R |
|--:|--:|--:|--:|:--:|:--:|--:|:--|--:|:--|:--|
| 70 | 11 | 15 | 3 | 1 | 0 | 0.023812 | -0.008% / -1.026% / -0.000% | +0.107% | -0.032% / -2.326% | -0.032% / -2.326% |
| 84 | 20 | 13 | 2 | 1 | 0 | 0.023812 | +0.067% / +1.460% / -3.019% | -1.947% | +0.015% / +3.641% | +0.000% / +0.000% |
| 151 | 13 | 12 | 2 | 1 | 0 | 0.000000 | +0.000% / -0.016% / +0.000% | +0.161% | -0.001% / -0.087% | -0.001% / -0.087% |

Aggregate pairwise selections: E `+0.020%`, RMSE `+0.056%`, consistency `-1.049%`, attempted-byte saving `-0.770%`, weakest formation E/R `-0.011% / -1.240%`; gate `0`.
### pairwise-leave-one-anchor-out

| Anchor | Teacher | Selected | Teacher rank | Top-3 | Tail-aligned | Score margin | Realized E / R / C | Byte saving | Target E / R | Minimum formation E / R |
|--:|--:|--:|--:|:--:|:--:|--:|:--|--:|:--|:--|
| 70 | 11 | 21 | 5 | 0 | 0 | 0.147020 | -0.001% / -0.271% / -0.002% | +1.277% | -0.000% / -0.002% | -0.002% / -2.322% |
| 84 | 20 | 2 | 15 | 0 | 0 | 0.176816 | -0.225% / -2.974% / -1.386% | -0.857% | -0.651% / -5.740% | -0.651% / -5.740% |
| 151 | 13 | 20 | 6 | 0 | 0 | 0.000000 | -0.002% / -0.536% / +0.362% | +0.059% | -0.007% / -1.552% | -0.007% / -1.552% |

Aggregate pairwise selections: E `-0.078%`, RMSE `-1.126%`, consistency `-0.371%`, attempted-byte saving `-0.035%`, weakest formation E/R `-0.215% / -2.095%`; gate `0`.

## posterior-rich

### in-sample

| Anchor | Teacher | Selected | Recalled | Tail-aligned | Admissible | Rank-only / safe-rank | Realized E / R / C | Byte saving | Target E / R | Minimum formation E / R |
|--:|--:|--:|:--:|:--:|--:|:--|:--|--:|:--|:--|
| 70 | 11 | 1 | 0 | 0 | 0 | 3 / 11 | +0.000% / +0.000% / +0.000% | +0.000% | +0.000% / +0.000% | +0.000% / +0.000% |
| 84 | 20 | 1 | 0 | 0 | 0 | 3 / 20 | +0.000% / +0.000% / +0.000% | +0.000% | +0.000% / +0.000% | +0.000% / +0.000% |
| 151 | 13 | 1 | 0 | 0 | 0 | 16 / 13 | +0.000% / +0.000% / +0.000% | +0.000% | +0.000% / +0.000% | +0.000% / +0.000% |

Aggregate selected actions: E `+0.000%`, RMSE `+0.000%`, consistency `+0.000%`, attempted-byte saving `+0.000%`, weakest formation E/R `+0.000% / +0.000%`; gate `0`.
### leave-one-anchor-out

| Anchor | Teacher | Selected | Recalled | Tail-aligned | Admissible | Rank-only / safe-rank | Realized E / R / C | Byte saving | Target E / R | Minimum formation E / R |
|--:|--:|--:|:--:|:--:|--:|:--|:--|--:|:--|:--|
| 70 | 11 | 1 | 0 | 0 | 0 | 2 / 11 | +0.000% / +0.000% / +0.000% | +0.000% | +0.000% / +0.000% | +0.000% / +0.000% |
| 84 | 20 | 7 | 0 | 0 | 1 | 4 / 20 | -0.001% / -0.458% / -0.038% | +0.012% | +0.000% / +0.000% | -0.005% / -2.338% |
| 151 | 13 | 1 | 0 | 0 | 0 | 16 / 13 | +0.000% / +0.000% / +0.000% | +0.000% | +0.000% / +0.000% | +0.000% / +0.000% |

Aggregate selected actions: E `-0.000%`, RMSE `-0.127%`, consistency `-0.013%`, attempted-byte saving `+0.005%`, weakest formation E/R `-0.002% / -0.465%`; gate `0`.
### pairwise-in-sample

| Anchor | Teacher | Selected | Teacher rank | Top-3 | Tail-aligned | Score margin | Realized E / R / C | Byte saving | Target E / R | Minimum formation E / R |
|--:|--:|--:|--:|:--:|:--:|--:|:--|--:|:--|:--|
| 70 | 11 | 11 | 1 | 1 | 1 | 0.058452 | +0.006% / +0.644% / +0.371% | +0.224% | +0.023% / +1.537% | -0.000% / -0.143% |
| 84 | 20 | 20 | 1 | 1 | 1 | 0.062448 | +0.139% / +1.932% / +0.003% | +0.037% | +0.001% / +0.180% | +0.000% / +0.000% |
| 151 | 13 | 13 | 1 | 1 | 1 | 0.309396 | +0.219% / +0.262% / +1.440% | +0.068% | +0.000% / +0.011% | +0.000% / +0.000% |

Aggregate pairwise selections: E `+0.120%`, RMSE `+0.855%`, consistency `+0.572%`, attempted-byte saving `+0.094%`, weakest formation E/R `-0.000% / -0.035%`; gate `1`.
### pairwise-leave-one-anchor-out

| Anchor | Teacher | Selected | Teacher rank | Top-3 | Tail-aligned | Score margin | Realized E / R / C | Byte saving | Target E / R | Minimum formation E / R |
|--:|--:|--:|--:|:--:|:--:|--:|:--|--:|:--|:--|
| 70 | 11 | 13 | 16 | 0 | 0 | 0.205908 | +0.045% / -3.432% / +0.435% | +1.048% | +0.000% / +0.000% | +0.000% / -16.187% |
| 84 | 20 | 17 | 9 | 0 | 0 | 0.329193 | +0.012% / +3.099% / +1.002% | +0.667% | +0.000% / +0.000% | -0.003% / -1.153% |
| 151 | 13 | 20 | 16 | 0 | 0 | 0.365891 | -0.002% / -0.536% / +0.362% | +0.059% | -0.007% / -1.552% | -0.007% / -1.552% |

Aggregate pairwise selections: E `+0.019%`, RMSE `-0.495%`, consistency `+0.610%`, attempted-byte saving `+0.571%`, weakest formation E/R `-0.003% / -4.008%`; gate `0`.

## Evidence boundary

V251 reuses all completed paired V250 H=3 arms from one M24 development seed. Ridge inputs contain only current directed-edge posterior summaries, current link reliability and geometry, past selected-edge persistence, action type and change fraction. Numeric sensor/formation identities, truth, future measurements, future physical pages and alternative-arm outcomes are not features. Truth defines eight offline outcome heads. In-sample fit tests representation capacity; leave-one-anchor-out is only a three-group development screen. Neither result is an online, full-episode, X36, validation or paper-level claim.
