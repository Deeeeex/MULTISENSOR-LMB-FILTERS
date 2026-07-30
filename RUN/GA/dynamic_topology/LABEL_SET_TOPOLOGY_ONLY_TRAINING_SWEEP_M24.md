# Label-set topology-only M24 training sweep

## Scope

This report evaluates the frozen topology-only candidate on all six
registered M24 training seeds. Development seeds 31/37, held-out M24 seeds,
and X36 remain sealed.

- Preset: `m24-hard`
- Seeds: `11, 17, 19, 23, 27, 29`
- Continuation window: `75:83`
- Dominant backbone weight: `0.70`
- Residual weight: fixed at `0.05`
- Candidate:
  `label-set-message-passing-safe-fixed-e05-a70`
- Controls:
  `backbone-residual-spliced-cycle-cw-a70-e05` and
  `backbone-residual-spliced-cycle-ccw-a70-e05`
- Generation source: `878cec4134ebb52f4b4949e42f12b473571b7f7d`

Every seed reuses one registered continuation cache and common random numbers
across its three arms. Seed 11 reuses the previously saved deterministic
controls and topology-only arm. The other five seeds were run as complete
three-arm paired screens with explicit `training` metadata.

## Per-seed result

The `Best static` column is a retrospective per-seed envelope. It is a
conservative comparison, not a deployable baseline-selection rule.

| Seed | Best static | Static mean | Learned mean | Mean gain | Static worst | Learned worst | Worst gain | Static consensus | Learned consensus | Consensus gain | Byte delta |
|--:|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| 11 | CCW | 19.7324 | **17.5854** | **10.88%** | 48.9410 | **47.3658** | **3.22%** | 24.8002 | **23.7068** | **4.41%** | +1.051% |
| 17 | CW | **18.3337** | 19.7514 | **-7.73%** | 36.9123 | **34.4441** | **6.69%** | **21.9400** | 22.5619 | **-2.83%** | +0.454% |
| 19 | CCW | 20.8133 | **16.7747** | **19.40%** | **36.1957** | 37.8015 | **-4.44%** | 23.5285 | **21.6592** | **7.94%** | +0.740% |
| 23 | CW | **6.0288** | 6.3496 | **-5.32%** | **8.5429** | 11.9289 | **-39.63%** | **7.7202** | 8.3278 | **-7.87%** | +0.026% |
| 27 | CCW | **15.4976** | 17.3614 | **-12.03%** | **28.0333** | 33.0238 | **-17.80%** | **19.4536** | 20.6293 | **-6.04%** | -0.245% |
| 29 | CCW | 13.0674 | **11.9111** | **8.85%** | 30.8219 | **26.8627** | **12.85%** | 16.8843 | **15.4520** | **8.48%** | +0.027% |

Relative to the retrospective per-seed static envelope:

- Mean static E-OSPA: `15.578864`
- Mean learned E-OSPA: `14.955606`
- Aggregate mean gain: `4.001%`
- Positive-mean-gain seeds: `3/6`
- Perfect seed-level static/dynamic oracle mean: `14.355228`
- Perfect seed-level oracle gain: `7.854%`

The learned policy therefore fails the stability gate against the strongest
static orientation available on each seed.

## Frozen single-control comparison

A paper baseline cannot choose its orientation after observing a seed's
outcome. If one static orientation is selected once using the training-set
average, CCW is the stronger registered control:

| Arm | Mean E-OSPA | Mean worst node | Mean consensus | Mean attempted bytes | Mean delivered bytes | Mean total time (s) |
|:--|--:|--:|--:|--:|--:|--:|
| CW | 16.295774 | 32.383183 | 19.345893 | 17,861,820 | 17,041,752 | 134.14 |
| **CCW** | **15.844439** | 33.856431 | 19.318898 | 17,867,432 | 17,090,259 | 133.83 |
| Topology-only | **14.955606** | **31.904465** | **18.722837** | 17,931,452 | 17,194,437 | 563.35 |

Relative to the frozen CCW control, topology-only yields:

- Mean E-OSPA gain: `5.610%`
- Mean worst-node gain: `5.765%`
- Mean consensus gain: `3.085%`
- Attempted-byte delta: `+0.358%`
- Delivered-byte delta: `+0.610%`
- Positive-mean-gain seeds: `4/6`
- Mean runtime ratio: `4.21x`

This aggregate result is promising but insufficient for method selection:
seed 17 and seed 27 still regress, and the experiment was run with limited
parallelism, so its runtime values are training diagnostics rather than
paper-quality timing measurements.

## Structural safety

Across all six learned runs:

- selected rolling-B3 sensor strong fraction: `1`
- truth-use fraction: `0`
- repair rate: `0`
- payload-emergency rate: `0`
- topology-infeasible rate: `0`
- attempted-byte mismatch relative to the per-seed best static: within `2%`

Delivered rolling-B3 can fall below one under packet loss (seed 19:
`0.6667`; seed 29: `0.7778`). The selected graph guarantee must not be
misstated as an unconditional delivered or effective-KLA graph guarantee.

## Gate headroom

The retrospective three-arm oracle chooses:

- topology-only for seeds `11, 19, 29`;
- CW for seeds `17, 23`;
- CCW for seed `27`.

Its mean E-OSPA is `14.355228`, or `9.399%` better than the frozen CCW
control. This is only an upper bound: seed identity and closed-loop outcome
cannot be gate inputs.

The current learned score does not provide a monotone confidence signal. Its
mean predicted task advantage is:

| Seed | Closed-loop mean gain vs per-seed static | Predicted task advantage |
|--:|--:|--:|
| 11 | 10.88% | -0.002689 |
| 17 | -7.73% | -0.036100 |
| 19 | 19.40% | -0.178447 |
| 23 | -5.32% | -0.058270 |
| 27 | -12.03% | -0.018551 |
| 29 | 8.85% | -0.016335 |

A hand-chosen confidence threshold is therefore not authorized. A new gate
must be trained and evaluated using only predecision observable state, with
whole-seed leave-one-out validation on the six opened training seeds.

## Decision

1. Reject the always-on topology-only policy as a stable M24 method.
2. Keep the fixed residual weight and safe topology projector; both remain
   structurally valid and communication-fair.
3. Add CW, CCW, and learned topology as explicit high-level actions.
4. Evaluate a predecision observable gate using whole-seed leave-one-out
   validation. The gate must not use seed identity, target truth, future
   outcomes, or post-run aggregate metrics.
5. Require the gated training policy to beat the frozen CCW control by at
   least 5% in mean E-OSPA, be positive on at least five of six training
   seeds, avoid aggregate worst-node and consensus regression, remain within
   2% attempted bytes, and preserve all selected-graph safety checks.
6. Keep development M24, held-out M24, and X36 sealed until that gate passes.

## Registered evidence hashes

| Seed | Report SHA-256 | MAT SHA-256 | Log SHA-256 |
|--:|:--|:--|:--|
| 17 | `6d51ab5cded1dc690092f658bc80c2f2ec6db8a2d4e31d7aec2b1a0bdaa1f761` | `e4ca7df5c5ec172805fc86634f5209d0992a5f509945359a0b7fc935600a31c0` | `43311b077ce18f01af475959978a7767fcced8554fab0950e6048bfa9640977f` |
| 19 | `01aee705f032506a6e44997c94feb2ed16f08b569c5cc1e41aee9d86626d9ab2` | `1a21469e2833fab2b2546b02771401c45d9e3116b95ad36571831596411dc942` | `b14ecbf89ead44612be87de26da710d797dd8a4f729c470394d37e75fade93e2` |
| 23 | `4bbdf84ea03e751617c3b4309f35700e50189cb9e4ee7ec3a10bbdb4cb9b7c56` | `d528b63dc400e1b7c074d6b37321185808480b64ae2d03b940f8f8e04942c9a3` | `6079e6824a7b38cdce8b102f76533ec04f3cb31e6af3449708c89a409c9358e1` |
| 27 | `1dcb1f2d083c23b2e617633db48cec3916d8fe32df91e317a996e136c81608e8` | `7e68e4619e4bdd0c2816629cee9bd382ebaafc8bb31939ec887668325e282c2e` | `2b1597a62ef88455c95acf1018ba4fc746e151ee5bd03aaf28558640426bac32` |
| 29 | `527b6adb42f70d989dbf6f0a0a0882c43206080ca61d0d3da93383a6fd35e7a7` | `fda63c66a2597eec0f9d035bda73a5a9c6b5a6e3795bf08a79e99c50a2cdedfc` | `1c1e1ad759e7930873ed795522343dcc0eb5f4be0029d759bf4571fc12a4e859` |

Seed 11 evidence and hashes are registered in
`LABEL_SET_TOPOLOGY_ONLY_PREFLIGHT_M24_SEED11.md`.
