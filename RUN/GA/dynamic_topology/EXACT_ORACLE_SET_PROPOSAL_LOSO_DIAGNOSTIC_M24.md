# Structured set-proposal LOSO diagnostic: M24

- Generated: 2026-07-29 17:21:37
- Contract: `rolling-safe-exact-oracle-set-proposal-model-m24-v2`
- Training commit: `acf182cbf321ec6a6bcea9557abd878c1925a3b6`
- Dataset SHA-256: `f75cb2eef4665bb1bbe624c92aa29348f1e00acd5ad066337bc108e7c5c89d99`
- Method variant: `exact-oracle-hard-negative-v2`
- Feature mode / loss: `relative-only / multi-positive-set-softmax`
- Selected hidden width / lambda: `32 / 0.01`
- Evaluation top-K: `16`
- Hard-negative rounds / additions: `3 / [310 323 319]`
- Hard-negative projection failures: `[14 1 4]`
- Final training candidates min/mean/max: `35 / 39.901 / 41`
- Expanded state capture: `0 / 81 (0.000%)`
- Minimum expanded seed capture: `0.000%`
- Expanded graph recall: `0 / 309 (0.000%)`
- Expanded mean best edge F1: `0.205761`
- Legacy value-state capture: `0 / 10 (0.000%)`
- Expanded / legacy gate: `0 / 0`
- H=3 return generation authorized: `0`
- Critic/X36 authorized: `0 / 0`
- Evidence boundary: LOSO selection uses unordered current-truth safe graph sets on states collected by a frozen truth-free behavior policy. Relative observable features use neither truth nor future returns. The older value-bearing states are an external architecture diagnostic only. A pass authorizes H=3 return generation for the frozen M24 proposal bank, not critic training, an M24 effect claim, or X36. Exact-projector hard negatives are mined only from each fold's fitting seeds; held-out and legacy-value states never enter constraint generation.

## Result-blind configuration screen

| Hidden | Lambda | State | Min seed | Graph recall | Best F1 | Distinct | Failures |
|--:|--:|--:|--:|--:|--:|--:|--:|
| 32 | 0.01 | 0.0% | 0.0% | 0.0% | 0.206 | 4.10 | 7 |

## Selected top-K expanded states

| Seed | Time | Targets | State captured | Captured graphs | Best F1 | Distinct | Failures |
|--:|--:|--:|--:|--:|--:|--:|--:|
| 7 | 75 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 7 | 76 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 7 | 77 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 7 | 78 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 7 | 79 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 7 | 80 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 7 | 81 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 7 | 82 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 7 | 83 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 11 | 75 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 11 | 76 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 11 | 77 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 11 | 78 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 11 | 79 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 11 | 80 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 11 | 81 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 11 | 82 | 4 | 0 | 0 | 0.333 | 3 | 1 |
| 11 | 83 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 17 | 75 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 17 | 76 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 17 | 77 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 17 | 78 | 4 | 0 | 0 | 0.667 | 4 | 0 |
| 17 | 79 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 17 | 80 | 3 | 0 | 0 | 0.333 | 6 | 0 |
| 17 | 81 | 4 | 0 | 0 | 0.333 | 3 | 1 |
| 17 | 82 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 17 | 83 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 19 | 75 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 19 | 76 | 4 | 0 | 0 | 0.333 | 6 | 0 |
| 19 | 77 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 19 | 78 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 19 | 79 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 19 | 80 | 4 | 0 | 0 | 0.000 | 3 | 1 |
| 19 | 81 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 19 | 82 | 4 | 0 | 0 | 0.000 | 6 | 0 |
| 19 | 83 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 23 | 75 | 4 | 0 | 0 | 0.000 | 6 | 0 |
| 23 | 76 | 3 | 0 | 0 | 0.000 | 4 | 0 |
| 23 | 77 | 4 | 0 | 0 | 0.000 | 3 | 1 |
| 23 | 78 | 3 | 0 | 0 | 0.000 | 4 | 0 |
| 23 | 79 | 4 | 0 | 0 | 0.000 | 6 | 0 |
| 23 | 80 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 23 | 81 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 23 | 82 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 23 | 83 | 4 | 0 | 0 | 0.667 | 4 | 0 |
| 27 | 75 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 27 | 76 | 3 | 0 | 0 | 0.000 | 4 | 0 |
| 27 | 77 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 27 | 78 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 27 | 79 | 3 | 0 | 0 | 0.333 | 4 | 0 |
| 27 | 80 | 3 | 0 | 0 | 0.333 | 4 | 0 |
| 27 | 81 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 27 | 82 | 4 | 0 | 0 | 0.000 | 4 | 1 |
| 27 | 83 | 3 | 0 | 0 | 0.000 | 4 | 0 |
| 29 | 75 | 3 | 0 | 0 | 0.333 | 4 | 0 |
| 29 | 76 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 29 | 77 | 3 | 0 | 0 | 0.333 | 4 | 0 |
| 29 | 78 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 29 | 79 | 4 | 0 | 0 | 0.000 | 6 | 0 |
| 29 | 80 | 3 | 0 | 0 | 0.000 | 4 | 0 |
| 29 | 81 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 29 | 82 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 29 | 83 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 31 | 75 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 31 | 76 | 3 | 0 | 0 | 0.000 | 4 | 0 |
| 31 | 77 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 31 | 78 | 3 | 0 | 0 | 0.667 | 4 | 0 |
| 31 | 79 | 4 | 0 | 0 | 0.667 | 4 | 0 |
| 31 | 80 | 4 | 0 | 0 | 0.333 | 2 | 2 |
| 31 | 81 | 3 | 0 | 0 | 0.333 | 4 | 0 |
| 31 | 82 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 31 | 83 | 4 | 0 | 0 | 0.333 | 6 | 0 |
| 37 | 75 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 37 | 76 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 37 | 77 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 37 | 78 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 37 | 79 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 37 | 80 | 3 | 0 | 0 | 0.000 | 4 | 0 |
| 37 | 81 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 37 | 82 | 3 | 0 | 0 | 0.333 | 4 | 0 |
| 37 | 83 | 4 | 0 | 0 | 0.333 | 4 | 0 |

## Selected top-K legacy value states

| Seed | Time | Targets | State captured | Captured graphs | Best F1 | Distinct | Failures |
|--:|--:|--:|--:|--:|--:|--:|--:|
| 7 | 76 | 4 | 0 | 0 | 0.333 | 6 | 0 |
| 7 | 77 | 3 | 0 | 0 | 0.000 | 4 | 0 |
| 11 | 75 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 11 | 76 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 11 | 77 | 1 | 0 | 0 | 0.000 | 6 | 0 |
| 17 | 75 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 17 | 77 | 3 | 0 | 0 | 0.000 | 4 | 0 |
| 23 | 75 | 4 | 0 | 0 | 0.000 | 6 | 0 |
| 29 | 75 | 3 | 0 | 0 | 0.333 | 4 | 0 |
| 29 | 76 | 4 | 0 | 0 | 0.333 | 4 | 0 |

## Decision

FAIL: do not generate returns, train a critic, or run X36 from this proposal model.
