# M24 structured formation-bridge choice analysis

- Generated: 2026-07-30 02:52:39
- Commit: `716762773edd9dbab3ec8d939523280687110bd7`
- Dataset-set SHA-256: `2eb144803806a2657e2fa74fa2a6b3a71008097ce5fa9006b64b60c8e286d511`
- Training seeds: `[11 17 19 23 27 29]`
- Development seeds opened: `0`
- Evidence boundary: This is a training-only architecture analysis on already-opened seeds. Hyperparameters are compared on the same outer LOSO results, so the best row is diagnostic rather than a frozen generalization estimate. No development labels are opened.

| Features | Lambda | Choice top-1 | Action recall | Orientation | Mean adv. | Min seed adv. | Median gap | P90 gap | Nonnegative | Tail safe | Fallback |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| edge-only | 0.01 | 0.007 | 0.007 | 0.583 | -0.1861 | -0.6055 | 0.3261 | 0.7257 | 0.306 | 0.750 | 0 |
| edge-only | 0.1 | 0.007 | 0.007 | 0.583 | -0.1853 | -0.6010 | 0.3261 | 0.7120 | 0.306 | 0.750 | 0 |
| edge-only | 1 | 0.014 | 0.007 | 0.500 | -0.1720 | -0.5776 | 0.2964 | 0.7066 | 0.306 | 0.694 | 0 |
| edge-only | 10 | 0.021 | 0.014 | 0.528 | -0.1487 | -0.4688 | 0.2610 | 0.6950 | 0.333 | 0.750 | 0 |
| edge-only | 100 | 0.021 | 0.014 | 0.500 | -0.1410 | -0.4688 | 0.2494 | 0.6950 | 0.333 | 0.722 | 0 |
| edge-update-relational | 0.01 | 0.007 | 0.021 | 0.500 | -0.1965 | -0.6643 | 0.3787 | 0.7180 | 0.222 | 0.778 | 0 |
| edge-update-relational | 0.1 | 0.007 | 0.021 | 0.500 | -0.1965 | -0.6643 | 0.3787 | 0.7180 | 0.222 | 0.778 | 0 |
| edge-update-relational | 1 | 0.007 | 0.021 | 0.472 | -0.1861 | -0.6694 | 0.2980 | 0.7191 | 0.306 | 0.778 | 0 |
| edge-update-relational | 10 | 0.000 | 0.021 | 0.444 | -0.1778 | -0.6669 | 0.3326 | 0.7120 | 0.306 | 0.778 | 0 |
| edge-update-relational | 100 | 0.007 | 0.014 | 0.417 | -0.1663 | -0.5533 | 0.3271 | 0.6780 | 0.278 | 0.694 | 0 |

Best training-only diagnostic: `edge-only`, lambda `100`, mean advantage `-0.1410`, minimum seed advantage `-0.4688`, tail-safe fraction `0.722`.
