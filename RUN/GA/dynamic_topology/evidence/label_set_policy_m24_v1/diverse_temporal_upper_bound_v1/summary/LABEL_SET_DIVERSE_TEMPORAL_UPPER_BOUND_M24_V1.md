# M24 diverse temporal H=3 upper bound

- Mean old / persistent / temporal oracle E-OSPA: `14.506045 / 14.319245 / 14.190508`
- Gain vs old two-option oracle: `2.175%`
- Incremental gain vs persistent-rank oracle: `0.899%` (gate `2.000%`)
- Refreshed persistent states: `2 / 4`
- Hard audit / temporal headroom gates: `1 / 0`

| Seed | Source | Rank | Old oracle | Persistent oracle | Temporal oracle | Best | Gain vs old | Incremental | Safe |
|--:|:--|--:|--:|--:|--:|:--|--:|--:|--:|
| 11 | learned | 6 | 15.371985 | 15.371985 | 14.897434 | `C-G6-G6` | +3.087% | +3.087% | 1 |
| 19 | ccw | 5 | 19.856173 | 19.440435 | 19.440435 | `G5-G5-G5` | +2.094% | +0.000% | 1 |
| 23 | ccw | 5 | 5.901460 | 5.896317 | 5.896317 | `G5-G5-G5` | +0.087% | +0.000% | 1 |
| 27 | learned | 4 | 16.894561 | 16.568242 | 16.527847 | `G4-C-C` | +2.171% | +0.244% | 1 |

## Decision

The temporal upper bound is `rejected`. The next opened training-only method direction is `quota-adaptive-backbone-preserving-residual-routing`.

## Boundary

This opened-training upper bound chooses each state rank after observing future return and tests only eight C/G timing patterns. It cannot establish a deployable selector, development, held-out M24 or X36 result.
