# V226 bounded dominant-edge transfer H=3

- State: `x36-formation-fov / seed 1301 / t=50`
- Donor formation: `F2`
- Evaluated / joint-positive / donor-increment-positive: `3 / 1 / 1`

| Row | Selected fraction min / median / max | Donor -> beneficiary | Source | Label | Joint E / R / C / terminal | Increment over donor E / R / C / terminal | Beneficiary increment E / R | Byte saving | Joint+ / Increment+ |
|--:|:--|:--|--:|:--|:--|:--|:--|--:|:--:|
| 3 | `0.025 / 0.300 / 0.300` | F2 -> F5 | 31 | `[1,3]` | `+0.006 / +1.232 / -0.134 / +0.000%` | `+0.006 / +1.232 / -0.134 / +0.000%` | `+0.033 / +5.749%` | +0.136% | 0 / 0 |
| 1 | `0.075 / 0.750 / 0.750` | F2 -> F5 | 3 | `[1,3]` | `+0.008 / +2.057 / -0.153 / -0.007%` | `+0.008 / +2.057 / -0.153 / -0.007%` | `+0.050 / +9.601%` | +0.050% | 0 / 0 |
| 2 | `0.025 / 0.025 / 0.250` | F2 -> F5 | 23 | `[19,15]` | `+0.006 / +1.942 / +0.035 / +0.052%` | `+0.006 / +1.942 / +0.035 / +0.052%` | `+0.033 / +9.067%` | +0.929% | 1 / 1 |

## Eta projection decisions

| Row | Weight | Evaluated receivers | Allowed | Rejected | Applied action |
|--:|--:|:--|:--|:--|--:|
| 3 | 0.100 | `25 26 27 28 29 30` | `25 27 28 29 30` | `26` | 1 |
| 1 | 0.100 | `25 26 27 28 29 30` | `25 26 27 28 29 30` | `-` | 1 |
| 2 | 0.100 | `25 26 27 28 29 30` | `25 26 28 29 30` | `27` | 1 |

## Replaced effective label edges

| Row | Replaced source IDs by receiver | Transferred topology weights |
|--:|:--|:--|
| 3 | `26 - 25 25 25 25` | `0.019 - 0.210 0.210 0.210 0.225` |
| 1 | `26 25 25 25 25 25` | `0.056 0.350 0.525 0.525 0.525 0.562` |
| 2 | `26 25 - 25 25 25` | `0.019 0.175 - 0.017 0.017 0.019` |

## Evidence boundary

V226 is a frozen development mechanism screen for receiver-specific partial label-graph rewiring. Each receiver preserves self evidence and total label-wise KLA weight, evaluates every registered transfer fraction, and selects the largest fraction whose fused label remains inside the two-sided ordinary-reference log-odds envelope. If no fraction passes, the receiver uses ordinary fusion. The complete Bernoulli Gaussian-mixture payload is charged before projection, so a smaller transfer fraction never creates artificial communication savings. The source and label remain offline teacher choices; this screen cannot support deployment or generalization claims.
