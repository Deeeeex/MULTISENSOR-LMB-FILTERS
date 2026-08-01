# M24 formation H=3 strict-oracle audit

## Decision

The frozen M24 sentinel fails before value-model training.  Across seeds
`211`, `223`, and `227` at predecision times `60` and `72`, the exact oracle
over the 19-action singleton-plus-pair bank has:

- positive strict-feasible gain in `4/6` states;
- strong gain of at least 3% in `0/6` states;
- mean strict-feasible gain of `+0.480%`;
- required M24 gates of at least `2/6`, at least `1/6`, and at least `+2%`,
  respectively.

The positive-count condition passes, but both the strong-state and mean-gain
conditions fail.  X36 teacher generation is therefore not authorized.

## Frozen protocol and provenance

- Generation commit: `c32a0649834e0119b3993a95e9e441e6269c8c3b`
- Preset: `m24-formation-fov` (24 sensors, four formations)
- Sentinel-training seeds: `211`, `223`
- Sentinel-development seed: `227`
- Opened times: `60`, `72`
- Return: `H=3`; the candidate is applied at the first step and the fixed
  counter-clockwise reference is restored for the next two steps
- Action bank: reference + 12 one-formation actions + six conservative
  two-formation actions
- Reserved final seeds, still unopened: `251`, `257`, `263`, `269`, `271`

An action is deployment-feasible only if all six gains are nonnegative:
network-mean tracking, minimum-formation tracking, worst-sensor tracking,
consensus, attempted bytes, and delivered bytes.  Reference fallback is
always feasible and has zero gain.

## Exact state results

| Seed | Time | Feasible actions | Strict-oracle action | Mean | Min. formation | Worst sensor | Consensus | Attempted bytes | Delivered bytes |
|--:|--:|--:|:--|--:|--:|--:|--:|--:|--:|
| 211 | 60 | 5/19 | formations 1-2, trust 0.30 | +1.591% | 0.000% | +5.494% | +2.807% | +0.696% | +0.742% |
| 211 | 72 | 2/19 | formation 2, trust 0.70 | +0.024% | 0.000% | +0.100% | +0.006% | +0.998% | +0.163% |
| 223 | 60 | 1/19 | reference | 0.000% | 0.000% | 0.000% | 0.000% | 0.000% | 0.000% |
| 223 | 72 | 3/19 | formations 1-2, trust 0.30 | +0.516% | 0.000% | +0.001% | +1.882% | +1.152% | +1.183% |
| 227 | 60 | 1/19 | reference | 0.000% | 0.000% | 0.000% | 0.000% | 0.000% | 0.000% |
| 227 | 72 | 6/19 | formations 2-4, trust 0.30 | +0.751% | +0.001% | +0.003% | +0.833% | +2.479% | +2.580% |

For scale context, the identically audited D12 control has strict-oracle gains
`[0, 3.752, 0.003, 5.460, 4.159, 1.122]%`: `5/6` positive, `3/6` strong, and
`+2.416%` mean.  This contrast is evidence of a scale-dependent limitation,
not evidence that the audit is incapable of finding feasible gain.

## Binding constraints

The table below counts violations among actions that improve network-mean
tracking.  Counts overlap because one action can violate multiple constraints.

| Seed | Time | Mean-positive actions | Min. formation | Worst sensor | Consensus | Attempted bytes | Delivered bytes |
|--:|--:|--:|--:|--:|--:|--:|--:|
| 211 | 60 | 15 | 10 | 2 | 5 | 2 | 6 |
| 211 | 72 | 15 | 11 | 7 | 8 | 5 | 6 |
| 223 | 60 | 16 | 16 | 5 | 11 | 11 | 13 |
| 223 | 72 | 11 | 7 | 6 | 4 | 0 | 0 |
| 227 | 60 | 14 | 13 | 5 | 14 | 2 | 2 |
| 227 | 72 | 13 | 1 | 5 | 6 | 1 | 1 |

The action bank frequently reduces average E-OSPA, but the most common prices
are a loss in at least one formation and increased network disagreement.  At
seed 227/time 60, every one of the 14 mean-improving actions worsens consensus.

## Claim boundary and next falsifier

This result does **not** show that dynamic topology is ineffective.  It
falsifies the narrower combination of fixed times `60/72`, one-step
interventions, and singleton/conservative-pair actions under the strict
six-target deployment rule.  It also shows that increasing predictor capacity
now would be scientifically invalid: even an oracle selector lacks the
pre-registered headroom.

The next experiment will select decision states from the fixed-reference
trajectory using only currently observable posterior disagreement, link
reliability, coverage/load change, and past selected/delivered topology.  The
same action bank and the same six-target gate will be reused first.

1. If event-conditioned states pass M24 headroom, the main defect was fixed-time
   sampling; the learned component should predict **when** and **which** safe
   intervention is useful.
2. If they fail, the action space must be broadened to coordinated
   multi-formation topology projections with an explicit consensus-risk
   constraint before any GNN training or X36 teacher generation.
3. Reserved seeds remain closed until the event rule, action space, feature
   contract, predictor, confidence calibration, and reference fallback are
   frozen.
