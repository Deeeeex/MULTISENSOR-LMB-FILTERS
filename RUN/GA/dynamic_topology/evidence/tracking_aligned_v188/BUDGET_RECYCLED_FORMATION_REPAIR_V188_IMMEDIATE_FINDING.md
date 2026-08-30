# V188 cross-scale executable-action finding

## Paired base layer

The M24 and X36 screens use the same seed, cached pre-action posterior,
measurements, delivery uniforms, filter RNG, static carrier, fusion weights,
and three-page horizon for the full-posterior reference and V99 causal online
admission arm.

| Scale | V99 E-OSPA gain | V99 RMSE gain | V99 consensus gain | V99 byte saving |
|:--|--:|--:|--:|--:|
| M24, seed 211, t=104 | +9.044% | +3.734% | +21.104% | +5.080% |
| X36, seed 211, t=72 | +2.802% | -0.666% | +5.149% | +6.550% |

The base admission layer is already strong on M24.  On X36 it saves more
communication and improves E-OSPA and consensus, but slightly worsens RMSE.
This is the specific residual error that the label-repair layer must address.

## Executable repair action space

At the first captured page, every sensor transmits the frozen 24-byte
per-label light synopsis only after the V99 saving has created sufficient
credit.  A candidate is executable only when all members of one formation
can reach the same source, every member has nonnegative observable Bayes-risk
reduction, the source still holds the exact complete Bernoulli GM object, and
the rich synopsis, request, and independent complete responses fit the
spendable balance.  Truth is read only after each action is frozen.

| Scale | Feasible / jointly positive | Selected action | Network E-OSPA | Network RMSE | Consensus | Affected formation E-OSPA / RMSE | Charged page saving |
|:--|--:|:--|--:|--:|--:|--:|--:|
| M24 | 4 / 1 | F2, source 3, label [25,15] | +0.003% | +0.055% | +1.673% | +0.011% / +0.083% | +6.533% |
| X36 | 6 / 2 | F3, source 29, label [25,18] | +0.268% | +0.699% | +0.458% | +1.628% / +13.393% | +5.180% |

The deterministic utility-per-byte projection selects the only jointly
positive M24 action and the stronger of the two jointly positive X36 actions.
For X36, the weakest affected-sensor gains are also positive: +0.143% E-OSPA
and +8.064% RMSE.  Thus the same truth-free proposal and projection path has
positive, fully charged action headroom at both scales.

## Method decision

This result separates three issues.  First, the V99 admission layer creates
enough real communication credit for the synopsis and complete GM repair on
both scales.  Second, a single X36 formation repair can remove a large local
RMSE error without sacrificing E-OSPA, consensus, or net communication.
Third, the network-average increment is necessarily diluted because only one
of six X36 formations is repaired on one page.  The next experiment must
therefore evaluate recursive propagation over the scale-aware horizon (M24
four pages, X36 six pages), not demand that a one-page one-formation edit
alone meet the final network gate.

M24 also shows why no-op must remain an explicit action.  Three of four
budget-feasible repairs are harmful after truth is opened, while the selected
positive repair has only a small immediate accuracy increment.  The eventual
value rule must spend credit only when a calibrated finite-horizon lower
confidence bound beats no-op; positive scalar Bayes-risk reduction is an
eligibility condition, not sufficient evidence to transmit.

These two opened seed-211 pages authorize a bounded recursive V188 pilot and
grouped finite-horizon teacher generation.  They do not authorize value-model
training, a development-gate claim, validation seeds, or paper-facing
cross-scene conclusions.
