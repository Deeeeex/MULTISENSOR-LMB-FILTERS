# Formation H=3 first-step mode-vector v22 audit

## Verdict

The frozen v22 probe passed its execution and provenance gates but did not
establish strong safe headroom.  It found one weak, strictly feasible
nonreference action and several high-value prefixes with distinct terminal
debts.  These are development labels for a debt-aware two-step teacher beam,
not evidence for a deployable selector.

## Contract checks

- generation commit: `796ef31abdae34940cef238ecca7d7b3380466d4`;
- opened state: `m24-formation-fov / seed 211 / t=72`;
- frozen candidates: all 67 mode vectors within Hamming distance two of
  `[1,1,1,1]` at the first step, followed by reference twice;
- exact reproduction: `[1,1,3,1] -> reference -> reference` reproduced the
  registered v17 six-target vector within `5e-6` percentage points;
- physical, payload, exact-execution, rolling-B3, truth-use, repair,
  emergency, and infeasibility gates passed for every candidate;
- strict feasibility: all six signed targets nonnegative;
- strong headroom: strict feasibility plus at least `3%` mean tracking gain.

## Main outcomes

| Role | First-step vector | Mean | Min. formation | Worst sensor | Consensus | Attempted | Delivered | Strict |
|:--|:--|--:|--:|--:|--:|--:|--:|:--:|
| Weak safe anchor | `[1,4,1,1]` | +0.024% | +0.000% | +0.100% | +0.006% | +0.998% | +0.163% | yes |
| Balanced near-strong | `[1,1,2,4]` | +2.936% | -0.001% | -0.003% | +1.101% | -0.981% | -1.026% | no |
| Best mean | `[1,1,3,4]` | +7.166% | +0.000% | -0.001% | -4.986% | -1.109% | -1.160% | no |
| High mean with byte credit | `[1,4,3,1]` | +6.013% | +0.000% | +0.100% | -11.452% | +1.165% | +0.338% | no |
| Misleading four-metric candidate | `[1,4,1,2]` | +1.549% | -1.236% | +0.100% | +5.222% | +0.061% | -0.817% | no |

The screen contained two strictly feasible rows out of 67: the reference and
the weak safe anchor.  No strict row met the strong-gain threshold.

## Research finding

The formations have nonexchangeable roles.  At this opened state, formation
3 creates most of the tracking gain, formation 4 can create consensus credit
but often spends bytes and harms a formation-local tail, and formation 2 can
create communication and worst-sensor slack but cannot by itself produce
tracking value.  Uniform trust changes therefore erase useful
complementarity.

The six targets also cannot be collapsed safely into the current scalar
posterior proxy.  The proxy admitted 11 candidates and had no false
positives for positive mean return, but missed 50 other positive-mean
candidates and rejected the only realized safe nonreference action.  A
future value model needs separate heads for mean gain, formation tail,
sensor tail, consensus debt, attempted bytes, and delivered bytes, followed
by a hard safety projection and reference fallback.

## Next decision

Freeze a two-step beam before observing new outcomes.  Use a small set of
v22 prefixes that span weak-safe, balanced-near-strong, high-mean, and
high-mean-with-byte-credit regimes.  At the second step, vary only the three
formations with observed complementary roles and use a structured factorial
repair bank.  The third step remains reference.  A positive v23 result must
be strictly feasible on all six targets and exceed `3%` mean gain; otherwise
the current three-step action family lacks teacher headroom and should not
advance to GNN training.

## Evidence boundary

All conclusions are local to the opened seed-211/time-72 development state.
Seeds 223/227, X36, and final seeds remain unopened.  No validation or
generalization claim is authorized.
