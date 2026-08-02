# Formation H=3 terminal debt-repair v24 audit

## Verdict

v24 passed its frozen-protocol, provenance, reproduction, execution, and
safety-instrumentation checks, but failed the strict and strong gates.  The
complete structured terminal-repair grid contains no safe teacher sequence.
The current H=3 mode-vector family is therefore closed and GNN training
remains unauthorized.

## Contract checks

- generation commit: `a2d9e227454c2259660e5e90edce868e59155633`;
- opened state: `m24-formation-fov / seed 211 / t=72`;
- frozen grid: four v23 two-step prefixes crossed with 18 structured terminal
  actions, for 72 new candidates;
- the all-reference sequence and saved v21 closest sequence were exact
  controls;
- all four prefix-plus-reference outcomes and the saved v21 control reproduced
  their registered targets within `5e-6` percentage points;
- all 74 sequences passed physical, payload, exact-execution, rolling-B3,
  truth-use, repair, emergency, and infeasibility checks;
- strict feasibility required all six signed targets to be nonnegative;
- strong headroom additionally required at least `3%` mean tracking gain.

## Main outcomes

| Role | First | Second | Terminal | Mean | Min. formation | Worst sensor | Consensus | Attempted | Delivered |
|:--|:--|:--|:--|--:|--:|--:|--:|--:|--:|
| Closest overall | `[1,1,2,2]` | `[1,4,1,1]` | `[1,1,1,4]` | +2.685% | 0 | -0.002% | -0.681% | -0.387% | -0.405% |
| Best high-return balance | `[1,4,3,1]` | `[1,1,1,4]` | `[1,1,4,2]` | +5.752% | 0 | +0.100% | -1.704% | +1.810% | +1.011% |
| Minimum high-return consensus debt | `[1,4,3,1]` | `[1,1,4,4]` | `[1,1,1,2]` | +6.083% | 0 | +0.099% | -2.092% | +1.140% | +0.311% |
| Minimum high-return delivered debt | `[1,4,3,1]` | `[1,1,2,4]` | `[1,1,1,2]` | +6.732% | 0 | +0.099% | -3.114% | +1.605% | +0.798% |
| Best mean | `[1,4,3,1]` | `[1,1,1,4]` | `[1,1,1,2]` | +8.649% | 0 | +0.100% | -3.682% | +1.021% | +0.186% |

The candidate strict count was `0/72`; the candidate strong-safe count was
also `0/72`.  The saved v21 control reproduced
`[+5.774941, 0, +0.042126, -1.527435, +1.027062, +1.073903]%`.

## Research finding

Opening the third action cannot remove the window-average consistency floor
within the present H=3 family.  The best balanced high-return candidate already
improves mean tracking, the formation tail, the sensor tail, attempted bytes,
and delivered bytes, yet consensus averaged over the three-step window remains
`1.704%` worse than the all-reference trajectory.  The saved v21 sequence
shows the same pattern at `1.527%`.  This repeated residual is more specific
than a generic negative result: tracking and communication headroom exist,
but the current metric does not reveal whether the final-step posterior has
already recovered or still needs more time.

The current scalar proxy is not a viable selector.  It predicted no positive
nonreference action, produced `0` true positives and `73` false negatives,
and had `0.000` action agreement.  A learned graph-value model would therefore
need separate heads for tracking, tail, consensus, and communication debt;
however, there is still no safe teacher target with which to justify training
that model.

## Next decision

Do not enumerate more H=3 mode vectors.  Generalize only the opened-return
screen's time horizon and test whether one or two appended reference actions
repair both window-average and final-step consensus for a small, frozen set of
v21/v24 boundary sequences.  Compare every H=4/H=5 candidate with an
all-reference trajectory of the same horizon, and retain all existing
execution and safety gates.

If the residual consensus debt crosses zero while the other five targets and
the `3%` mean threshold remain nonnegative, broader opened-state teacher
collection becomes admissible.  If it does not, redesign the action
representation or abandon this action family before learning.

## Evidence boundary

All findings are local to the opened seed-211/time-72 development state.
Seeds 223/227, X36, and final seeds remain unopened.  This audit authorizes
only the specified recovery-horizon mechanism probe, not validation,
generalization, GNN training, or paper-level claims.
