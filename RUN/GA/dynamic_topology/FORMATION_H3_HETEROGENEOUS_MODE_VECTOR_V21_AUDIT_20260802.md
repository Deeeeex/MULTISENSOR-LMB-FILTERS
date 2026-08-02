# M24 H=3 heterogeneous mode-vector radius-2 audit

## Decision

The v21 terminal mode-vector neighborhood contains no strict-feasible
candidate.  The strict oracle remains all-reference with `0%` gain, so there
is no safe label for a selector that keeps the outcome-inspected `[9,13]`
prefix fixed and optimizes only the third step.

## Provenance and controls

- Generation commit: `c2e040f58a2104d439548fefac8cb901c1ae79b4`
- Cache generation commit: `c9c6d4dcdc7ad1cb04fb88a22823e99c7fc5bc53`
- Preset / state: `m24-formation-fov / seed 211 / t=72`
- Prefix mode vectors: `[1,1,3,1]`, then `[1,1,1,4]`
- Terminal center: `[1,2,2,2]`
- Candidate set: complete Hamming radius two, `1 + 12 + 54 = 67`
- Full teacher dictionary: `4^4 = 256` mode vectors
- Candidate strict-feasible count: `0/67`
- Runtime failures: none

The all-reference and prefix-plus-reference controls reproduce prior outcomes.
The center vector also reproduces the v20 2+3+4 action within the frozen
`5e-6` percentage-point tolerance.  All 67 actions pass physical, reference
payload, exact execution, and rolling-B3 checks.

## Best candidates

| Rank | Terminal mode vector | Mean | Min. formation | Worst sensor | Consensus | Attempted | Delivered |
|--:|:--|--:|--:|--:|--:|--:|--:|
| 1 | `[1,4,4,2]` | +5.775% | 0.000% | +0.042% | -1.527% | +1.027% | +1.074% |
| 2 | `[1,3,4,2]` | +5.772% | 0.000% | +0.038% | -1.528% | +1.027% | +1.074% |
| 3 | `[1,2,4,3]` | +5.759% | 0.000% | +0.031% | -1.547% | +1.027% | +1.074% |
| 4 | `[1,2,4,2]` | +5.760% | 0.000% | +0.031% | -1.565% | +1.027% | +1.074% |

The best vector repays about 86.7% of the original `-11.486%` consensus debt.
Every other target is nonnegative and mean tracking remains above the `3%`
strong threshold, so the residual failure is sharply localized to terminal
consensus.  Increasing formation 2 from mode 2 to modes 3/4 helps only
marginally; changing formation 4 from mode 2 to mode 3 also helps marginally.
Their combined improvement is far smaller than the remaining gap.

## Mechanism finding

Mode effects are formation-dependent and interactive.  In particular,
formation 3 at mode 4 produces the largest repair, while applying comparable
modes to formation 1 worsens the result.  Treating a mode as a scalar trust
knob or scoring formations independently is therefore inadequate.  A graph
model is justified because it can condition a formation's action value on
the neighboring formations, selected sources, and current topology history.

However, v21 also shows that a terminal-only graph model would solve the wrong
problem.  The best available third action still cannot repay the debt created
by the first two actions.  Learning must be sequence-aware: the first action
needs a predicted consensus-debt budget, and later actions should update that
budget rather than attempt a final repair after unconstrained exploitation.

The current posterior-risk proxy allows only one of 67 candidates and rejects
the best vector `[1,4,4,2]` (proxy objective `-0.007798`).  It is not a suitable
teacher label or pruning rule for the temporal controller.

## Consequence

The next bounded mechanism should use a debt-aware beam over mode vectors at
the first two steps.  Candidate selection may be privileged on this opened
state, but the future deployment model must consume only current posterior,
link, payload, formation, and recent-topology features.  A safe fallback to
the registered reference remains mandatory.

This is outcome-inspected single-state mechanism evidence.  It does not
establish M24 or X36 performance, and seeds 223/227 and final seeds remain
unopened.
