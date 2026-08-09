# V59 event-conditioned formation-level headroom result

## Decision

The formation-level action family is closed for the current method search.
V58 selected three fresh M24 convoy states with the largest observable
formation retention debt.  V59 then enumerated all 16 formation protection
subsets at each state, retained only candidates with nonnegative mean,
worst-sensor, minimum-formation, window-consensus, terminal-consensus and
attempted-byte changes, and scored the resulting one-step intervention under
the registered three-step tracking horizon.

| Time | Safe subsets | Strict candidates | Best strict action | Mean E-OSPA gain | Attempted-byte saving |
|--:|--:|--:|:--|--:|--:|
| 40 | 16 | 1 | `suspend-f4` | +0.054% | +1.027% |
| 88 | 16 | 4 | `suspend-f1-f2-f3` | +0.384% | +2.415% |
| 128 | 16 | 5 | `suspend-f2-f3-f4` | +0.364% | +2.864% |

The mean best strict gain is only `+0.267%`; no state reaches the registered
`5%` strong-gain threshold.  The formation-level headroom gate therefore
fails with `0/3` strong states.

This failure is stronger than another weak learned policy result: the entire
safe formation-subset action space was opened at the selected states.  More
formation-level selectors, longer training or another combination search
cannot recover headroom that is absent from the action family itself.

## Mechanism interpretation

Formation retention debt detects states in which the registered cross-input
changes expected label-existence mass, but it does not establish that the
rescued labels have current measurement support.  A formation suspension also
applies the same decision to every label and every affected receiver.  Helpful
and harmful label-level effects can therefore cancel inside one coarse action.

The next action space is receiver--sender--label routing.  A candidate removes
or retains an individual sender label for an individual receiver while keeping
the transmitted label's full Gaussian mixture.  The first diagnostic will
reweight V58's rescued-existence signal by the receiver's current
`detectionAssociationMass`; this tests whether V58 selected unsupported or
spurious labels before another tracking experiment is opened.

## Claim boundary

This is M24 convoy development evidence for seed 1201 at times 40, 88 and 128.
It does not establish an X36 result, a learned-policy result, a held-out result
or a paper-level generalization claim.  The original objective remains open:
the replacement method must show clear, repeatable gains on M24 and X36 before
multi-scene evaluation.
