# V83 route-and-trust headroom finding

## Decision

Reject self-funded trust amplification of the frozen merge-split replacement
route.  Neither M24 nor X36 has material H=3 task headroom, so no relative
value model is trained on this action family.

## Paired result

V83 keeps the V75 direct-safe replacement formations, the 0.70
within-formation backbone, exact directed-message count, and a rolling-B3-safe
`candidate -> reference -> candidate` sequence.  It varies only the selected
cross-formation weight, funded from receiver self-weight.

| Scale | Best trust | Mean tracking | Worst | Minimum formation | Window consensus | Terminal consensus | Gate |
|:--|--:|--:|--:|--:|--:|--:|:--:|
| M24 | 0.05 | +0.668% | +0.000% | +0.000% | -0.625% | -1.352% | no |
| X36 | 0.05 | +0.201% | +0.000% | +0.000% | +0.295% | +0.167% | no |

On M24, increasing trust to 0.10, 0.15, and 0.20 changes mean tracking by
`-0.027%`, `-0.582%`, and `-0.748%`.  On X36, the corresponding gains are
`+0.118%`, `-0.034%`, and `-0.183%`.  Both response curves peak at the
original 0.05 residual weight and turn downward.  The result is therefore not
an underpowered-weight counterexample: the selected alternative senders do
not carry enough distinct task-relevant information at these anchors.

## Scenario implication

The merge-split scene remains useful as a negative control for attractive but
low-value source-side opportunities.  V70 reported only 1.176% M24 and 1.675%
X36 local quarantine pressure at the selected anchors, and V72/V83 now show
that their alternative-sender opportunities do not propagate into material
tracking gain.  Further tuning route weights, recovery rules, or a learned
selector on these two actions cannot create missing action-space headroom.

The next development scene is `braided-handover`.  Its frozen geometry has no
blackout, a sparse formation chain, and approximately 50--57% of target-time
pairs visible to exactly one formation, with at least one formation handover
per target group.  That creates the mechanism the method is intended to
control: unique information must cross an effective KLA graph during a real
handover.  The next experiment first scans observable current-state routing
leverage on M24/X36 braided-handover caches, freezes event anchors without
tracking outcomes, and only then opens a small signed action bank containing
reference, selective cross-input protection, and safe transport/trust actions.

## Evidence boundary

V83 is opened merge-split development evidence.  It rejects one frozen action
family at two anchors; it is not a validation or generalization result and
does not invalidate the earlier strong radial V65 results.
