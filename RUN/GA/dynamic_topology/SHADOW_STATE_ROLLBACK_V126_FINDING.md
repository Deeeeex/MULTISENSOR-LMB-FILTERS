# V126 finding: safe state recovery closes the X36 local-regret gap

## Registered result

| Metric | V105 protection only | V126 with exact static-state rollback | V126 gate |
|:--|--:|--:|:--:|
| Mean E-OSPA gain vs static | +5.259% | +5.705% | pass |
| Minimum mature-page gain | +5.188% | +5.984% | pass |
| Minimum formation gain | -0.931% | +0.125% | pass |
| Minimum formation-time gain | -15.75% | +0.000% | pass |
| F6 peer terminal gain | -2.940% | +0.000% | pass |
| Worst-sensor gain | positive | +14.717% | pass |
| Window / terminal consensus gain | +9.650% / +17.214% | +9.205% / +16.183% | pass |
| Main-path attempted-byte saving | +6.117% | +6.032% | pass |

V126 reuses the frozen X36 seed-211, t=72, H=8 static arm.  It keeps the
complete V105 topology, fusion weights and protection schedule, then restores
the paired static post-fusion state at only 36 opened node-time cells: F2 on
page 5, F1 on pages 6--8, and F6 on pages 7--8.  The applied rollback counts
are exactly `[0 0 0 0 6 6 12 12]`.  All 48 formation-time gains become
nonnegative while the network mean and mature-page gains remain above 5%.

## Mechanism conclusion

The X36 action space contains enough estimation headroom.  V105's aggregate
gain is not intrinsically coupled to its local failures: removing accumulated
harmful state at the affected formation-time cells preserves and slightly
increases the network gain.  This narrows the missing method component from
another topology or payload rule to a temporal safety mechanism that detects
when a protected trajectory should rejoin a conservative state path.

The exact shadow arm is deliberately privileged.  Its rollback cells are
selected from opened V105 outcomes, and its replacement posteriors come from a
counterfactual static full-payload run.  The reported 6.032% byte saving counts
only the V105 main path; it does not include any communication or computation
needed to maintain that exact shadow state.  V126 is therefore a mechanism
upper bound, not a deployable communication result and not a generalization
claim.

## Method decision

The next candidate should preserve V105's high-gain working state while adding
a causal, low-cost safety anchor.  It must use only online local posterior,
observation, delivery and route-history features to decide whether to retain
the working state or return to the anchor.  An exact parallel static network is
inadmissible because it would erase the communication claim.  The preferred
implementation is a checkpoint-and-release controller: retain a locally
available conservative posterior before protection, propagate it without
extra inter-node messages, and trigger release or rollback from observable
state divergence and support-loss signals.  A learned GNN remains optional
only after this causal action and feature space shows a positive closed-loop
gate on X36.
