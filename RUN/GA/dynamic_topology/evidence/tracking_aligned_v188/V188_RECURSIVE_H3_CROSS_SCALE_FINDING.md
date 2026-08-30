# V188 cross-scale recursive H=3 finding

## Question

The immediate action-bank screen showed a fully charged, truth-free repair
with positive current-page headroom on both M24 and X36.  This experiment
tests the missing claim: does that action remain beneficial after the
repaired complete Bernoulli GM state is propagated through two later V99
pages?

## Paired result

Both rows use the same scene, seed, cached posterior, measurements, link
uniforms and filter RNG as their frozen static and V99 arms.  Repair is
applied only on the first page; its synopsis and complete-label traffic are
included in attempted bytes.  Delivery is ideal but charged, so these are
recursive teacher results rather than deployable evidence.

| Scale | Metric | Static | V99 | V188 | V188 vs V99 |
|:--|:--|--:|--:|--:|--:|
| M24 | Mean E-OSPA | 71.665 | 65.183 | 65.664 | -0.738% |
| M24 | Mean RMSE | 41.837 | 40.275 | 43.759 | -8.649% |
| M24 | Consensus gain vs static | 0 | +21.104% | +20.137% | -1.226% |
| M24 | Attempted-byte saving vs static | 0 | +5.080% | +4.268% | +38,224 B |
| X36 | Mean E-OSPA | 85.970 | 83.562 | 83.701 | -0.167% |
| X36 | Mean RMSE | 57.902 | 58.288 | 57.849 | +0.754% |
| X36 | Consensus gain vs static | 0 | +5.149% | +4.991% | -0.167% |
| X36 | Attempted-byte saving vs static | 0 | +6.550% | +5.697% | +90,472 B |

The M24 action changes only formation 2.  Relative to V99, that formation's
three-page E-OSPA and RMSE degrade by 2.338% and 11.545%.  The action is
slightly positive at time 104, but network RMSE degrades by 31.333% at time
105 and E-OSPA degrades by 2.417% at time 106.  The X36 action changes only
formation 3: its RMSE improves by 8.196%, but its E-OSPA degrades by 1.025%.
Thus V188 repairs the X36 localization gap but does not preserve the joint
objective.

## Mechanism diagnosis

The V188 proxy ranks the current-page minimum Bernoulli-risk reduction.  It
does not bound how far the replacement state will move under prediction.
For the selected M24 action (formation 2, source 3, label `[25,15]`), the
source-to-receiver moment-mean position gap is about 154--159 m and grows to
about 159--166 m after one prediction.  For the selected X36 action
(formation 3, source 29, label `[25,18]`), the corresponding gaps are about
36--59 m and 38--63 m.  The registered position-error cutoff is 150 m for
both scales.

This separation explains why covariance-normalized current compatibility is
not sufficient: a broad posterior can make a large displacement look
compatible even though the copied velocity and position subsequently alter
the recursive KLA trajectory.  The result rules out unconditional spending
based only on current-page risk reduction.

## Alternative X36 teacher and falsified trust gate

The alternative formation-1 action is a Pareto improvement over V99 across
the same recursive window.  Mean E-OSPA, RMSE and consensus improve by
`0.035%`, `0.288%` and `0.778%`, while attempted communication remains
`6.010%` below the static reference.  Within formation 1, RMSE gain relative
to static moves from `-0.069%` under V99 to `+2.142%`.  This action therefore
has genuine finite-horizon value even though its source-to-receiver predicted
position distances are `176.984--198.816 m`, all outside the proposed
`150 m` trust region.

The Euclidean cutoff is consequently not a necessary safety condition.  It
rejects the beneficial formation-1 action, while it accepts the original
formation-3 action that improves RMSE but sacrifices E-OSPA and consensus.
The causal propagation diagnostic remains useful as a feature, but it cannot
be a hard accuracy gate.

The per-formation result also exposes the next target.  Formation-1 repair
fixes formation 1, and formation-3 repair moves formation-3 RMSE gain from
`-1.489%` to `+6.829%`; neither touches formation 2, whose RMSE gain remains
`-14.198%` and dominates the weakest-formation gate.

The formation-2 teacher is also finite-horizon positive, despite being
slightly negative on its action page.  Relative to V99, it improves mean
E-OSPA, RMSE and consensus by `0.155%`, `0.030%` and `0.005%`, while keeping
`6.191%` communication saving relative to static.  However, formation-2
RMSE gain moves only from `-14.198%` to `-13.859%`.  The current top-one
label is useful but far too weak to close the formation deficit.

## Repeated-page falsification

Repeating the same formation-2 top-one repair on all three H=3 pages does
not close that deficit.  The action is selected and applied on every page,
with `60,232 / 60,280 / 60,112 B` of charged repair traffic.  Relative to
V99, mean E-OSPA improves only `0.099%`, while mean RMSE and consensus
degrade by `0.369%` and `1.164%`; communication saving relative to static
falls from `6.550%` to `5.039%`.  This is worse than the one-page F2
teacher, which was weakly positive on all three tracking objectives.

The result rejects the hypothesis that the F2 gap is mainly caused by
insufficient repetitions of the selected `[19,16]` label.  Reusing the same
repair consumes more credit and eventually overwrites useful evolution with
an increasingly stale cross-formation state.  The next teacher must change
the source-label action, not increase the dose of the current one.

## Two-formation action-set teacher

The causal V188 proxy ranks formation 3 first and formation 1 second.  Letting
the charged teacher projector take both actions produces the first V188 arm
that improves every aggregate objective relative to static:

| X36 H=3 arm | Selected formation(s) | E-OSPA gain | RMSE gain | Consensus gain | Byte saving |
|:--|:--|--:|--:|--:|--:|
| V99 base | none | +2.802% | -0.666% | +5.149% | +6.550% |
| F1 teacher | 1 | +2.836% | -0.377% | +5.887% | +6.010% |
| F2 teacher | 2 | +2.953% | -0.636% | +5.154% | +6.191% |
| F3 teacher | 3 | +2.639% | +0.092% | +4.991% | +5.697% |
| F3 + F1 teacher | 3, 1 | +2.673% | +0.382% | +5.712% | +5.530% |

Relative to V99, the two-action set improves RMSE and consensus by `1.041%`
and `0.594%`, at a `0.132%` E-OSPA cost.  It establishes genuine
multi-objective action-set headroom, but it is not the new overall best:
formation 2 remains at `-14.198%` RMSE and formation 5 at `-1.000%`, and the
aggregate E-OSPA/RMSE gains remain below the registered `5%/2%` paper gate.

## Revised method decision

The next method keeps only properties that can actually be guaranteed outside
the value model: rolling graph connectivity, complete-label delivery,
communication-credit conservation and an explicit no-op.  Positional
propagation distance becomes one causal feature among others.  A shared
finite-horizon value model must predict the separate E-OSPA, RMSE and
consensus effects of each formation action; a calibrated lower-confidence
rule admits an action only when the joint predicted value is positive.

Because the formation-1 and formation-3 teachers improve complementary
objectives and disjoint formations, the teacher projector permits at most two
formations on a page.  This expansion is restricted to ideal, charged teacher
runs; the deployable default remains one action.  The paired result confirms
finite-horizon action-set headroom, but the formation-2 attribution shows that
one preselected label per formation is the new bottleneck.

At X36 t=72, each formation has `101--124` executable common source-label
candidates before the V188 hand-crafted proxy collapses them to one.  The
next version therefore uses a hierarchical action space: a scale-equivariant
formation graph model allocates repair opportunities across formations, and
a label-level ranker selects from a small, diverse causal Top-K within each
chosen formation.  The repeated-top-one teacher has now failed, so the next
bounded teacher evaluates a different F2 shortlist member before any model is
fit.  The strongest causal contrast is source 19, label `[13,12]`: its
minimum observable risk reduction is `0.31444`, versus `0.00098` for the
current source-20 `[19,16]` action, and its normalized precision gain is
`0.37389`, versus `0.00130`.  These are selection features rather than a
tracking claim.  Model training remains closed until the expanded bank
demonstrates cross-scale finite-horizon headroom.

## Evidence boundary

The temporal and formation metrics use truth only after each V188 action was
frozen.  The propagation-gap diagnostic is causal and truth-free, but its
interpretation and the alternative teacher choice were made after opening
seed 211.  The result authorizes targeted recursive action and action-set
teachers only; it does not authorize model training, validation claims, or
promotion to the main Lark current-best table.
