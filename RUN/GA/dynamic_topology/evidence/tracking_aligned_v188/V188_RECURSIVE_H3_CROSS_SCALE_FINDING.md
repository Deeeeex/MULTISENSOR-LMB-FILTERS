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

## Method decision

The next method should retain V99 as the safe base and add a two-stage,
receiver-certified propagation trust region:

1. The existing light synopsis shortlists one formation action.
2. The source sends a charged rich motion synopsis for that label.
3. Each receiver predicts its own and the source's moment means over the
   registered propagation horizon and returns a small charged safety vote.
4. The complete label is requested only when every receiver remains inside
   the pre-existing position-error cutoff and the learned horizon-value
   estimate is positive; otherwise the explicit action is no-op.
5. Rich-synopsis, vote, request and complete-label bytes are all debited from
   V99 communication credit before transmission.

The cutoff is a trust-region boundary, not a claimed sufficient condition
for tracking improvement.  It would reject the harmful M24 action while
retaining the selected X36 action on this opened pair.  A separate recursive
teacher bank must determine whether the alternative X36 formation-1 action
offers a better E-OSPA/RMSE/consensus balance before this gate is frozen.

## Evidence boundary

The temporal and formation metrics use truth only after the V188 action was
frozen.  The propagation-gap diagnosis is causal and truth-free, but its
interpretation was made after opening seed 211.  It authorizes a new method
version and targeted recursive teacher comparisons only; it does not
authorize model training, validation claims, or promotion to the main Lark
current-best table.
