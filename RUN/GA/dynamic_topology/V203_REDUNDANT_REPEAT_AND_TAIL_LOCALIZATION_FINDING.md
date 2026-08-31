# V203 redundant-repeat and remaining-tail finding

## The last F3 repeat is redundant

The corrected X36 `seed=211, t=72, H=8` observation-handover sequence was
rerun after deleting only the F3 action at `t=79`.  The retained actions are
F2 at `t=72` and F3 at `t=76/78`, all using complete-label residual KLA with
source weight `0.5`.

| Metric | F2+F3 with t=79 repeat | Without t=79 repeat | Change after deletion |
|:--|--:|--:|--:|
| Mean E-OSPA | 78.032340 | 78.032626 | +0.000286 |
| Mean RMSE | 56.738264 | 56.739546 | +0.001282 |
| Static-relative consensus gain | +11.383% | +11.383% | unchanged at report precision |
| Static-relative byte saving | +4.004% | +4.217% | +0.213 percentage points |
| Attempted bytes | 27,434,480 | 27,373,768 | -60,712 B |
| F3 E-OSPA gain | +9.549% | +9.546% | -0.002 percentage points |
| F3 RMSE gain | +5.795% | +5.764% | -0.031 percentage points |

The tracking change is negligible, F3 remains positive on both metrics, and
the F1/F5/F6 tails are numerically unchanged.  The last-page action should
therefore be removed.  A one-page semantic-action cooldown (same formation
and label, regardless of which source currently supplies it) is now a causal
candidate for preventing this kind of redundant repeat without using time,
formation, or label identifiers as learned features.

## Remaining-tail localization

The corrected F2+F3 rollout was used to score 4,703 shortlisted current
physical-neighbor label replacements over 42 receiver-time cells.  Candidate
features are truth-free.  Current truth supplies only the immediate E-OSPA
and matched-RMSE targets after candidate construction.  The table below
collapses receiver rows to actions executable by all six receivers in one
formation.

| Tail | Formation-common action | Minimum risk proxy | Immediate E-OSPA sum / minimum | Immediate RMSE sum / minimum | Joint-positive receivers | Interpretation |
|:--|:--|--:|--:|--:|--:|:--|
| F1, t=77 | source 13, label `[7,7]` | 0.026886 | +35.880962 / +5.974390 | +7.724154 / +0.228923 | 6/6 | MAP-sensitive set-entry candidate; ordinary maximum-risk ranking misses it |
| F1, t=79 | source 24, label `[25,19]` | 0.498770 | +35.597374 / +5.920303 | +7.514103 / +0.137656 | 6/6 | A high-risk alternative exists late, but competes with the stronger F5 tail |
| F5, t=78 | source 19, label `[19,16]` | 0.498755 | +47.243813 / +7.037599 | -0.408344 / -0.642444 | 4/6 | Too early: set accuracy improves but the RMSE sign is not formation-safe |
| F5, t=79 | source 22, label `[19,16]` | 0.498849 | +47.418628 / +7.129655 | +0.725895 / +0.097040 | 6/6 | Clean late support-restore candidate in the corrected state |
| F6, t=78 | source 28, label `[7,5]` | 0.439397 | +33.125643 / +5.451272 | +242.033257 / +31.263074 | 6/6 | Strong precision-refresh opportunity |
| F6, t=79 | source 21, label `[7,5]` | 0.498629 | +35.005447 / +5.823658 | +167.711171 / +2.549446 | 6/6 | The same semantic precision defect persists late |

The per-receiver maximum-risk choices are not sufficient.  F1's maximum-risk
label `[25,18]` is shared by all receivers but makes RMSE worse at three of
six nodes.  Conversely, the low-risk `[7,7]` action is positive at all six.
F6 at `t=78` also has only three-of-six agreement on the receiver-local top
action, even though a formation-common `[7,5]` action is positive everywhere.
This supports formation-level minimum/quantile aggregation followed by an
explicit mode decision, rather than receiver voting or one global risk rule.

The old V187 F5 action `[31,24]` must not be copied into the new rollout.  In
the corrected F2+F3 state the observable common action is `[19,16]`, and its
safe timing is `t=79`, not `t=78`.

Receiver support exposes a hard operator boundary.  F1 `[7,7]` remains present
at all six receivers with existence `0.0279--0.1869`, so residual KLA is
defined.  F5 `[19,16]` is absent (`r=0`) at every receiver while source 22 has
existence `0.999966`, evidence quality `0.998604`, and observation opportunity
`0.777504`.  A KLA update cannot resurrect zero support and the current KLA
implementation correctly rejects it.  F5 must therefore use a protected
complete-label insertion/replacement.  This is a sparse set/cardinality
restore action, not an observation-handover KLA action.

## Next recursive decision

Two paired H=8 teachers are authorized.  Both use the already verified F2 to
F6 prefix, retain the minimal F3 schedule, and spend the freed final-page slot
on F5:

1. F2 `[13,12] <- 19` at `t=72`;
2. F6 `[7,5] <- 19` at `t=73`;
3. F3 `[19,13] <- 23` at `t=76/78`;
4. F5 `[19,16] <- 17` at `t=79`, using hard support restoration rather than
   residual KLA.  Source 17 is the opened teacher choice because its
   formation-wide minimum RMSE margin and peer consensus exceed source 22;
   source selection remains a learned within-mode decision at runtime.

The first paired arm adds only F1 `[7,7] <- 13` at `t=77` while keeping the
other supported-label actions on residual KLA.  It determines whether the
MAP-sensitive F1 action belongs in the complete mode set.  The subsequent
mixed-operator teacher adds the F5 hard restore after the per-page update
schedule is implemented.  These are operator/mode ablations, not a parameter
sweep.

## Evidence boundary

All results are opened X36 seed-211 development evidence.  The no-repeat
ablation is recursive and fully charged but teacher-routed with ideal
delivery.  Tail localization uses hard replacement only for immediate causal
attribution; it does not prove that residual KLA has the same effect.  The
paired recursive teachers must therefore confirm the complete sequence before
any online selector is trained or any main-document best row is replaced.
