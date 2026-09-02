# V245 information-coupled formation-braid finding

## Decisive result

The completed M24 seed-1301 comparison does not support the minimum backbone
as a balanced solution on the information-coupled scene.

| Arm | E-OSPA | RMSE | Focus consistency | Attempted bytes | Messages / step |
|:--|--:|--:|--:|--:|--:|
| Fixed formation tree | 126.724 | 9.052 | 135.624 | 34,363,568 | 46--48 |
| Full causal repair | 125.394 | 8.947 | 133.109 | 38,421,672 | 48 |
| Minimum causal backbone | 125.991 | 9.579 | 132.815 | 26,465,520 | 30 |

Relative to the fixed tree, full causal repair improves E-OSPA, RMSE, and
focus consistency by 1.050%, 1.161%, and 1.854%, but increases attempted
bytes by 11.809%.  The minimum backbone improves E-OSPA and focus consistency
by 0.578% and 2.071% and saves 22.984% attempted bytes, but worsens RMSE by
5.826%.  Its weakest-formation E-OSPA and RMSE also worsen by 0.191% and
9.886%.  The balanced-direction and paper gates therefore both fail.

## What changed relative to the original formation-braid

On the original formation-braid scene, V242 was the current balanced M24
record: E-OSPA, RMSE, consistency, and attempted bytes improved by 0.479%,
7.513%, 2.307%, and 14.373% relative to its matched fixed tree.  The coupled
scene retains E-OSPA, consistency, and communication gains but reverses the
RMSE direction.  Absolute metric values are not ranked across the two scenes;
the relevant evidence is the within-scene paired direction.

The V244 gate only required a target source--destination pair to cross each
initial-tree cut that later fails.  V245 shows that this topological condition
is insufficient to create a useful posterior dependency.  Even the full
causal repair arm has smaller tracking and consistency gains than on the
original scene.  A stronger scene diagnostic would need to align the failure
interval with formation-specific visibility, posterior complementarity, and
the time required for information to propagate, rather than count route pairs
alone.

## Method decision

The 30-message backbone leaves 22.984% byte headroom relative to the fixed
tree, while the 48-message causal route shows that extra inputs can recover
position precision.  The next method should therefore retain V242 as the
guaranteed strongly connected base and restore only ordinary residual inputs
whose posterior value justifies their bytes.  This is exactly the question
registered by V246.

V246 is promoted only as the next development experiment, not as a validated
method.  Its immediate success condition is to close the RMSE loss relative
to the fixed tree while keeping E-OSPA and consistency gains and positive
communication saving.  If its exact one-step guard selects almost no
residuals or fails to improve V242, the next change should be a causal
finite-horizon value ranker; lowering the safety threshold would not address
the known delayed-value failure mode.

## Evidence boundary

This is one opened M24 seed on an exploratory scene.  It supports a method
choice and rejects a scene-design assumption.  It does not establish
held-out, X36, cross-scene, or paper-level benefit.
