# V116 finding: truth-aligned labels do not repair the F6 boundary

## Result

V116 keeps the V113 clockwise carrier and F2--F5 abstention schedule fixed,
then admits the truth-ranked top 5, 10 or 15 complete sender-27 labels on
`27 -> 32` during pages 5--8.  None passes the X36 H=8 gate.

| Boundary action | Mean E-OSPA | Gain vs CW | vs V113 full | vs V114 empty | Mature min. | Min. formation | F6 peers | Byte saving |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|
| V113 full boundary | 78.479689 | +4.063% | -- | -0.205% | +2.588% | -1.495% | -7.206% | +3.419% |
| V114 empty boundary | 78.319230 | +4.259% | +0.205% | -- | +2.960% | -0.311% | -5.522% | +4.244% |
| V116 truth top-5 | 78.401889 | +4.158% | +0.099% | -0.106% | +2.769% | -0.921% | -5.594% | +4.030% |
| V116 truth top-10 | 78.462017 | +4.085% | +0.023% | -0.182% | +2.769% | -1.365% | -7.240% | +3.802% |
| V116 truth top-15 | 78.442341 | +4.109% | +0.048% | -0.157% | +2.960% | -1.219% | -7.209% | +3.570% |

The selected top-5 arm sends exactly five labels on every boundary page.  It
improves worst-sensor error by `8.965%`, window consensus by `12.135%`, and
terminal consensus by `17.974%`, while passing rolling B3.  Its failure is
therefore not caused by connectivity, consensus, communication, or an
isolated worst sensor.  The binding failures are the sub-five-percent mean
and mature gains, the `-0.921%` minimum formation, and especially the
`-5.594%` F6 non-gateway terminal result.

## Interpretation

Truth alignment removes the main ambiguity left by V115.  The candidate
labels are not selected by a weak observable proxy: they are the sender's
best currently truth-aligned tracks, frozen before the H=8 outcomes are
opened.  Even this privileged ordering cannot beat the empty F6 boundary.
Increasing the quota mostly moves the result back toward the harmful V113
full-boundary endpoint.  The top-15 non-monotonicity is too small to create a
safe intermediate point.

This closes current-state truth-alignment ranking as a useful upper bound for
label-wise control on `27 -> 32`.  It does not mathematically enumerate every
possible label subset, but it is sufficient to reject more quota or threshold
sweeps and to reject training a temporal GNN on this label-action family.

## Next decision

The next bounded action-space screen must change the physical entry while
preserving the F5-to-F6 formation edge, message count, fusion-weight multiset,
physicality and rolling B3.  In the captured V113 trajectory, sender 27 is
already the best F5 source over pages 5--8 (`69.029` mean E-OSPA), whereas
receiver 32 is the worst F6 node (`85.380`).  V117 should therefore keep
sender 27 and replace only the F6 gateway with a small frozen set of
physically reachable receivers, restoring the displaced internal residual
edge each time.  Only a gateway arm that beats clockwise full by at least
five percent with nonnegative formation and F6-peer tails authorizes a
causal gateway-value model.

V116 is privileged opened-development evidence at X36 seed 211, t=72, H=8.
It is not deployable, validation, or generalization evidence.
