# V111 finding: H=6 safety is transient

## Paired X36 result

| Arm | Mean E-OSPA | Gain vs static | Byte saving |
|:--|--:|--:|--:|
| Static full payload | 84.037151 | -- | -- |
| V111 alternating shield/broadcast | 79.663569 | +5.204% | +5.483% |

V111 preserves at least 5% gain on every page from t74 onward and improves
window/terminal consensus by `10.685% / 20.684%`. It does not satisfy the
formation safety gate: F1 is `-0.8158%`, F6 is `-0.1541%`, and the F6
non-gateway terminal metric is `-2.955%`.

## Research conclusion

The V102 H=6 result was a transient short-window success. Extending the exact
alternating one-step-delayed broadcast/reference-recovery cadence to H=8 does
not prevent the protected posterior from creating downstream debt. The mean
and consensus benefits remain, but the same F1/F6 failure reappears.

Together, V109--V111 establish three boundaries:

1. source abstention is the real aggregate-gain mechanism;
2. perfect local formation classification cannot certify downstream nodes;
3. a fixed periodic broadcast/recovery cadence cannot certify a longer window.

The next method should therefore predict finite-horizon downstream risk on the
time-expanded effective KLA graph. A GNN is justified only for this relational
prediction problem, not as a generic replacement for the hand-designed rule.
Its output should be projected onto a safety set that preserves the static
carrier, rolling B3 reachability, a cap on unresolved influence-cone debt and a
conservative full-payload fallback under uncertainty.

Before training, the next upper-bound experiment should use paired
counterfactual rollouts to label each candidate action by its worst downstream
formation regret over a fixed horizon. This tests whether an observable
time-expanded risk gate has sufficient headroom on X36; M24 and unseen X36
seeds then become the first generalization targets.
