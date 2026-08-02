# V31 safe reconnect-pulse probe

## Question

V30 keeps formations 2, 3, and 4 isolated at t=72 and t=73, then returns to the registered reference route at t=74 because another suspension would violate rolling-B3 connectivity. This improves the H=3 tracking window by 4.341% and window consensus by 15.606%, but terminal consensus remains 1.595% worse than reference.

V31 tests one narrow source-only hypothesis: can the forced t=74 reconnect repay that residual consensus deficit by temporarily increasing cross-formation trust on the unchanged reference graph?

## Candidate family

For each formation, the controller raises its registered cross-edge weight from 0.05 to one of 0.075, 0.10, or 0.15 and removes the same amount from receiver self weight. The selected topology and directed message count therefore remain exactly equal to reference. The probe evaluates the reference, 12 single-formation pulses, and only then an exact joint combination of eligible singles.

A pulse is eligible only when it:

- reduces exact one-round expected posterior-summary disagreement by at least 0.25%;
- has reference-relative existence-retention risk at most 0.01;
- lowers no formation's mean expected cardinality by more than 0.05;
- preserves at least 80% of every reference-supported label's existence probability;
- causes no label to cross from at least 0.5 existence to below 0.5; and
- preserves physical feasibility, row-stochastic weights, reference message count, and rolling-B3 connectivity.

Reference is the mandatory fallback. The probe reads the causal pre-fusion posterior at t=74, current link reliability, and the selected t=72--73 topology history. It does not read target truth or any later outcome.

## Frozen exploratory finding

Only formation 1 at weights 0.075 and 0.10 is label-safe, and their predicted disagreement improvements are only 0.042% and 0.110%. Every pulse on formations 2, 3, or 4 violates the label-retention projection even when it improves expected disagreement. No pulse reaches both the safety and 0.25% improvement gates, so the controller falls back to reference.

The official clean preflight must reproduce this mask exactly and stop before any tracking rerun. The result means that increasing trust on the same cross inputs that caused the original existence suppression is not an admissible recovery mechanism. It does not rule out reconnecting through a different, posterior-compatible source under the same hard projection.
