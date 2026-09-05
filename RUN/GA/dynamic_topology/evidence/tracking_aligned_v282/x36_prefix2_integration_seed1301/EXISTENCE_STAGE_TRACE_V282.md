# V282 early existence-stage trace

x36-formation-fov-temporal-coupled-formation-braid, seed 1301, steps 1--2 of the original 160-step scene. Runtime 21.6 s. Source `fbf17cdd98439080b0befb2b5246f459aa8d074e`.

Unchanged-reference prefix correspondence: `1`; max E-OSPA difference 0, finite RMSE difference 0, finite-mask match `1`. This is not full-state equivalence or a new policy gain.

| Time | Predicted mass | Local mass | Pre-spatial pooled mass | Fused mass | Local MAP count | Output MAP count |
|--:|--:|--:|--:|--:|--:|--:|
| 1 | 1.440000 | 3.260977 | 3.228033 | 3.093662 | 3.056 | 2.778 |
| 2 | 3.062726 | 3.874945 | 3.946089 | 3.794163 | 3.556 | 3.611 |

Zero expected-pD label stages: 1488; max absolute local existence change there: 6.93889e-18. pD is evaluated at predicted mixture-component means, matching the existing local-update approximation.

The columns are averages over receivers, not true-target recall. The predicted state precedes the current measurement update; local precedes topology and fusion. The pre-spatial column applies the logistic function to each actual weighted input log odds; the fused column includes the spatial overlap term. Pooling can add neighbor-only labels, so its difference from local mass is not a per-target loss. Output uses MAP cardinality, not an r>0.5 threshold; retained unselected labels remain in the filter. Only offline metrics use truth. This opened-seed prefix cannot establish full-episode causality, multiseed performance or a deployable improvement.
