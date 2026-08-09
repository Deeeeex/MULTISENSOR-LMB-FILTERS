# V54 X36 oracle result and method decision

## Result

The frozen `x36-formation-fov-convoy`, seed 1009 run reused the saved paired
V46 delivery and filter seeds.  V54 kept the V46 route and changed only the
cross-residual label payloads plus the receiver existence projection.

| Metric | V46 | V54 | Improvement |
|:--|--:|--:|--:|
| Full-horizon position E-OSPA | 126.370 | 128.785 | -1.91% |
| Focus-window position E-OSPA | 123.494 | 126.760 | -2.64% |
| Mean absolute cardinality error | 14.534 | 15.438 | -6.22% |
| Total attempted bytes | 233,938,560 | 139,786,200 | +40.25% |
| Selective-path bytes | 8,206,352 full | 4,024,472 synopsis plus GM | +50.96% |

V54 produced 6,356 existence clamps, zero unresolved retention violations,
and no receiver-only fallback.  It improved inter-formation disagreement by
3.73%, but this did not translate into better tracking or cardinality.

## Interpretation

This result rejects the concrete V54 teacher and projection combination.  It
does not reject all label-selective communication: the byte result shows that
the residual cross-formation payload contains substantial redundancy.

Two design choices are now suspect.

1. The V54 reference admits labels according to current evidence type.  In
   this run it saw 1,154 positive-support inputs but 6,001
   unsupported-absence inputs.  An unsupported current observation is not the
   same as an invalid predicted track, so deleting these labels can create a
   systematic positive-evidence bias.
2. The existence floor was enforced 6,356 times.  It therefore acted as a
   persistent estimator modification rather than a rare safety fallback and
   is a plausible cause of the worsened cardinality error.

The consensus improvement is not sufficient evidence of tracking benefit.
It instead demonstrates that stronger agreement can make the network agree
on a worse posterior.

## Next method gate

No GNN is trained on V54 targets.  V55 uses the actual full V46 label inputs,
including the fixed incoming fusion context, as its rate-distortion reference.
The next paired experiment separates:

| Arm | Cross-residual payload | Existence projection | Purpose |
|:--|:--|:--|:--|
| projection-only | full V46 | clamp only | Test whether the floor creates cardinality bias |
| selection-only | context-aware selective | off | Test whether full-reference compression preserves tracking |
| combined | context-aware selective | clamp only | Test whether protection adds value after compression |

The legacy V54 oracle remains unchanged for reproducibility.  In the new
combined arm, already received inputs are not greedily removed; the optional
existence correction is applied directly to the final fused Bernoulli label.

Development evidence only.  The paired report is
`evidence/formation_b4_v54_oracle_tracking_development/FORMATION_B4_V54_ORACLE_TRACKING_20260809_121514.md`.
