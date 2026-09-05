# Packet-level KLA weight realization

Source: `eb7bc57f541af00f37636b61a1d2edaa919c51e6`; paired input seed: `1301`.

| Scene | Arm | Messages | Non-double pages: planned / renormalize / self | Packet-strong pages | Maximum surviving-neighbor weight increase |
|:--|:--|--:|:--|:--|--:|
| m24-formation-fov-temporal-coupled-formation-braid | Full causal | 48 | 160 / 160 / 160 | 41 / 160 | 0.116667 |
| m24-formation-fov-temporal-coupled-formation-braid | Minimum backbone | 30 | 17 / 122 / 110 | 41 / 160 | 0.116667 |
| x36-formation-fov-temporal-coupled-formation-braid | Full causal | 72 | 160 / 160 / 160 | 18 / 160 | 0.116667 |
| x36-formation-fov-temporal-coupled-formation-braid | Minimum backbone | 46 | 22 / 143 / 136 | 18 / 160 | 0.116667 |

These are packet-level matrices, assuming a delivered packet supplies an input. They are not realized label-wise fusion matrices: empty payloads, support censoring and label-specific renormalization can further change the active inputs. The exact scenario generator is called to reproduce the registered directed delivery uniforms, but its posterior, tracking output and scoring functions are not run. Generated truth and measurements are not used by this analysis.

V240/V242 inherit missingNeighborWeightMode=renormalize. Planned removal of a local residual edge returns mass to self inside the routing policy, whereas an unexpected missing packet is omitted and remaining weights are renormalized. The self column is a counterfactual on the same packet realization, not a tracking result. Neither handling rule guarantees per-round double stochasticity.
