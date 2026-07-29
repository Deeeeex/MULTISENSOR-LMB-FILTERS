# Label-set topology-only M24 preflight

## Scope

This is a training-split preflight on registered M24 seed 11 over the
continuation window \(t=75{:}83\). The message-passing initialization was
trained using the six registered M24 training seeds, so these results are not
development, held-out, or cross-scale evidence.

The raw single-arm report says `Evidence split: development` because the
screen runner did not yet expose a `training` metadata value at generation
commit `eb61899c47c6770d06e99f6642093855c4610ff6`. The seed, continuation
cache, model, random-number offset, and numerical result are training-only.
The runner now accepts and validates an explicit `training` split.

## Question

The initial message-passing arm learned both the residual route and a residual
fusion weight from \([0.05, 0.10, 0.20, 0.25]\). It selected 0.25 at every
focus step. That arm improved mean tracking error but regressed both the
worst-node error and network consensus. This preflight asks whether the useful
signal is the learned topology itself by fixing the residual fusion weight at
the registered Adaptive-KLA-compatible value 0.05.

## Common evidence

- Preset: `m24-hard`
- Seed: `11` (registered training seed)
- Continuation window: `75:83`
- Continuation cache:
  `RUN/GA/dynamic_topology/cache/m24_hard_seed11_n1_sig75.mat`
- Filter RNG offset: `100000`
- Dominant backbone weight: `0.70`
- Fixed residual weight: `0.05`
- Directed messages per step: `40`
- Cross-formation messages per step: `4`
- Model:
  `RUN/GA/dynamic_topology/models/label_set_message_passing_policy_m24_init_v1.mat`
- Model SHA-256:
  `9d6732320e9f5f846e6dbd46f1e7711da372ffbc95c39a31bbe2174788cc6af4`

The static controls and both learned arms use the same cached local
posteriors, measurements, trajectories, link uniforms, and filter RNG offset.
The static controls are deterministic and are reused from the saved
three-arm run rather than recomputed.

## Result

| Arm | Residual weight | Mean E-OSPA | Worst node | MAP-set consensus | Attempted bytes | Delivered bytes | Policy time (s) | Total time (s) |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|
| Fixed residual cycle, clockwise | 0.05 | 22.415966 | 45.600355 | 26.079525 | 17,679,888 | 16,878,568 | 0.0608 | 95.04 |
| Fixed residual cycle, counter-clockwise | 0.05 | 19.732356 | 48.941010 | 24.800230 | 17,676,096 | 16,938,792 | 0.0584 | 95.80 |
| Message passing, learned weight grid | 0.25 | 18.661955 | 59.101854 | 26.079991 | 17,741,736 | 17,038,592 | 0.1158 | 446.61 |
| **Message passing, topology-only** | **0.05** | **17.585406** | **47.365786** | **23.706832** | **17,861,856** | **17,103,816** | **0.1005** | **391.85** |

Relative to the mean-tracking-best static control (counter-clockwise), the
topology-only arm provides:

| Readout | Relative change |
|:--|--:|
| Mean E-OSPA | **10.880% better** |
| Worst-node E-OSPA | **3.219% better** |
| MAP-set consensus | **4.409% better** |
| Attempted bytes | 1.051% more |
| Delivered bytes | 0.974% more |
| Total runtime | 4.090x |

The topology-only arm does not dominate every specialized static readout: its
worst-node E-OSPA is 3.872% above the clockwise control, whose network-mean
tracking error is 27.47% above the topology-only arm. This distinction must
remain visible in later tail-safety claims.

## Safety and attribution

- Truth-use fraction: `0`
- Repair rate: `0`
- Payload-emergency rate: `0`
- Topology-infeasible rate: `0`
- Selected rolling-B3 sensor/formation strong fraction: `1 / 1`
- Delivered rolling-B3 sensor/formation strong fraction: `1 / 1`
- Cross-formation messages per step: `4`
- Within-window route-change fraction: `0.21875`
- Distinct directed route maps: `8`
- Receiver coverage: `1`
- Posterior-content use fraction: `1`
- Current-link-reliability use fraction: `1`

The 0.1005 seconds of policy time rules out GNN inference as the direct cause
of the 391.85-second total runtime. The changing sender sequence alters the
downstream mixture posterior and therefore the cost of subsequent filtering.
Runtime or posterior-complexity protection is consequently a real method
constraint, not merely an implementation optimization.

## Decision

1. Reject joint route-and-weight selection for the current model. Its
   always-0.25 solution is confounded with Adaptive-KLA's weight allocation
   role and fails the observed tail/consensus check.
2. Freeze the candidate as **message-passing edge scoring plus safe
   message-count-preserving topology projection at residual weight 0.05**.
3. Authorize evaluation on the remaining registered training seeds only.
4. Keep development M24, held-out M24, and X36 sealed until the training
   sweep establishes positive mean gain, communication fairness, structural
   safety, and an acceptable runtime/posterior-complexity profile.

## Artifact hashes

Reused static/adaptive comparison:

- Report:
  `RUN/GA/dynamic_topology/evidence/label_set_policy_m24_v1/closed_loop_init_seed11_t75_t83/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260730_064638.md`
- Report SHA-256:
  `5788186803266bd3d921f25e41967eeb9d67ec35cf4d143966c683da5683fd9e`
- MAT SHA-256:
  `8c7b6e61b5cae0e161f9879fb4c7eed097585903a477db2ebfaae0732b8fc929`
- Log SHA-256:
  `e90381ae672896dacbc4f351007d41a836c9f18cfbb67edc6bdf491af1153e1f`

Topology-only arm:

- Report:
  `RUN/GA/dynamic_topology/evidence/label_set_policy_m24_v1/closed_loop_fixed_e05_seed11_t75_t83/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260730_071528.md`
- Report SHA-256:
  `ef4a40b694d2c95e22bc39dd66e26c34543c900a2fbaa2fb2a82dda7a97d378f`
- MAT SHA-256:
  `c0d67c4f0eac1e81394bb167712c064be7f1687a3766ee6149f24dd1f23dc600`
- Log SHA-256:
  `7fe2bf2d52d8a049db0f5584df181ab9bcce17c81bc9cddaf976c5769fcd7f73`
