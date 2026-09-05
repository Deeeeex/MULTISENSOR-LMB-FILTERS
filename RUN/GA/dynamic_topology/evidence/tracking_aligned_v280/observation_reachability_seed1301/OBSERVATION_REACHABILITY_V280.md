# V280: geometric observation and causal packet reachability

Opened M24/X36 seed 1301 only. No filter is rerun, no scenario parameter changes, and no new policy is selected.

## Sensing opportunity, independent of routing

| Scale | Mean true count | Locally visible targets / sensor-time | Globally visible targets / time | Global blackout / active target-time | Expected detections / active target-time | Probability of no network detection |
|:--|--:|--:|--:|--:|--:|--:|
| N=24 | 16.000 | 4.029 | 15.556 | 2.773% | 4.254 | 3.639% |
| N=36 | 24.000 | 3.877 | 23.344 | 2.734% | 4.058 | 3.936% |

Visibility and detection probability use `evaluateSensorQuality`, the same function as measurement generation. The nominal detection probability/noise standard deviation are modified by range and off-axis angle. No-network-detection probability is the product of per-sensor miss probabilities, conditional on the deterministic target state; it is not the realized detection count from the experiment.

## Ideal geometric-evidence propagation over actual packet opportunities

| Scale | Arm | Age <=0 / <=3 / <=8 / <=16 steps | No prior reachable source | Mean possible target count within 8 steps | Inferred output count |
|:--|:--|:--|--:|--:|:--|
| N=24 | Fixed tree | 31.13% / 52.61% / 66.78% / 72.89% | 3.98% | 10.684 | 4.869--5.722 |
| N=24 | Full causal repair | 32.21% / 59.85% / 87.34% / 95.94% | 3.98% | 13.974 | 5.329--5.329 |
| N=24 | Sparse causal repair | 31.13% / 50.15% / 81.26% / 95.31% | 4.47% | 13.002 | 5.280--5.280 |
| N=36 | Fixed tree | 20.59% / 36.03% / 49.72% / 62.57% | 5.05% | 11.932 | 5.544--8.275 |
| N=36 | Full causal repair | 21.66% / 43.10% / 68.39% / 91.44% | 5.05% | 16.413 | 5.537--5.824 |
| N=36 | Sparse causal repair | 20.83% / 35.69% / 63.18% / 88.52% | 5.59% | 15.164 | 5.403--6.369 |

A source is eligible when a true target has positive detection probability at that sensor. We assume it is detected perfectly and retained indefinitely. At each step, local evidence age increments, currently eligible sources reset to age zero, and one synchronous packet round propagates the youngest source age. Scheduled edges are intersected with the same directed delivery uniforms. A new observation can cross only one edge that step. Age zero includes directly visible targets at self or a sender whose packet arrived.

Coverage percentages use active sensor-target-time triples. Counts average over sensor-time cells. Reachability is an optimistic communication opportunity, not actual label availability, detector recall, posterior quality, or a bound on tracking error. False labels, association, negative evidence, pruning, density overlap and stale motion uncertainty are not modeled.

## Cells where every active target has an age <=8 geometric source path

| Scale | Arm | Sensor-time fraction | True count on these cells | Output-count interval on these cells |
|:--|:--|--:|--:|:--|
| N=24 | Fixed tree | 20.260% | 16.000 | 6.167--6.167 |
| N=24 | Full causal repair | 52.422% | 16.000 | 5.772--5.772 |
| N=24 | Sparse causal repair | 39.766% | 16.000 | 5.634--5.634 |
| N=36 | Fixed tree | 1.562% | 24.000 | 6.556--6.556 |
| N=36 | Full causal repair | 7.205% | 24.000 | 7.692--7.692 |
| N=36 | Sparse causal repair | 4.844% | 24.000 | 6.846--6.846 |

The eight-step slice is descriptive, not a tuned selection gate. The frozen horizons 0, 3, 8 and 16 are all reported. V279 count-sign ambiguity is retained. A reachable true target is not necessarily one of the emitted estimates, so these counts do not measure label recall.

## Reproduction and decision boundary

`octave --no-gui --quiet --eval "addpath(genpath(pwd)); analyzeObservationReachabilityV280();"`

Route message counts are checked against the saved paired metric artifacts. Self-check only. This offline diagnostic can expose a gross sensing or temporal-path bottleneck, but cannot attribute specific missing estimates to transport versus fusion. It does not change the V278 stopping rule or authorize a new parameter sweep.

- `/Users/dex/.config/superpowers/worktrees/MULTISENSOR-LMB-FILTERS/v248-temporal-coupled-routing/RUN/GA/dynamic_topology/evidence/tracking_aligned_v248/m24_temporal_task_coupled_formation_braid_seed1301/minimum_causal_backbone/CAUSAL_MINIMUM_FORMATION_BACKBONE_V242_FULL_EPISODE.mat`
- `/Users/dex/.config/superpowers/worktrees/MULTISENSOR-LMB-FILTERS/v274-x36-minimum-backbone/RUN/GA/dynamic_topology/evidence/tracking_aligned_v274/x36_minimum_backbone_seed1301/minimum_causal_backbone/CAUSAL_MINIMUM_FORMATION_BACKBONE_V242_FULL_EPISODE.mat`
