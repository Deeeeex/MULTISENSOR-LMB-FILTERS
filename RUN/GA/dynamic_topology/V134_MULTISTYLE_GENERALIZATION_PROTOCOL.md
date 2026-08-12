# V134 cross-scale and cross-geometry generalization protocol

## Purpose

The final claim is not that one carefully chosen state admits a useful
posterior-scheduling action. The claim must be that one frozen online policy
produces a visible and stable benefit at both M24 and X36, and that the same
decision rule transfers from the radial-surround development geometry to
qualitatively different motion and sensing geometries.

This protocol is conditional. No learner or validation run is authorized
unless the V134 radial action bank first passes the registered joint M24/X36
headroom gate. A failed joint gate leaves V134 as a repository experiment
record and stops this validation branch.

## What is frozen before outcomes are opened

One scale-normalized policy checkpoint is shared by M24 and X36. Separate
per-scale checkpoints, scene-specific thresholds, preset identifiers, raw
sensor or formation indices, truth, future measurements and future delivery
outcomes are forbidden policy inputs. The checkpoint, feature normalizers,
fallback threshold, source commit and training manifest are frozen before the
first unseen radial result is opened. They cannot change after radial,
convoy, relay or crossing outcomes are observed.

The v5 convoy and relay scene contracts intentionally reject tracking-outcome
access. A separate execution permit is issued only after the radial M24/X36
method gate passes, the policy is frozen, and the geometry gate is verified.
That permit binds the method commit, policy and normalizer digests, this
protocol digest, and the exact preset-seed manifest. It exposes truth only to
the scorer, never to the policy, and it cannot authorize a validation claim
before the complete aggregate gate passes.

The reference is the full-mixture LMB posterior exchanged over the stronger
static carrier direction selected independently for each scale from the V133
radial development trajectories. The direction cannot be reselected for a
scene or seed. Method and reference share truth, measurements, delivery
uniforms, filter randomness, physical reachability and nominal Metropolis KLA
weights. Control synopses are charged to the method's communication total.

## Frozen scene hierarchy

| Stage | Geometry | M24/X36 presets | Frozen seeds | Role |
|---|---|---|---|---|
| 1 | Radial surround | `m24-formation-fov`, `x36-formation-fov` | 2203, 2207, 2221 | Unseen in-distribution confirmation |
| 2a | Parallel convoy | `*-formation-fov-convoy` | 2309, 2341, 2351 | Moving formations and targets with sustained lateral overlap |
| 2b | Linear relay | `*-formation-fov-relay` | 2401, 2411, 2417 | Sequential handover through a long coverage corridor |
| 3 | Orthogonal crossing | `*-formation-fov-crossing` | 2503, 2521 | Non-gating failure-envelope stress test |

Convoy and relay already passed the frozen v5 geometry eligibility gate on a
different twenty-seed manifest. That result establishes only safe,
observable and scale-matched scene geometry; it does not establish tracking
benefit. Crossing remains deliberately outside the formal success average.
Merge-split, curved-corridor and braided-handover remain development scene
assets until they receive their own held-out geometry contract.

## Sequential gates

The radial confirmation is evaluated first. On M24 and X36 separately, the
three unseen seeds must yield at least 5% pooled full-episode E-OSPA gain and
5% focus-window gain, at least two positive seeds, no seed below -1%, no
aggregate regression in the weakest sensor, weakest formation, window
consensus or terminal consensus, and positive attempted-byte saving. Failure
on either scale stops the protocol before non-radial tracking is run.

After radial passage, convoy and relay are opened without retraining. For each
scale separately, their pooled full-episode and focus gains must both reach
5%. Every scale-by-style cell must have at least 2% mean gain, at least two of
three positive seeds and no seed below -2%. The same local non-regression and
positive-byte-saving requirements remain. M24 cannot compensate for X36, and
convoy cannot compensate for relay.

Only the complete radial plus transfer decision can support a main-document
generalization result. Below-gate candidates, individual bright seeds and
stress outcomes remain repository records. If the formal gates pass, crossing
is reported separately as a failure envelope and never included in the mean.

## Attribution to the current method

After the primary radial gate, a selection-free 2-by-2 ablation separates
message admission from the earlier Adaptive-KLA weight-allocation mechanism:

1. static full posterior with fixed KLA weights;
2. static full posterior with Adaptive-KLA weights;
3. binary admission with fixed KLA weights;
4. binary admission with Adaptive-KLA weights.

These arms are not used to change the policy. They test whether the new gain
comes from deciding when complete posteriors enter fusion, rather than from
reusing adaptive fusion-weight allocation.
