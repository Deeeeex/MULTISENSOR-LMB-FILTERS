# V292: weighted observation transport, without another tracker run

## Question

Does binary temporal reachability conceal weak weighted access to recent
observations in the already-opened M24/X36 episodes? The decision is whether
transport strength remains a plausible bottleneck, not which weight to use.

## Scope

Reuse V280's geometric visibility, activity and path ages for seed 1301.
Replay the unchanged fixed, full causal and sparse causal policies using the
original scenario generator and directed packet uniforms. The generator is
needed to preserve its RNG sequence; no filter or tracking scorer is run.
Truth is confined to the cached offline observer and never enters a policy.
Report every existing horizon: maximum ages 0, 3, 8 and 16 steps.

For one target, let `o_k` indicate geometric observation opportunity and let
`W_k` be the delivered packet-level row-stochastic matrix, using the existing
missing-packet renormalization. Within a window `max(1,t-h):t`, start at zero:

\[
z_k=W_k\{o_k+(1-o_k)\odot z_{k-1}\}.
\]

This is a backwards-source-path hitting probability: sample a pre-fusion
sender using the receiver's row of W; stop successfully at an observation
opportunity, otherwise continue through that sender's previous state.
One synchronous round follows the current local observation opportunity.
Binary path coverage asks whether any such path exists; the weighted score
averages over paths according to their actual packet-level mixing weights.

It is not actual posterior mass, label recall, detection probability,
mutual information, a tracking-error bound or a new theoretical result.
Empty packets, active-label weights, local likelihoods, overlap, pruning and
association are omitted. Visibility also does not guarantee detection.

## Risk Tier

L2 internal mechanism analysis, self-check only. No new control or deployed
recommendation. Any paper use must retain the packet-level limitations.

## Claims

- C1: V280's binary path indicator and a weighted path score answer different
  questions. Positive score should reproduce binary reachability (E1, E2).
- C2: A weak score would motivate studying useful information transfer, not
  conclude that stronger generic mixing improves tracking (E3).

## Evidence Ledger

- E1: `evidence/tracking_aligned_v280/observation_reachability_seed1301/OBSERVATION_REACHABILITY_V280.md`.
- E2: `RUN/GA/analyzeWeightedObservationTransportV292.m`; reports and source
  CSV are generated under `evidence/tracking_aligned_v292/weighted_observation_transport_seed1301/`.
- E3: `V220_SCALE_AWARE_EDGE_VALUE_ROUTING_DESIGN.md`, V27 stronger-mixing
  controls: old-scene negative evidence, not an exclusion for every new scene
  or estimator. V291 independently tests a different spatial pooling rule.

## Verification Record

Self-check only. Check row sums, probability range and exact correspondence
of positive-score masks to V280 path ages, plus route message counts. These
are focused orientation/timing checks; no broad audit or hash validation.
The two-source recursion directly follows the law of total probability.

## Risk and Escalation

Calling this score an actual fusion coefficient or a detector probability
would misrepresent the approximation. Do not select thresholds, tune routing
or promote tracking gains from these post-hoc values.

## Reproducibility

```sh
/opt/homebrew/bin/octave --no-gui --quiet --eval "addpath(genpath(pwd)); analyzeWeightedObservationTransportV292();"
```

The script saves the realized matrices and the eight-step score to permit
follow-up analysis without regenerating observations. Figures use Python
and the exported CSV only.

## Open Issues

No completed result at registration. Source-hit scores ignore all
label-specific estimator dynamics and cannot resolve their causal role.

## Recommendation

Complete this offline comparison while the frozen V291 result is scored.
Keep unsuccessful fusion controls in experiment records. Use the combined
evidence to narrow the next question before authorizing another filter run.
