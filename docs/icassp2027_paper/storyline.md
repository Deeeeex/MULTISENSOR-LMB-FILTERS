# ICASSP 2027 Storyline

## Working title

Fusion-Sufficient Moment Exchange for Distributed Projected KLA-LMB Tracking

## One-sentence claim

For the specified single-round projected Gaussian KLA-LMB receiver, sender-side per-label moment projection commutes with receiver fusion, yielding identical fusion outputs while reducing attempted application-layer bytes by 58.28% over 50 paired confirmatory trials.

## Core story

The contribution is an operator-induced interface, not a new KLA algorithm and not a generic compression heuristic. The receiver studied here is

`F_omega = G_omega o P`,

where `P` performs label-wise moment projection and `G_omega` performs the implemented projected Gaussian KLA-LMB fusion. Under matched source-label presence, spatial/existence weights, delivery masks, and numerical conventions, `P` preserves every input to `K`, `h`, `eta`, `q0`, and `q1`; idempotence then gives `F_omega = F_omega o P`. Moving `P` to the sender therefore removes mixture structure that cannot affect this receiver's output. A shared versioned codec makes the communication comparison real rather than a scalar-count estimate.

## Evidence to foreground

- Frozen paired confirmatory run: 50 trials, seeds 82--131, 100 steps, 8 receivers.
- Mean per-trial attempted application-layer byte reduction: 58.277264%; paired percentile-bootstrap 95% interval [57.923222, 58.636095]%; minimum 55.921689%.
- Mean delivered-byte reduction: 58.267212%.
- Attempted totals, full/moment: 1,254,185,200 / 522,888,880 bytes. Delivered totals: 1,004,548,968 / 418,898,448 bytes.
- Exact audit: 40,000 paired sensor-time snapshot comparisons, 1,119,037 matched label-instance comparisons, no label mismatch, and zero maximum residual in existence, mean, covariance, or tracking metrics.

## Claims to avoid

- Do not claim general Gaussian-mixture KLA density equivalence.
- Do not call the moment message "minimal" or invoke Fisher--Neyman sufficiency.
- Do not claim radio, network, latency, or energy savings from application-layer bytes.
- Do not claim equivalence under quantization, covariance inflation, a different projection convention, mixture-aware fusion, or multi-round reuse.
- Do not use "light posterior," "held-out," "effective graph," dynamic-topology diagnostics, or safety language in the main story.
- Do not claim to be the first state/covariance projection method; the novelty is that the receiver operator induces the message and an exact output-equivalence class.

## Short-paper structure

1. Introduction: the receiver, not only the network policy, determines which message fields matter.
2. Related work: distinguish event scheduling, component selection, generic state/covariance projection, and receiver-induced interfaces.
3. Method: define `P`, the actual projected fusion operator `G_omega`, prove `F_omega = F_omega o P`, then specify the typed codec and byte boundary.
4. Experiments: frozen two-arm paired protocol, per-seed byte distribution, and exact output audit.
5. Discussion: explain the non-circular serialized control, workload-dependent byte ratio, failure modes, and reusable receiver-contract procedure.
6. Conclusion: exact interface result for one receiver; name the conditions that require richer messages.
