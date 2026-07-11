# ICASSP 2027 Storyline

## Working title

Receiver-Induced Moment Exchange for Distributed Projected KLA-LMB Tracking

## One-sentence claim

For the specified projected Gaussian KLA-LMB receiver with one synchronous fusion round per filtering step, sender-side label-wise moment projection preserves every executed receiver output while reducing attempted application-layer bytes by 58.28% over 50 paired confirmatory trials.

## Core story

The contribution is a receiver-induced interface and executable certificate, not a new KLA algorithm or generic compression heuristic. The receiver studied here is

`F_(omega,R) = G_(omega,R) o P`, with the wire claim
`F_(omega,R)(T pi) = F_(omega,R)(T P pi)`.

where `P` applies weight sanitation, covariance symmetrization, and label-wise moment projection; `R` is the deterministic Cholesky regularizer; `T` is the versioned encode/decode content map; and `G_(omega,R)` is the executed projected Gaussian fusion. On admissible messages, `P o T = P` and `T o P = P`. The confirmatory path applies KLA over the participating-source subset for each label and uses the same presence-normalized Metropolis weights for Gaussian and Bernoulli terms; it is not full-source LMB-density KLA with missing labels represented by zero existence. Under matched active labels, delivery masks, `C/P/R/T`, and disabled covariance inflation, both wire paths preserve every input to `K`, `h`, `eta`, `q0`, and `q1`. Moving `P` to the sender therefore removes only structure this receiver cannot observe while testing the real serialization boundary.

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
