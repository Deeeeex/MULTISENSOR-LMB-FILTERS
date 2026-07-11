# ICASSP 2027 Storyline

## Working title

Receiver-Induced Moment Exchange for Distributed LMB Fusion

## One-sentence claim

For the authors' custom presence-conditioned geometric-average LMB receiver, sender-side moment projection preserves every receiver-observable field under an explicit numerical and transport contract; in 50 paired trials it reduced attempted application-layer bytes by 58.28%, while 1,119,037 retained post-step label fields matched exactly.

## Core story

The contribution is a receiver-induced interface and executable certificate, not a new KLA algorithm, moment-matching method, or generic compression heuristic. The receiver is

`F_(omega,R) = G_(omega,R) o P`,

and remote messages satisfy

`F_(omega,R)(pi_i, T pi_-i) = F_(omega,R)(pi_i, T P pi_-i)`.

`P` performs weight sanitation, covariance symmetrization, and per-label moment projection; `R` is the deterministic Cholesky regularizer; `T` is the versioned encode/decode content map; and `G_(omega,R)` is the executed presence-conditioned geometric-average update. The frozen regular 4+4 graph uses fixed symmetric degree-based base weights (`gamma_ii=1/3`, `gamma_ij=1/6` for four neighbors), then normalizes them over sources carrying each label. This is not full-source LMB-density KLA with missing labels represented by zero existence.

Admissible messages must yield finite projected fields and finite fusion intermediates, with every call to `R` succeeding. Under matched active labels, delivery masks, effective source weights, `C/P/R/T`, and disabled mode-aware weighting and covariance inflation, both wire paths preserve every input to `K`, `h`, `eta`, `q0`, and `q1`. Moving `P` to the sender therefore removes only structure this receiver cannot observe, while crossing the real serialization boundary.

## Evidence to foreground

- Frozen paired confirmatory run: 50 trials, seeds 82--131, 100 steps, 8 receivers.
- Mean per-trial attempted application-layer byte reduction: 58.277264%; paired percentile-bootstrap 95% interval [57.923222, 58.636095]%; minimum 55.921689%.
- Mean delivered-byte reduction: 58.267212%.
- Attempted totals, full/moment: 1,254,185,200 / 522,888,880 bytes. Delivered totals: 1,004,548,968 / 418,898,448 bytes.
- Retained-field audit: 40,000 post-step sensor-time snapshots and 1,119,037 matched retained label instances; no label mismatch and zero maximum residual in existence, mean, covariance, or tracking metrics.
- Raw pre-retention equality is supplied by Proposition 1; the N50 snapshot audit occurs after the common `r>1e-2` retention step.

## Claims to avoid

- Do not call the custom receiver a standard or full-source KLA-LMB implementation.
- Do not call its degree-based weights standard Metropolis weights.
- Do not claim general Gaussian-mixture KLA density equivalence.
- Do not call the moment message minimal or invoke Fisher--Neyman sufficiency.
- Do not call the N50 fields raw fusion outputs; they are retained post-step fields.
- Do not claim radio, network, latency, energy, entropy-coding, or rate-optimality gains from application-layer bytes.
- Do not claim equivalence under nonfinite intermediates, quantization, covariance inflation, different numerical maps, mixture-aware fusion, or multi-round reuse.
- Do not use light posterior, held-out, effective graph, dynamic-topology diagnostics, or safety language in the main story.

## Short-paper structure

1. Introduction: the receiver determines which message fields matter; moment matching itself is standard.
2. Related work: distinguish density rules, event scheduling, component selection, compact track inputs, and receiver-induced interfaces.
3. Method: define `P`, the custom presence-conditioned receiver, admissible domain, transport identity, codec, and byte boundary.
4. Experiments: frozen two-arm paired protocol, per-seed byte distribution, and retained post-step field audit.
5. Discussion: explain the serialized control, recursive consequence, workload-specific ratio, failure modes, and reusable receiver-contract procedure.
6. Conclusion: exact interface result for one receiver; name the conditions that require richer messages.
