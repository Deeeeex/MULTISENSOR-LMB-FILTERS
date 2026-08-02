# Formation reference-recovery mixing v26 audit

## Verdict

The v26 audit rejects the current topology-only safety state as sufficient.
All 256 mode-vector actions satisfy the same coarse formation-level mixing
matrix, short-horizon Dobrushin certificates are vacuous, and two alternative
centered product metrics admit counterexamples.  The current action family
cannot control the mechanism responsible for its consensus debt.

## Provenance and checks

- audit generation commit: `b22a34c0b66d8a21f10582f9fcff160ab4740c0d`;
- source screen commit: `27b2c879c634a4e32705c53cf413caa7df9fb6d1`;
- source screen SHA-256:
  `098c4f374b19b9160b316f091db75c15ccfb529c068daa4b1b14541db47c8a39`;
- source state: `m24-formation-fov / seed 211 / t=72 / H=5`;
- action count: 256; arm count: five;
- maximum fusion-matrix row-sum error: `1.11e-16`;
- analysis steps: `1, 3, 5, 7, 10, 20, 30`;
- steps after H=5 are weight-only fixed-reference extrapolations, not new
  filter outcomes.

## Coarse mixing is action invariant

Every action collapses to the following receiver-formation by sender-formation
matrix:

```text
0.9917  0.0083  0       0
0       0.9917  0.0083  0
0       0       0.9917  0.0083
0.0083  0       0       0.9917
```

The maximum deviation across all 256 actions is `1.11e-16`.  Thus, mode-vector
actions move the gateway role among sensors and alter fine-scale weights, but
the average cross-formation mass remains exactly `0.0083333` for every
receiver formation.  No action can strengthen or weaken coarse
formation-to-formation information flow.

## Short-horizon mixing is non-discriminative

| Arm | H=5 Dobrushin | Centered spectral | Row dispersion | Window consensus | Final-step consensus | First Dobrushin below 1 |
|--:|--:|--:|--:|--:|--:|--:|
| reference | 1.000000 | 1.581765 | 0.560145 | 0 | 0 | 7 |
| candidate 1 | 1.000000 | 1.599928 | 0.565939 | -5.038% | -11.147% | 7 |
| candidate 2 | 1.000000 | 1.572840 | 0.562398 | -3.282% | -9.238% | 6 |
| candidate 3 | 1.000000 | 1.573351 | 0.556211 | -3.810% | -6.589% | 6 |
| candidate 4 | 1.000000 | 1.534495 | 0.538321 | -2.800% | -4.824% | 6 |

The Dobrushin coefficient cannot rank any H=5 arm because it is saturated at
one.  Candidates 2--4 have centered spectral norms no worse than the
reference but all have worse final-step consensus.  Candidates 3--4 remain
counterexamples even under maximum row dispersion.  A lower topology-product
proxy is therefore not sufficient to certify posterior consensus.

## Method implication

The disagreement dynamics contain both mixing and disturbance.  Fusion
weights control how existing disagreement propagates, while heterogeneous
local measurements, label support, and posterior innovations inject new
state-dependent disagreement.  A graph-only score observes the first term but
not the second.

The next method must make two changes together:

1. expose cross-formation edge multiplicity or total fusion mass as a
   scale-aware action, instead of choosing only a gateway identity;
2. predict tracking, tail, consensus, and payload effects from the current
   posterior/innovation state, with physical, payload, rolling-connectivity,
   and minimum-mixing constraints enforced by a hard projection.

This supports an action-space redesign, not immediate learning.  GNN training
remains unauthorized until the redesigned family shows repeated strong-safe
headroom on development states.

## Evidence boundary

This is deterministic posthoc diagnosis of one opened M24 state.  It does not
establish the magnitude of the same mechanism on X36 and cannot support
generalization or performance claims.  Seeds 223/227, X36, and final seeds
remain unopened.
