# V138 post-fusion label readout

V137 scores W and R before the current fusion.  Formation-2 harm first appears
only after same-formation W messages are fused, so that decision is causally
too early.  Replacing the chosen state also lets one conservative fallback
erase later useful W propagation.

V138 keeps the V136 hidden-state dynamics unchanged.  W continues to
propagate inside a protected formation, R continues as the exact reference
relay, and ordinary whole-formation reentry still replaces W with R.  The only
new operation is the current protected node's extracted output.

For every label with current positive support or credible negative local
evidence, V138 compares the two already available post-fusion candidates by

```text
estimated D_KL(local updated Bernoulli-GM || W-fused Bernoulli-GM)
versus
estimated D_KL(local updated Bernoulli-GM || R-fused Bernoulli-GM).
```

The deterministic Gaussian-mixture cubature approximation already used by
the repository supplies these two divergences.  The label closer to the
receiver's current local update is used for the
current reported estimate; exact ties retain W.  Labels without admissible
current local evidence retain W.  The choice mutates neither hidden state and
uses neither target truth nor future measurements.  Thresholds governing the
existing local-evidence classifier are frozen before outcomes open.

V138 remains a charged dual-payload mechanism upper bound.  It must preserve
at least 5% M24 intervention gain, make every sensor and formation
nonnegative, and preserve exact reentry.  M24 failure closes the branch before
X36.  A pass authorizes the paired X36 screen but does not yet justify a
communication-saving claim.
