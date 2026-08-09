# V54 runtime integration finding

## Decision

The receiver-safe V54 oracle is now connected to the real distributed LMB
filter, but the first tracking experiment must evaluate two coupled effects:

1. label-selective payload exchange on V46 cross-formation residual edges;
2. a constrained Bernoulli projection for receiver labels with current
   positive measurement support.

The short mechanism run is sufficient to start an X36 oracle headroom run. It
is not tracking evidence and does not establish that either effect improves
E-OSPA or cardinality.

## Important correction to the control synopsis

The original synopsis used 24 double-precision scalars per four-dimensional
label, or 192 bytes per label. A one-component full GM label also costs 192
bytes in the repository's payload model. Therefore the earlier comparison to
the *mean* late-horizon V46 message hid an important failure mode: while tracks
are mostly unimodal, that synopsis can consume essentially the whole full
message budget before any selected GM label is sent.

An eight-step M24 convoy run exposed this directly:

| Synopsis | Baseline full bytes | Synopsis bytes | Selected GM bytes | Saving |
|---|---:|---:|---:|---:|
| original double/full-covariance | 10,176 | 9,600 | 0 | 5.7% |
| compact typed/position-covariance | 10,176 | 3,264 | 0 | 67.9% |

The compact synopsis uses a 16-byte header and a padded 64-byte record per
four-dimensional label. Continuous features are represented as float32;
label IDs and discrete codes use compact integer fields. It retains existence,
position and velocity moments, position covariance, velocity variance,
mixture complexity, expected detection opportunity, association support and
predicted existence. It does not carry GM components. Its wire cost is

`B_synopsis(L) = 16 + 64 L` bytes.

Quantization error remains a later calibration obligation for the deployable
GNN. The current offline oracle uses full posteriors to construct targets and
uses the compact synopsis only for honest byte accounting.

## Focused 24-step mechanism result

The final M24 convoy mechanism probe used the V46 route and message
opportunities without changing the dominant backbone or local residual edges.

| Quantity | Result |
|---|---:|
| Selective cross-residual edge-times | 36 |
| Paired V46 full-message budget | 177,792 bytes |
| Compact synopsis bytes | 19,008 bytes |
| Selected GM-label bytes | 88,544 bytes |
| Selective-path byte saving | 39.5% |
| Mean / maximum active labels per synopsis sender | 8.0 / 12 |
| Positive-support label inputs | 35 |
| Credible-negative label inputs | 0 |
| Unsupported-absence label inputs | 198 |
| Ambiguous label inputs | 55 |
| Retention violations before projection | 278 |
| Selective inputs removed | 16 |
| Bernoulli existence clamps | 278 |
| Receiver-only spatial fallbacks | 0 |
| Unresolved retention violations | 0 |

The evidence counts explain why label granularity is materially different
from edge scheduling: most active cross-formation label inputs in this prefix
carry no valid observation basis, while a smaller positive-support subset is
worth transmitting.

## Receiver-safety correction

Two overly strong early rules were rejected during integration:

- high local existence alone is not current measurement support;
- requiring the fused existence to retain 95% of local existence is not
  aligned with Bernoulli KLA geometry and suppresses normal fusion updates.

The installed rule now protects only receiver labels with current positive
association support. It bounds the decrease in Bernoulli log-odds: the fused
existence odds may be at most four times lower than the receiver's supported
local odds. If removing harmful V54-selective inputs is insufficient because
fixed backbone inputs also violate the bound, the receiver retains the fused
spatial density and clamps only its existence probability to the feasible
boundary. This is the closest boundary point to the unconstrained Bernoulli
result for a one-dimensional existence constraint; it does not revert the
label's spatial estimate to the local posterior.

## Risk that the X36 oracle must resolve

The 278 clamps show that the constrained Bernoulli layer is active rather than
a rare emergency fallback. It may prevent the known KLA suppression of direct
positive evidence, but it may also create positive cardinality bias. The X36
oracle gate must therefore report the constrained projection count together
with E-OSPA, cardinality error, focus-window performance and total bytes. No
learned utility model should be trained unless the paired X36 result clears
the frozen headroom gate.
