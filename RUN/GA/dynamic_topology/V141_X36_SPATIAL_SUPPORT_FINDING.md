# V141 X36 spatial-support finding

## Decision

V141 fails the frozen X36 mechanism gate and is rejected as a candidate
method.  The experiment establishes a narrower but decisive conclusion:
correcting the output cardinality alone is insufficient.  A useful rescue
message must also recover spatially supported label states or mixture modes.

This result is repository-only.  It must not be promoted to the main Lark
progress document, and it does not support a communication-method claim.

## Frozen result

- Source commit: `bf4171734226a8ed5809d290dc20de56c2dd8089`.
- Preset / seed / anchor / action: `x36-formation-fov` / `1601` / `80` / `51`.
- Intervention E-OSPA gain: `+4.901%`.
- Full-window / mature-window gain: `+0.453% / +0.000%`.
- Minimum sensor / formation gain: `-0.161% / +0.234%`.
- Exact reentry match: `100.000%`.
- Charged attempted-byte delta: `+89.403%`.
- Registered gate: failed.

The aggregate result and all saved sensor-time E-OSPA values reproduce V139
to numerical precision.  Routes, nominal weights, attempted and delivered
edges, abstention masks, and attempted, delivered, and auxiliary bytes are
also identical.  V141 therefore executed as designed; its lack of gain is not
caused by a pairing or accounting mismatch.

## Intervention-level evidence

V141 requested seven one-sided undercount augmentations and appended seven
labels, with no insufficient-hybrid fallback.  One augmentation was later
overwritten by the inherited predictive fallback.  The other six changed the
final output cardinality from 18 to 19:

| Sensor | Page | Local | Hybrid | Relay | Readout | Final | Added |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 4 | 4 | 18 | 18 | 19 | 18 | 19 | 1 |
| 5 | 4 | 18 | 18 | 19 | 18 | 19 | 1 |
| 6 | 4 | 18 | 18 | 19 | 18 | 19 | 1 |
| 3 | 5 | 18 | 18 | 19 | 18 | 19 | 1 |
| 4 | 5 | 18 | 18 | 19 | 18 | 19 | 1 |
| 5 | 5 | 18 | 18 | 19 | 18 | 19 | 1 |

Despite those effective cardinality changes, the six E-OSPA values are
unchanged (the largest observed difference is floating-point noise of
`1.42e-14`):

| Sensor | Page | V139 E-OSPA | V141 E-OSPA | Difference |
|---:|---:|---:|---:|---:|
| 4 | 4 | 75.861294955934 | 75.861294955934 | 0 |
| 5 | 4 | 75.769984678900 | 75.769984678900 | 0 |
| 6 | 4 | 75.862206568849 | 75.862206568849 | 0 |
| 3 | 5 | 75.732534120092 | 75.732534120092 | `1.42e-14` |
| 4 | 5 | 75.722642623304 | 75.722642623304 | 0 |
| 5 | 5 | 75.817315664133 | 75.817315664133 | 0 |

## Why the corrected cardinality gives zero gain

For these cells the truth cardinality is 24 and the OSPA cutoff is 150.  Moving
from 18 to 19 estimates reduces the squared cardinality term by
`150^2 / 24 = 937.5`.  However, the appended hybrid conditional-MAP state has
no useful spatial support for the missing target and is assigned at the same
cutoff.  The squared localization term rises by exactly 937.5, cancelling the
cardinality reduction.  The six localization components change as follows:

| Sensor | Page | Before | After |
|---:|---:|---:|---:|
| 4 | 4 | 11.3990 | 32.6716 |
| 5 | 4 | 10.7745 | 32.4591 |
| 6 | 4 | 11.4050 | 32.6738 |
| 3 | 5 | 10.5079 | 32.3715 |
| 4 | 5 | 10.4364 | 32.3484 |
| 5 | 5 | 11.1025 | 32.5694 |

The earlier cardinality-only diagnosis was therefore incomplete.  It located
where the aggregate loss appears, but it did not identify the missing
information needed to repair the estimate.

## Method consequence

Do not continue threshold or cardinality-rule sweeps.  The next bounded
mechanism test should preserve the hybrid output and append the relay-only
conditional-MAP label *with its relay-supported spatial density or component*.
This directly tests whether a sparse mixture-aware Bernoulli correction can
recover the missing target support.  Exact relay information remains a
privileged mechanism teacher; any successful effect must subsequently be
distilled into observable edge-label decisions and charged as a single-payload
communication method before it is eligible for the main document.

The intended deployable direction remains a learned edge-label value model
followed by a deterministic safety projection that preserves the effective
KLA graph and byte budget.  V141 sharpens the teacher target: it must predict
rare, spatially useful label-state corrections, not merely a desired output
cardinality.
