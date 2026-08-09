# V54 control-synopsis byte feasibility

## Decision

The original double-precision, full-covariance synopsis is rejected. The V54
runtime uses a compact typed synopsis and charges it on the same V46 message
opportunities before allocating bytes to selected GM labels.

## Evidence and accounting

The saved X36 convoy V46 reference run (seed 1009) attempted 7,200 posterior
messages and 233,938,560 payload bytes, or 32,491.47 bytes per message on
average. Its saved result explicitly reports that separate control-plane bytes
were not included.

For a four-dimensional label, `buildLmbLabelControlSynopsis` now uses a
16-byte header and a padded 64-byte record per active label:

`B_synopsis(L) = 16 + 64 L` bytes.

Continuous features use float32; label IDs, component count and evidence code
use compact integer fields. The feature set keeps position/velocity moments,
position covariance, velocity variance, mixture complexity and current
observation evidence, but no GM components.

| Active labels in one message | Synopsis bytes | Fraction of mean V46 full message | Bytes left under the same mean-message budget |
|--:|--:|--:|--:|
| 4 | 272 | 0.84% | 32,219 |
| 24 | 1,552 | 4.78% | 30,939 |
| 48 | 3,088 | 9.50% | 29,403 |
| 96 | 6,160 | 18.96% | 26,331 |

The mean-message comparison is only descriptive. The decisive correction came
from an early unimodal runtime prefix: the original 192-byte label synopsis
was the same size as a one-component full-GM label and consumed 9,600 of a
10,176-byte budget. The compact synopsis consumed 3,264 bytes on the same 12
edge-times. A later 24-step probe used 19,008 synopsis bytes plus 88,544
selected-GM bytes under a 177,792-byte paired full-message budget, a 39.5%
saving on the selective path. See `V54_RUNTIME_INTEGRATION_FINDING.md`.

## Boundary of the conclusion

These are byte-feasibility and short mechanism results, not tracking evidence
and not a total-network communication claim. It does not yet decide whether a
synopsis is sent only on the V46 dominant backbone, on every physically
available residual edge, or through a cached multi-hop control plane. Sending
synopses on more edges than V46 sends posterior messages can erase the
per-message saving. The V54 runtime ledger must therefore report separately:

- attempted and delivered synopsis messages and bytes;
- attempted and delivered selected-GM messages and bytes;
- dominant-backbone and residual-edge contributions;
- total bytes compared with the paired V46 reference.

The first implementation should reuse the V46 message opportunity set and
reserve synopsis bytes before applying the exact label-option projection. This
keeps the initial comparison causal and avoids claiming savings from a free or
unbounded control channel.
