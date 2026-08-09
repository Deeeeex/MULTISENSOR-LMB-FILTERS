# V54 control-synopsis byte feasibility

## Decision

The fixed-shape online synopsis is small enough to retain for the first V54
tracking implementation. The next method decision should focus on where the
synopsis is routed and which selected GM labels consume the remaining byte
budget, rather than compressing the synopsis before any tracking evidence.

## Evidence and accounting

The saved X36 convoy V46 reference run (seed 1009) attempted 7,200 posterior
messages and 233,938,560 payload bytes, or 32,491.47 bytes per message on
average. Its saved result explicitly reports that separate control-plane bytes
were not included.

`buildLmbLabelControlSynopsis` uses four header scalars and 24 scalars per
active four-dimensional label. Its estimated wire payload is therefore

`B_synopsis(L) = 8 * (4 + 24 L) = 32 + 192 L` bytes.

| Active labels in one message | Synopsis bytes | Fraction of mean V46 full message | Bytes left under the same mean-message budget |
|--:|--:|--:|--:|
| 24 | 4,640 | 14.28% | 27,851 |
| 48 | 9,248 | 28.46% | 23,243 |
| 96 | 18,464 | 56.83% | 14,027 |
| 169 | 32,480 | 99.96% | 11 |

The break-even point is 169 active labels; 170 labels would exceed the mean
V46 full-message size. The X36 convoy geometry contains 24 target labels, but
that is not a claim that the tracker always has exactly 24 active labels.
False or stale tracks can increase the online count, so the first V54 smoke
must record the actual synopsis label-count distribution.

## Boundary of the conclusion

This is a deterministic byte-feasibility calculation, not tracking evidence
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
