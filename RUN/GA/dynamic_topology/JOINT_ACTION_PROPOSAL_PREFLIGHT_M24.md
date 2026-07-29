# Joint-action proposal preflight: M24

- Generated: 2026-07-29 09:52:43
- Audit contract: `rolling-safe-joint-action-proposal-preflight-v1`
- Protocol: `m24-rolling-safe-joint-action-critic-v1-centralized-metadata-unaccounted`
- Dataset SHA-256: `71be84164cb2d46d8da4d83448e78f983a40b3760712e91078724c2620cc94e0`
- Source state SHA-256: `ca08e2eec2c7a3e63ceca8b3ede292cea9cdb690666f58c64be8ec4df83866c9`
- Generation commit: `211aac475d2cfe3eeba85ae8d71bc8604379a938`
- Blocks: `18`
- Distinct candidates: `341`
- Selectable candidates: `338`
- Reference-only candidates: `3`
- Distinct candidates min/max: `12 / 23`
- Selectable candidates min/max: `11 / 23`
- Repaired reference blocks: `3`
- Minimum distinct-graph feature distance: `0.020888600673`
- Feature collisions at <= 1e-06: `0`
- Maximum feature replay difference: `0`
- Accepted for paired H=3 returns: `1`
- Accepted for critic training: `0`
- Accepted for X36 zero-shot: `0`

## State blocks

| Seed | Time | Distinct | Selectable | Duplicates | Unavailable | Reference repaired | Minimum feature distance |
|--:|--:|--:|--:|--:|--:|--:|--:|
| 7 | 75 | 21 | 21 | 7 | 0 | 0 | 0.09557373 |
| 7 | 76 | 19 | 19 | 6 | 3 | 0 | 0.09602643 |
| 7 | 77 | 17 | 16 | 4 | 7 | 1 | 0.063009699 |
| 11 | 75 | 23 | 23 | 5 | 0 | 0 | 0.31877977 |
| 11 | 76 | 23 | 23 | 5 | 0 | 0 | 0.096826531 |
| 11 | 77 | 16 | 16 | 7 | 5 | 0 | 0.063780054 |
| 17 | 75 | 16 | 16 | 9 | 3 | 0 | 0.22055228 |
| 17 | 76 | 22 | 22 | 6 | 0 | 0 | 0.03029335 |
| 17 | 77 | 12 | 11 | 5 | 11 | 1 | 0.10452443 |
| 19 | 75 | 17 | 17 | 8 | 3 | 0 | 0.1687434 |
| 19 | 76 | 17 | 17 | 8 | 3 | 0 | 0.066189545 |
| 19 | 77 | 20 | 20 | 4 | 4 | 0 | 0.14015527 |
| 23 | 75 | 20 | 20 | 8 | 0 | 0 | 0.054664333 |
| 23 | 76 | 23 | 23 | 5 | 0 | 0 | 0.095289767 |
| 23 | 77 | 18 | 17 | 3 | 7 | 1 | 0.046128331 |
| 29 | 75 | 21 | 21 | 7 | 0 | 0 | 0.020888601 |
| 29 | 76 | 21 | 21 | 5 | 2 | 0 | 0.22670387 |
| 29 | 77 | 15 | 15 | 4 | 9 | 0 | 0.077394612 |

## Decision

PASS: all 18 frozen M24 states retain at least one repair-free selectable action, every complete graph is unique within its state, all 341 feature vectors replay exactly, and no distinct graph pair collides at the frozen threshold. Paired H=3 return generation may proceed.

## Evidence boundary

This preflight verifies construction, safety attestation, exact feature replay and within-state representation separation. It does not evaluate tracking return, top-K coverage, candidate-oracle gain, critic accuracy, X36 support or control-metadata traffic. Three repaired action-24 graphs are retained only as matched references and are not critic-selectable.
