# Privileged joint-action proposal preflight: M24

- Generated: 2026-07-29 12:15:39
- Audit contract: `rolling-safe-privileged-proposal-preflight-m24-v1`
- Protocol: `m24-rolling-safe-joint-action-critic-v1-centralized-metadata-unaccounted`
- Dataset SHA-256: `ee1e684c1c1a6e78d21efed4f3ab0c6b32a1e3222cf9a5dc76bb8983f06ba90e`
- Generation commit: `90b0cce9a948400d0b2fbfc606bc327007f8bbe2`
- Blocks: `18`
- Distinct graphs / targets: `70 / 70`
- Targets per block min/max: `3 / 4`
- Truth-free exact graph matches: `0 / 70`
- Cached diverse targets: `52`
- Minimum distinct-graph feature distance: `0.0272772313523`
- Feature collisions at <= 1e-06: `0`
- Maximum feature replay difference: `0`
- Accepted for proposal distillation: `1`
- Accepted for paired H=3 returns: `1`
- Accepted for critic training: `0`
- Accepted for deployment: `0`
- Accepted for X36 zero-shot: `0`

## State blocks

| Seed | Time | Distinct | Targets | Truth-free exact | Cached diverse | Minimum feature distance |
|--:|--:|--:|--:|--:|--:|--:|
| 7 | 75 | 4 | 4 | 0 | 3 | 0.13185913 |
| 7 | 76 | 4 | 4 | 0 | 3 | 0.20712205 |
| 7 | 77 | 4 | 4 | 0 | 3 | 0.31878157 |
| 11 | 75 | 4 | 4 | 0 | 3 | 0.14396295 |
| 11 | 76 | 4 | 4 | 0 | 3 | 0.16822175 |
| 11 | 77 | 4 | 4 | 0 | 3 | 0.10435698 |
| 17 | 75 | 4 | 4 | 0 | 3 | 0.10400132 |
| 17 | 76 | 4 | 4 | 0 | 3 | 0.049706056 |
| 17 | 77 | 4 | 4 | 0 | 3 | 0.069613435 |
| 19 | 75 | 4 | 4 | 0 | 3 | 0.083581039 |
| 19 | 76 | 4 | 4 | 0 | 3 | 0.098477685 |
| 19 | 77 | 4 | 4 | 0 | 3 | 0.14178565 |
| 23 | 75 | 4 | 4 | 0 | 3 | 0.10182765 |
| 23 | 76 | 4 | 4 | 0 | 3 | 0.13687982 |
| 23 | 77 | 4 | 4 | 0 | 3 | 0.027277231 |
| 29 | 75 | 3 | 3 | 0 | 2 | 0.078715531 |
| 29 | 76 | 4 | 4 | 0 | 3 | 0.094655155 |
| 29 | 77 | 3 | 3 | 0 | 2 | 0.11223114 |

## Decision

PASS: all 18 frozen M24 states contain at least three distinct, repair-free, exact rolling-B=3 privileged targets; every action feature replays exactly and all diverse targets reuse the state-matched teacher score. Paired H=3 return generation may proceed.

## Research finding and evidence boundary

The frozen truth-free proposal bank exactly matches 0 of 70 privileged target graphs. This establishes an action-space coverage gap, not a tracking-return gap. The artifact uses truth offline and is not deployable. This audit does not establish H=3 gain, top-K proposal recall, critic accuracy, X36 transfer or control-metadata traffic cost.
