# V191 X36 H=3 recursive formation-release pilot

- Release page offsets: `[0 1]`
- Requested releases by page: `{[2 5], 3, []}`
- Applied releases by page: `{[2 5], 3, []}`
- Effective withheld formations by page: `{[1 4], [1 4 5], [1 3 4 5]}`
- Release actually applied: `1`

| Metric | Reference | V99 base | V191 release |
|:--|--:|--:|--:|
| Mean E-OSPA | 85.970277 | 83.561598 | 84.791800 |
| E-OSPA gain | 0 | +2.802% | +1.371% |
| Mean RMSE | 57.902417 | 58.288297 | 58.724047 |
| RMSE gain | 0 | -0.666% | -1.419% |
| Consensus gain | 0 | +5.149% | +2.414% |
| Byte saving | 0 | +6.550% | +4.615% |

- Formation E-OSPA gains: `[1.62646 0 0.650331 4.00729 2.1339 0]`
- Formation RMSE gains: `[-0.0689284 0 -24.4237 1.81777 0.547294 0]`

## Incremental effect over V99

- Mean E-OSPA: `-1.472%`
- Mean RMSE: `-0.748%`
- Consensus: `-2.883%`
- Attempted bytes: `+205232 B`

- Development gate passed: `0`

## Evidence boundary

V191 is an opened recursive teacher that removes a named formation from the current V99 withheld-payload set on registered pages.  The ordinary full posterior and its ordinary byte accounting are thereby restored; topology, fusion weights, measurements, links and filter RNG remain paired.  The release identifiers are teacher inputs, so V191 cannot support deployable or cross-scene claims.
