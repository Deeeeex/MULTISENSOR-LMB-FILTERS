# V250 causal sensor-gateway embedding preflight

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`
- Source commit: `9a9a7bd1d231901fe73d3b9d486fa5476ae5d899`
- Scene / structural gate: `1 / 1`
- N / F / exact messages: `24 / 4 / 30`
- Candidate counts: `[21 21 21]`
- Raw global assignment counts: `[1511654400 1511654400 1428840000]`
- Minimum receiver coverage: `[2 2 2]`
- H=3 tracking oracle authorized: `1`
- Candidate construction truth / posterior / future physical use: `0 / 0 / 0`

| Anchor | Candidates | Raw assignments | Receiver coverage by formation | V242 reselection |
|--:|--:|--:|:--|:--:|
| 70 | 21 | 1511654400 | `[4 2 6 2]` | 0 |
| 84 | 21 | 1511654400 | `[4 3 5 2]` | 0 |
| 151 | 21 | 1428840000 | `[4 3 2 4]` | 0 |

## Candidate composition

- `t=70`: v242-reference=1; global-rank-profile=2; receiver-local-assignment=12; single-directed-arc=6
- `t=84`: v242-reference=1; global-rank-profile=2; receiver-local-assignment=12; single-directed-arc=6
- `t=151`: v242-reference=1; global-rank-profile=2; receiver-local-assignment=12; single-directed-arc=6

## Method decision

The corrected M24 formation tree is fixed, but its sensor-level gateway embedding has a large executable action space. The bounded causal bank passes the exact-message, physicality, strong-connectivity, KLA-weight and receiver-diversity gates. A paired H=3 tracking oracle is therefore authorized; ridge or GNN training remains unauthorized until that oracle shows repeatable joint value.

## Evidence boundary

V250 candidate-bank preflight uses only the current physical graph, current link reliability, current positions, immutable physical identities and the causal V242 incumbent. It changes only the sensor-level embedding of cross-formation messages while preserving the V242 formation tree, local cycles, KLA weight scale and exact N+2(F-1) message count. No truth, future page, measurement, tracking outcome or posterior proxy is used. Structural diversity is not a tracking-gain or generalization claim.
