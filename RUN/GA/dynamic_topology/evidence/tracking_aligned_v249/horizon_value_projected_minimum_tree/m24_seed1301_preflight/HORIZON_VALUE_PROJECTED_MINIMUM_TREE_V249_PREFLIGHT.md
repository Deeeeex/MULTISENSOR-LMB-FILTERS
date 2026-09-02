# V249 H=3 minimum-tree oracle preflight

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`
- Source commit: `19f5eb59ac6513af85206f316d12e7653799a30a`
- Scene / structural gate: `1 / 1`
- N / F / exact messages: `24 / 4 / 30`
- Alternative trees by anchor: `[0 0 0]`
- Nontrivial action space / H=3 authorized: `0 / 0`
- Candidate construction truth / future physical use: `0 / 0`

| Anchor | Feasible trees | V242 index | V242 tree | Reselection |
|--:|--:|--:|:--|:--:|
| 70 | 1 | 1 | `700101-700202,700101-700303,700303-700404` | 0 |
| 84 | 1 | 1 | `700101-700202,700101-700303,700303-700404` | 0 |
| 151 | 1 | 1 | `700101-700202,700101-700404,700303-700404` | 0 |

## Frozen candidate banks

**t=70.** `T01=700101-700202,700101-700303,700303-700404 (V242)`

**t=84.** `T01=700101-700202,700101-700303,700303-700404 (V242)`

**t=151.** `T01=700101-700202,700101-700404,700303-700404 (V242)`


## Method decision

Each registered physical formation graph contains exactly F-1 edges and therefore has one spanning tree. Formation-tree selection has no action at the task-coupled cut windows, so the H=3 tracking oracle and GNN training are stopped before execution. The next action space should keep this formation skeleton and select its sensor-level gateway embedding under the same message budget.

## Evidence boundary

V249 is an opened-seed structural action-space screen. Candidate trees use only the current physical graph and the causal V242 incumbent; no posterior, truth, future physical page, measurement or tracking outcome is scored. The screen proves that the registered M24 cut windows have no alternative formation tree and therefore stops the planned H=3 oracle and GNN before execution. It is a method-design falsification, not a tracking or generalization claim.
