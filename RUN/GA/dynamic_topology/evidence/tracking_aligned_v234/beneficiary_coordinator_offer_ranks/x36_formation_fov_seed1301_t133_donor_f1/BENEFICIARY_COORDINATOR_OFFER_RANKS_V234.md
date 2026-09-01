# V234 beneficiary-coordinator offer ranks

- State: `x36-formation-fov / seed 1301 / t=133`
- Coordinator rule: `first-registered-beneficiary-receiver`
- Top-3 mode recall: `1 / 2 = 50.0%`
- Top-6 mode recall: `2 / 2 = 100.0%`

| Row | Beneficiary / coordinator | Source / label | Sources | Offers | Mode ranks E/P/D/R | Composite rank | Top-3 / top-6 |
|--:|:--|:--|:--|--:|:--|--:|:--|
| 1 | F5 / S25 | S2 / `[1,4]` | `2` | 16 | `12/12/4/4` | 7 | `0 / 1` |
| 4 | F6 / S31 | S1 / `[25,20]` | `1/3/5` | 71 | `60/2/10/4` | 12 | `1 / 1` |

## Decision

Every opened teacher label is visible within the top six of at least one causal compact-data mode. Retain these features for H=3 outcome collection; do not train or claim an online policy until material teacher headroom passes.

## Evidence boundary

V234 checks whether one frozen coordinator can distinguish the opened teacher source-label pairs from compact V233 records. It does not select a payload online, use exact eta, run tracking, validate a learned model, or establish generalization.
