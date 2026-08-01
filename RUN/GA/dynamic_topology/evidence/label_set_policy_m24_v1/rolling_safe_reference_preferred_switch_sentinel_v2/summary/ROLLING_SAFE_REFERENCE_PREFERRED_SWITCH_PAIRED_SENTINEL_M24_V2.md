# M24 reference-preferred safe-switch sentinel

- Reference / fixed-duration envelope / adaptive oracle mean E-OSPA: `15.271324 / 14.398346 / 14.398346`
- Fixed envelope / adaptive gain: `+5.716% / +5.716%`
- Fixed-envelope recovery: `1.000` (gate `>=0.90`)
- States improving reference: `2 / 4` (gate `>=2`)
- Best states selecting candidate / reference later: `1 / 3`
- Hard audit / headroom / selector-development: `1 / 1 / 1`

| Seed-time | Source | Reference | One-step | Persistent | Fixed env. | Adaptive | Candidate | Executed codes | Gain | vs env. | Worst | Consensus | Unavailable | Safe |
|:--|:--|--:|--:|--:|--:|--:|:--|:--|--:|--:|--:|--:|--:|--:|
| 11-78 | learned | 14.983228 | 14.983228 | 14.983228 | 14.983228 | 14.983228 | reference-only-safe-switch | `[24 24 24]` | +0.000% | +0.000 pp | +0.000% | +0.000% | 1 | 1 |
| 19-78 | ccw | 21.026617 | 21.026617 | 21.026617 | 21.026617 | 21.026617 | reference-only-safe-switch | `[24 24 24]` | +0.000% | +0.000 pp | +0.000% | +0.000% | 2 | 1 |
| 23-78 | ccw | 8.427007 | 8.427007 | 5.961354 | 5.961354 | 5.961354 | deterministic-burst-reference-preferred-switch | `[22 22 22]` | +29.259% | +0.000 pp | +40.949% | +34.236% | 2 | 1 |
| 27-78 | learned | 16.648442 | 15.622186 | 16.284410 | 15.622186 | 15.622186 | history-continuity-reference-preferred-switch | `[77 24 24]` | +6.164% | +0.000 pp | +10.413% | +5.439% | 3 | 1 |

## Decision

The mechanism falsifier passes and authorizes only opened-training first-action selector development.

## Boundary

This paired opened-training mechanism falsifier compares a non-absorbing, reference-preferred strict nominal-free switching bank with same-commit one-step and persistent controls. The four sentinel states were already used to design the rule, so success only authorizes opened-training selector development. Best-of-eight first-action selection remains an offline oracle. Fail-closed arms are retained as unavailable rather than silently projected. No formation-FoV M24, X36, X48, recursive-feasibility or deployable fallback claim is made. Coordinator metadata bytes are uncharged.
