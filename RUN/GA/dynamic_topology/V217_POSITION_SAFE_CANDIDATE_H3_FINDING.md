# V217 position-safe candidate H=3 finding

## Paired result

The frozen `m24-formation-fov / seed 1301 / t=119 / F2` bank contains `39`
causal complete-label KLA candidates. The 99% two-dimensional position
support guard retains `31`; the three registered proposal modes reduce these
to eight distinct, credit-feasible actions. Every action was evaluated from
the same continuation cache against the same full-payload reference over
`H=3`. Candidate availability used the realized delivery graph, the compact
synopsis cost was `7,056 B`, and each forced action executed exactly once.

| candidate | source / label | E-OSPA | RMSE | consistency | terminal consistency | byte saving | repair vs withholding E / R / C | admit |
|--:|:--|--:|--:|--:|--:|--:|:--|:--:|
| 5 | S1 `[25,14]` | -1.830% | -31.103% | -3.807% | -4.493% | +1.215% | -2.274 / -9.660 / -4.281% | no |
| 16 | S13 `[25,14]` | +0.431% | -29.303% | +0.517% | -2.159% | -0.073% | -0.003 / -8.155 / +0.062% | no |
| 21 | S14 `[25,14]` | +0.454% | -29.009% | +0.633% | -2.032% | -0.073% | +0.020 / -7.909 / +0.178% | no |
| 22 | S15 `[9,5]` | +0.437% | -19.503% | +0.463% | -2.227% | +0.264% | +0.003 / +0.043 / +0.008% | no |
| 26 | S15 `[25,14]` | +0.446% | -19.267% | +0.609% | -2.081% | +0.039% | +0.012 / +0.240 / +0.154% | no |
| 34 | S17 `[17,9]` | +0.346% | -30.184% | +0.107% | -2.525% | -0.073% | -0.088 / -8.892 / -0.350% | no |
| 35 | S17 `[25,14]` | +0.457% | -28.977% | +0.646% | -2.025% | -0.073% | +0.023 / -7.882 / +0.192% | no |
| 38 | S18 `[17,9]` | +0.341% | -20.471% | +0.102% | -2.554% | -0.073% | -0.094 / -0.767 / -0.355% | no |

## Method decision

This state is a no-op label for the hierarchical controller. Position
support, direct executability and communication credit are feasibility
conditions, not evidence that available credit should be spent. Candidate
26 is the strongest incremental repair relative to withholding-only, but it
still leaves mean RMSE `19.267%` worse than the full-payload reference, F2
RMSE `74.727%` worse, terminal consistency `2.081%` worse and only `0.039%`
attempted-byte saving. Candidate 5 is the only action retaining the frozen
`1%` byte reserve, yet all four aggregate tracking/consistency coordinates
regress.

The failure is therefore upstream: withholding F2 at this state is not a
safe base action that a single supported-label update can rescue. The value
model must compare three complete alternatives from the same state:

1. no-op / ordinary full-posterior routing;
2. one-page formation withholding;
3. the same withholding plus one position-safe complete-label KLA action.

The repair head cannot be invoked merely because the withholding head earned
credit, and an admitted repair cannot retroactively certify a harmful graph
action. This training state supplies eight negative repair labels and one
no-op preference; it does not establish cross-state, cross-scale or online
performance and does not update the paper-facing current-best table.
