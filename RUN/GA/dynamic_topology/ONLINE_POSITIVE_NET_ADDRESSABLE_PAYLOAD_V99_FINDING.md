# V99 finding: online selection helps, but the gain is delayed

## Matched X36 t72 result

V99 keeps the same static carrier graph and fusion weights as the full-payload
baseline.  It differs from V97 only by recomputing the safe positive-net
receiver-formation set after each local update.

| Arm | Mean E-OSPA | Gain over static | Attempted-byte saving |
|:--|--:|--:|--:|
| Static full payload | 85.970277 | -- | -- |
| Fixed V97 | 83.896827 | +2.412% | +5.305% |
| Online V99 | 83.561598 | +2.802% | +6.550% |

The online sets are `[1 2 4 5]`, `[1 2 3 4 5]` and `[1 2 3 4 5]` at
t=72, 73 and 74.  V99 improves on fixed V97 and preserves every registered
tail gate: worst-sensor gain is +7.669%, minimum-formation gain is 0%, window
and terminal consensus gains are +5.149% and +10.177%, and rolling B3 passes.
It nevertheless fails the required 5% mean E-OSPA gain.

## Mechanism diagnosis

The aggregate result hides a monotone temporal response:

| Time | Static | Fixed V97 | Online V99 | V99/static |
|---:|---:|---:|---:|---:|
| 72 | 86.118620 | 85.071354 | 85.071354 | +1.216% |
| 73 | 85.408155 | 84.028817 | 83.601644 | +2.115% |
| 74 | 86.384056 | 82.590310 | 82.011795 | +5.061% |

Only a small subset of nodes changes immediately; the protected gateway state
must propagate through later fusion rounds.  The terminal step already exceeds
5%, while the three-step mean is diluted by the first two transient steps.
The V98 temporal-migration hypothesis is therefore real but incomplete: it
accounts for another 0.400% in the window, not the remaining 2.198 percentage
points needed by the mean gate.

## Decision

Do not run the four-anchor V99 screen and do not expand scenes yet.  First test
whether the improvement persists over a horizon tied to effective graph
propagation time rather than the inherited fixed H=3 window.  If the longer
matched window remains above 5% without tail regressions, the method needs a
scale-aware decision horizon.  If it decays, the next action must become more
spatially granular, such as receiver-sender or label-selective participation.

The exact V99 online arm takes 502.57 seconds versus 87.97 seconds for fixed
V97.  Even if its longer-horizon tracking effect survives, this evaluator must
serve as a teacher for a cheaper causal selector; its present runtime is not a
deployable engineering contribution.
