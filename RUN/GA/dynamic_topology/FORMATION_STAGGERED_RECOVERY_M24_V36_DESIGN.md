# V36 multi-state M24 replication

## Question

V35 passes every frozen gate on the already-opened M24 seed-211 t=72 state.
V36 asks whether the unchanged causal controller also creates safe nonreference
actions and closed-loop headroom at three temporally separated, already-cached
states: t=60, t=104, and t=124.

## Source-only gate

For each state, the controller uses the current posterior, physical graph,
link reliability, and two selected-topology history pages. It first chooses the
initial suspension subset with the unchanged 2%/1% retention-debt hysteresis.
It then executes the complete H=3 v35 runtime without tracking or ground-truth
scoring. Exact initial execution, label retention, message bounds, current-data
attestation, and every one-step rolling-B3 reserve must pass.

The source-only initial actions are frozen before any new outcome is opened:

| Anchor | Initial suspended formations | Bank action |
|--:|:--|--:|
| 60 | `[2,4]` | 11 |
| 104 | `[1,2,4]` | 12 |
| 124 | `[2,3,4]` | 15 |

The complete runtime traces are frozen after a dirty exploratory source-only
run and reproduced from a clean commit. A state may open exactly one paired
reference-versus-v35 H=3 outcome only if its full clean rollout passes.

| Anchor | H=3 suspended-formation trace | Explicit release trace | Safe fallback |
|--:|:--|:--|:--|
| 60 | `[2,4] -> [2,4] -> []` | `[] -> [] -> []` | reference at t=62 |
| 104 | `[1,2,4] -> [1,2] -> [4]` | `[] -> [4] -> []` | none |
| 124 | `[2,3,4] -> [2,3] -> [4]` | `[] -> [4] -> []` | none |

The t=60 terminal reference is the controller's declared conservative
fallback, not a post-outcome edit. It is frozen together with both staggered
release traces before any tracking metric is evaluated.

## Replication gate

Each state retains the v35 thresholds: at least 2% mean tracking gain, no
formation or worst-sensor regression, nonnegative window and terminal
consensus gain, nonnegative attempted-byte saving, and selected rolling-B3.

The multi-state development gate requires at least two of three strict-strong
states, median mean gain of at least 2%, no state below -1% mean gain, and at
least two states with positive terminal consensus. Passing may authorize an
X36 source-only protocol; it does not itself authorize GNN training,
validation, X48, or reserved seeds.

## Frozen clean preflight

Commit `3fc4a0e280dea00bfc934e5a0fb8989f6887acb0` reproduced all three
truth-free H=3 traces with clean source and authorized exactly one paired
outcome per state. The preflight did not score tracking or open a new
posterior state.

Earlier t=60/104/124 reference artifacts retain mean OSPA and total-byte
results, but predate terminal-consensus time series and the current runtime
attestation fields. V36 therefore reruns each reference once under the current
schema and freezes it in the aggregate screen; later analyses must reuse that
frozen reference rather than rerun it again.
