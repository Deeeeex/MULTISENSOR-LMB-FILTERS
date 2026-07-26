# M24 rolling-safe teacher attainability checkpoint

## Result

Using the same `m24-hard` continuation snapshot, source weight, three-step
focus window, directed-message budget, rolling-\(B=3\) safety projection and
communication accounting, the privileged current-risk teacher outperformed
the strongest registered rolling control observed in the complete matched
family.

| Arm | Focus E-OSPA | Worst node | Consensus OSPA | Attempted bytes | Delivered B3 |
|:--|--:|--:|--:|--:|--:|
| Posterior-analytic rolling-safe | 25.1206 | 38.6379 | 26.3899 | 3,406,752 | 1.0000 |
| Best registered control: burst r4, ccw, phase 2 | 22.3449 | **34.5380** | 22.9908 | 3,472,848 | 0.6667 |
| Privileged current-risk rolling-safe teacher | **21.0283** | 35.1210 | **22.5080** | **3,387,552** | 0.6667 |
| Privileged open-loop \(H=2\) rolling-safe teacher | **21.0283** | 35.1210 | **22.5080** | **3,387,552** | 0.6667 |

The current-risk teacher improves focus E-OSPA by approximately **5.89%**
relative to the strongest matched control while attempting fewer bytes.
Its worst-node E-OSPA is nevertheless about **1.69% worse** than that
control, so this checkpoint establishes mean-tracking headroom but does not
pass the registered tail-safety gate.
Removing the arbitrary fixed successor schedule also improves the analytic
arm from the earlier 27.0178 result to 25.1206, but the analytic scorer
remains about 12.4% worse than the strongest control.

## Experimental contract

- Scenario: `m24-hard`
- Seed: `7`
- Continuation cache:
  `RUN/GA/dynamic_topology/cache/m24_hard_seed7_n1_sig75.mat`
- Continuation cache contract:
  `static-prefix-behavior-cache-v2-predecision-history`
- Focus and executed interval: \(t=75{:}77\)
- Source weight: \(0.70\)
- Rolling connectivity window: \(B=3\)
- Current and successor routes: exact joint rolling-safe projection
- Online payload projection cap: disabled; realized attempted bytes are
  compared directly
- Raw execution log:
  `/tmp/m24_rolling_teacher_20260726.log`
- Automatically generated evidence report:
  `RUN/GA/dynamic_topology/evidence/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260726_215248.md`
- Local MAT telemetry artifact:
  `RUN/GA/dynamic_topology/evidence/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260726_215248.mat`

The command compared:

```matlab
{
  'rolling-safe-analytic-w70',
  'directed-rolling-burst-r4-dccw-p2-w70',
  'oracle-rolling-safe-current-w70',
  'oracle-rolling-safe-h2-w70'
}
```

## Interpretation boundary

This is a single development seed and is not paper-level evidence. Both
teacher arms read target truth when constructing edge-value labels and are
not deployable. The identical current and open-loop \(H=2\) results show no
incremental evidence for the more expensive open-loop prediction label in
this window. Their independently audited selected-route signatures are also
identical (`2810-4810-0882`), so the \(H=2\) ranking actually collapses to
the current-risk ranking over these three decisions.

The valid conclusion is narrower and important: the rolling-\(B=3\) action
space and exact safety layer can support a practically meaningful mean
closed-loop gain. The next method gate is stricter: a risk-sensitive teacher
must retain that mean gain without regressing the worst sensor, after which a
truth-free structured scorer can be trained only on declared development
blocks. Unseen-seed M24 validation and an independently frozen X36 transfer
test remain subsequent gates.
