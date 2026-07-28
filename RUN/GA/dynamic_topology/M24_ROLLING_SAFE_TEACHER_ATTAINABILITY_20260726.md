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

## Risk-sensitive one-step diagnostics

Several stricter one-step objectives were tested before increasing the
teacher's action horizon.

| Diagnostic | Focus E-OSPA | Mean gain vs control | Worst node | Tail gain vs control | Decision |
|:--|--:|--:|--:|--:|:--|
| Current-risk teacher | 21.0283 | **5.89%** | 35.1210 | -1.69% | Mean gate only |
| Receiver-risk priority, \(c=1\) | 21.0283 | **5.89%** | 35.1210 | -1.69% | Same realized route |
| Receiver-risk priority, \(c=3\) | 21.0508 | 5.79% | 35.1210 | -1.69% | Tail unchanged |
| Exact current-step minimax | 21.2706 | 4.81% | 35.1210 | -1.69% | Misses both gates |
| Corrected global-tail cap | 21.0283 | **5.89%** | 35.1210 | -1.69% | Same realized route |

The corrected global-tail-capped projection lowers the predicted one-step
worst risk from the scheduled anchor's 0.321158 to 0.222130. Nevertheless,
the realized three-step worst-node E-OSPA increases from 34.5380 to 35.1210.
This is direct evidence that the present one-step expected-risk surrogate is
temporally misaligned with the closed-loop tail gate. Further tuning of its
linear coefficient or MILP cap does not address the observed failure.

Two earlier control-anchored diagnostics generated at `20260726_224527` and
`20260726_225402` are invalidated. Their additive objective used
`anchorRisk - candidateRisk`, although the unselected action falls back to
the formation cycle. The correct edge advantage is therefore
`cycleRisk - candidateRisk`; the scheduled anchor may appear only in the
risk constraint. The implementation and tests now enforce this distinction,
so those two numerical results must not be cited as method evidence.

Supporting evidence:

- Receiver-risk priority:
  `evidence/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260726_221300.md`
- Exact current-step minimax:
  `evidence/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260726_223017.md`
- Corrected global-tail cap:
  `evidence/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260726_230419.md`

## Real closed-loop switching diagnostic

The next diagnostic enumerated every three-step binary switching sequence
between the registered burst control and the current-risk teacher. Unlike
the earlier open-loop \(H=2\) score, each activated teacher step was
recomputed from that hybrid trajectory's actual current posterior and
executed topology history.

| Mask | Focus E-OSPA | Mean gain | Worst node | Tail gain | Attempted bytes | Gate |
|:--:|--:|--:|--:|--:|--:|:--|
| 000 | 22.3449 | 0.00% | 34.5380 | 0.00% | 3,472,848 | Control |
| 001 | 22.0600 | 1.28% | 34.5380 | 0.00% | 3,464,568 | Mean fails |
| 010 | 21.9329 | 1.84% | 34.5729 | -0.10% | 3,409,368 | Mean and tail fail |
| 011 | 21.2387 | 4.95% | 34.7381 | -0.58% | 3,401,136 | Just below mean; tail fails |
| 100 | 22.2890 | 0.25% | 31.3041 | **9.36%** | 3,427,536 | Mean fails |
| 101 | 21.6043 | 3.31% | 31.2587 | **9.50%** | 3,430,008 | Mean fails |
| 110 | 21.4818 | 3.86% | 35.1210 | -1.69% | 3,391,440 | Mean and tail fail |
| 111 | **21.0283** | **5.89%** | 35.1210 | -1.69% | **3,387,552** | Tail fails |

All eight sequences satisfy selected rolling-\(B=3\) safety, attempt no more
bytes than the control, and reproduce the control's delivered-\(B=3\)
fraction of 0.6667. No binary sequence simultaneously reaches the registered
5% mean-improvement threshold and preserves the control's worst-node result.
The failure is therefore not resolved by choosing when to activate the same
one-step teacher.

Supporting evidence:

- `evidence/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260728_232256.md`

## Current decision

The rolling-\(B=3\) projector and message budget admit meaningful mean
tracking gains and, under other action sequences, meaningful tail gains.
However, the two-policy switching set contains no action sequence that
achieves both on this development block. The next teacher must expand the
action set itself: generate multiple alternative rolling-safe edge
assignments, evaluate their actual finite-horizon closed-loop mean and tail
risk, and use the best feasible sequence as privileged training supervision.
Only after that attainability gate passes should a truth-free structured
scorer be trained on declared development blocks. Unseen-seed M24 validation
and a frozen X36 transfer test remain subsequent gates.
