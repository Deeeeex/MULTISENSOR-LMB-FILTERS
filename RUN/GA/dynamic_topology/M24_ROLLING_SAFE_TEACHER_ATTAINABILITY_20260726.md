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

## Expanded safe-action rollout result

The action set was subsequently expanded in two stages.

1. Codes `01` through `24` enumerate every registered burst root,
   orientation and temporal phase for four formations.
2. Codes `90` through `92` generate nearby teacher actions by forbidding,
   one at a time, the default teacher's three selected cross-formation
   edges (ordered by increasing edge-score contribution) and resolving the
   same exact current/successor rolling-\(B=3\) projection.

All action codes modify only the proposal or admissible edge set. The
message budget, one-message-per-receiver contract, payload accounting and
joint rolling-safety projection remain common.

The fixed burst codebook alone did not contain a three-step sequence that
passed both tracking gates. It did, however, localize the useful branch:
the first action `02` created up to 11.17% mean gain but irreversibly harmed
sensor 13, while prefix `02-14` preserved the tail but saturated at 4.82%
mean gain. Leave-one-selected-edge-out alternatives were therefore tested
at each decision time. Diversifying the first teacher action produced the
first joint feasible sequence:

| Arm | Focus E-OSPA | Mean gain | Worst node | Tail gain | Consensus OSPA | Attempted bytes | Byte reduction |
|:--|--:|--:|--:|--:|--:|--:|--:|
| Registered burst control `24-24-24` | 22.3449 | 0.00% | 34.5380 | 0.00% | **22.9908** | 3,472,848 | 0.00% |
| Default current-risk teacher `00-00-00` | 21.0283 | 5.89% | 35.1210 | -1.69% | 22.5080 | 3,387,552 | 2.46% |
| Diverse alternative 1 `90-00-00` | 20.5468 | 8.05% | 35.1210 | -1.69% | **22.3436** | 3,392,256 | 2.32% |
| **Diverse alternative 2 `91-00-00`** | **20.5260** | **8.14%** | **34.5380** | **0.00%** | 23.2549 | **3,432,840** | **1.15%** |
| Diverse alternative 3 `92-00-00` | 21.2732 | 4.80% | 35.1242 | -1.70% | **22.2117** | 3,373,392 | 2.86% |

Sequence `91-00-00` has selected-route signature
`0A10-0A10-4042`, which differs from both the registered control
(`1000-0000-0842`) and default current-risk teacher
(`2810-4810-0882`). It satisfies selected sensor/formation \(B=3\) on all
three mature windows, matches the control's delivered-\(B=3\) fraction of
0.6667, and uses no infeasibility fallback, repair or payload emergency.

An independent rerun of only the new arm reproduced the exact values:

- Evidence report:
  `evidence/action_search/reproduction/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260729_004650.md`
- Raw log:
  `/tmp/m24_diverse_first_alt_91_reproduction_20260729.log`
- Reproduced tuple:
  E-OSPA `20.5259870081`, worst node `34.5380426781`,
  attempted bytes `3432840`, selected \(B=3\) `1.0`,
  delivered \(B=3\) `0.666667`.

This establishes the registered M24 development-block attainability gate:
the safe action space contains a sequence with at least 5% mean gain,
no worst-node regression and lower attempted bytes. It does not yet
establish a deployable method. Codes `90` through `92` use truth-labelled
teacher scores, and the sequence was selected using the same development
block's final tracking outcomes.

There is also a secondary caveat: the passing sequence's consensus OSPA is
1.15% worse than the registered control, although this metric was not part
of the current joint tracking-tail-byte gate. The learned objective should
therefore retain consensus as a monitored auxiliary loss rather than hide
this trade-off.

## Multi-seed rollout-candidate attainability

The first leave-one-edge-out screen was then repeated on the declared M24
development seeds 19, 23 and 29. A fixed alternative index did not
generalize. In particular, all four `90/91/92-00-00` candidates on seed 19
improved network-mean E-OSPA by about 14--16%, but increased the worst-node
value from 43.0102 to 48.7124. The exact current-step minimax teacher showed
the same tail failure (E-OSPA 18.6463, worst node 48.7124), so a stronger
one-step risk scalar was still temporally misaligned.

An exhaustive binary switch screen on seed 19 localized the failure. The
teacher should not be activated at the first continuation step. Sequence
`011`, which executes the registered control at \(t=75\) and recomputes the
teacher at \(t=76,77\), passes the joint gate without repair:

| Seed | Best observed safe rollout | Mean gain | Tail gain | Byte reduction | Selected \(B=3\) | Repair |
|--:|:--|--:|--:|--:|--:|--:|
| 7 | diverse alt. 2, then teacher (`91-00-00`) | 8.14% | 0.00% | 1.15% | 1.0000 | 0 |
| 19 | control, then teacher (`011`) | 7.15% | 0.15% | 1.27% | 1.0000 | 0 |
| 23 | diverse alt. 1, then teacher (`90-00-00`) | 26.10% | 37.71% | 0.65% | 1.0000 | 0 |
| 29 | diverse alt. 3, then teacher (`92-00-00`) | 36.54% | 65.53% | 3.26% | 1.0000 | 0 |

The table is an action-space attainability result on four development
seeds, not a learned-policy result. Each row was selected after observing
that seed's closed-loop tracking outcome, and codes `00` and `90:92` read
truth. Its methodological implication is nevertheless sharp: the common
safe projector admits useful routes on every development seed, while the
best first action and activation time vary across seeds. The remaining
problem is therefore state-conditioned finite-horizon ranking, not another
fixed schedule or another coefficient on the one-step edge score.

Supporting evidence:

- Seed 19 diverse-action screen:
  `evidence/rollout_dataset/seed19/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260729_013509.md`
- Seed 19 hybrid screen:
  `evidence/rollout_dataset/seed19_hybrid/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260729_015415.md`
- Seed 19 minimax reproduction:
  `evidence/rollout_dataset/seed19_risk/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260729_015004.md`
- Seeds 23 and 29 candidate screens:
  `evidence/rollout_dataset/seed23/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260729_013515.md`
  and
  `evidence/rollout_dataset/seed29/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260729_013525.md`

### Truth-free proposal check

The seed-19 timing result could in principle have reduced the learning
problem to a binary gate over an existing analytic proposal. That simpler
hypothesis was tested by replacing the teacher at the successful delayed
activation times with two truth-free rolling-safe scores:

| Sequence | Proposal | E-OSPA | Worst node | Attempted bytes | Mean gate |
|:--|:--|--:|--:|--:|:--|
| `24-24-24` | registered control | 21.7219 | 43.0102 | 3,482,928 | reference |
| `24-24-80` | posterior-analytic at \(t=77\) | 22.3948 | 42.9460 | 3,483,432 | fail |
| `24-80-80` | posterior-analytic at \(t=76,77\) | 23.5697 | 44.8667 | 3,471,072 | fail |
| `24-24-81` | link-aware at \(t=77\) | 22.0004 | 42.9460 | 3,479,208 | fail |
| `24-81-81` | link-aware at \(t=76,77\) | 22.0004 | 42.9460 | 3,479,208 | fail |

The truth-free candidates preserve the safety contract but do not reproduce
the teacher's 7.15% tracking gain. A learned timing gate over either
existing heuristic is therefore insufficient. The data-driven component
must learn graph or edge value, while the deterministic projector remains
responsible for admissibility.

Supporting evidence:

- `evidence/rollout_dataset/seed19_truthfree/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260729_015745.md`

## Method implication

The evidence supports a different learning target from direct one-step edge
regression. A finite-horizon **diverse safe rollout teacher** should:

1. generate the default projected action and a small set of structured
   leave-one-selected-edge-out alternatives;
2. roll each candidate forward under the same future data and safety layer;
3. select by mean tracking risk subject to explicit tail, communication and
   rolling-connectivity constraints;
4. supervise a truth-free graph scorer or action ranker from observable
   posterior, link, geometry and executed-history features.

At deployment, the learned model proposes edge values or ranks a small
candidate set; the deterministic joint projector remains responsible for
the rolling-\(B=3\), message-budget and payload contracts. This separation
preserves the theoretical safety story while giving the data-driven
component a nontrivial finite-horizon target.
