# M24 truth-free counterfactual coverage audit: seed 7

## Evidence contract

- Scenario: `m24-hard`
- Seed: `7` (development-only and already design-seen)
- Common prefix: static execution through the predecision state at `t=75`
- Counterfactual continuation: candidate action at `t=75`, followed by
  scheduled control code `24` at `t=76,77`
- Candidates: scheduled codes `01:24`, posterior/link-aware analytic policies
  `80:81`, and their leave-one-selected-edge-out alternatives `82:87`
- Candidate information: deployment-observable only; no truth or future outcome
  is consumed by any action
- Safety contract: every selected route is passed through the exact rolling
  sensor- and formation-level \(B=3\) projector
- Reference: code `24`, with E-OSPA `22.3449`, worst-node E-OSPA `34.5380`,
  and `3,472,848` attempted bytes
- Evidence boundary: this audit tests coverage of a fixed truth-free action
  codebook at one development state. It neither trains nor validates a policy.

All 32 actions satisfied the truth-free information contract and the selected
rolling-\(B=3\) check. They produced only 19 distinct route signatures. No
candidate simultaneously achieved the registered 5% mean E-OSPA improvement,
no tail regression, no repair, and the communication tolerance.

## Complete candidate table

Positive gain and byte-reduction values are improvements over code `24`.

| Code | E-OSPA | Mean gain | Worst node | Tail gain | Byte reduction | Repair |
|--:|--:|--:|--:|--:|--:|--:|
| 01 | 23.1663 | -3.68% | 41.9307 | -21.40% | -1.26% | 0.3333 |
| 02 | 21.6076 | 3.30% | 35.7900 | -3.62% | 0.81% | 0.0000 |
| 03 | 25.6242 | -14.68% | 40.6094 | -17.58% | -0.72% | 0.3333 |
| 04 | 23.1663 | -3.68% | 41.9307 | -21.40% | -1.26% | 0.3333 |
| 05 | 23.5580 | -5.43% | 38.6540 | -11.92% | 0.49% | 0.0000 |
| 06 | 22.9881 | -2.88% | 37.8442 | -9.57% | 0.33% | 0.3333 |
| 07 | 23.1663 | -3.68% | 41.9307 | -21.40% | -1.26% | 0.3333 |
| 08 | 24.7988 | -10.98% | 37.8442 | -9.57% | -0.09% | 0.3333 |
| 09 | 23.0419 | -3.12% | 34.5380 | 0.00% | 0.02% | 0.3333 |
| 10 | 23.1663 | -3.68% | 41.9307 | -21.40% | -1.26% | 0.3333 |
| 11 | 22.8704 | -2.35% | 38.6540 | -11.92% | 1.42% | 0.0000 |
| 12 | 23.5118 | -5.22% | 37.8442 | -9.57% | -0.73% | 0.3333 |
| 13 | 23.1663 | -3.68% | 41.9307 | -21.40% | -1.26% | 0.3333 |
| 14 | 24.5587 | -9.91% | 40.6094 | -17.58% | 0.92% | 0.3333 |
| 15 | 24.0680 | -7.71% | 37.8442 | -9.57% | -0.27% | 0.3333 |
| 16 | 23.1663 | -3.68% | 41.9307 | -21.40% | -1.26% | 0.3333 |
| 17 | 23.0795 | -3.29% | 34.5380 | 0.00% | 1.92% | 0.0000 |
| 18 | 23.4459 | -4.93% | 38.6540 | -11.92% | -0.95% | 0.3333 |
| 19 | 23.1663 | -3.68% | 41.9307 | -21.40% | -1.26% | 0.3333 |
| 20 | 25.2164 | -12.85% | 37.8442 | -9.57% | 1.39% | 0.3333 |
| 21 | 22.4716 | -0.57% | 40.6131 | -17.59% | -0.94% | 0.3333 |
| 22 | 23.1663 | -3.68% | 41.9307 | -21.40% | -1.26% | 0.3333 |
| 23 | 24.1805 | -8.21% | 38.6540 | -11.92% | 0.97% | 0.3333 |
| 24 | 22.3449 | 0.00% | 34.5380 | 0.00% | 0.00% | 0.0000 |
| 80 | 24.7166 | -10.61% | 37.8095 | -9.47% | 1.21% | 0.0000 |
| 81 | 23.1663 | -3.68% | 41.9307 | -21.40% | -1.26% | 0.3333 |
| 82 | 27.0059 | -20.86% | 40.6131 | -17.59% | -0.77% | 0.3333 |
| 83 | 24.7228 | -10.64% | 37.8095 | -9.47% | 0.49% | 0.0000 |
| 84 | 25.2653 | -13.07% | 44.6065 | -29.15% | 1.26% | 0.0000 |
| 85 | 23.1663 | -3.68% | 41.9307 | -21.40% | -1.26% | 0.3333 |
| 86 | 23.1663 | -3.68% | 41.9307 | -21.40% | -1.26% | 0.3333 |
| 87 | 23.1663 | -3.68% | 41.9307 | -21.40% | -1.26% | 0.3333 |

The best mean candidate is code `02`: it improves mean E-OSPA by only 3.30%
and regresses the tail by 3.62%. The only no-repair action with no tail
regression is the code-`24` reference itself. Code `17` reduces bytes by
1.92% without tail regression, but its mean E-OSPA is 3.29% worse.

## Decision

This fixed candidate codebook has no admissible action-space headroom at the
seed-7 decision state. A learned selector over these action IDs cannot recover
the much stronger privileged rollout because that action is absent from its
support. The next method must therefore generate new sensor-level score
directions from observable edge features and counterfactual returns, rather
than imitate one selected graph, predict cardinality, or rank this fixed list.

Before fitting any selector, the expanded generator must pass the same coverage
audit: at least one truth-free, no-repair candidate must clear the registered
mean, tail, and communication gates. Only then is cross-seed value learning
scientifically meaningful.

## Source reports

- `evidence/rollout_dataset/seed7_counterfactual_b01of04/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260729_051154.md`
- `evidence/rollout_dataset/seed7_counterfactual_b02of04/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260729_051151.md`
- `evidence/rollout_dataset/seed7_counterfactual_b03of04/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260729_051156.md`
- `evidence/rollout_dataset/seed7_counterfactual_b04of04/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260729_051206.md`
