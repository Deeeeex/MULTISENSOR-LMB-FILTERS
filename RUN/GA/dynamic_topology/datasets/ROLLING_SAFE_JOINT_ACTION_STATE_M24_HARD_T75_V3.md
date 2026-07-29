# Rolling-safe joint-action state dataset v3

- Generated: 2026-07-29 08:01:45
- Contract: `rolling-safe-joint-action-state-v3-selected-delivered-history-centralized-coordinator`
- Preset: `m24-hard`
- Seeds: `[7 11 17 19 23 29]`
- Times: `[75 76 77]`
- Source weight: 0.70
- Payload tolerance fraction: `inf`
- Filter RNG offset: `100000`
- Protocol: `m24-rolling-safe-joint-action-critic-v1-centralized-metadata-unaccounted`
- Source rollout protocol: `m24-rolling-safe-rollout-v2-projector-replay-f1`
- Generation git commit: `55e6dc4830c6255450dfad9429c5ed8c01871097`
- Generation research worktree dirty: `0`
- Feature context: `raw`
- Feature count: 42
- Features use truth: `0`
- Labels use truth: `1`
- Future outcomes selected labels: `1`
- Joint-projector replay context: physical action set, group IDs, sender payload estimates and executed B=3 history
- Selected/delivered history contract: separate two-page predecision histories; delivered is a subset of selected
- Coordination mode: `centralized-truth-free-predecision-summary-coordinator`
- Coordinator inputs: `all-node-posterior-summaries, current-geometry, current-link-reliability, sender-payload-metadata, selected-topology-history`
- Posterior payload traffic accounted: `1`
- Control metadata traffic accounted: `0`
- Evidence boundary: Features are deployment-observable. Executed graph labels come from seed-specific action sequences selected after closed-loop outcome inspection and are privileged training supervision only. The v3 state contract separately stores the two predecision selected and delivered history pages; delivered edges are attested subsets of selected edges. The study assumes a centralized truth-free coordinator with all-node posterior summaries, geometry, link reliability, payload metadata and selected history. Its control-metadata traffic is not included in posterior attempted-byte totals.

## Frozen continuation caches

| Seed | SHA-256 |
|--:|:--|
| 7 | `a33665ae7709a2bbfd2bb7c0ad79f51613d6eb2534f9c7919fdd95f70b323055` |
| 11 | `c0b67093bc055d23a920f2bf695108e74084acf7068e6ad19093645674c3fdef` |
| 17 | `5e6649a27b42cefbac5e2f4cb88c0c0eed2fc81f84a21902dddf6d028be7a605` |
| 19 | `0624fee8f6ebb5c6104c3b7889ebbe4c71100c8a09c6756824197f770958d963` |
| 23 | `0508cbc0b4ec2647fa66e1ab227da5574a62f4463f064e098e0db8d6d2239f0b` |
| 29 | `16a9f2bd7838943da714523dd1a3a3caf26b1f121f62bf8f1beb412ab505878e` |

## Rollout outcomes

| Seed | Codes | E-OSPA | Worst node | Attempted bytes | Selected B3 | Repair | Infeasible |
|--:|:--|--:|--:|--:|--:|--:|--:|
| 7 | `91-00-00` | 20.5260 | 34.5380 | 3432840 | 1.0000 | 0.0000 | 0.0000 |
| 11 | `00-00-00` | 12.6290 | 37.6304 | 3398928 | 1.0000 | 0.0000 | 0.0000 |
| 17 | `00-24-00` | 15.9849 | 35.7370 | 3416016 | 1.0000 | 0.0000 | 0.0000 |
| 19 | `24-00-00` | 20.1698 | 42.9460 | 3438576 | 1.0000 | 0.0000 | 0.0000 |
| 23 | `90-00-00` | 5.1936 | 13.5208 | 3509328 | 1.0000 | 0.0000 | 0.0000 |
| 29 | `92-00-00` | 5.8384 | 13.2801 | 3438072 | 1.0000 | 0.0000 | 0.0000 |

## Supervision blocks

| Seed | Time | Code | Examples | Selected cross | Action truth used |
|--:|--:|--:|--:|--:|--:|
| 7 | 75 | 91 | 432 | 3 | 1 |
| 7 | 76 | 00 | 432 | 3 | 1 |
| 7 | 77 | 00 | 432 | 3 | 1 |
| 11 | 75 | 00 | 432 | 3 | 1 |
| 11 | 76 | 00 | 432 | 3 | 1 |
| 11 | 77 | 00 | 432 | 3 | 1 |
| 17 | 75 | 00 | 432 | 3 | 1 |
| 17 | 76 | 24 | 432 | 0 | 0 |
| 17 | 77 | 00 | 432 | 3 | 1 |
| 19 | 75 | 24 | 432 | 1 | 0 |
| 19 | 76 | 00 | 432 | 3 | 1 |
| 19 | 77 | 00 | 432 | 3 | 1 |
| 23 | 75 | 90 | 432 | 3 | 1 |
| 23 | 76 | 00 | 432 | 3 | 1 |
| 23 | 77 | 00 | 432 | 3 | 1 |
| 29 | 75 | 92 | 432 | 3 | 1 |
| 29 | 76 | 00 | 432 | 3 | 1 |
| 29 | 77 | 00 | 432 | 3 | 1 |
