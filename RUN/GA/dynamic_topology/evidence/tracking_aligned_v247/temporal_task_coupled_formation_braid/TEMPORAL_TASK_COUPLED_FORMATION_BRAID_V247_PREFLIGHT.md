# V247 temporally task-coupled formation-braid preflight

- Seed: `1301`
- Cross-scale geometry gate passed: `1`
- Tracking outcome authorized: `0`
- Evidence boundary: V247 checks scene validity and realised target-owner transfers during initial-tree cut failures. It does not establish a tracking, consistency, communication-byte, learned-policy, or generalization benefit.

| Scale | Valid | Blackout | Single / multi formation | Target speed / acceleration | Min target / sensor-target separation | Failed / temporally coupled cuts | Lane signs |
|:--|--:|--:|--:|--:|--:|--:|:--|
| m24-formation-fov-temporal-coupled-formation-braid | 1 | 0.028 | 0.752 / 0.220 | 13.432 / 1.098 | 17.909 / 60.223 | 1 / 1 | `[-1 1 1 -1]` |
| x36-formation-fov-temporal-coupled-formation-braid | 1 | 0.027 | 0.744 / 0.228 | 13.934 / 1.098 | 18.000 / 58.828 | 2 / 2 | `[-1 1 -1 1 1 -1]` |
| x48-formation-fov-temporal-coupled-formation-braid | 1 | 0.029 | 0.692 / 0.279 | 13.741 / 1.098 | 18.000 / 39.232 | 3 / 3 | `[-1 1 -1 1 -1 1 1 -1]` |

## Realised cut/handover alignment

### m24-formation-fov-temporal-coupled-formation-braid

- Coupling contract: `actual-owner-side-transition-during-failed-initial-tree-cut-v2`
- Scene hard failures: `--`

| Initial-tree cut | Failure interval | Cohort | Pre-failure source support | During-failure destination support | Aligned targets | First aligned transition | Cross-cut visibility overlap | Pass |
|:--|:--|:--|--:|--:|--:|--:|--:|--:|
| 2--3 | 70--160 | G2: 2->3 | 0.990 | 0.595 | 1.000 | 84 | 0.243 | 1 |

### x36-formation-fov-temporal-coupled-formation-braid

- Coupling contract: `actual-owner-side-transition-during-failed-initial-tree-cut-v2`
- Scene hard failures: `--`

| Initial-tree cut | Failure interval | Cohort | Pre-failure source support | During-failure destination support | Aligned targets | First aligned transition | Cross-cut visibility overlap | Pass |
|:--|:--|:--|--:|--:|--:|--:|--:|--:|
| 2--3 | 55--160 | G2: 2->3 | 1.000 | 0.912 | 1.000 | 55 | 0.236 | 1 |
| 4--5 | 95--160 | G4: 4->5 | 0.990 | 0.926 | 0.750 | 96 | 0.236 | 1 |

### x48-formation-fov-temporal-coupled-formation-braid

- Coupling contract: `actual-owner-side-transition-during-failed-initial-tree-cut-v2`
- Scene hard failures: `--`

| Initial-tree cut | Failure interval | Cohort | Pre-failure source support | During-failure destination support | Aligned targets | First aligned transition | Cross-cut visibility overlap | Pass |
|:--|:--|:--|--:|--:|--:|--:|--:|--:|
| 2--3 | 46--160 | G2: 2->3 | 1.000 | 0.669 | 1.000 | 53 | 0.041 | 1 |
| 4--5 | 71--160 | G4: 4->5 | 1.000 | 0.541 | 1.000 | 82 | 0.014 | 1 |
| 6--7 | 101--160 | G6: 6->7 | 1.000 | 0.851 | 1.000 | 102 | 0.108 | 1 |

## Interpretation

The old V244 gate counted a target route when its registered source and destination lay on opposite sides of a cut. That was insufficient for moving formations. V247 instead requires realised source-side ownership before the cut failure and a realised destination-side transition while the cut is unavailable. Passing this report licenses the scene for tracker experiments only; it is not itself evidence that dynamic routing improves tracking.
