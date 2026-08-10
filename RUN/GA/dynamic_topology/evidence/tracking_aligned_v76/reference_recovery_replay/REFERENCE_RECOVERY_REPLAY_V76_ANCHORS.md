# V76 reference-recovery KLA replay

- Virtual fusion rounds: `3`
- Candidate / reference-recovery rounds: `1 / 2`
- Prediction / new measurement / future link page: `0 / 0 / 0`
- Deterministic current-reliability weighting: `1`

## m24-formation-fov-merge-split / t=80

- Direct-safe formations replayed: `3`

### Historical exact V71/V72 route

- Applied slot triples: `[14 10 21;17 19 3]`

| Round | Mean arm gap | Tail arm gap | Max node gap | Reference network mean | Candidate network mean | Mean excess | Tail excess |
|--:|--:|--:|--:|--:|--:|--:|--:|
| 1 | 0.014856 | 0.059424 | 0.223676 | 0.916260 | 0.924131 | +0.007871 | +0.007232 |
| 2 | 0.016916 | 0.067663 | 0.203335 | 0.844076 | 0.843566 | -0.000510 | -0.002728 |
| 3 | 0.025933 | 0.102718 | 0.159503 | 0.785657 | 0.802216 | +0.016559 | +0.030327 |

- Mean/tail gap monotone: `0 / 0`
- Terminal mean/tail contraction: `-74.560% / -72.855%`
- Recovery-safe: `0`

### Prospective exact V73 route

- Applied slot triples: `[14 10 21;17 19 20]`

| Round | Mean arm gap | Tail arm gap | Max node gap | Reference network mean | Candidate network mean | Mean excess | Tail excess |
|--:|--:|--:|--:|--:|--:|--:|--:|
| 1 | 0.010012 | 0.040048 | 0.223676 | 0.916260 | 0.923160 | +0.006900 | +0.001544 |
| 2 | 0.013071 | 0.052285 | 0.203335 | 0.844076 | 0.842400 | -0.001677 | -0.010862 |
| 3 | 0.026630 | 0.105509 | 0.159503 | 0.785657 | 0.803589 | +0.017933 | +0.033093 |

- Mean/tail gap monotone: `0 / 0`
- Terminal mean/tail contraction: `-165.982% / -163.453%`
- Recovery-safe: `0`

## x36-formation-fov-merge-split / t=52

- Direct-safe formations replayed: `4`

### Historical exact V71/V72 route

- Applied slot triples: `[20 16 34;23 25 35]`

| Round | Mean arm gap | Tail arm gap | Max node gap | Reference network mean | Candidate network mean | Mean excess | Tail excess |
|--:|--:|--:|--:|--:|--:|--:|--:|
| 1 | 0.003927 | 0.015709 | 0.101726 | 0.633526 | 0.635678 | +0.002152 | +0.004335 |
| 2 | 0.004330 | 0.017319 | 0.089340 | 0.622365 | 0.620703 | -0.001662 | -0.005489 |
| 3 | 0.009818 | 0.039274 | 0.079256 | 0.594657 | 0.595954 | +0.001297 | -0.004254 |

- Mean/tail gap monotone: `0 / 0`
- Terminal mean/tail contraction: `-150.000% / -150.000%`
- Recovery-safe: `0`

### Prospective exact V73 route

- Applied slot triples: `[20 16 34;23 25 36]`

| Round | Mean arm gap | Tail arm gap | Max node gap | Reference network mean | Candidate network mean | Mean excess | Tail excess |
|--:|--:|--:|--:|--:|--:|--:|--:|
| 1 | 0.004250 | 0.016999 | 0.101726 | 0.633526 | 0.636097 | +0.002571 | +0.005351 |
| 2 | 0.004436 | 0.017744 | 0.089340 | 0.622365 | 0.620787 | -0.001579 | -0.005371 |
| 3 | 0.009836 | 0.039343 | 0.079256 | 0.594657 | 0.596003 | +0.001345 | -0.004173 |

- Mean/tail gap monotone: `0 / 0`
- Terminal mean/tail contraction: `-131.439% / -131.439%`
- Recovery-safe: `0`

## Summary

- All historical routes recovery-safe: `0`
- All aligned routes recovery-safe: `0`
- Route executed / tracking outcome read: `0 / 0`
- Validation claim allowed: `0`

## Evidence boundary

V76 starts from the two opened current posterior states. The reference arm uses the current physical-tree route for all three virtual fusion rounds. The candidate arm uses only the V75 direct-safe formation replacements in round one and the identical reference route for rounds two and three. Every round preserves the registered directed weights and formal mixture-aware heavy fusion. Current link reliability scales weights deterministically; no packet draw, prediction, target motion, new measurement, truth, future link page, or tracking outcome is used. This diagnoses KLA information-flow recovery only, not closed-loop tracking recovery.
