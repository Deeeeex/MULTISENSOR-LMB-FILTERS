# V75 replacement innovation-energy anchors

- Conditional energy limit: `5.991464547`
- Limit chosen after opened-anchor inspection: `1`
- Validation preregistration: `0`

## m24-formation-fov-merge-split / t=80

### Historical exact V71/V72 route

| Formation | Receiver | Incumbent | Candidate | Labels | Conditional RIE | Reliability | Expected RIE | Safe |
|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| 3 | 14 | 10 | 21 | 14 | 3.298216 | 0.948 | 3.125536 | 1 |
| 3 | 17 | 19 | 3 | 15 | 0.675438 | 0.942 | 0.636047 | 1 |

| Formation | Covered slots | Maximum conditional RIE | Maximum expected RIE | Safe |
|--:|--:|--:|--:|:--:|
| 3 | 2/2 | 3.298216 | 3.125536 | 1 |

- Expected historical formation safety: `true`
- Observed historical formation safety: `true`
- Retrospective direction match: `1`

### Prospective exact V73 route

| Formation | Receiver | Incumbent | Candidate | Labels | Conditional RIE | Reliability | Expected RIE | Safe |
|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| 3 | 14 | 10 | 21 | 14 | 3.298216 | 0.948 | 3.125536 | 1 |
| 3 | 17 | 19 | 20 | 15 | 0.143606 | 0.942 | 0.135267 | 1 |

| Formation | Covered slots | Maximum conditional RIE | Maximum expected RIE | Safe |
|--:|--:|--:|--:|:--:|
| 3 | 2/2 | 3.298216 | 3.125536 | 1 |

- Prospective route tracking label available: `0`

## x36-formation-fov-merge-split / t=52

### Historical exact V71/V72 route

| Formation | Receiver | Incumbent | Candidate | Labels | Conditional RIE | Reliability | Expected RIE | Safe |
|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| 4 | 20 | 16 | 34 | 22 | 1.096073 | 0.941 | 1.031339 | 1 |
| 4 | 23 | 25 | 35 | 19 | 0.755945 | 0.936 | 0.707900 | 1 |
| 5 | 26 | 22 | 35 | 18 | 8.594396 | 0.949 | 8.159288 | 0 |
| 5 | 29 | 31 | 17 | 18 | 0.670267 | 0.936 | 0.627561 | 1 |

| Formation | Covered slots | Maximum conditional RIE | Maximum expected RIE | Safe |
|--:|--:|--:|--:|:--:|
| 4 | 2/2 | 1.096073 | 1.031339 | 1 |
| 5 | 2/2 | 8.594396 | 8.159288 | 0 |

- Expected historical formation safety: `[true false]`
- Observed historical formation safety: `[true false]`
- Retrospective direction match: `1`

### Prospective exact V73 route

| Formation | Receiver | Incumbent | Candidate | Labels | Conditional RIE | Reliability | Expected RIE | Safe |
|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| 4 | 20 | 16 | 34 | 22 | 1.096073 | 0.941 | 1.031339 | 1 |
| 4 | 23 | 25 | 36 | 19 | 1.052758 | 0.935 | 0.983901 | 1 |
| 5 | 26 | 22 | 35 | 18 | 8.594396 | 0.949 | 8.159288 | 0 |
| 5 | 29 | 31 | 17 | 18 | 0.670267 | 0.936 | 0.627561 | 1 |

| Formation | Covered slots | Maximum conditional RIE | Maximum expected RIE | Safe |
|--:|--:|--:|--:|:--:|
| 4 | 2/2 | 1.096073 | 1.031339 | 1 |
| 5 | 2/2 | 8.594396 | 8.159288 | 0 |

- Prospective route tracking label available: `0`

## Summary

- All retrospective directions match: `1`
- All historical/aligned slots covered: `1 / 1`
- Route executed / tracking read: `0 / 0`
- Validation claim allowed: `0`

## Evidence boundary

V75 reads only the current local LMB posteriors and link probabilities at the two opened V72/V73 anchors. For each exact receiver--incumbent--candidate replacement it computes a covariance-normalized two-dimensional position innovation over shared labels, weighted by receiver existence and the smaller sender existence. The gate uses the conditional maximum slot energy; reliability-weighted energy is diagnostic only. The 5.991 scale is the conventional 95-percent chi-square reference for two dimensions, but it was selected after inspecting these opened anchors and is development evidence, not a calibrated p-value or validation preregistration. No route, truth, future measurement, tracking outcome, or model training is used.
