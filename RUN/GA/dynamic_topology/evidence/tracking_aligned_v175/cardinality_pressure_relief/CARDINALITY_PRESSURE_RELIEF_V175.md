# V175 cardinality-pressure relief preflight

- Preset / seed: `x36-formation-fov / 211`
- Synopsis bytes per raw source-label: `24`
- Policy gate passed: `1`

| t | F | Receiver | Pressure | Selected | First R | Second E/R | Safety | Bytes |
|--:|--:|--:|:--:|:--:|--:|:--|--:|--:|
| 76 | 3 | 13 | 0 | 0 | +6.606 | +0.000/+0.000 | NaN | 22920 |
| 76 | 3 | 14 | 0 | 0 | +6.220 | +0.000/+0.000 | NaN | 22944 |
| 76 | 3 | 15 | 0 | 0 | +0.594 | +0.000/+0.000 | NaN | 22944 |
| 76 | 3 | 16 | 0 | 0 | +6.895 | +0.000/+0.000 | NaN | 22944 |
| 76 | 3 | 17 | 0 | 0 | +6.401 | +0.000/+0.000 | NaN | 22944 |
| 76 | 3 | 18 | 0 | 0 | +6.522 | +0.000/+0.000 | NaN | 22920 |
| 78 | 3 | 13 | 0 | 0 | +0.424 | +0.000/+0.000 | NaN | 23184 |
| 78 | 3 | 14 | 0 | 0 | +0.402 | +0.000/+0.000 | NaN | 23184 |
| 78 | 3 | 15 | 0 | 0 | +140.221 | +0.000/+0.000 | NaN | 23184 |
| 78 | 3 | 16 | 0 | 0 | +0.439 | +0.000/+0.000 | NaN | 23184 |
| 78 | 3 | 17 | 0 | 0 | +0.435 | +0.000/+0.000 | NaN | 23184 |
| 78 | 3 | 18 | 0 | 0 | +0.434 | +0.000/+0.000 | NaN | 23184 |
| 78 | 5 | 25 | 1 | 1 | +0.251 | +0.102/+0.910 | 0.875 | 24664 |
| 78 | 5 | 26 | 1 | 1 | +0.197 | +0.106/+0.873 | 0.851 | 24664 |
| 78 | 5 | 27 | 1 | 1 | +0.240 | +0.084/+0.619 | 0.890 | 24664 |
| 78 | 5 | 28 | 1 | 1 | +0.248 | +0.101/+0.731 | 0.879 | 24664 |
| 78 | 5 | 29 | 1 | 1 | +0.267 | +0.144/+0.998 | 0.886 | 24664 |
| 78 | 5 | 30 | 1 | 1 | +0.256 | +0.122/+0.895 | 0.870 | 24664 |
| 79 | 3 | 13 | 0 | 0 | +9.911 | +0.000/+0.000 | NaN | 23304 |
| 79 | 3 | 14 | 0 | 0 | +0.396 | +0.000/+0.000 | NaN | 23304 |
| 79 | 3 | 15 | 0 | 0 | +0.392 | +0.000/+0.000 | NaN | 23304 |
| 79 | 3 | 16 | 0 | 0 | +145.743 | +0.000/+0.000 | NaN | 23304 |
| 79 | 3 | 17 | 0 | 0 | +0.398 | +0.000/+0.000 | NaN | 23304 |
| 79 | 3 | 18 | 0 | 0 | +0.403 | +0.000/+0.000 | NaN | 23304 |
| 79 | 5 | 25 | 1 | 1 | +0.137 | +0.200/+1.905 | 0.626 | 26800 |
| 79 | 5 | 26 | 1 | 1 | +0.249 | +0.150/+1.174 | 0.666 | 26800 |
| 79 | 5 | 27 | 1 | 1 | +0.255 | +0.125/+1.449 | 0.753 | 26800 |
| 79 | 5 | 28 | 1 | 1 | +0.236 | +0.134/+1.105 | 0.746 | 26800 |
| 79 | 5 | 29 | 1 | 1 | +0.248 | +0.186/+1.485 | 0.748 | 26800 |
| 79 | 5 | 30 | 1 | 1 | +0.233 | +0.145/+1.212 | 0.677 | 26800 |

- F5 first / second / total RMSE gain: `2.817287 / 13.356331 / 16.173618`
- F5 total E-OSPA gain: `87.353614`
- Additional attempted bytes: `678800`
- Projected byte saving: `+0.083%`
- Projected formation RMSE gain: `[25.846 5.7725 14.275 1.789 0.030441 23.294]%`

## Evidence boundary

V175 is an opened, nonrecursive policy-composition screen. The first and conditional classifiers use only present posterior and topology summaries. A frozen high-cardinality-pressure gate enables a second action; the conditional classifier filters candidates and minimum source evidence quality is an analytic cardinality-relief tie-break. Truth only scores the chosen actions. Bytes conservatively charge a 24-byte packed synopsis for every raw physical-neighbor label, two sequential requests and both complete-label responses. Snapshot deltas are added to V162, so a pass only authorizes an actual recursive implementation and is not validation.
