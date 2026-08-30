# V189 propagation-trust preflight

This truth-free replay checks whether each already-opened V188 source and all intended receivers remain within the registered position cutoff across the H=3 recursive window. It does not score tracking performance.

## M24 seed 211 t=104

- preset `m24-formation-fov`, seed `211`, t=`104`, cutoff `150.000 m`
- V188-selected proposal(s): `2`; selected trust result: `reject`; maximum distance `172.395 m`

| proposal | formation | source | label | V188 selected | max distance (m) | cutoff ratio | trust | preflight bytes | accepted total bytes |
|---:|---:|---:|---|---|---:|---:|---|---:|---:|
| 1 | 1 | 10 | `[25,15]` | no | 166.743 | 1.112 | reject | 112 | 20656 |
| 2 | 2 | 3 | `[25,15]` | yes | 172.395 | 1.149 | reject | 112 | 16624 |
| 3 | 3 | 3 | `[25,15]` | no | 170.638 | 1.138 | reject | 112 | 16624 |
| 4 | 4 | 10 | `[25,15]` | no | 178.562 | 1.190 | reject | 112 | 20656 |

## X36 seed 211 t=72

- preset `x36-formation-fov`, seed `211`, t=`72`, cutoff `150.000 m`
- V188-selected proposal(s): `3`; selected trust result: `pass`; maximum distance `68.134 m`

| proposal | formation | source | label | V188 selected | max distance (m) | cutoff ratio | trust | preflight bytes | accepted total bytes |
|---:|---:|---:|---|---|---:|---:|---|---:|---:|
| 1 | 1 | 20 | `[19,16]` | no | 198.816 | 1.325 | reject | 112 | 20656 |
| 2 | 2 | 20 | `[19,16]` | no | 49.991 | 0.333 | pass | 112 | 20656 |
| 3 | 3 | 29 | `[25,18]` | yes | 68.134 | 0.454 | pass | 112 | 20656 |
| 4 | 4 | 34 | `[31,24]` | no | 69.538 | 0.464 | pass | 112 | 20656 |
| 5 | 5 | 20 | `[7,7]` | no | 95.694 | 0.638 | pass | 112 | 20656 |
| 6 | 6 | 7 | `[13,10]` | no | 87.380 | 0.583 | pass | 112 | 20656 |

## Finding boundary

This causal replay certifies only open-loop positional compatibility of already opened V188 proposals. Passing is a necessary safety condition, not evidence of recursive tracking improvement.
