# V79 balanced recovery operator

- Source mechanism gate passed: `0`
- Tracking outcome available: `0`

## m24-formation-fov-merge-split / t=80

- Nodes / formations: `24 / 4`
- Selected balanced phase pair: `[2 2]`
- Reference centered norm / column deviation: `1.634147 / 2.786743`
- Balanced peak / terminal linear bound: `0.995666 / 0.991186`
- Exact message-count parity: `1`

### Historical exact V71/V72 route

- Applied slot triples: `[14 10 21;17 19 3]`
- Round-1 RIE safe / maximum: `1 / 3.298216`

| Recovery | Centered energy by round | Existence-centered | Spatial-centered | Common energy | Peak | Terminal | Monotone |
|:--|:--|:--|:--|:--|--:|--:|:--:|
| CRR | `[0.0050363425 0.0055767847 0.0040530802]` | `[0.0047328524 0.0054848932 0.0037567671]` | `[0.00030349019 9.1891479e-05 0.00029631319]` | `[0.00012434649 0.00018425346 0.00082432292]` | 1.107308 | 0.804767 | 0 |
| CBB | `[0.0050363425 0.042401057 0.042279715]` | `[0.0047328524 0.013881537 0.014831339]` | `[0.00030349019 0.02851952 0.027448376]` | `[0.00012434649 0.0062984679 0.0071836172]` | 8.419018 | 8.394924 | 0 |

- CBB peak / terminal factor improvement: `-7.311709 / -7.590158`

### Prospective exact V73 route

- Applied slot triples: `[14 10 21;17 19 20]`
- Round-1 RIE safe / maximum: `1 / 3.298216`

| Recovery | Centered energy by round | Existence-centered | Spatial-centered | Common energy | Peak | Terminal | Monotone |
|:--|:--|:--|:--|:--|--:|--:|:--:|
| CRR | `[0.0040008815 0.0042128558 0.0041649305]` | `[0.0038586327 0.0041368503 0.0038590339]` | `[0.0001422488 7.6005436e-05 0.00030589661]` | `[0.00017125992 0.00034206267 0.00086866041]` | 1.052982 | 1.041003 | 0 |
| CBB | `[0.0040008815 0.040378707 0.044089563]` | `[0.0038586327 0.011822462 0.016081791]` | `[0.0001422488 0.028556245 0.028007771]` | `[0.00017125992 0.0062158672 0.0074253529]` | 11.019962 | 11.019962 | 0 |

- CBB peak / terminal factor improvement: `-9.966980 / -9.978959`

## x36-formation-fov-merge-split / t=52

- Nodes / formations: `36 / 6`
- Selected balanced phase pair: `[2 2]`
- Reference centered norm / column deviation: `1.641035 / 2.786744`
- Balanced peak / terminal linear bound: `0.998002 / 0.995970`
- Exact message-count parity: `1`

### Historical exact V71/V72 route

- Applied slot triples: `[20 16 34;23 25 35]`
- Round-1 RIE safe / maximum: `1 / 1.096073`

| Recovery | Centered energy by round | Existence-centered | Spatial-centered | Common energy | Peak | Terminal | Monotone |
|:--|:--|:--|:--|:--|--:|--:|:--:|
| CRR | `[0.0017982536 0.0014470601 0.0028974039]` | `[0.00170656 0.0013923478 0.0027795802]` | `[9.1693691e-05 5.4712262e-05 0.0001178237]` | `[5.1629983e-05 5.7355868e-05 0.00037833125]` | 1.611232 | 1.611232 | 0 |
| CBB | `[0.0017982536 0.012425245 0.018541495]` | `[0.00170656 0.0066378165 0.0099472461]` | `[9.1693691e-05 0.0057874284 0.0085942491]` | `[5.1629983e-05 0.00075518319 0.0012639122]` | 10.310834 | 10.310834 | 0 |

- CBB peak / terminal factor improvement: `-8.699602 / -8.699602`

### Prospective exact V73 route

- Applied slot triples: `[20 16 34;23 25 36]`
- Round-1 RIE safe / maximum: `1 / 1.096073`

| Recovery | Centered energy by round | Existence-centered | Spatial-centered | Common energy | Peak | Terminal | Monotone |
|:--|:--|:--|:--|:--|--:|--:|:--:|
| CRR | `[0.0019874828 0.0014755969 0.0028995362]` | `[0.0018956884 0.0014208729 0.0027817134]` | `[9.1794374e-05 5.4724008e-05 0.00011782276]` | `[5.7497226e-05 5.9617392e-05 0.00037845346]` | 1.458899 | 1.458899 | 0 |
| CBB | `[0.0019874828 0.012456041 0.01851847]` | `[0.0018956884 0.0066695275 0.0099247358]` | `[9.1794374e-05 0.0057865133 0.0085937338]` | `[5.7497226e-05 0.00075168283 0.0012648068]` | 9.317550 | 9.317550 | 0 |

- CBB peak / terminal factor improvement: `-7.858651 / -7.858651`

## Evidence boundary

V79 keeps the V75-safe candidate pulse in round one but replaces the two fixed-star recovery rounds by a structurally selected balanced recovery pair. Each recovery round retains the same self, dominant, and residual weights and the same directed-message count. The residual Hamiltonian tour is unchanged; only the within-formation dominant star is replaced by a physical balanced cycle. Candidate phase pairs are selected from current physical links and current link reliabilities by the centered spectral bounds of the one- and two-round expected fusion operators. No posterior, measurement, truth, future link, packet draw, route execution, tracking outcome, or model training enters recovery construction. The source-only replay uses formal mixture-aware heavy receivers and diagnoses mechanism headroom only.
