# V121 exact-budget mixed carrier insertion: X36 t72 H=8

- V113/V114 endpoints reused: `1 / 1`
- V121 candidate screen reused: `0`
- Any candidate passed: `0`
- Oracle action: `v121-insert-f6-after-f4-h8`

| Insert F6 after | Order | Mean E-OSPA | Gain vs CW | vs fixed CW | vs V114 | Mature min | Min form. | Terminal form. | F6 peers | Worst | Bytes | Gate |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| F1 | `F1→F6→F2→F3→F4→F5` | 78.782991 | +3.692% | -0.386% | -0.592% | +1.596% | -6.219% | -12.605% | -13.886% | +7.931% | +3.394% | 0 |
| F2 | `F1→F2→F6→F3→F4→F5` | 79.255185 | +3.115% | -0.988% | -1.195% | +2.377% | -13.151% | -27.851% | -23.731% | -20.571% | +5.826% | 0 |
| F3 | `F1→F2→F3→F6→F4→F5` | 79.705439 | +2.565% | -1.562% | -1.770% | +0.416% | -13.029% | -19.623% | -21.074% | -11.474% | +4.977% | 0 |
| F4 | `F1→F2→F3→F4→F6→F5` | 78.812743 | +3.656% | -0.424% | -0.630% | +1.951% | -6.439% | -9.751% | -4.979% | -7.113% | +3.804% | 0 |

## Oracle diagnostics

- Formation gains: `[2.558 0.9785 10.62 4.576 9.913 -6.439]%`
- Terminal formation gains: `[10.1 4.858 16.38 10.84 6.645 -9.751]%`
- Per-page gains: `[0.1819 2.291 1.246 1.951 2.111 7.389 7.756 6.853]%`
- F6 gateway sensor: `32`
- Window / terminal consensus: `+7.287% / +10.813%`
- Message / weights / sensor cycle / formation cycle / rolling B3: `1 / 1 / 1 / 1 / 1`
- Gate passed: `0`

## Evidence boundary

V121 is a privileged opened-development X36 seed-211 t=72 H=8 mixed-carrier action-space screen. It retains the relative clockwise order of F1--F5 and relocates F6 after F1, F2, F3 or F4 for the full horizon. Each arm reuses the clockwise sensor-level cut gateways and reconnects exactly six residual cross-formation tokens into one Hamiltonian formation cycle. The F2--F5 abstention schedule, 60-message budget, fusion-weight multiset, measurements, delivery uniforms, filter RNG and communication accounting remain paired. V113 fixed clockwise and V114 early F6 shield outcomes are reused. The candidate must beat both and pass mature-page, formation-tail, consensus, byte, physicality and rolling-B3 gates. Candidate orders were chosen after V120 opened outcomes, so V121 is not deployable, validation or generalization evidence.
