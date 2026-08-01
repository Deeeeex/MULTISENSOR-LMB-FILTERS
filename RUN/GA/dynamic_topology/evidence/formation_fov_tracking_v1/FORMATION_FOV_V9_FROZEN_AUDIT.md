# Formation-FoV v9 frozen audit

## Decision

The old v9 value-gated joint source-trust policy **fails** the registered
formation-FoV gate at both M24 and X36.  M24 has a positive aggregate tracking
mean but fails seed stability and tail safety.  X36 is worse than the fixed
reference in aggregate tracking, tail tracking and consensus, while also
missing the two-percent communication-saving gate.  The v9 line is therefore
retired as a final method; these results are retained as frozen negative
evidence and are not used to retune the current H=3 teacher protocol.

## Provenance and boundary

- Generation commit for every arm: `106c0508d3585bf9592c71c0f032affec31a20b4`.
- Paired seeds: `83, 89, 97, 101, 103`.
- M24 continuation starts at `t=70`; X36 continuation starts at `t=60`.
- Every reference/candidate pair shares the same cached predecision posterior,
  measurement stream, link randomness and filter randomness.
- Results are conditional continuation-window evidence, not full-episode
  performance.
- Mixture-aware fusion uses the repository componentwise powered-GM KLA
  approximation; it is not exact arbitrary-mixture density exponentiation.

All percentages below use the fixed residual-CCW reference as denominator;
positive values mean improvement or byte saving.

## Per-seed paired results

### M24

| Seed | Mean tracking | Worst node | Consensus | Attempted bytes | Delivered bytes |
|---:|---:|---:|---:|---:|---:|
| 83  | -2.248% | -10.697% | +3.498% | +2.707% | +3.108% |
| 89  | +7.217% | -7.443%  | +3.157% | +2.814% | +3.085% |
| 97  | +7.927% | +2.357%  | +6.661% | +2.212% | +1.955% |
| 101 | +16.304% | -4.198% | +4.054% | +3.656% | +3.179% |
| 103 | +14.062% | -0.029% | +0.847% | +1.525% | +1.588% |

### X36

| Seed | Mean tracking | Worst node | Consensus | Attempted bytes | Delivered bytes |
|---:|---:|---:|---:|---:|---:|
| 83  | +0.923% | +4.359% | +0.485% | +0.911% | +1.029% |
| 89  | +1.114% | +7.054% | +2.996% | +2.567% | +2.268% |
| 97  | -3.351% | -3.744% | -4.789% | +0.893% | +0.597% |
| 101 | -0.538% | -7.059% | +0.338% | +2.688% | +3.121% |
| 103 | -2.697% | -5.111% | -7.623% | +1.465% | +1.524% |

## Aggregate results

The aggregate is the arithmetic mean of each per-seed metric, matching the
repository screen aggregator.

| Scale / arm | Mean E-OSPA | Worst-node E-OSPA | MAP-set disagreement | Attempted bytes | Delivered bytes |
|---|---:|---:|---:|---:|---:|
| M24 reference | 28.6459 | 64.9233 | 31.1507 | 42,928,891 | 41,036,712 |
| M24 v9 | 26.1100 | 67.4272 | 30.0440 | 41,818,635 | 39,975,558 |
| X36 reference | 77.2230 | 92.3963 | 63.3199 | 76,826,146 | 74,451,085 |
| X36 v9 | 77.9127 | 93.2273 | 64.3420 | 75,519,507 | 73,181,339 |

| Registered readout | M24 | X36 |
|---|---:|---:|
| Aggregate mean-tracking gain | **+8.852%** | **-0.893%** |
| Positive seeds | **4 / 5** | **2 / 5** |
| Aggregate worst-node gain | **-3.857%** | **-0.899%** |
| Aggregate consensus gain | **+3.553%** | **-1.614%** |
| Attempted-byte saving | **+2.586%** | **+1.701%** |
| Delivered-byte saving | **+2.586%** | **+1.705%** |
| Truth / repair / emergency / infeasible violations | 0 | 0 |
| Overall gate | **FAIL** | **FAIL** |

## Gate diagnosis

| Gate | Requirement | M24 | X36 |
|---|---|---:|---:|
| Mean tracking | at least +5% | pass | fail |
| Seed stability | 5 / 5 positive | fail | fail |
| Worst node | no aggregate regression | fail | fail |
| Consensus | no aggregate regression | pass | fail |
| Attempted bytes | at least 2% saving | pass | fail |
| Per-seed communication | no seed exceeds reference by 0.5% | pass | pass |
| Hard runtime checks | all pass | pass | pass |

The important mechanism lesson is that a one-step posterior proxy can select
actions that save communication and satisfy graph/runtime checks while still
hurting recursive tracking, especially at X36.  A deployable selector must
learn multi-step value and jointly protect the network mean, formation tail,
sensor tail, consensus and both communication measures; explicit topology and
fallback constraints remain necessary but are not sufficient.
