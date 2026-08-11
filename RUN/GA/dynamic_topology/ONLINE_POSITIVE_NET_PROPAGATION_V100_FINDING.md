# V100 finding: propagation delay is real but not sufficient

## Matched H=6 result

The X36 formation-level effective graph is a directed six-cycle with diameter
five.  V100 therefore evaluates one action step plus five propagation steps
without changing V99's spatial selector.

| Arm | Mean E-OSPA | Gain over static | Attempted-byte saving |
|:--|--:|--:|--:|
| Static full payload | 84.581111 | -- | -- |
| Fixed initial set | 81.443184 | +3.710% | +3.936% |
| Online V100 | 80.807326 | +4.462% | +5.250% |

Online re-selection improves fixed control by another 0.781%.  Its worst
sensor metric improves 13.668%, the weakest formation improves 0.165%, window
and terminal consensus improve 7.376% and 11.377%, and rolling B3 passes.  The
method nevertheless fails both registered efficacy conditions: six-step mean
gain is below 5%, and the minimum gain after formation-level propagation is
4.623%.

## Temporal evidence

| Time | Static | Online | Online/static |
|---:|---:|---:|---:|
| 72 | 86.118620 | 85.071354 | +1.216% |
| 73 | 85.408155 | 83.601644 | +2.115% |
| 74 | 86.384056 | 82.011795 | +5.061% |
| 75 | 85.605271 | 80.345140 | +6.145% |
| 76 | 82.342302 | 78.535347 | +4.623% |
| 77 | 81.628263 | 75.278676 | +7.779% |

The gain persists beyond t=74 but fluctuates below 5% at t=76.  H=3 was too
short to expose the full response, yet a graph-diameter-matched window still
does not establish stable significant improvement.  Longer windows would now
be outcome-driven tuning and are closed.

## Spatial evidence

The formation gains are `[2.265, 2.866, 5.512, 8.367, 7.942, 0.165]%`.
Online re-selection adds material benefit over the fixed set mainly in
formation 3, adds only 0.165% in formation 6, and is 0.894% worse than the
fixed set in formation 5.  Several non-gateway members of formations 1--3
also regress slightly even though the worst-sensor and formation-average gates
pass.  The remaining error is therefore spatially structured, not a missing
global duration.

## Decision

Do not extend the horizon and do not open the four-anchor or new-scene runs.
The binary receiver-formation action is too coarse: it either admits or
suppresses every complete cross-formation label posterior arriving at a
gateway.  The next headroom test should decompose that action using the signed
effect of each sender-label input on the receiver's KLA log odds and spatial
agreement, then combine it with the already validated online formation
schedule.

This is distinct from V62's failed `sender-supported-only` exception.  V62
kept labels based on sender association support and collapsed a 10.393% M24
gain to 1.694%.  A successor must use signed receiver--sender--label influence
under the current KLA counterfactual, retain complete Gaussian mixtures for
selected labels, and fall back to the control-only action unless the exception
has positive predicted net value.

The exact V100 online arm takes 1015.21 seconds versus roughly 183 seconds for
the static and fixed arms.  It remains a teacher/oracle; deployment requires a
cheaper learned or analytic approximation after headroom is demonstrated.
