# V258 event-window residual reinforcement

- Preset: `m24-formation-fov-temporal-coupled-formation-braid`
- Seed: `1301`
- Generation commit: `7e3b8c32657cfb5734b83f405d5d0f56998c8d2a`
- Posthoc event / formation: `58--73 / 4`
- Balanced direction over fixed passed: `1`
- Tail repair over V242 passed: `0`
- Mechanism supported: `0`
- Paper threshold passed: `0` (mechanism evidence only)

| Arm | Full E-OSPA | Full RMSE | Focus consistency | Attempted bytes | Messages / step |
|:--|--:|--:|--:|--:|--:|
| Fixed formation tree | 125.478 | 22.640 | 133.599 | 40769168 | 46--48 |
| Full causal repair | 122.380 | 14.081 | 131.913 | 44867136 | 48--48 |
| Minimum causal backbone | 122.462 | 12.183 | 131.664 | 36675624 | 30--30 |
| V258 event reinforcement | 122.557 | 11.922 | 132.423 | 37579520 | 30--31 |

## V258 over fixed tree

| Metric | Gain |
|:--|--:|
| Full E-OSPA | `+2.328%` |
| Full RMSE | `+47.341%` |
| Focus consistency | `+0.881%` |
| Attempted-byte saving | `+7.824%` |
| Weakest formation E-OSPA | `-0.391%` |
| Weakest formation RMSE | `-23.851%` |

## V258 over V242 minimum backbone

| Metric | Gain |
|:--|--:|
| Full E-OSPA | `-0.077%` |
| Full RMSE | `+2.140%` |
| Focus consistency | `-0.577%` |
| Attempted-byte saving | `-2.465%` |
| Weakest formation E-OSPA | `-0.665%` |
| Weakest formation RMSE | `+0.188%` |

## Decision

The direct tail-repair hypothesis is rejected. V258 selected one local
residual on only `1` of the `16` posthoc event pages. Although the
complete-episode RMSE improved by `+2.140%` relative to V242, the diagnosed
formation-4 event RMSE changed by `-2.122%`, while full E-OSPA and focus
consistency changed by `-0.077%` and `-0.577%`. The isolated intervention
therefore has delayed, sign-changing recursive effects rather than repairing
the active localization tail. Do not tune this posthoc window or promote
V258. The next controller must separate localization-tail risk from
label-support risk and estimate finite-horizon action value; the V242 backbone
remains the fallback.

## Diagnosed event: formation 4, t=58--73

| Arm | Event E-OSPA | Event RMSE | Absolute cardinality error | Peak time-mean RMSE |
|:--|--:|--:|--:|--:|
| Fixed formation tree | 129.958 | 7.288 | 12.000 | 8.343 |
| Full causal repair | 129.958 | 7.288 | 12.000 | 8.343 |
| Minimum causal backbone | 123.683 | 26.636 | 10.677 | 32.331 |
| V258 event reinforcement | 123.425 | 27.201 | 10.625 | 33.359 |

V258 over V242 in the diagnosed event: E-OSPA `+0.209%`, RMSE `-2.122%`,
cardinality `+0.488%`, peak RMSE `-3.181%`.
## Runtime selection

- Event pages selecting one residual: `0.062`
- Selected residual times: `59`
- Mean / maximum policy seconds: `0.965 / 9.459`
- Current-step Pareto guard passed on every page: `1`

## Evidence boundary

V258 is a falsification-oriented mechanism probe on the already opened V248 M24 seed. Its time window (58--73) and receiver formation (4) were selected after inspecting V248 tracking errors. Within that fixed window it may restore at most one currently physical V240 local residual input into the V242 minimum backbone, using the V246 current-posterior one-round Pareto guard; outside the window it is exactly V242. The executed policy reads no truth or future outcomes, but the posthoc schedule makes the result mechanism evidence only, not a deployable causal method or validation claim.
