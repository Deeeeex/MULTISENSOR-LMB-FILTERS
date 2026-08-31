# V198 temporal observation-supported repair finding

## Short-horizon preservation

V198 adds one causal distinction to V197: cooldown expiry makes a repair
token available, but does not authorize spending it.  Before a token is
spent, label support is aggregated over the current and immediately preceding
local-posterior pages, including currently reachable cross-edge senders.

The frozen H=3 paired runs preserve the independently useful actions exactly:

- M24 releases `{F4, empty, empty}` and improves E-OSPA, RMSE, consensus and
  attempted bytes by `+7.521%`, `+10.620%`, `+17.429%` and `+3.562%`;
- X36 releases `{F2, empty, empty}` and improves the same four metrics by
  `+2.181%`, `+0.589%`, `+2.648%` and `+5.397%`.

The policy receives no truth, future measurement, future outcome, numeric
formation feature or result feedback.

## X36 H=8 result

On the frozen X36 seed-211, t=72, H=8 window, V198 releases only F2:
`{F2, empty, empty, empty, empty, empty, empty, empty}`.  It therefore removes
the harmful F3 and F5 releases made by V197 without changing the carrier,
KLA weights, message builder or byte ledger.

| Arm | Mean E-OSPA | E-OSPA gain | Mean RMSE | RMSE gain | Consensus gain | Byte saving |
|:--|--:|--:|--:|--:|--:|--:|
| Static full payload | 84.037151 | 0 | 59.967347 | 0 | 0 | 0 |
| V99 base | 79.451115 | +5.457% | 62.172152 | -3.677% | +8.803% | +5.423% |
| V197 cooldown-only repair | 79.652782 | +5.217% | 61.870974 | -3.174% | +7.657% | +4.886% |
| V198 temporal-support repair | 79.535696 | +5.357% | 60.832025 | -1.442% | +7.809% | +5.341% |

Relative to V99, V198 improves mean RMSE by `+2.156%` while adding only
`23,384 B`; its mean E-OSPA and consensus change by `-0.106%` and `-1.090%`.
Relative to V197, suppressing the later releases also improves E-OSPA, RMSE,
consensus and communication.

The remaining formation RMSE gains are
`[+1.847%, -2.720%, -29.799%, +1.789%, -2.561%, -0.749%]`.  Formation 3 is
now the dominant failure even though the temporal support test abstains from
restoring it.  Presence or recent observation support is therefore not an
accuracy certificate: a label can remain supported while its spatial density
is still too inaccurate for the downstream recursive estimate.

## Method decision

V198 validates temporal abstention as a controller component, but closes the
formation-level full-posterior-only branch.  Do not tune the opened support
threshold, history depth, cooldown or token count.  The next action space must
contain, for every risky formation and page:

1. keep the V99 omission unchanged;
2. release the formation and restore its ordinary full posterior; or
3. transmit one complete source label and apply residual label-wise KLA.

The first controlled H=8 teacher should replace the V198 F2 formation release
with the already identified source-19 label `[13,12]`, using the frozen
0.5-weight residual KLA operator and the same V99 state, link realization and
byte ledger.  This isolates action granularity before any online ranker is
implemented.  If the finer action has long-horizon headroom, an online
three-action selector may use only causal risk reduction, precision gain,
source evidence quality, motion compatibility and temporal support.  Truth
remains an offline target only.

V198 does not replace the current balanced best V187 and does not enter the
main progress document as a new best result.

