# V267 asymmetric eta retention

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`
- Generation commit: `53beeded2591e07e426408751a8ef08aa38acbf4`
- Continuation: `t=57--73` from the V259 V242 state
- Label-route active / delivered / applied pages: `[57 58 59 60 61 62 63 64 65 66 67] / [57 59 60 62 63 64 65 66 67] / [57 59 65]`
- Label payload attempted / delivered bytes: `9352 / 8064`
- Mechanism / full-M24 authorization: `0 / 0`

| Arm | E-OSPA | RMSE | Consistency | F4 event E / RMSE | Window bytes | Spliced static saving |
|:--|--:|--:|--:|:--|--:|--:|
| V242 minimum backbone | 118.651 | 16.844 | 130.254 | 123.683 / 26.636 | 2991480 | 10.041% |
| V267 asymmetric eta retention | 118.700 | 15.912 | 130.178 | 123.839 / 22.722 | 3027328 | 9.953% |

## Gains over V242

| Scope | E-OSPA | RMSE |
|:--|--:|--:|
| Network | `-0.041%` | `+5.536%` |
| F4 event | `-0.126%` | `+14.694%` |
| Weakest formation | `-0.118%` | `-0.108%` |

- Consistency gain: `+0.058%`
- Window byte change vs V242: `-1.198%`
- Short-horizon gate: `0`
- Next method decision: `inspect-source-value-or-packet-level-routing-after-support-protection`

## Receiver projection diagnostics

| t | Source to receiver | Selected share | Source r / risk | Target / donor risk | Reference / candidate r | Change in log odds | log eta | Outcome |
|--:|:--|--:|:--|:--|--:|--:|--:|:--|
| 57 | S2 to S13 | 0.05000 | 0.915 / 0.009 | 0.273 / 0.008 | 0.234 / 0.258 | +0.127 | -0.051 | accepted |
| 58 | S1 to S13 | 0.00000 | 0.838 / 0.009 | 0.311 / 0.009 | NaN / NaN | +NaN | NaN | transport-drop |
| 59 | S1 to S13 | 0.05000 | 0.977 / 0.008 | 0.355 / 0.008 | 0.272 / 0.272 | +0.003 | -0.243 | accepted |
| 60 | S2 to S13 | 0.00000 | 0.995 / 0.007 | 0.384 / 0.007 | 0.284 / 0.264 | -0.098 | -0.274 | no-source-weight-passed |
| 61 | S3 to S13 | 0.00000 | 0.997 / 0.007 | 0.432 / 0.007 | NaN / NaN | +NaN | NaN | transport-drop |
| 62 | S3 to S13 | 0.00000 | 0.989 / 0.008 | 0.456 / 0.006 | 0.345 / 0.244 | -0.490 | -0.551 | no-source-weight-passed |
| 63 | S6 to S13 | 0.00000 | 0.999 / 0.004 | 0.522 / 0.005 | 0.376 / 0.193 | -0.927 | -0.981 | no-source-weight-passed |
| 64 | S5 to S13 | 0.00000 | 0.991 / 0.005 | 0.588 / 0.006 | 0.304 / 0.168 | -0.773 | -0.824 | no-source-weight-passed |
| 65 | S5 to S13 | 0.01250 | 0.997 / 0.004 | 0.647 / 0.006 | 0.258 / 0.271 | +0.065 | -0.050 | accepted |
| 66 | S5 to S13 | 0.00000 | 0.990 / 0.005 | 0.718 / 0.007 | 0.234 / 0.167 | -0.421 | -0.514 | no-source-weight-passed |
| 67 | S5 to S13 | 0.00000 | 0.985 / 0.005 | 0.800 / 0.009 | 0.254 / 0.167 | -0.524 | -0.579 | no-source-weight-passed |

## Decision

The causal label-only action does not pass the joint mechanism gate. Follow the registered next-method decision before introducing a learned ranker.

## Evidence boundary

V267 is an outcome-opened M24 continuation mechanism screen. It keeps the V242 backbone, V265 causal route selection and V266 frozen source-share grid. The only change is the intended asymmetric existence-retention semantics: an ordinary MAP-negative label may gain support but may not lose additional log odds, while a MAP-positive label retains the registered 0.25 drop allowance and cannot cross the 0.5 extraction threshold. Every attempted complete-label payload remains charged and centralized risk-synopsis cost remains excluded. No truth or future outcome is visible to the controller, so the result is mechanism evidence only.
