# V259 dense risk-alignment result

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`
- Trace commit: `c34983dcd0a08f7576ea7d48f6b15754c3f14518`
- Passive V242 reproduction: `1`
- Localization / support / byte screen: `1 / 1 / 1`
- Bounded causal action screen authorized: `1`

## Dense alignment

| Observable decision | Result |
|:--|:--|
| Localization entry (`tail >= 0.25`, `tail / formation-median >= 2.5`) | first selects F4 at `t=57`; F4 is selected on `100%` of `t=58--73`; all-page duty `26.9%` |
| Support entry (`prominence >= 0.20`, `coverage/count deficit >= 0.25`) | selects F1 on `100%` of `t=131--160`; agreement with the largest future-H=3 cardinality-error formation is `100.0%` |
| Localization diagnostic precision | selected formation matches the largest future-H=3 RMSE formation on `53.5%` of trigger pages; this is a candidate-generation signal, not an action-value guarantee |

## Passive communication headroom

| Quantity | Value |
|:--|--:|
| Strong localization trigger pages | 43 / 160 |
| Period-2 residual-bundle pulse pages | 23 |
| Residual edges per pulse | 4--5 |
| F4 event passive incremental bytes | 185552 |
| Full-trace passive incremental bytes | 550432 |
| Fraction of V242-to-static byte headroom used | 13.446% |
| Passive projected saving versus static | 8.691% |

## Method decision

The risk representation passes the opened-seed action-space screen. It detects the F4 localization event one page before the registered interval and keeps F4 first throughout the interval. The earlier one-edge action is therefore replaced by a formation-level residual bundle pulsed every two pages. This restores four or five omitted local inputs together, matching the systemic spatial failure while consuming only a bounded part of the static-route byte headroom on the passive trajectory.

Support risk remains a separate zero-message gateway-re-embedding action family. It must not be mixed with localization risk or automatically fired from this report: finite-horizon mixture-aware value and fallback still decide whether any action is executed. The next experiment compares a period-2 mass-matched pulse with a bounded strong pulse from the first causal trigger, reusing the V242 continuation state.

## Evidence boundary

This is an outcome-opened development alignment screen. The online risk traces use current posteriors only, while event alignment and future H=3 precision use recorded truth offline. Passive pulse bytes are evaluated on the unchanged V242 posterior trajectory and do not include recursive payload-size changes. The result authorizes a bounded causal action screen, not a tracking or generalization claim.
