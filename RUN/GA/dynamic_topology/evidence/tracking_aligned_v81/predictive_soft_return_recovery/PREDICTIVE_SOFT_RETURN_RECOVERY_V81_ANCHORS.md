# V81 predictive soft-return recovery

- Source mechanism gate passed: `0`
- Tracking outcome available: `0`

## m24-formation-fov-merge-split / t=80

- Nodes / formations: `24 / 4`
- Reference / balanced centered norm: `1.634147 / 0.995614`

### Historical exact V71/V72 route

- Applied slot triples: `[14 10 21;17 19 3]`
- Round-1 RIE safe / maximum: `1 / 3.298216`
- Selected recovery actions: `R -> R`
- Centered energy by round: `[0.0050363425 0.0055767847 0.0040530802]`
- Existence-centered by round: `[0.0047328524 0.0054848932 0.0037567671]`
- Spatial-centered by round: `[0.00030349019 9.1891479e-05 0.00029631319]`
- Peak / terminal factor / monotone: `1.107308 / 0.804767 / 0`

#### Recovery round 1

| Action | Type | Parameter | Centered | Existence | Spatial | Common |
|:--|:--|--:|--:|--:|--:|--:|
| R | reference | 0.000 | 0.0055767847 | 0.0054848932 | 9.1891479e-05 | 0.00018425346 |
| G02 | global-balance | 0.020 | 0.0055906021 | 0.0054908506 | 9.9751448e-05 | 0.00019366992 |
| G05 | global-balance | 0.050 | 0.0056603713 | 0.0055126824 | 0.00014768894 | 0.0002184379 |
| G10 | global-balance | 0.100 | 0.0059041227 | 0.0055831352 | 0.00032098746 | 0.00028574673 |
| L10 | local-soft-return | 0.100 | 0.0099429446 | 0.009633025 | 0.00030991961 | 0.00061969313 |
| L25 | local-soft-return | 0.250 | 0.0094248495 | 0.0091759454 | 0.00024890416 | 0.00058103381 |
| L50 | local-soft-return | 0.500 | 0.0076286961 | 0.0074572665 | 0.00017142962 | 0.00044683673 |
| L75 | local-soft-return | 0.750 | 0.0057754109 | 0.0056556551 | 0.00011975577 | 0.00024248338 |

- Selected: `R`

#### Recovery round 2

| Action | Type | Parameter | Centered | Existence | Spatial | Common |
|:--|:--|--:|--:|--:|--:|--:|
| R | reference | 0.000 | 0.0040530802 | 0.0037567671 | 0.00029631319 | 0.00082432292 |
| G02 | global-balance | 0.020 | 0.0040741484 | 0.0037766568 | 0.00029749158 | 0.00082204049 |
| G05 | global-balance | 0.050 | 0.0041247285 | 0.0038090627 | 0.00031566573 | 0.00082287278 |
| G10 | global-balance | 0.100 | 0.004254133 | 0.0038704606 | 0.00038367244 | 0.00083365912 |
| L10 | local-soft-return | 0.100 | 0.0064401503 | 0.0058529367 | 0.00058721361 | 0.0014809623 |
| L25 | local-soft-return | 0.250 | 0.0055539567 | 0.0050391278 | 0.00051482887 | 0.0012954486 |
| L50 | local-soft-return | 0.500 | 0.0042823382 | 0.0038692475 | 0.00041309071 | 0.00095923229 |
| L75 | local-soft-return | 0.750 | 0.0041228387 | 0.0037817627 | 0.00034107603 | 0.00085580811 |

- Selected: `R`

### Prospective exact V73 route

- Applied slot triples: `[14 10 21;17 19 20]`
- Round-1 RIE safe / maximum: `1 / 3.298216`
- Selected recovery actions: `R -> R`
- Centered energy by round: `[0.0040008815 0.0042128558 0.0041649305]`
- Existence-centered by round: `[0.0038586327 0.0041368503 0.0038590339]`
- Spatial-centered by round: `[0.0001422488 7.6005436e-05 0.00030589661]`
- Peak / terminal factor / monotone: `1.052982 / 1.041003 / 0`

#### Recovery round 1

| Action | Type | Parameter | Centered | Existence | Spatial | Common |
|:--|:--|--:|--:|--:|--:|--:|
| R | reference | 0.000 | 0.0042128558 | 0.0041368503 | 7.6005436e-05 | 0.00034206267 |
| G02 | global-balance | 0.020 | 0.0042301938 | 0.0041473696 | 8.2824189e-05 | 0.00034938596 |
| G05 | global-balance | 0.050 | 0.0043150293 | 0.0041856356 | 0.00012939368 | 0.00037169364 |
| G10 | global-balance | 0.100 | 0.0046015885 | 0.0043009151 | 0.00030067337 | 0.0004364571 |
| L10 | local-soft-return | 0.100 | 0.0089556529 | 0.0086614103 | 0.00029424258 | 0.0010081783 |
| L25 | local-soft-return | 0.250 | 0.0083493177 | 0.0081165752 | 0.00023274251 | 0.00093970088 |
| L50 | local-soft-return | 0.500 | 0.006425921 | 0.006273108 | 0.00015281307 | 0.00070601581 |
| L75 | local-soft-return | 0.750 | 0.0044564965 | 0.0043555431 | 0.00010095339 | 0.00043236575 |

- Selected: `R`

#### Recovery round 2

| Action | Type | Parameter | Centered | Existence | Spatial | Common |
|:--|:--|--:|--:|--:|--:|--:|
| R | reference | 0.000 | 0.0041649305 | 0.0038590339 | 0.00030589661 | 0.00086866041 |
| G02 | global-balance | 0.020 | 0.0041872232 | 0.0038794831 | 0.00030774003 | 0.00086582353 |
| G05 | global-balance | 0.050 | 0.00424044 | 0.003913478 | 0.00032696203 | 0.00086588962 |
| G10 | global-balance | 0.100 | 0.0043759067 | 0.0039790721 | 0.00039683463 | 0.00087563428 |
| L10 | local-soft-return | 0.100 | 0.0070747509 | 0.0064815064 | 0.00059324457 | 0.0016485844 |
| L25 | local-soft-return | 0.250 | 0.0060375686 | 0.0055159411 | 0.00052162753 | 0.0014330522 |
| L50 | local-soft-return | 0.500 | 0.0045534795 | 0.0041324102 | 0.00042106936 | 0.0010558372 |
| L75 | local-soft-return | 0.750 | 0.0042784986 | 0.0039285552 | 0.00034994333 | 0.00092081594 |

- Selected: `R`

## x36-formation-fov-merge-split / t=52

- Nodes / formations: `36 / 6`
- Reference / balanced centered norm: `1.641035 / 0.997658`

### Historical exact V71/V72 route

- Applied slot triples: `[20 16 34;23 25 35]`
- Round-1 RIE safe / maximum: `1 / 1.096073`
- Selected recovery actions: `G02 -> R`
- Centered energy by round: `[0.0017982536 0.0014367095 0.0028996494]`
- Existence-centered by round: `[0.00170656 0.0013798163 0.0027776071]`
- Spatial-centered by round: `[9.1693691e-05 5.6893143e-05 0.00012204229]`
- Peak / terminal factor / monotone: `1.612481 / 1.612481 / 0`

#### Recovery round 1

| Action | Type | Parameter | Centered | Existence | Spatial | Common |
|:--|:--|--:|--:|--:|--:|--:|
| R | reference | 0.000 | 0.0014470601 | 0.0013923478 | 5.4712262e-05 | 5.7355868e-05 |
| G02 | global-balance | 0.020 | 0.0014367095 | 0.0013798163 | 5.6893143e-05 | 5.8938398e-05 |
| G05 | global-balance | 0.050 | 0.001441944 | 0.001374283 | 6.7660935e-05 | 6.2608506e-05 |
| G10 | global-balance | 0.100 | 0.0015049947 | 0.0014011817 | 0.00010381307 | 7.2270959e-05 |
| L10 | local-soft-return | 0.100 | 0.0031905089 | 0.0029527227 | 0.00023778622 | 0.0001828694 |
| L25 | local-soft-return | 0.250 | 0.0028488174 | 0.0026716927 | 0.00017712464 | 0.00016358329 |
| L50 | local-soft-return | 0.500 | 0.0018804082 | 0.0017717743 | 0.00010863388 | 9.1866401e-05 |
| L75 | local-soft-return | 0.750 | 0.0016099716 | 0.0015394402 | 7.0531405e-05 | 6.5226287e-05 |

- Selected: `G02`

#### Recovery round 2

| Action | Type | Parameter | Centered | Existence | Spatial | Common |
|:--|:--|--:|--:|--:|--:|--:|
| R | reference | 0.000 | 0.0028996494 | 0.0027776071 | 0.00012204229 | 0.00037601486 |
| G02 | global-balance | 0.020 | 0.002902104 | 0.0027758398 | 0.00012626419 | 0.00037290638 |
| G05 | global-balance | 0.050 | 0.002924201 | 0.0027871446 | 0.00013705639 | 0.0003696104 |
| G10 | global-balance | 0.100 | 0.0030048338 | 0.0028400519 | 0.00016478193 | 0.00036749697 |
| L10 | local-soft-return | 0.100 | 0.0039357226 | 0.0036793936 | 0.00025632906 | 0.00053383395 |
| L25 | local-soft-return | 0.250 | 0.0035910346 | 0.0033733472 | 0.00021768737 | 0.00045124105 |
| L50 | local-soft-return | 0.500 | 0.0033224492 | 0.0031525129 | 0.00016993624 | 0.00040135185 |
| L75 | local-soft-return | 0.750 | 0.0030613247 | 0.0029221937 | 0.00013913099 | 0.00038500044 |

- Selected: `R`

### Prospective exact V73 route

- Applied slot triples: `[20 16 34;23 25 36]`
- Round-1 RIE safe / maximum: `1 / 1.096073`
- Selected recovery actions: `G05 -> R`
- Centered energy by round: `[0.0019874828 0.0014586633 0.0029110531]`
- Existence-centered by round: `[0.0018956884 0.0013910015 0.0027796161]`
- Spatial-centered by round: `[9.1794374e-05 6.7661764e-05 0.00013143702]`
- Peak / terminal factor / monotone: `1.464693 / 1.464693 / 0`

#### Recovery round 1

| Action | Type | Parameter | Centered | Existence | Spatial | Common |
|:--|:--|--:|--:|--:|--:|--:|
| R | reference | 0.000 | 0.0014755969 | 0.0014208729 | 5.4724008e-05 | 5.9617392e-05 |
| G02 | global-balance | 0.020 | 0.0014613658 | 0.0014044654 | 5.6900368e-05 | 6.1086455e-05 |
| G05 | global-balance | 0.050 | 0.0014586633 | 0.0013910015 | 6.7661764e-05 | 6.4473352e-05 |
| G10 | global-balance | 0.100 | 0.0015061076 | 0.0014022934 | 0.00010381416 | 7.348561e-05 |
| L10 | local-soft-return | 0.100 | 0.0031958465 | 0.0029579739 | 0.00023787258 | 0.00018457039 |
| L25 | local-soft-return | 0.250 | 0.0028599097 | 0.0026827145 | 0.00017719522 | 0.00016542689 |
| L50 | local-soft-return | 0.500 | 0.0019081519 | 0.0017994705 | 0.00010868143 | 9.4162847e-05 |
| L75 | local-soft-return | 0.750 | 0.0016544484 | 0.0015838892 | 7.0559201e-05 | 6.7993025e-05 |

- Selected: `G05`

#### Recovery round 2

| Action | Type | Parameter | Centered | Existence | Spatial | Common |
|:--|:--|--:|--:|--:|--:|--:|
| R | reference | 0.000 | 0.0029110531 | 0.0027796161 | 0.00013143702 | 0.00037382489 |
| G02 | global-balance | 0.020 | 0.0029181185 | 0.0027804434 | 0.00013767507 | 0.00037100727 |
| G05 | global-balance | 0.050 | 0.0029471788 | 0.0027958119 | 0.00015136693 | 0.0003681549 |
| G10 | global-balance | 0.100 | 0.0030396844 | 0.0028560648 | 0.00018361953 | 0.00036678093 |
| L10 | local-soft-return | 0.100 | 0.0039096133 | 0.003645897 | 0.00026371634 | 0.00052038763 |
| L25 | local-soft-return | 0.250 | 0.0035970271 | 0.0033714292 | 0.00022559788 | 0.00044264884 |
| L50 | local-soft-return | 0.500 | 0.0033355872 | 0.0031571337 | 0.00017845355 | 0.00039752151 |
| L75 | local-soft-return | 0.750 | 0.0030733792 | 0.0029251991 | 0.00014818016 | 0.00038216784 |

- Selected: `R`

## Evidence boundary

V81 keeps the V75-safe first-round pulse and uses one frozen recovery action bank on M24 and X36. The bank contains ordinary reference recovery, three small reliability-balanced global weight corrections retained from V80, and four intervention-local soft returns. A soft return changes only the receiver rows touched by the pulse: the incumbent residual input is restored with fractional trust and the unused weight returns to self. Every action keeps the reference senders, physical support, and directed-message count. At each of two recovery rounds, formal mixture-aware virtual KLA evaluates all actions from the current virtual posterior against the causal parallel reference arm and chooses minimum next-round centered energy. No truth, prediction, new measurement, future link, packet draw, route execution, tracking outcome, or model training is used.
