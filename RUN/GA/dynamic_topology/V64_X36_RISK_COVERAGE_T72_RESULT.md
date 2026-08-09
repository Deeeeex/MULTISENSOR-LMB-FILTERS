# V64 X36 cumulative-risk coverage: t=72 result

The frozen V64 selector passed its first strict X36 gate on
`x36-formation-fov / seed 211 / t=72 / H=3`.

## Observable action

- formation ranking: `[4,2,3,5,1,6]`;
- support-weighted rescue scores:
  `[0.0212754,0.0190650,0.0180059,0.0173466,0.0083419,0.0078573]`;
- cumulative-risk target: `80%`;
- selected minimum prefix: `[4,2,3,5]`;
- achieved current risk coverage: `82.3715%`;
- schedule: the same selected set remains protected for all three steps;
- physical carrier graph and registered fusion weights remain unchanged.

The action was constructed before its tracking outcome and used no truth,
future measurements, future links, or prior action outcomes.

## Strict paired result

| Metric | Gain relative to reference |
|:--|--:|
| Mean tracking | **+5.847%** |
| Worst sensor | **+27.843%** |
| Minimum formation | +0.000% |
| Window consensus | **+15.719%** |
| Terminal consensus | **+17.567%** |
| Attempted posterior bytes | **+3.623%** |
| Rolling sensor/formation B3 | passed |

The registered `5%` mean-gain gate and every nonnegative tail, consensus,
byte, and connectivity gate passed.  Unlike the V63 top-3 result, the fourth
formation was not added by identifier after inspecting tracking results; the
same cumulative 80% rule selected it from current observable risk mass.

## Interpretation and next gate

Together with V63, the result establishes a clean scale mechanism on this
opened state:

1. gain grows with the fraction of current harmful-input risk covered;
2. a fixed top-k is not scale invariant;
3. protecting the minimum set covering most current risk crosses the X36
   practical-effect threshold without sacrificing any registered metric;
4. early recovery is harmful over this H=3 window, so persistence is part of
   the action rather than a cosmetic duration choice.

This is still one opened X36 development state.  The unchanged 80% rule may
now be evaluated at t=100 and t=128, where it selected `[6,4,5,2]` and
`[3,4,5]` before either tracking outcome was opened.  Model training and
validation claims remain unauthorized.
