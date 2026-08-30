# V193 observation-supported LMB set-entry probe

| Scale | F | V99 | Rescue | Useful loss | Supported rescue share | Weighted rescue | MAP changes | Entries | Unsupported | Unsupported mass | Max risk | Min support | Worst receiver | Ref -> cand MAP |
|:--|--:|:--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--|
| M24 | 1 | 1 | 2.529% | 0.000% | 99.913% | 10.727% | 1 | 9 | 0 | 0.0000 | 0.0000 | 0.9389 | 1 | 13 -> 13 |
| M24 | 2 | 0 | 0.057% | 0.064% | 3.257% | 0.232% | 1 | 4 | 4 | 3.5348 | 0.3882 | 0.0000 | 8 | 9 -> 12 |
| M24 | 3 | 1 | 0.359% | 0.000% | 74.421% | 1.275% | 0 | 0 | 0 | 0.0000 | 0.0000 | 1.0000 | 13 | 15 -> 15 |
| M24 | 4 | 1 | 0.554% | 0.031% | 74.732% | 2.321% | 1 | 3 | 1 | 0.5297 | 0.0549 | 0.0000 | 20 | 9 -> 12 |
| X36 | 1 | 1 | 0.183% | 0.024% | 82.794% | 1.100% | 0 | 0 | 0 | 0.0000 | 0.0000 | 1.0000 | 1 | 17 -> 17 |
| X36 | 2 | 1 | 0.314% | 0.080% | 58.541% | 2.065% | 1 | 4 | 2 | 1.5946 | 0.1227 | 0.0000 | 8 | 13 -> 16 |
| X36 | 3 | 0 | 0.141% | 0.158% | 31.323% | 0.857% | 1 | 4 | 3 | 2.3434 | 0.1584 | 0.0000 | 14 | 15 -> 18 |
| X36 | 4 | 1 | 0.168% | 0.000% | 74.293% | 0.958% | 1 | 1 | 0 | 0.0000 | 0.0000 | 0.7853 | 19 | 18 -> 18 |
| X36 | 5 | 1 | 0.087% | 0.006% | 21.513% | 0.514% | 1 | 3 | 2 | 1.9585 | 0.1206 | 0.0000 | 26 | 16 -> 19 |
| X36 | 6 | 0 | 0.014% | 0.150% | 49.912% | 0.083% | 1 | 0 | 0 | 0.0000 | 0.0000 | 1.0000 | 31 | 17 -> 17 |

## Worst-receiver entry diagnostics

- `M24 F1 / sensor 1`: labels `[]`, support `[]`
- `M24 F3 / sensor 13`: labels `[]`, support `[]`
- `M24 F4 / sensor 20`: labels `[25 14;25 13;1 1]`, support `[0.7609 0.9737 0]`
- `X36 F1 / sensor 1`: labels `[]`, support `[]`
- `X36 F2 / sensor 8`: labels `[31 21;7 7;25 18;7 5]`, support `[0.8467 0.9998 0 0]`
- `X36 F4 / sensor 19`: labels `[]`, support `[]`
- `X36 F5 / sensor 26`: labels `[19 13;25 20;13 11]`, support `[0 0 0.999]`

## Evidence boundary

V193 combines the observable V192 cardinality change with current receiver measurement-association support for labels entering the marginal LMB extraction.  Numeric label values only align Bernoulli components.  Opened paired outcomes are not used by the feature and remain development-only interpretation evidence.
