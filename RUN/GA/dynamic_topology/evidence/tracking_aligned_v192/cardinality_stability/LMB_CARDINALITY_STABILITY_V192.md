# V192 observable LMB cardinality-stability probe

A positive stability slack certifies that the candidate keeps the reference MAP cardinality.  A non-positive value only means that the sufficient certificate failed.

| Scale | F | V99 | Positive net | Rescue | Useful loss | MAP changes | Uncertified | Min slack | Worst receiver | Ref -> cand MAP | Ref margin | PMF shift | Mean card. delta |
|:--|--:|:--:|:--:|--:|--:|--:|--:|--:|--:|:--|--:|--:|--:|
| M24 | 1 | 1 | 1 | 2.529% | 0.000% | 1 | 1 | -0.905476 | 2 | 4 -> 13 | 0.457892 | 0.681684 | +1.2741 |
| M24 | 2 | 0 | 0 | 0.057% | 0.064% | 1 | 1 | -0.900255 | 8 | 9 -> 12 | 0.624126 | 0.762190 | +0.4594 |
| M24 | 3 | 1 | 1 | 0.359% | 0.000% | 0 | 1 | -0.054579 | 14 | 14 -> 14 | 0.319658 | 0.187119 | +0.2351 |
| M24 | 4 | 1 | 1 | 0.554% | 0.031% | 1 | 1 | -0.794171 | 20 | 9 -> 12 | 0.100817 | 0.447494 | +0.4069 |
| X36 | 1 | 1 | 1 | 0.183% | 0.024% | 0 | 1 | -0.154818 | 2 | 17 -> 17 | 0.085470 | 0.120144 | +0.2223 |
| X36 | 2 | 1 | 1 | 0.314% | 0.080% | 1 | 1 | -0.519011 | 8 | 13 -> 16 | 0.058313 | 0.288662 | +0.5360 |
| X36 | 3 | 0 | 0 | 0.141% | 0.158% | 1 | 1 | -0.501738 | 14 | 15 -> 18 | 0.068383 | 0.285061 | +0.4540 |
| X36 | 4 | 1 | 1 | 0.168% | 0.000% | 1 | 1 | -0.519662 | 20 | 17 -> 18 | 0.018210 | 0.268936 | +0.2925 |
| X36 | 5 | 1 | 1 | 0.087% | 0.006% | 1 | 1 | -0.901235 | 26 | 16 -> 19 | 0.079266 | 0.490250 | +0.4151 |
| X36 | 6 | 0 | 0 | 0.014% | 0.150% | 1 | 1 | -0.170024 | 32 | 18 -> 17 | 0.167924 | 0.168974 | -0.1257 |

## Evidence boundary

V192 computes an exact outcome-marginal LMB cardinality PMF from the current observable routing surrogate.  The MAP certificate is sufficient but not necessary and does not claim that the full-payload reference is correct.  Opened M24/X36 anchors are development-only.
