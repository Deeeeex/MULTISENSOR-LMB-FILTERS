# V151 M24 temporal-repeat closure

## Decision

The fixed repeated label-omission mechanism is closed at M24.  Repeating the
best legal V150 source--label action over all eight opened steps does not
accumulate material tracking gain and creates full-horizon communication debt.
X36 and a learned label-omission policy are not opened from this route.

## Paired result

| Metric | One-step V150 | Eight-step V151 repeat |
|:--|--:|--:|
| Mean H=8 E-OSPA gain | +0.724% | +0.767% |
| Worst-sensor gain | +0.000% | +0.000% |
| Minimum-formation gain | +0.000% | +0.000% |
| Window consensus gain | +0.878% | +1.813% |
| Terminal consensus gain | +3.959% | +5.256% |
| Attempted bytes saved | +0.032% | -0.326% |

Seven additional omissions add only about 0.043 percentage points of mean
E-OSPA gain while changing a byte saving into a byte increase.  The benefit
saturates after the first intervention, whereas posterior-complexity debt
continues to accumulate.

## Consequence

Explicit source--label omission remains a corrected and useful protocol
primitive, and the best V150 singleton remains valid development evidence that
label participation can matter.  It is not retained as the primary method
because the bounded M24 upper bounds stay far below the registered 5% gate.

The next primary search returns to higher-leverage routing actions: select the
current physical/effective KLA graph while preserving deterministic
connectivity, byte and fallback constraints.  Any analytic or learned edge
value must be trained against final recursive tracking outputs and include the
full-horizon posterior-complexity debt discovered by V150/V151.  Label
omission may later serve only as a constrained secondary action or ablation,
not as the main contribution.
