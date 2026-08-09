# V68 alternative transport headroom result

## Decision

Do not open relay tracking and do not implement a connectivity-safe transport
route for the current relay seed.  At the strongest signed reference-edge
transport state (`m24-formation-fov-relay`, seed `1301`, time `124`), replacing
each residual cross-formation sender by its best currently physical but unused
alternative yields only `0.073%` optimistic net information headroom.  This is
well below the preregistered `1%` materiality threshold.

## Source-only result

| Quantity | Result |
|---|---:|
| Residual input slots checked | 4 |
| Slots with a safe positive replacement | 4 |
| Optimistic transport gain | 0.075% |
| Receiver/incumbent-supported harm | 0.002% |
| Optimistic net headroom | 0.073% |
| Material 1% threshold passed | No |
| Connectivity projection executed | No |
| Tracking outcome read | No |

The largest individual opportunity replaces sender `15` by sender `17` for
receiver `8`.  It contributes `0.066%` transport value and `0.002%` supported
harm, for `0.064%` net value.  The other three best safe replacements contribute
only `0.003%`, less than `0.001%`, and `0.006%` net value respectively.

## Interpretation

V67 showed that the direction of useful intervention can reverse: at time
`124`, the already selected reference edge contains more sender-supported
information worth retaining (`0.781%`) than receiver-supported information
worth quarantining (`0.053%`).  V68 then asks the missing question: whether an
unused physical sender could carry materially better information at the same
message count and `0.05` residual weight.  The answer for this relay state is
no, even before connectivity constraints are imposed.

This means the relay scene is already close to the current one-step
existence-support upper bound.  It does **not** show that transport routing is
unnecessary in general, and it does not establish that suppression is the
universally correct action.  It shows only that this particular relay seed does
not contain enough actionable information contrast for the present method to
produce a meaningful tracking improvement.

## Evidence boundary and next experiment

V68 uses the installed projected-Gaussian, label-wise KLA implementation and
current posteriors only.  Each candidate preserves the receiver, message count,
residual weight, and all other inputs, and is evaluated before connectivity
projection.  The result does not close higher-weight transport with compensating
weight reallocation, multi-step store-and-forward routing, localization or
covariance-specific transport, or other scene families.

The next source-only experiment therefore moves to the existing merge-split
geometry.  Its converging branches and later separation should create a genuine
time-varying information-flow problem: redundant cross-formation messages near
the dense merge, followed by complementary information requirements during the
split.  The method and thresholds remain frozen; tracking is opened only if the
new scene first shows material signed opportunity on unseen states.

