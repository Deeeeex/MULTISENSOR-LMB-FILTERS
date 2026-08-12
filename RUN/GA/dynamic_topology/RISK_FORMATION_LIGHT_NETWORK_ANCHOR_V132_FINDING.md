# V132 finding: close the parallel compressed-anchor route

V132 does not pass the registered X36 development gate. It is retained only as
a repository experiment record and must not be added to the canonical progress
document.

## Paired result

Against the matched static full-payload reference on X36 seed 211, `t=72`,
`H=8`:

- mean E-OSPA gain: `+6.221%`;
- post-maturity minimum gain: `+4.997%` (below the `+5%` gate);
- formation gains: `[+2.855, +6.267, +7.711, +8.970, +11.250, +0.506]%`;
- minimum formation-time gain: `-6.111%`;
- F6 non-gateway terminal gain: `-2.968%`;
- window / terminal consensus gains: `+12.652% / +30.318%`;
- attempted-byte saving after charging the anchor: `+7.309%`;
- auxiliary traffic: exactly `30` attempted messages per page, `240` total,
  and `1,040,640` attempted bytes;
- candidate runtime: `314.13 s` versus `251.51 s` for the reused static arm.

## Interpretation

Reallocating V131's message count from alternating whole-network refreshes to
continuous F1/F2/F6 receiver-row refreshes improves the mature aggregate and
F1 tail, while preserving more byte saving. It does not remove the decisive
local failures: F2 remains negative at page 5 (`-6.111%`), F1 is negative at
page 6 (`-2.785%`), and F6 terminal peers return to `-2.968%`. The same light
anchor can help one risk region while harming another even under continuous
refresh, so the remaining problem is the compressed state itself rather than
only message timing or budget placement.

## Decision

Close the parallel moment-compressed anchor branch. Further opened-mask tuning
would add privileged patches without resolving deployability, and the extra
filter state raises runtime by about 25% while memory remains unquantified.
Return to the stronger first-principles direction: keep the effective graph and
standard mixture-aware LMB-KLA state, and learn/optimize a causal topology or
payload action under an explicit communication budget with a conservative
static fallback. Future experiments should begin from an online observable
risk signal, not another rollback oracle.
