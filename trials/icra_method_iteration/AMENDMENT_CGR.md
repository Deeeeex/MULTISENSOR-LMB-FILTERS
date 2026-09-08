# Confirmation-gated recency, 2026-09-08

Registered after the complete CR synthetic-case audit, before any CGR output
and before any reserved-cohort tracking. This iteration is bounded to IR,
CR and this third candidate; retain all three outcomes. No parameter search
is authorized by this protocol.

CR reduces false cost but gives up much of ER's discovery/retention gain:
its three synthetic mean OSPA values are 2.66619, 1.09522, 0.78553, compared
with ER's 2.56915, 0.84560, 0.66787. This motivates testing a qualified
exception to the CR upper bound, rather than changing the age kernel.

Confirmation-gated recency (CGR) retains the same fixed-input r_0 and r_ER.
If r_ER <= r_0, use r_ER. An upward change is allowed only when an eligible
represented source (i) receives more recency weight than ordinary weight,
(ii) currently has r_j >= 0.5, and (iii) has detection-association mass at
least 0.5 in each of two consecutive **local** updates with direct sensing
opportunity. Otherwise use r_0. These two-hit and 0.5 association settings
are reused from the already evaluated confirmation ablation; no grid or
outcome-based threshold fit is performed. The r>=0.5 condition requires
the source's own existence decision to be positive.

The two-hit counter and previous local-update frame remain local. A source
transmits its current two-hit bit in the already allocated confirmation
field, so no packet scalar is added. A received bit cannot advance the
receiver's streak. Misses, lack of direct opportunity, empty measurements,
and gaps in a label's local update sequence break the streak. After fusion
restore the receiver's own current bit, just as for direct-opportunity time.
Independent-label matching does not transfer a remote streak into the local
counter. This bit is not the earlier permanent, propagated confirmation
certificate and does not defer untouched-prior exclusion.

For fixed inputs this is ER's scalar objective with r<=r_0 unless the above
gate admits an increase. Spatial weights/densities, negative censors,
observation-history qualification, prediction, likelihood, births, pruning,
label matching and the rho=0.25/lambda=5 s kernel remain unchanged. There is
no claim of calibrated confirmation or independence of consecutive detector
errors. Persistent false detections can pass the gate; report their effects.

Run every development sequence under both existing link conditions and all
60 cached synthetic cases. Before any reserved-cohort output, freeze both
CR and CGR as candidates in that cohort alongside Local, no-age, ER, MIL-AM
and both TC windows. Report both candidates, not only whichever wins on the
reserved data. Six nonduplicate sequences are the primary reserved cohort;
the selected duplicate remains a separate overlap control.
