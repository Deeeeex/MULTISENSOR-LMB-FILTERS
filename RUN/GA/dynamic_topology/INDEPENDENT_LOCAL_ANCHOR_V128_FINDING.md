# V128 finding: a fully independent local anchor is too conservative

## Registered result

| Metric | V128 independent local anchor | Strict gate |
|:--|--:|:--:|
| Mean E-OSPA gain vs static | +3.683% | fail |
| Minimum mature-page gain | +2.797% | fail |
| Minimum formation gain | -3.005% | fail |
| Minimum formation-time gain | -22.646% | fail |
| F6 peer terminal gain | -5.784% | fail |
| Worst-sensor gain | +10.530% | pass |
| Window / terminal consensus gain | +4.585% / +3.968% | pass |
| Attempted-byte saving | +6.874% | pass |

V128 retains the opened V126 rollback mask, but replaces the privileged
paired-static state with a genuinely independent per-node anchor.  From the
common t=72 continuation state onward, every anchor uses only its own
prediction and measurement update; it receives no neighbor posterior.  All
36 anchors are maintained on every page.  The candidate runtime rises from
251.51 s to 386.05 s (+53.5%) even though it adds no posterior message or
payload byte.  Additional anchor memory remains unquantified.

## Causal conclusion

V127 failed because its same-step local posterior was not independent of the
accumulated working-state bias.  V128 removes that contamination, but fails in
the opposite direction: replacing a working posterior with a purely local
state discards useful cross-node information along with the harmful input.
The exact paired-static shadow used by V126 is therefore not merely a local
checkpoint.  Its benefit comes from preserving a safer *network-informed*
state.

This closes full-posterior local rollback as the next implementation path.
The result is retained as an experiment record and is not promoted to the
canonical progress document because it does not clear the registered mean,
maturity, or local-safety gates.

## Method decision

The next candidate should keep the working posterior and soften only the
protected cross-formation input.  In particular, a moment-compressed posterior
on a protected edge can retain label-wise existence and spatial information
at lower byte cost, while an attenuated fusion weight interpolates between the
V105 control-only action and full static fusion.  This directly tests whether
the missing safe network information can be restored without a second filter
chain or privileged rollback state.
