# V107 finding: binary protection release is not the missing controller

## Matched result

| Arm | Mean gain | Min. formation | F6 peer terminal | Byte saving |
|:--|--:|--:|--:|--:|
| V105 persistent control-only | +5.259% | -0.931% | -2.940% | +6.117% |
| V106 reactive full release | +5.246% | -0.865% | -2.947% | +5.286% |
| V107 one-page-early release | +5.179% | -0.865% | -5.752% | +4.848% |

V107 releases F1 at t=76 and F6 at t=77, one scheduled page earlier than
V106.  It reuses the exact static H=8 outcome and changes no topology or
fusion-weight row.  The candidate keeps positive network gain at every page,
retains 5.179% mean improvement and saves 4.848% attempted bytes, but the
local gate still fails.

## Delivered-opportunity attribution

The static cross-formation gateway into F1 is receiver 2 from sender 9.  Its
delivered pages over t=72--79 are `[0 0 1 1 0 1 0 0]`.  V107's nominal early
release at t=76 therefore carries no full posterior; the first effective full
input remains t=77, exactly as in V106.  F1 consequently reproduces V106 at
every page after release, including the -12.181% t=77 switch shock and
-5.681% terminal loss.

The F6 gateway is receiver 32 from sender 3 and is delivered on every page.
Its t=77 early full release is real.  Instead of helping, F6 becomes -4.708%
at t=79 and its five non-gateway peers fall to -5.752%.  Persistent
control-only was less harmful than full restoration for this group.

## Method conclusion

The missing controller is not another global dwell threshold:

1. a scheduled release has no effect when the required cross link is not
   delivered;
2. when delivery is available, restoring every label at once can amplify the
   downstream mismatch;
3. further page enumeration would overfit the frozen delivery realization and
   would not solve the empty/full discontinuity.

The binary formation-level release family is therefore closed.  The next
controlled object is a sparse exception layer on top of the successful
control-only action.  For each delivered gateway edge and label, it either
keeps the control-only fallback or admits that label's complete Gaussian-
mixture Bernoulli posterior.  The decision must use the mixture-aware KLA
counterfactual: Bernoulli log-odds change, spatial-overlap normalization,
receiver and sender association support, protection age and recent delivered
opportunity.  It must reject downward existence crossings and stay within the
full-payload byte budget.

The first label experiment remains an offline headroom test.  It should target
the F1/F6 gateway pages that actually deliver, keep all other protected inputs
control-only, and require at least 5% network gain with nonnegative formation
and F6-peer outcomes.  A GNN is justified only after this exact action space
shows safe headroom.
