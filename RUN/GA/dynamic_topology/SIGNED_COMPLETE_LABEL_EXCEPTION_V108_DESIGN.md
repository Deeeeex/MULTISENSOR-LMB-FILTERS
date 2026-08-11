# V108 design: signed complete-label exceptions

## Why the action space changes

V105 establishes that the useful part of the current X36 intervention is the
formation-conditioned control-only payload, not gateway handoff.  V106 and
V107 establish that returning an entire formation to full payload cannot make
the intervention locally safe: a nominal release can miss its delivery
opportunity, while a delivered full restoration can worsen downstream peers.
The remaining discontinuity is therefore inside the message.  A protected
cross-formation edge currently sends either no Bernoulli Gaussian-mixture
labels or every active label.  V108 keeps control-only as the default and
restores only complete labels with positive signed marginal value.

## LMB-KLA basis

For one label, ideal LMB-KLA has spatial density and existence odds

\[
\bar p_\ell(x) = \eta_\ell^{-1}
  \prod_i p_{i,\ell}(x)^{\omega_i},\qquad
\frac{\bar r_\ell}{1-\bar r_\ell} =
  \eta_\ell \prod_i
  \left(\frac{r_{i,\ell}}{1-r_{i,\ell}}\right)^{\omega_i},
\]

where

\[
\eta_\ell = \int \prod_i p_{i,\ell}(x)^{\omega_i}\,dx.
\]

This factorization makes the label the smallest theoretically coherent
communication action.  V108 never transmits a moment-only replacement for a
selected label: it sends the sender's complete Bernoulli Gaussian mixture so
that both the spatial normalizer and existence update remain represented.
The repository evaluates this action with its componentwise powered-GM KLA
approximation.  That numerical path preserves multiple modes but is not an
exact closed-form power of an arbitrary Gaussian mixture; the experiment must
not be described as exact full-density GM fusion.

## Signed oracle value

The first experiment is an offline headroom test, not an online controller.
For a delivered protected edge and a candidate label, it reconstructs the
current one-round fusion inputs from the opened V105 local posterior snapshot.
All other cross-formation labels remain absent, exactly as in the control-only
arm.  Let `q0` be the fused Bernoulli without the candidate label and `q1` the
fused Bernoulli after adding the sender's complete label mixture.  Against the
opened target state, the per-label capped expected risk is

\[
R(q;x^*)=(1-r)c^2+r\sum_k \alpha_k
\min\{\|m_{k,pos}-x^*_{pos}\|^2+
\operatorname{tr}(P_{k,pos}),c^2\}.
\]

The signed value is `v = R(q0;x*) - R(q1;x*)`.  Positive values mean that the
complete label is useful under the same approximate KLA path that will execute
the experiment; negative values mean that it should remain control-only.  The
frozen V108 oracle retains at most three positive labels per delivered edge
and page, rejects a downward `r=0.5` decision crossing, and never exceeds the
corresponding full-payload byte budget.

The first plan targets only the two formations that fail V105's local gate.
For F1, the delivered cross-edge pages are t=74, 75 and 77.  For F6, they are
t=75--79.  This restriction preserves V105's already positive F2--F5 behavior
and tests whether fine-grained complete-label restoration can repair the
remaining local losses without reopening harmful labels.

## Interpretation gate

V108 establishes useful action-space headroom only if all of the following
hold against the matched static fixed-route full-payload baseline:

1. mean E-OSPA gain is at least 5%;
2. every formation has nonnegative window gain;
3. the F6 non-gateway terminal gain is nonnegative;
4. attempted communication remains below the full-payload reference; and
5. the selected rolling-B3 checks remain satisfied.

Passing this gate authorizes work on a truth-free signed-value estimator using
receiver/sender association support, Bernoulli log-odds change, mixture
overlap, protection age and recent delivery opportunity.  It does not itself
authorize a GNN or a generalization claim.  Failing the gate means the
remaining X36 loss is not repairable by sparse complete-label exceptions under
the current control-only carrier, so the protection mechanism or fusion
semantics must change before any learned policy is attempted.

## Opened result

V108 sends all 24 frozen exception actions and retains 5.237% mean E-OSPA gain
with 5.870% byte saving, but it fails the local gate.  F1 remains -0.930%, F6
falls to -0.151%, and the F6 non-gateway terminal result remains -2.904%.
Relative to V105, mean gain drops by 0.022 percentage points and byte saving
drops by 0.247 points; the F6 peer change is only +0.036 points.  Positive
one-round label risk under opened truth is therefore not sufficient to predict
recursive network E-OSPA.

The result also exposes a semantic issue in the current carrier.  A
control-only selective edge delivers an empty heavy payload.  Under
`fov-aware-censored` missing-label fusion, an omitted label can be represented
as a low-existence censored observation when the sender is judged able to see
it.  Control-only is therefore not pure abstention: it can inject negative
existence evidence for every omitted label.  Sparse positive exceptions leave
that bulk negative-evidence mechanism intact.  The next attribution must
separate three cases explicitly: complete positive label evidence, credible
negative evidence, and source abstention with zero label-wise KLA weight.
