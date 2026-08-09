# V61: label-conditioned effective KLA routing

## Research decision

V59 closes the formation-level action family rather than merely rejecting one
selector.  At three fresh high-debt M24 convoy states, exhaustive enumeration
of all 16 safe formation protection subsets produces only `+0.267%` mean best
strict gain.  A single formation action applies the same communication change
to every target label and every affected receiver, so helpful and harmful
label effects can cancel before the tracker is evaluated.

V61 changes the controlled object from one graph per time step to one
effective graph per target label.  The physical communication graph remains a
shared carrier and safety layer.  For a physical sender--receiver edge, a
binary variable decides whether the sender's posterior for label `ell`
participates in that receiver's KLA update:

```text
physical graph A_t  ->  label masks Z_t^(ell)  ->  effective graphs G_t^(ell)
```

This is not a return to the full/light payload-equivalence claim.  Every
selected label transmits its complete Gaussian-mixture Bernoulli posterior;
an unselected label is absent from that receiver's fusion input.

## KLA interface and theoretical target

For a selected source set `S_i^(ell)` at receiver `i`, ideal label-wise KLA
has the normalized spatial density and existence probability

```text
p_bar_i^(ell)(x) proportional to
    product_{j in S_i^(ell)} p_j^(ell)(x) ^ omega_ij^(ell)

logit r_bar_i^(ell) = log eta_i^(ell)
    + sum_{j in S_i^(ell)} omega_ij^(ell) logit r_j^(ell).
```

The source weights are renormalized within each selected label.  This gives
three concrete theoretical obligations for the paper:

1. show that independently normalized label factors still define a valid LMB;
2. express the effect of adding or removing one sender label as a change in
   Bernoulli log odds plus the spatial-overlap term `log eta`;
3. give a convergence or bounded-error condition for time-varying per-label
   effective graphs under positive weights and rolling label information flow.

The repository's heavy fusion is a mixture-aware approximation to the spatial
KLA integral.  Experiments must call it that; no exact full-density
implementation or full/light equivalence is claimed.

## Observable action score

The first candidate generator uses only the current receiver and sender label
posteriors, current link reliability and registered reference route.  A
receiver--sender--label omission is eligible only when:

- the receiver has current positive measurement-association support;
- the sender does not have positive support for the same label;
- the sender label participates on a selected cross-formation residual edge;
- omitting it improves the exact current implementation's counterfactual
  fused existence, receiver-anchored spatial agreement, or both.

Candidates are ranked by the association-support-weighted counterfactual
improvement.  The action bank contains cumulative top-scoring bundles rather
than an exhaustive power set.  Each bundle retains all Gaussian-mixture
components of every transmitted label and must save enough full-label bytes to
pay for its compact control synopsis.  The reference action sends the original
full messages and pays no synopsis overhead.

Offline H=3 paired outcomes are used only as teacher labels.  An eventual
shared set/GNN predictor may replace the analytic ranker, but it receives the
same current observable features and cannot bypass deterministic projection.

## First headroom experiment

The first positive-control state is `m24-formation-fov / seed 211 / t=104`.
It has a previously opened strict H=3 formation-level gain of `+10.394%` and,
under V60's current-support attribution, `8.493%` positive-supported rescue
and `7.544%` association-weighted rescue.  It therefore tests whether the new
label action can preserve the real mechanism while removing coarse collateral
effects.

Each nonreference candidate executes for one step and then returns to the
registered reference route for two steps.  A useful label action must achieve:

- at least `5%` mean E-OSPA improvement;
- nonnegative worst-sensor and minimum-formation improvement;
- nonnegative window and terminal consensus improvement;
- no increase in aggregate attempted bytes after synopsis accounting.

If this gate passes, the score and bundle construction freeze before opening
fresh M24 convoy states.  If it fails, the next diagnostic compares existence
and spatial components of the label score; it does not return to formation
subset expansion.

## Scale and scene path

The method remains development-only until a frozen label router shows clear,
repeatable gains on both M24 and X36.  Only then does evaluation expand to the
registered radial, convoy, relay, merge-split and curved-corridor primary
scenes, with crossing retained as a stress test.  Required comparisons include
the fixed reference, the best formation-level controller, a byte-matched
random label mask, the analytic label router, and the learned router if the
label action space first demonstrates sufficient oracle headroom.
