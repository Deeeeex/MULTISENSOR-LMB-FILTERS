# V160 observable one-hop label-source gate

## Question

V159 establishes that every high-value V157 label restore has a current
one-hop source.  V160 asks the narrower causal question:

> Conditioned on a receiver-label pair that is already known to be valuable,
> can present-time, truth-free metadata choose a useful one-hop source?

This separation matters.  A failure here means source selection itself needs a
learned model or richer metadata.  A pass here leaves only the harder trigger
question: deciding which receiver-label pairs deserve payload bytes.

## Frozen inputs and observable recipes

The analysis replays the 35 V159 restores whose privileged marginal reference
gain exceeds `0.1`.  For each decision it enumerates only current physical
one-hop neighbors that hold the complete label in their current local
posterior.  Five predeclared rankers use only current quantities:

1. minimum per-label posterior Bayes-risk proxy;
2. maximum source evidence/FoV/precision quality;
3. a handover-rescue score combining existence, precision, association,
   observation-opportunity and guarded disagreement terms;
4. the same rescue score gated by same-label Gaussian compatibility;
5. handover-rescue value per estimated KiB.

The metadata is obtainable from a compact control synopsis.  It contains no
Gaussian-mixture components and no target truth.  Full mixture components are
needed only after the receiver requests a selected label.

## Evidence boundary and next gate

Truth evaluates the immediate E-OSPA gain of each candidate source offline and
is also used after the fact to identify the best development recipe.  V160 is
therefore not validation and not deployable.  More importantly, it is
conditioned on the privileged high-value label set, so it cannot establish a
trigger policy or a communication saving.

If a simple observable recipe retains nearly all capped source value, V161
will freeze that source rule and enumerate all one-hop receiver-label actions
to test truth-free label ranking, false-positive control and synopsis cost.  If
all simple recipes fail, a compact learned source scorer is justified before
the label trigger is attempted.

## Outcome and frozen source rule

V160 enumerated `1136` current one-hop source-label candidates for the `35`
conditioned high-value decisions.  The simplest per-label Bayes-risk rule was
positive on all `35/35` decisions, retained at least half of the privileged
reference value on `33/35`, and covered `97.803%` of capped source value.  It
never selected the exact truth-best source, which is not a defect: many
neighbors hold nearly interchangeable good posteriors, so exact source
identity is much less important than avoiding a harmful source.

The maximum evidence/FoV-quality rule reached `99.526%` aggregate capped
coverage but made two negative choices on low-value labels.  Compatibility
gating reduced coverage to `91.891%`, showing that receiver-source
disagreement is often the stale-state signal to rescue rather than a reason to
reject the source.  Dividing the rescue score by payload size was much worse
(`71.374%` capped coverage and only `24/35` positive choices), so byte limits
must be enforced as a receiver/network selection constraint rather than as a
naive per-source ratio.

V161 therefore freezes **minimum current per-label posterior Bayes risk** as
the one-hop source selector.  It uses the equal-cost Bernoulli decision risk
plus existence-weighted, cutoff-normalized position covariance; it contains no
truth or future information.  Learning is not justified for source selection
at this stage.  The unresolved problem is label value: deciding which
receiver-label pairs should trigger a request without being handed the V159
positive set.
