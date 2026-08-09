# V64 first-principles method note

## 1. What the action controls

The physical carrier graph answers whether two sensors can exchange a message.
The effective fusion graph answers whether a received posterior actually
participates in the receiver's KLA update.  V64 leaves the carrier graph and
registered fusion weights unchanged.  Its action only controls complete
cross-formation posterior participation for selected receiver formations.

This separation matters because deleting a physical edge simultaneously
changes reachability, payload, and fusion content.  A data-plane intervention
isolates the estimation mechanism: control synopses still flow, while a
currently harmful posterior can be withheld from KLA.

## 2. Observable per-formation rescue score

For each formation `f`, the current posterior supplies a counterfactual:
compare reference fusion with the same fusion after withholding that
formation's registered cross-formation input.  For receiver labels with
current measurement-association mass `a`, retain only positive existence
rescue and weight it by `a`:

`s_f = sum_(receiver,label in f) a * [r_without_cross - r_reference]_+ / Z_f`.

`Z_f` is that formation's reference existence mass used for local scale
normalization.
Every `s_f` is nonnegative and uses only the current local posteriors, current
link reliability, current geometry, and past selected graphs.  Truth and
future data are absent.

The score is an **observable harmful-input surrogate**.  It is not claimed to
be a bound on future E-OSPA.  V60--V63 provide empirical evidence that it is
useful for ranking which receiver domains are currently harmed.  Because the
denominator is formation-specific, the current implementation covers a sum of
locally normalized ratios.  It does **not** yet measure an additive fraction
of total network risk when formations carry different reference cardinality.

## 3. Minimal risk-coverage action

Let `rho` be a dimensionless coverage target and `A` the protected formation
set.  V64 solves

`minimize |A|  subject to  sum_(f in A) s_f >= rho * sum_f s_f`.

For unit intervention cost per formation, sorting scores in descending order
and taking the shortest feasible prefix is optimal.  If a feasible set of
size `k` omitted a larger score in favour of a smaller one, exchanging them
could not reduce coverage.  Repeating the exchange produces the top-`k`
prefix; therefore no smaller set can meet the same constraint.

Two direct guarantees follow for the current locally normalized surrogate:

1. the selected action changes the fewest formation domains among all actions
   meeting the registered coverage target;
2. the residual unprotected observable risk is at most
   `(1-rho) * sum_f s_f`.

Because `rho` is a fraction rather than a node count, the rule can select four
formations when X36 rescue scores are diffuse and three when they are
concentrated.  This removes the most direct fixed-top-k dependence on network
size, but the formation-specific denominators prevent a full cross-scale risk
interpretation.

## 4. Why the action persists for H=3

V63 held the protected set fixed while varying its size, then held the size
fixed while varying recovery speed.  On X36 t=72, persistent top-3 protection
reached `+4.703%`, whereas `3 -> 2 -> 1` and `3 -> 2 -> 0` recovery fell to
`+3.152%` and `+2.746%`.  The fastest release also degraded terminal
consensus.  The current evidence therefore describes persistent harmful input
over the three-step window, not a one-step impulse.

V64 consequently keeps its minimal risk-coverage set for all three steps.
This H=3 duration is still an experimental design choice, not a general
theorem.  A final online controller should recompute current risk and use
hysteresis to release formations once covered risk falls, without reading
future outcomes.

## 5. Claim boundary

The minimal-prefix and residual-risk properties are exact for the observable
nonnegative scores.  They do not establish that formation effects add exactly
inside recursive LMB-KLA, nor that 80% observable coverage guarantees 5%
tracking improvement.  Those links require paired experiments across states,
scales, and scene families.  A learned model, if later used, should predict
temporal residuals or interactions around this analytic action rather than
replace the interpretable coverage constraint.

## 6. Successor correction: additive risk with a useful-information guard

The next method revision should not tune an absolute trigger against the V64
outcome.  It should first repair the quantity being covered.  For each
receiver formation, compute two current-only counterfactual masses:

`h_f = sum a_receiver * [r_without_cross - r_reference]_+`

`b_f = sum a_cross * [r_reference - r_without_cross]_+`.

Here `h_f` measures locally observed existence that cross-formation fusion
suppresses, while `b_f` measures existence supported by currently active
cross-formation senders that would be lost by withholding their posteriors.
Both use absolute existence mass.  A single network denominator

`Z = sum_(all receivers,labels) r_reference`

then gives additive quantities `H_f=h_f/Z` and `B_f=b_f/Z`.  Consequently
`sum_f H_f` has one meaning on M24 and X36: the fraction of network reference
existence mass exposed to currently observable negative transfer.

The protected set can be chosen by the deterministic constrained problem

`minimize (|A|, sum_(f in A) B_f, -sum_(f in A) H_f)`

subject to

`sum_(f in A) H_f >= rho * sum_f H_f`

and

`sum_(f in A) B_f <= sum_(f in A) H_f`.

The first constraint retains the V64 coverage idea.  The second is a
truth-free useful-information guard: protection is allowed only when its
observable local rescue is at least as large as the observable cross-supported
mass it may remove.  If no feasible nonempty set exists, the reference policy
is used.  With four or six formations the lexicographic optimum can be found
exactly by enumeration; at larger scale it becomes a small covering problem.

This correction separates three claims cleanly: V64 tests whether persistent
coverage has headroom, the successor defines an additive and scale-comparable
online decision rule, and any later GNN predicts only temporal persistence or
non-additive formation interactions around that constrained analytic action.
