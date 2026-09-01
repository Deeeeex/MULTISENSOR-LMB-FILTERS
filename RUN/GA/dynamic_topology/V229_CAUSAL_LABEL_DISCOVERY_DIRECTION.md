# V229 causal label-discovery direction

## First-principles question

V228 can make a teacher-selected complete-label transfer affordable, but an
online receiver must choose at most three label keys before it sees remote
label records.  The missing question is therefore not payload feasibility or
value prediction.  It is whether the beneficiary's own causal state contains
enough information to nominate the valuable label.

The first frozen receiver-only rule assigns one query key to each of three
observable needs: the largest cross-receiver existence disagreement, the
largest covariance-normalized spatial disagreement, and the largest median
position uncertainty.  Labels must be present at four of six receivers and
have mean existence at least 0.10.  Numeric label values break exact ties only;
they are not score features.  Remote label inventories, truth and future
outcomes are unavailable until after the three keys have been chosen.

## t=133 diagnostic

The rule is evaluated on the same X36 seed-1301 t=133 reference local
posteriors used by the V228 headroom queue.  Teacher rows are read only after
selection to measure recall.

- F5 teacher label `[1,4]` is present at all six receivers with mean existence
  about 0.736, but ranks only 8th by existence disagreement, 9th by spatial
  disagreement and 7th by uncertainty.  It is absent from the three keys.
- F6 teacher label `[25,20]` is present at all six receivers with mean
  existence about 0.658.  It ranks 6th, 10th and 3rd respectively, but the
  one-key-per-mode rule still does not select it.
- Overall teacher-key recall is therefore 0/2.

The F5 miss is the decisive one.  Its beneficiary posteriors are mutually
similar, while source S2 carries strongly different existence evidence.  A
beneficiary-only selector has no direct observation of that remote surprise.
Increasing model capacity cannot create information that is absent before the
query; it risks fitting trajectory or label history instead.

## Method decision

Do not authorize this beneficiary-only top-three rule for V228.  Keep the
teacher-keyed V228 run as an action-space headroom experiment, but make the
next control-plane candidate source-offer or query/offer hybrid:

1. choose the donor and beneficiary from causal formation-level state and
   exact byte credit;
2. let each physically eligible source nominate at most one compact label
   surprise relative to its latest causally cached beneficiary posterior;
3. let the beneficiary coordinator select at most one offer, pay one complete
   Bernoulli-GM payload, and run the existing receiver-specific KLA projection;
4. retain the query path for labels whose need is already visible within the
   beneficiary, but do not require every useful action to be receiver-nominated.

This direction deliberately reuses the “current versus last broadcast” idea
from event-triggered LMB work.  The possible contribution is not the trigger
itself; it is the byte-certified source-beneficiary-label acquisition and the
safe change it induces in the effective label-wise KLA input graph.

The first frozen byte contract allows each common source to return at most two
24-byte offers.  A 16-byte solicitation and 16-byte response header are
charged per source, hence the worst-case control cost is
`S × (16 + 16 + 2 × 24) = 80S` bytes.  At t=133 both F5 and F6 have nine
common sources, so the control plane costs 720 B.  With the 1,456 B complete
label payload and 10,400 B admission credit, the action retains 8,224 B of
certified net saving and 6,144 B of spendable feasibility margin.  This is
736 B better than the three-key query-first bound, but it proves only byte
feasibility; source-side causal ranking remains unimplemented.

The source-offer protocol must be specified and charged before any additional
tracking outcome is opened.  This single development state cannot establish
recall, tracking gain or generalization; it only rejects the current simple
receiver-only discovery rule and identifies the missing information channel.
