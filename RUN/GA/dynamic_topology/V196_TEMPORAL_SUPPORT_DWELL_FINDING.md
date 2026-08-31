# V196 temporal support dwell and joint-release finding

## Observable replay

V196 replaces V194's one-frame absence test with a causal two-page support
dwell: a set-entry label is unsupported only when neither its receiver nor a
currently contributing cross-formation sender has positively associated it on
the current or preceding page.

The replay preserves the known useful first-page releases on both scales.  It
reduces later formation releases from four to zero on M24 and from three to one
on X36.  The surviving X36 decision is `F3` at the second page.

## Exact paired sequence test

The replay identifies release decisions but does not establish their joint
tracking value.  Two opened teacher arms therefore isolate the X36 interaction
under the same H=3 reference outcome, V99 base, measurements, filter RNG and
byte ledger.

| X36 release sequence | E-OSPA gain | RMSE gain | Consensus gain | Attempted-byte saving |
|:--|--:|--:|--:|--:|
| `t72: F2,F5` | +1.655% | +0.673% | +2.819% | +5.211% |
| `t72: F2,F5; t73: F3` | +1.371% | -1.419% | +2.414% | +4.615% |

The paired static-reference means are E-OSPA `85.970277` and RMSE
`57.902417`.  Adding the temporally certified `F3` release makes every reported
axis worse than the first-page-only arm.  It is therefore a false repair for
the present objective.

The first-page pair is itself non-additive.  Earlier single-release teachers
gave:

| Scale and release | E-OSPA gain | RMSE gain | Consensus gain | Attempted-byte saving |
|:--|--:|--:|--:|--:|
| M24 `F4` | +7.521% | +10.620% | +17.429% | +3.562% |
| X36 `F2` | +2.181% | +0.589% | +2.648% | +5.397% |
| X36 `F5` | +2.276% | -0.583% | +5.333% | +6.365% |

Releasing both X36 formations does not compose the single-release benefits:
the intervention changes the recursive posterior and hence the following V99
decisions.

## Decision

Do not promote the unrestricted temporal-support rule.  The next deployable
candidate is a budgeted repair controller:

1. identify observation-unsupported set entries as in V194;
2. release at most one formation while a short repair token is available;
3. choose the largest certified set-entry risk, which selects M24 `F4` and
   X36 `F2` at the opened anchors;
4. enforce a three-page cooldown before the token can be reused.

This converts the discovered non-additivity into an explicit action-budget
constraint, preserves positive communication saving, and targets the two
single-release teacher sequences that improve E-OSPA, RMSE and consensus on
their respective scales.  All teacher results remain opened development
evidence and do not establish cross-scene generalization.

