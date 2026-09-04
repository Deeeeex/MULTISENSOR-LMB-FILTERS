# V262 risk-episode latched formation shortcut

## Decision inherited from V261

V261 established that formation-scale path shortening has direct tracking
leverage.  Over the paired M24 `t=57--73` continuation it improved network
E-OSPA by 0.356%, RMSE by 5.488%, consistency by 0.503%, and reduced posterior
bytes by 0.984%.  F4 event RMSE improved by 16.119%.  The complete gate failed
because F4 event E-OSPA regressed by 0.294% and F2 RMSE regressed by 3.619%.

The online action trace identifies a concrete temporal defect.  The target
remained F4 for eleven pages, while the greedy lowest-risk donor alternated
`F1,F2,F1,F2,...,F1`.  Each new donor was one hop closer after changing the
tree, but that change lengthened the previous donor's route.  V261 therefore
changed the tree on every active page.  This is a chattering controller, not a
stable information route.

## Parameter-free risk episode

V262 retains the V261 trigger, donor support conditions, physical projection,
KLA weights and exact `N+2(F-1)` posterior-message budget.  It changes only the
temporal decision rule:

1. when no episode is active, V261 may initiate one donor-target shortcut;
2. the accepted donor, target, label and projected formation tree are written
   into the policy schedule state;
3. on the next page, the same tree is retained if the target formation still
   passes the original localization-tail gate, the same label remains its
   largest localization risk, the donor still meets the original coverage,
   existence and risk-ratio gates, and the tree is still physically
   projectable;
4. otherwise the latch is released and the ordinary causal selector may keep
   V242 or initiate a new valid episode.

There is no tuned hold length and no future look-ahead.  The risk condition is
both the entry and release clock.  A one-page schedule certificate is enough
to carry the state, so no target truth, future measurement, future outcome or
realized future delivery is exposed to the policy.

## Runtime interface

The common filter receives an opt-in policy-schedule history channel.  It is
disabled by default and therefore does not change existing policies.  V262
enables a depth-one history and reads only the immediately preceding schedule
certificate.  This makes stateful hysteresis reusable by later controllers
without encoding method state in sensor identifiers or process-global memory.

The development controller still assumes a centralized current-network
posterior synopsis, and that control traffic is not charged.  V262 is therefore
an action-mechanism test rather than a deployment-cost or generalization claim.

## Paired stopping rule

Run exactly one continuation from the same V259/V242 state at `t=57`.  The run
must exercise both latch retention and suppression of an instantaneous donor
switch while keeping exactly 30 posterior messages.  It may proceed to one
complete M24 arm only if F4 event E-OSPA and RMSE both improve, network E-OSPA,
RMSE and consistency stay within the registered 2% guards, no formation
regresses by more than 3%, and spliced full-episode traffic remains below the
static baseline.  Failure closes the single-donor latch and motivates a
multi-source risk-aware tree objective; thresholds are not retuned on this
window.
