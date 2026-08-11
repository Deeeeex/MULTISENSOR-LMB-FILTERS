# V107: one-page-early protection release oracle

V106 restores the static full payload on the first page where the opened V105
tracking response becomes negative.  It retains 5.246% network gain but makes
F1 substantially worse on the switch page and does not recover the F6 peers.
The current outcome sign is therefore a lagging indicator: the harmful state
already exists before the fusion page that exposes it.

V107 advances each vulnerable release by exactly one page.  F1 returns to the
static payload at t=76 instead of t=77; F6 returns at t=77 instead of t=78.
All other formation schedules, topology rows, fusion weights, cached inputs,
random streams and the H=8 frozen static baseline remain unchanged.  The
release pages still come from opened V105 outcomes, so this is a retrospective
headroom oracle rather than an online policy.

The same strict gate applies: at least 5% mean gain, nonnegative window gain
for every formation, nonnegative F6 non-gateway terminal gain, nonnegative
consensus gains, positive communication saving and all registered B3 checks.

- A pass establishes temporal headroom for a one-step-ahead protection-debt
  predictor.
- A failure that shifts the negative spike to t=76/t77 closes abrupt binary
  release and motivates gradual or receiver-selective reintroduction.
- A failure that removes local harm but drops the mean below 5% establishes a
  real fairness--tracking tradeoff rather than an implementation defect.
