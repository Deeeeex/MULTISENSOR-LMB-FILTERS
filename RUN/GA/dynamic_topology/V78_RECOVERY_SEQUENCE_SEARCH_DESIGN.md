# V78 recovery-sequence search design

V77 shows that a direct-safe pulse followed by two fixed-reference rounds can
re-amplify centered label-existence perturbations.  V78 asks whether the
existing route action already contains a better recovery schedule before
expanding the topology or weight space.

The first round is always the V75-safe candidate.  Each of the next two rounds
uses either the reference route (`R`) or the same candidate route (`C`), giving
four schedules: `CRR`, `CRC`, `CCR`, and `CCC`.  Historical V71/V72 and aligned
V73 exact slot generations are searched separately on M24 formation 3 and X36
formation 4.

Before every `C` round, V78 recomputes the frozen V75 replacement innovation
energy on the current virtual posteriors.  A schedule is ineligible if any
candidate round exceeds `5.991464547` or loses shared-label coverage.  This
prevents persistence from silently violating the direct spatial trust region.

Eligible schedules are ranked without a fitted percentage threshold:

1. minimum maximum centered-energy factor after the first round;
2. minimum terminal centered-energy factor;
3. minimum number of candidate recovery rounds.

The report also states whether any schedule is monotonically non-increasing.
A positive answer means the current binary route action has source-only
recovery headroom.  If all four schedules rebound, later work must add a
balanced recovery route or weight action; changing the centered-energy
threshold would not solve the missing control authority.

V78 uses the same deterministic current-link, no-measurement KLA replay as
V77.  It is an architecture headroom screen, not an online policy, tracking
result, or validation claim.
