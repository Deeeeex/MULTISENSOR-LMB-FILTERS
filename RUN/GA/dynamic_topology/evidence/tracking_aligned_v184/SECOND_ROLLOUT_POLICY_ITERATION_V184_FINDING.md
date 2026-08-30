# V181--V184 second-rollout policy-iteration finding

## What changed

The second DAgger-style aggregate contains 4,817 label-source actions from
42 receiver-time cells.  It promotes the previously opened V176 rollout
cells into training, excludes the feature-identical V180 t=78 duplicate,
and reserves only the newly visited V180 t=79 formation-5 cells as the next
heldout group.

The compact V181 sign classifier remains conservative on that heldout group:
it selects four actions, causes no harmful action, and yields immediate
E-OSPA/RMSE gains of `+23.306 / +0.490`.  Its RMSE utility capture is only
`5.6%`, however.  Ranking the same safe candidates with either joint-positive
utility (V182) or RMSE utility (V183) is not a valid remedy: both variants
select one harmful heldout action and fail the safety gate.

## Mechanism finding

The failure is set-structured rather than a lack of individually safe
actions.  Across all six formation-5 nodes in the second recursive t=79
state, the same already-present label `[31,24]` can be replaced from source
31.  That coordinated action improves every node, for aggregate immediate
E-OSPA/RMSE gains of `+1.047 / +9.861`; the weakest node still gains
`+0.136 / +1.349`.

V184 therefore evaluates one fixed formation-level rule.  It requires an
exact `(label, source)` pair shared by all six receivers, classifier lower
safety probability at least `0.60` for every receiver, at least four nodes
inside training support, an already-present receiver label, and observable
risk reduction no worse than `-0.05`.  Eligible actions are ranked by the
median of

```text
peer consensus x (1 - receiver/source compatibility) x source evidence quality.
```

Truth is not used by the rule.  Over the six opened V166/V176/V180 contexts,
the rule selects one common action in every context; all 36 receiver actions
are truth-safe, with positive minimum-node E-OSPA and RMSE gains.  The
duplicate V180 t=78 context is a consistency check, not an extra sample.

## Method decision

Do not continue scalar per-node utility regression.  Freeze the V184 rule as
a formation-coordinated posterior-repair selector and test it recursively as
the next development arm.  It should replace, not stack on top of, the V179
formation-5 per-node action so that the experiment measures the value of
coordination without silently increasing the edit budget.  The recursive
arm must still beat the static reference in aggregate and weakest-formation
E-OSPA/RMSE, consensus, and actual attempted bytes before it is eligible for
the main document.

## Evidence boundary

All states and targets here come from opened X36 seed-211 development data.
This finding supports a frozen recursive probe only.  It is neither
independent validation nor a paper-facing result.
