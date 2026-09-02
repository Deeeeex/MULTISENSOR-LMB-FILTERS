# V258 event-window residual reinforcement

## Decision being tested

V248 established the strongest corrected-scene M24 network average so far:
the V242 minimum causal backbone improves E-OSPA, RMSE, focus consistency and
attempted bytes relative to the matched fixed formation tree.  Its failure is
localized rather than global.  Formation 4 has a large localization-RMSE tail
during `t=58--73`, even though its E-OSPA and cardinality are better than the
fixed reference.  This pattern is consistent with enough information reaching
the formation to retain targets, but too little redundant local information to
localize the matched states accurately.

V258 asks one narrow mechanism question: can a single local residual input,
restored only during that diagnosed event and only at receivers in formation
4, repair the localization tail without giving up the sparse backbone's
network-level and communication gains?

## Intervention

- Base route: V242, with `N + 2(F-1)` directed messages per step.
- Event: the already diagnosed `t=58--73` interval.
- Eligible receivers: formation 4 only.
- Action: restore at most one currently physical local V240 residual input.
- Selection: reuse the V246 exact current-posterior one-round Pareto guard.
- Outside the event: return V242 exactly and do not enumerate residuals.
- Maximum route size on M24: 31 messages per step, versus 30 for V242 and
  48 for the full causal route.

The policy executed by the filter reads no truth or future outcomes.  However,
the event and receiver formation were selected from the already opened V248
tracking result.  V258 is therefore a posthoc mechanism experiment, not a
deployable method and not validation evidence.

## Falsification gate

The mechanism is supported only if all of the following hold on the complete
160-step paired episode:

1. formation-4 RMSE over `t=58--73` improves over V242;
2. formation-4 E-OSPA over that event does not worsen relative to V242;
3. full-horizon E-OSPA, RMSE and focus consistency remain better than the
   matched fixed tree;
4. attempted bytes remain below the matched fixed tree;
5. the weakest full-horizon formation E-OSPA does not regress versus the fixed
   tree.

If V258 fails even with this outcome-informed schedule, local residual
reinforcement is not the right mechanism for the V248 tail.  The next
intervention should be coordinated cross-cut reinforcement.  If it passes, the
next step is to replace the posthoc time/formation labels with a causal event
detector based on observable localization uncertainty, disagreement and
handover persistence before opening any new seed.
