# V106: protection-release timing headroom oracle

## First-principles question

V105 shows that control-only protection, without any topology handoff, is
sufficient for both the 5.259% network gain and the F1/F6 regressions.  The
per-formation timeline is not a monotone duration effect: F1 benefits through
t=76 and reverses at t=77, while F6 enters later and its non-gateway peers
reverse at t=78--79.  F3--F5 continue to benefit.  A universal dwell limit
would therefore discard useful protection or release a vulnerable formation
too late.

The useful control variable is the marginal value of one more protected page:

```text
continue protection while current rescue value exceeds accumulated
cross-formation information debt; otherwise return to the static payload.
```

Before designing an observable approximation, V106 tests whether timely
release has enough tracking headroom at all.

## Frozen oracle arm

V106 starts from the exact V105 schedule.  It releases F1 from t=77 onward and
F6 from t=78 onward, immediately before the opened V105 formation or peer
outcomes reverse sign.  All other formations retain the V105 schedule.  Every
adjacency and fusion-weight row stays on the static fixed-counter-clockwise
route, no handoff occurs, and the exact H=8 static outcome is reused.

Because opened V105 tracking outcomes choose the release pages, V106 is a
retrospective headroom oracle, not an online method.  A positive result only
authorizes the next step: replace the oracle pages with a causal lightweight
debt rule based on protection age, current receiver support and cross-input
opportunity.

## Decision gate

The arm must retain at least 5% mean E-OSPA gain, keep every formation
nonnegative over the H=8 window, make the F6 non-gateway terminal gain
nonnegative, preserve nonnegative consensus gains and save communication.
The per-formation/per-time matrix is reported as a stronger diagnostic but is
not tuned after the result opens.

- If V106 passes, implement a causal debt controller and validate on unseen
  X36 and M24 windows before any GNN work.
- If V106 preserves the mean but not local safety, temporal release alone is
  insufficient; the protected payload itself must become receiver-selective.
- If V106 loses the mean gain, the current benefit requires long-lived
  protection and the objective must explicitly optimize a fairness tradeoff.

## Opened result

V106 retains 5.246% mean E-OSPA gain and 5.286% attempted-byte saving, but
the strict gate fails.  F1 is -12.181% on its release page and remains
-0.865% over the window; F6 reaches -2.406% at the terminal page and its
non-gateway peers remain -2.947%.  Restoring the full static payload on the
first observed negative page is therefore too late and produces a switch
shock.  V107 will test the single remaining temporal hypothesis by advancing
both release pages by one step.
