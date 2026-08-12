# V125 outward-reference carrier

V124 restores the F6 receiver row but still leaves a negative F6 tail. The
remaining causal ambiguity is whether F5's protected posterior, rather than
the F6 row itself, contaminates the persistent F5-to-F6 relay.

V125 keeps the complete V124 topology, weights and receiver-side protection.
For each page from t=72 to t=79, only the full posterior transmitted from
sensor 27 (F5) to sensor 32 (F6) is replaced by the paired clockwise-static
sender posterior captured at the same page. F5 continues to use its V124 local
posterior, so the intervention separates local estimation state from outward
relay state.

This is a counterfactual upper bound, not a deployable algorithm: it requires
a shadow posterior trajectory from another arm. It passes only with at least
5% aggregate and mature-page gain over clockwise static, no formation,
terminal-formation, F6-peer, worst-sensor or consensus regression, nonnegative
attempted-byte saving, and rolling-B3 safety. A failed result remains an
experiment record and is not promoted to the main progress document.
