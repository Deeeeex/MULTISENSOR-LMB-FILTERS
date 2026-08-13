# V137 causal predictive-evidence fallback

V136 proves that formation-isolated W propagation has enough M24 aggregate
headroom, but formation 2 contains both beneficial gateways and harmed peers.
A scalar W--R disagreement threshold cannot separate them: harmful and useful
cells overlap strongly in disagreement magnitude.

V137 therefore treats W/R selection as online model comparison.  At time
`t`, each node already has two causal predictions from its previous W and R
states.  Before the current measurement update, it scores both predictions on
the current measurement set using the cardinality-matched IID-cluster LMB log
score.  The node continues from W only when

```text
log score(W prediction; current measurements)
  >= log score(R prediction; current measurements).
```

Otherwise it performs the ordinary current update from R and uses R for the
current working output.  The score is proper for the registered IID-cluster
projection; it uses neither target truth nor a future measurement.  The rule
applies only while that node's formation is protected.  It adds no message or
payload beyond V136's already charged compound W+R representation and retains
exact whole-formation reentry.

The threshold is fixed at zero before outcomes open; no margin sweep is
allowed.  V137 must preserve V136's at-least-five-percent intervention gain,
make every sensor and formation nonnegative, and keep exact reentry.  An M24
failure closes the branch before X36.  A pass authorizes the paired X36
mechanism screen, but still does not establish a communication-saving method
because the compound payload remains expensive.
