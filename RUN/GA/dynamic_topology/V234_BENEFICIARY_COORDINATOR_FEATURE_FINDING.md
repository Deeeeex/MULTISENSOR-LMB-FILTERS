# V234 beneficiary coordinator features

## What the coordinator can see

V233 guarantees that the participating donor sources can expose every active
label within two bounded offer phases, but it does not identify which label is
valuable.  V234 freezes the pre-fetch information available to one beneficiary
coordinator.  Each donor record is exactly 24 bytes:

- a four-byte semantic label key;
- quantized existence probability and current evidence quality;
- two float32 position coordinates and one float32 position-covariance trace;
- one uint32 complete-label payload size.

The source identity is carried by the response envelope.  The coordinator
compares these records with its own current fused posterior and derives only
bounded, scale-normalized quantities: positive existence deficit/excess,
absolute log-odds gap, precision and evidence gaps, isotropic position
compatibility, local risk reduction and payload-credit fraction.  Numeric
label identifiers are routing keys and deterministic tie-breaks, not model
features.  The full source mixture and realized eta remain unavailable until
after one payload has been selected and fetched.

The coordinator rule in this first screen is the existing V228 convention:
the first registered receiver in the beneficiary formation.  Four causal
proposal modes are retained—existence rescue, precision refresh, credible
disagreement and risk reduction.  These scores identify different action
hypotheses; they are not H=3 value predictions.

## Same-state rank result

At X36 `seed=1301, t=133`, the F5 coordinator S25 receives 16 offers from S2.
The teacher S2/[1,4] label ranks fourth in both credible-disagreement and
risk-reduction modes, but only twelfth in existence and precision modes.  The
F6 coordinator S31 receives 71 offers from S1/S3/S5.  The teacher S1/[25,20]
label ranks second for precision refresh, fourth for risk reduction, tenth
for credible disagreement and sixtieth for existence rescue.

Thus a fixed top-three bank recovers only one of two teacher rows, whereas the
union of top-six per mode recovers both.  This is the important design
boundary: compact causal features are expressive enough, but an aggressively
narrow global shortlist is not.  The top-six mode bank may be used to collect
H=3 labels only if V228 first confirms material teacher headroom.  Until then,
no ridge/GNN training or online tracking claim is authorized.
