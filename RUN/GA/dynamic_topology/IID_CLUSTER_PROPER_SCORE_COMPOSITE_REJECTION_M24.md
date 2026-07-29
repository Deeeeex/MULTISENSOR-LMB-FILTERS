# M24 proper-score composite audit

- Source protocols:
  - `predictive-reward-action-alignment-m24-training-states-v1`
  - `iid-cluster-reward-action-alignment-m24-training-states-v1`
- Training seeds: `[11 17 19 23 27 29]`
- Snapshot times: `[78 79 80 81 82 83]`
- Compared score:
  \[
  S_\alpha=(1-\alpha)S_{\mathrm{Poisson}}+
  \alpha S_{\mathrm{IID}},\qquad
  \alpha\in\{0,0.01,\ldots,1\}.
  \]
- Target truth was used only to audit alignment on the already-opened
  training states. No development or held-out seed was opened.

| Composite | Teacher positive | Global Spearman | Pairwise preference | Positive-seed fraction |
|:--|--:|--:|--:|--:|
| \(\alpha=0\), Poisson | 0.5417 | 0.4196 | 0.5903 | 1.0000 |
| \(\alpha=0.25\) | 0.5347 | 0.4242 | 0.6111 | 1.0000 |
| \(\alpha=0.50\) | 0.5347 | 0.4297 | 0.6111 | 1.0000 |
| \(\alpha=0.75\) | 0.5347 | 0.4367 | 0.6042 | 1.0000 |
| \(\alpha=1\), IID-cluster | 0.5347 | 0.4384 | 0.6111 | 1.0000 |

The frozen gates were teacher-positive fraction at least 0.55, global
Spearman at least 0.25, pairwise preference at least 0.60, and positive
per-seed Spearman on at least four of six seeds. No value on the complete
101-point grid passed every gate. The largest teacher-positive fraction
was 0.5417 at \(\alpha=0\); the largest pairwise preference was 0.6111,
first reached at \(\alpha=0.24\); and the largest global Spearman was
0.4384 near \(\alpha=0.99\).

Therefore a post-hoc cardinality weight cannot rescue the rejected
absolute-reward bandit. The admissible next use of the IID-cluster score
is narrower: test whether its delayed pairwise/full-information signal is
learnable across seeds for ranking safe candidates. This does not
authorize a bandit, development evaluation, held-out M24, or X36.
