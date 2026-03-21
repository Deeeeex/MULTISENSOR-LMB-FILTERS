# Introduction

## Target Story Arc

The introduction should move in this order:

1. Distributed multi-sensor tracking needs robust fusion under limited communication.
2. KLA or geometric-average fusion is attractive, but fixed weights ignore time-varying sensor reliability.
3. Naive uncertainty-aware weighting is not enough because NIS and covariance are structurally coupled.
4. We introduce a consistency-aware adaptive weighting mechanism for KLA-based GA-LMB fusion.
5. Experiments show strong consensus gains and show why `robust NIS` is better than plain `NIS`.

## Suggested Paragraph Plan

Paragraph 1:

- Introduce distributed multi-sensor multi-object tracking.
- Mention communication constraints, packet loss, outages, and heterogeneous local quality.
- Motivate adaptive fusion weighting.

Paragraph 2:

- Introduce KLA or geometric-average fusion as a practical distributed fusion tool.
- State the limitation of fixed or topology-only weights.
- Emphasize that local posterior quality and link quality change over time.

Paragraph 3:

- Explain the pitfall of directly using NIS as a quality reward.
- Briefly state the covariance coupling issue.
- Set up the need for decoupled consistency-aware design.

Paragraph 4:

- Summarize the proposed method.
- Mention the factorized weight model.
- Mention robust NIS, EMA smoothing, and minimum-weight safeguard.

Paragraph 5:

- Preview the main empirical findings.
- Mention strong consensus gains in GA formation scenarios.
- Mention that `robust NIS` is stable while plain `NIS` is not.

Paragraph 6:

- List contributions in 3 concise bullets or sentences.

## Introduction Tone

- Keep the emphasis on distributed consensus quality.
- Avoid overselling local metric gains.
- Do not let AA dominate the story.

## Results To Preview

- Main GA formation result from `docs/FORMATION_4PLUS4_RUN.md`
- GA NIS ablation from `RUN/GA/GA_NIS_COMPARE_20260309_164119.md`
