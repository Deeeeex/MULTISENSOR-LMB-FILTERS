# Introduction

## Target Story Arc

The introduction should move in this order:

1. Distributed multi-sensor tracking needs robust fusion under limited communication.
2. KLA or geometric-average fusion is attractive, but fixed weights ignore time-varying sensor reliability and heterogeneous packet loss.
3. Covariance and link quality alone still miss whether a local posterior makes a decisive existence judgment.
4. We introduce an adaptive weighting mechanism for KLA-based GA-LMB fusion using covariance, realized link quality, and existence-confidence.
5. Experiments show strong consensus gains and show that existence-confidence improves over the `covariance + link quality` baseline.

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

- Explain why posterior covariance alone does not capture existence reliability.
- Explain why communication statistics alone do not capture local posterior decisiveness.
- Set up the need for a third factor tied to target existence confidence.

Paragraph 4:

- Summarize the proposed method.
- Mention the factorized weight model.
- Mention covariance, link quality, existence-confidence, EMA smoothing, and minimum-weight safeguard.

Paragraph 5:

- Preview the main empirical findings.
- Mention strong consensus gains in tiered GA formation scenarios.
- Mention that `existence confidence` further improves all three consensus metrics over `covariance + link quality`.

Paragraph 6:

- List contributions in 3 concise bullets or sentences.

## Introduction Tone

- Keep the emphasis on distributed consensus quality.
- Avoid overselling local metric gains.
- Do not let AA dominate the story.

## Results To Preview

- Tiered GA ablation from `RUN/GA/GA_TIERED_LINK_ABLATION_20260322_001613.md`
- Tiered communication setup from `docs/COMMUNICATION_TIERED_DROP_UPDATE_CN.md`
