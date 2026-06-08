# Conclusion

## Paper-Ready Conclusion Draft

This paper studies adaptive weight allocation for distributed KLA-based LMB fusion under heterogeneous packet loss. The revised evidence supports a narrower conclusion than a monolithic “all modules help” claim. Realized link quality is the dominant factor in the tested communication-constrained scenario. Covariance quality provides an independent posterior-concentration signal and gives a further cardinality benefit when combined with link quality. The combined branch-aware refinement then supplies a small but repeatable improvement in OSPA consensus error and matched localization disagreement.

Existence confidence has little isolated incremental effect on the covariance-link backbone in the present experiment. Branch decoupling and weak structure-aware modulation are therefore evaluated together as one branch-aware refinement rather than assigned separate empirical claims. The detailed ablation makes this distinction explicit and separates architectural motivation from measured effect size.

The retained Balanced configuration uses instantaneous branch weights without EMA smoothing or final-weight floors. Relative to Fixed Metropolis, it reduces OSPA consensus error, matched localization disagreement, and cardinality dispersion by `28.8%`, `35.5%`, and `85.4%`, respectively, with approximately `1.10x` fixed filtering/fusion runtime. Its truth-referenced local E-OSPA and cardinality error remain substantially below the fixed baseline, while local RMSE stays close to the covariance-link backbone.

The Cardinality-critical mode retains the same no-stabilization backbone and adds FID-FIA only to the existence branch. Relative to Balanced, it reduces cardinality dispersion by `34.8%`, local E-OSPA by `2.0%`, and local cardinality error by `26.2%`. The tradeoff is explicit: OSPA consensus error increases by `1.0%`, localization disagreement by `8.9%`, and local RMSE by `6.6%`. Balanced is therefore the spatial-consensus and lower-cost mode, whereas Cardinality-critical is appropriate when target-number errors dominate.

The former EMA/floor pair is not retained in either mode. The revised modes are currently confirmed only in the main tiered-loss scenario. Repeating ideal-communication, communication-level, and arithmetic-average studies is the next validation step. Further work should also consider richer sensor-parameter heterogeneity, asynchronous or multi-rate communication, and target-wise information measures once reliable label association across neighboring LMB posteriors is available.
