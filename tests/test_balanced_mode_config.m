function test_balanced_mode_config()
% TEST_BALANCED_MODE_CONFIG Prevent EMA/floor from returning to Balanced.

base = struct( ...
    'emaAlpha', 0.7, ...
    'minWeight', 0.05, ...
    'spatialEmaAlpha', 0.7, ...
    'existenceEmaAlpha', 0.7, ...
    'spatialMinWeight', 0.05, ...
    'existenceMinWeight', 0.05, ...
    'existenceConfidenceMinScore', 0.6, ...
    'existenceConfidencePower', 1.0, ...
    'spatialDecouplingStrength', 1.0, ...
    'existenceDecouplingStrength', 1.0, ...
    'spatialStructureStrength', 0.0, ...
    'existenceStructureStrength', 0.0, ...
    'structureReliabilityPower', 0.0);

cfg = buildBalancedModeConfig(base);

assert(cfg.enabled);
assert(strcmp(cfg.method, 'factorized'));
assert(cfg.useCovariance);
assert(cfg.useLinkQuality);
assert(cfg.useExistenceConfidence);
assert(cfg.useDecoupledKla);
assert(cfg.useStructureAwareKla);
assert(~cfg.usePosteriorStructureConsistency);
assert(~cfg.useFidFiaExistence);

assert(cfg.emaAlpha == 0.0);
assert(cfg.minWeight == 0.0);
assert(cfg.spatialEmaAlpha == 0.0);
assert(cfg.existenceEmaAlpha == 0.0);
assert(cfg.spatialMinWeight == 0.0);
assert(cfg.existenceMinWeight == 0.0);

assert(cfg.existenceConfidenceMinScore == 0.85);
assert(cfg.existenceConfidencePower == 2.0);
assert(cfg.spatialDecouplingStrength == 0.5);
assert(cfg.existenceDecouplingStrength == 0.15);
assert(cfg.spatialStructureStrength == 0.45);
assert(cfg.existenceStructureStrength == 0.08);
assert(cfg.structureReliabilityPower == 0.30);

fidCfg = buildFidFiaExistenceModeConfig(base);
assert(fidCfg.useFidFiaExistence);
assert(fidCfg.fidFiaExistenceStrength == 4.0);
assert(fidCfg.fidFiaExistenceMinScore == 0.0);
assert(fidCfg.emaAlpha == 0.0);
assert(fidCfg.minWeight == 0.0);
assert(fidCfg.spatialEmaAlpha == 0.0);
assert(fidCfg.existenceEmaAlpha == 0.0);
assert(fidCfg.spatialMinWeight == 0.0);
assert(fidCfg.existenceMinWeight == 0.0);

fprintf('test_balanced_mode_config passed\n');
end
