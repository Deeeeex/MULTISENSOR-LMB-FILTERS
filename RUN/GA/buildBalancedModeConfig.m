function cfg = buildBalancedModeConfig(baseAdaptiveFusionConfig)
% BUILDBALANCEDMODECONFIG Build the paper-facing Balanced operating mode.
%
% Balanced intentionally uses instantaneous adaptive weights. EMA smoothing
% and final-weight floors are disabled on both branches because the paired
% component ablation showed that they degrade network disagreement,
% cardinality dispersion, local E-OSPA, and local cardinality error.

if nargin < 1 || isempty(baseAdaptiveFusionConfig)
    baseAdaptiveFusionConfig = struct();
end

cfg = baseAdaptiveFusionConfig;
cfg.enabled = true;
cfg.method = 'factorized';
cfg.useCovariance = true;
cfg.useLinkQuality = true;
cfg.useFreshness = false;
cfg.useNIS = false;
cfg.useExistenceConfidence = true;
cfg.useDecoupledKla = true;
cfg.useStructureAwareKla = true;
cfg.usePosteriorStructureConsistency = false;
cfg.useFidFiaExistence = false;

if ~isfield(cfg, 'existenceConfidenceMinScore') || ...
        abs(cfg.existenceConfidenceMinScore - 0.6) < 1e-9
    cfg.existenceConfidenceMinScore = 0.85;
end
if ~isfield(cfg, 'existenceConfidencePower') || ...
        abs(cfg.existenceConfidencePower - 1.0) < 1e-9
    cfg.existenceConfidencePower = 2.0;
end
if ~isfield(cfg, 'spatialDecouplingStrength') || ...
        abs(cfg.spatialDecouplingStrength - 1.0) < 1e-9
    cfg.spatialDecouplingStrength = 0.5;
end
if ~isfield(cfg, 'existenceDecouplingStrength') || ...
        abs(cfg.existenceDecouplingStrength - 1.0) < 1e-9
    cfg.existenceDecouplingStrength = 0.15;
end
if ~isfield(cfg, 'spatialStructureStrength') || cfg.spatialStructureStrength <= 0
    cfg.spatialStructureStrength = 0.45;
end
if ~isfield(cfg, 'existenceStructureStrength') || cfg.existenceStructureStrength <= 0
    cfg.existenceStructureStrength = 0.08;
end
if ~isfield(cfg, 'structureReliabilityPower') || cfg.structureReliabilityPower <= 0
    cfg.structureReliabilityPower = 0.30;
end

% Do not reintroduce temporal smoothing or final-weight protection into the
% Balanced operating mode without a new paired ablation demonstrating gain.
cfg.emaAlpha = 0.0;
cfg.minWeight = 0.0;
cfg.spatialEmaAlpha = 0.0;
cfg.existenceEmaAlpha = 0.0;
cfg.spatialMinWeight = 0.0;
cfg.existenceMinWeight = 0.0;
end
