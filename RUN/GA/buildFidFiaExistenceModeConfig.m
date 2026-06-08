function cfg = buildFidFiaExistenceModeConfig(baseAdaptiveFusionConfig)
% BUILDFIDFIAEXISTENCEMODECONFIG Add FID-FIA to no-stabilization Balanced.
%
% This operating point differs from Balanced only on the existence branch.
% It must retain instantaneous weights and zero final-weight floors.

if nargin < 1 || isempty(baseAdaptiveFusionConfig)
    baseAdaptiveFusionConfig = struct();
end

cfg = buildBalancedModeConfig(baseAdaptiveFusionConfig);
cfg.useFidFiaExistence = true;
cfg.fidFiaExistenceStrength = 4.0;
cfg.fidFiaExistenceMinScore = 0.0;
cfg.fidFiaExistencePower = 1.0;
cfg.fidFiaQuadraturePoints = 3;
cfg.fidFiaUseDetectionProbability = true;
cfg.fidFiaUseExistenceWeight = true;

% Keep this extension directly comparable with the retained Balanced mode.
cfg.emaAlpha = 0.0;
cfg.minWeight = 0.0;
cfg.spatialEmaAlpha = 0.0;
cfg.existenceEmaAlpha = 0.0;
cfg.spatialMinWeight = 0.0;
cfg.existenceMinWeight = 0.0;
end
