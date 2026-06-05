function arms = buildRigorousAblationArms(baseAdaptiveFusionConfig)
% BUILDRIGOROUSABLATIONARMS Build the reviewer-facing component ablation.
%
% Arms 1-8 intentionally disable EMA and final weight floors. This keeps
% each transition attributable to the named factor or architectural change.
% Arm 9 adds the paper-facing stabilization settings. Arm 10 adds FID-FIA
% only to the existence branch while retaining stabilization, so its delta
% isolates the FID-FIA refinement.
%
% Arm 11 is an optional deployment control. It matches the current
% Cardinality-critical operating mode, where the existence branch removes
% EMA/floor protection to permit stronger suppression. It should not replace
% arm 10 in the causal ten-row ablation table.

if nargin < 1 || isempty(baseAdaptiveFusionConfig)
    baseAdaptiveFusionConfig = struct();
end

arms = repmat(struct('name', '', 'purpose', '', 'adaptiveFusion', struct()), 1, 11);
rawBase = resetFactorizedConfig(baseAdaptiveFusionConfig);

cfg = rawBase;
cfg.enabled = false;
arms(1) = makeArm('Fixed Metropolis', ...
    'Fixed topology-weight baseline.', cfg);

cfg = rawBase;
cfg.useCovariance = true;
arms(2) = makeArm('Cov only', ...
    'Isolate posterior concentration.', cfg);

cfg = rawBase;
cfg.useLinkQuality = true;
arms(3) = makeArm('Link only', ...
    'Isolate realized communication quality.', cfg);

cfg = rawBase;
cfg.useExistenceConfidence = true;
arms(4) = makeArm('Exist only', ...
    'Isolate Bernoulli existence confidence under shared scalar weights.', cfg);

cfg = rawBase;
cfg.useCovariance = true;
cfg.useLinkQuality = true;
arms(5) = makeArm('Cov + Link', ...
    'Measure the covariance-link adaptive backbone.', cfg);

cfg.useExistenceConfidence = true;
arms(6) = makeArm('Cov + Link + Exist, shared weights', ...
    'Test existence confidence before branch decoupling.', cfg);

cfg.useDecoupledKla = true;
cfg.spatialDecouplingStrength = 0.5;
cfg.existenceDecouplingStrength = 0.15;
arms(7) = makeArm('Cov + Link + Exist, branch-decoupled', ...
    'Isolate branch-specific spatial and existence weights.', cfg);

cfg.useStructureAwareKla = true;
cfg.usePosteriorStructureConsistency = false;
cfg.spatialStructureStrength = 0.45;
cfg.existenceStructureStrength = 0.08;
cfg.structureReliabilityPower = 0.30;
cfg.structureReliabilityMinScore = 0.25;
arms(8) = makeArm('+ structure-aware', ...
    'Isolate the weak topology and communication-reliability prior.', cfg);

cfg.emaAlpha = 0.7;
cfg.minWeight = 0.05;
cfg.spatialEmaAlpha = 0.7;
cfg.existenceEmaAlpha = 0.7;
cfg.spatialMinWeight = 0.05;
cfg.existenceMinWeight = 0.05;
arms(9) = makeArm('+ EMA/floor', ...
    'Isolate temporal smoothing and final-weight protection.', cfg);

cfg.useFidFiaExistence = true;
cfg.fidFiaExistenceStrength = 4.0;
cfg.fidFiaExistenceMinScore = 0.0;
cfg.fidFiaExistencePower = 1.0;
cfg.fidFiaQuadraturePoints = 3;
cfg.fidFiaUseDetectionProbability = true;
cfg.fidFiaUseExistenceWeight = true;
arms(10) = makeArm('+ FID-FIA existence only', ...
    'Isolate FID-FIA on the existence branch while retaining stabilization.', cfg);

cfg.existenceEmaAlpha = 0.0;
cfg.existenceMinWeight = 0.0;
arms(11) = makeArm('Cardinality-critical deployed control', ...
    'Match the deployed mode by removing existence EMA/floor after FID-FIA.', cfg);
end

function cfg = resetFactorizedConfig(base)
cfg = base;
cfg.enabled = true;
cfg.method = 'factorized';
cfg.useCovariance = false;
cfg.useLinkQuality = false;
cfg.useExistenceConfidence = false;
cfg.useDecoupledKla = false;
cfg.useStructureAwareKla = false;
cfg.usePosteriorStructureConsistency = false;
cfg.useFidFiaExistence = false;
cfg.useFreshness = false;
cfg.useCtFiDecay = false;
cfg.useCardinalityConsensus = false;
cfg.useAssociationAmbiguity = false;
cfg.useHistory = false;
cfg.useHistorySmoothedExistenceConfidence = false;
cfg.useNIS = false;
cfg.robustNIS = false;

cfg.existenceConfidenceMinScore = 0.85;
cfg.existenceConfidencePower = 2.0;
cfg.spatialCovariancePower = 1.0;
cfg.spatialLinkQualityPower = 1.0;
cfg.existenceLinkQualityPower = 1.0;
cfg.existenceConfidenceWeightPower = 1.0;
cfg.spatialDecouplingStrength = 0.0;
cfg.existenceDecouplingStrength = 0.0;
cfg.spatialStructureStrength = 0.0;
cfg.existenceStructureStrength = 0.0;
cfg.structureReliabilityPower = 0.0;

cfg.emaAlpha = 0.0;
cfg.minWeight = 0.0;
cfg.spatialEmaAlpha = 0.0;
cfg.existenceEmaAlpha = 0.0;
cfg.spatialMinWeight = 0.0;
cfg.existenceMinWeight = 0.0;
end

function arm = makeArm(name, purpose, cfg)
arm = struct('name', name, 'purpose', purpose, 'adaptiveFusion', cfg);
end
