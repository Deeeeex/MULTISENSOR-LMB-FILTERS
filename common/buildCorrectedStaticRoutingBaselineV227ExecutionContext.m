function context = ...
        buildCorrectedStaticRoutingBaselineV227ExecutionContext( ...
            presetName, seed, splitName, armId, measurementTimeCount)
% BUILDCORRECTEDSTATICROUTINGBASELINEV227EXECUTIONCONTEXT Full episode.

protocol = getCorrectedStaticRoutingBaselineV227Protocol();
[allowedPresets, allowedSeeds] = splitMembership(protocol, splitName);
if ~ischar(presetName) || ~ismember(presetName, allowedPresets) || ...
        ~isscalar(seed) || ~ismember(seed, allowedSeeds) || ...
        ~ischar(armId) || ...
        ~any(strcmp(armId, {protocol.dynamicArmId, ...
            protocol.staticArmId})) || ...
        ~isscalar(measurementTimeCount) || ...
        ~isfinite(measurementTimeCount) || ...
        measurementTimeCount < 1 || ...
        measurementTimeCount ~= round(measurementTimeCount)
    error('CorrectedStaticRoutingV227:InvalidExecutionContextInput', ...
        'V227 requires a registered split, scene, seed, arm and time axis.');
end
if strcmp(armId, protocol.dynamicArmId)
    policyName = protocol.dynamicPolicyName;
else
    policyName = protocol.staticPolicyName;
end
context = struct();
context.contractVersion = ...
    'corrected-static-routing-v227-full-episode-context-v1';
context.capability = ...
    'corrected-static-routing-v227-full-episode-development';
context.action = ...
    'filter-corrected-static-routing-v227-full-episode-development';
context.protocolId = protocol.id;
context.presetName = presetName;
context.seed = seed;
context.splitName = splitName;
context.armId = armId;
context.measurementTimeCount = measurementTimeCount;
context.policyName = policyName;
context.developmentEvidenceOnly = true;
end

function [presets, seeds] = splitMembership(protocol, splitName)
switch splitName
    case 'training'
        presets = protocol.split.mainPresets;
        seeds = protocol.split.trainingSeeds;
    case 'calibration'
        presets = protocol.split.mainPresets;
        seeds = protocol.split.calibrationSeeds;
    case 'evaluation'
        presets = protocol.split.mainPresets;
        seeds = protocol.split.evaluationSeeds;
    case 'stress'
        presets = protocol.split.stressPresets;
        seeds = protocol.split.stressSeeds;
    case 'scale-extrapolation'
        presets = protocol.split.scaleExtrapolationPresets;
        seeds = protocol.split.scaleExtrapolationSeeds;
    otherwise
        error('CorrectedStaticRoutingV227:UnknownSplit', ...
            'The requested V227 split is not registered.');
end
end
