% RUNPREDICTIVEREWARDALIGNMENTM24ARM Run one frozen M24 score-gate arm.
%
% Environment:
%   ARM_MODE   One mode registered by getPredictiveRewardAlignmentProtocol.
%   OUTPUT_DIR Destination directory for the one-arm evidence package.

addpath(genpath(pwd));
protocol = getPredictiveRewardAlignmentProtocol();
armMode = strtrim(getenv('ARM_MODE'));
if isempty(armMode)
    error('ARM_MODE is required.');
end
if ~any(strcmp(protocol.armModes, armMode))
    error('ARM_MODE is outside the frozen score-gate arm set: %s.', ...
        armMode);
end
outputDirectory = strtrim(getenv('OUTPUT_DIR'));
if isempty(outputDirectory)
    outputDirectory = fullfile( ...
        'RUN', 'GA', 'dynamic_topology', 'evidence', ...
        'predictive_reward_alignment_m24_seed7_h3_v1', ...
        strrep(armMode, '-', '_'));
end

options = struct();
options.maxTimeSteps = protocol.continuationEndTime;
options.continuationStartTime = ...
    protocol.continuationStartTime;
options.behaviorCacheDirectory = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'cache');
options.outputDirectory = outputDirectory;
options.armNames = {armMode};
options.primaryAttributionFamily = 'rolling-safe';
options.evidenceSplit = 'development';
options.filterSeedOffset = protocol.filterSeedOffset;
options.rollingSafeBackboneMode = ...
    protocol.rollingSafeBackboneMode;
options.rollingPayloadProjectionToleranceFraction = ...
    protocol.maximumAttemptedByteDeviationFraction;
options.generateMissingBehaviorCache = false;

runDynamicTopologyOracleGapScreen( ...
    protocol.presetName, protocol.seed, options);
