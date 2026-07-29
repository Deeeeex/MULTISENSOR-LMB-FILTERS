% RUNBACKBONERESIDUALSPLICEDCYCLEHEADROOM Frozen M24 cycle screen.

addpath(genpath(pwd));
protocol = getBackboneResidualSplicedCycleProtocol();
outputDirectory = strtrim(getenv('OUTPUT_DIR'));
if isempty(outputDirectory)
    outputDirectory = fullfile( ...
        'RUN', 'GA', 'dynamic_topology', 'evidence', ...
        'backbone_residual_spliced_cycle', ...
        'm24_hard_seed7');
end

options = struct();
options.maxTimeSteps = protocol.continuationEndTime;
options.continuationStartTime = ...
    protocol.continuationStartTime;
options.behaviorCacheDirectory = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'cache');
options.outputDirectory = outputDirectory;
options.armNames = protocol.armNames;
options.primaryAttributionFamily = 'rolling-safe';
options.evidenceSplit = 'development';
options.rollingSafeBackboneMode = ...
    'fixed-balanced-cycle';
options.rollingPayloadProjectionToleranceFraction = ...
    protocol.maximumAttemptedByteDeviationFraction;
options.generateMissingBehaviorCache = false;

runDynamicTopologyOracleGapScreen( ...
    protocol.presetName, protocol.seed, options);
