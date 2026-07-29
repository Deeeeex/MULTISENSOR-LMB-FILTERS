% RUNBACKBONEPRESERVINGRESIDUALHEADROOM
% Reproducible M24/X36 development gate for additive residual routing.
%
% Example:
%   PRESET=m24-hard SEED=7 \
%     octave --quiet RUN/GA/runBackbonePreservingResidualHeadroom.m
%   PRESET=x36-clean-scale SEED=7 \
%     octave --quiet RUN/GA/runBackbonePreservingResidualHeadroom.m
%   PROTOCOL_VARIANT=v2-e10 PRESET=m24-hard SEED=7 \
%     octave --quiet RUN/GA/runBackbonePreservingResidualHeadroom.m

addpath(genpath(pwd));

protocolVariant = strtrim(getenv('PROTOCOL_VARIANT'));
if isempty(protocolVariant)
    protocolVariant = 'v1-e05';
end
protocol = getBackbonePreservingResidualProtocol( ...
    protocolVariant);
presetName = lower(strtrim(getenv('PRESET')));
if ~ismember(presetName, protocol.scenarioPresets)
    error('PRESET must be one of: %s.', ...
        strjoin(protocol.scenarioPresets, ', '));
end
seed = str2double(strtrim(getenv('SEED')));
if ~isscalar(seed) || ~isfinite(seed) || ...
        seed < 1 || seed ~= round(seed)
    error('SEED must be one positive integer.');
end

outputDirectory = strtrim(getenv('OUTPUT_DIR'));
if isempty(outputDirectory)
    outputDirectory = fullfile( ...
        'RUN', 'GA', 'dynamic_topology', 'evidence', ...
        'backbone_residual_headroom', sprintf( ...
            '%s_%s_seed%d', strrep(protocol.variant, '-', '_'), ...
            strrep(presetName, '-', '_'), seed));
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
options.evidenceSplit = protocol.evidenceSplit;
options.rollingSafeBackboneMode = ...
    protocol.residualBackboneMode;
options.rollingPayloadProjectionToleranceFraction = ...
    protocol.maximumAttemptedByteDeviationFraction;
options.generateMissingBehaviorCache = true;

runDynamicTopologyOracleGapScreen( ...
    presetName, seed, options);
