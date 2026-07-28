% RUNM24ROLLOUTCANDIDATESCREEN
% Reproducible M24 continuation screens used to expand the
% finite-horizon rolling-safe rollout candidate set.
%
% Invoke with, for example:
%   SEED=19 SCREEN=risk octave --quiet RUN/GA/runM24RolloutCandidateScreen.m
%   SEED=19 SCREEN=hybrid octave --quiet RUN/GA/runM24RolloutCandidateScreen.m
%   SEED=19 SCREEN=diverse octave --quiet RUN/GA/runM24RolloutCandidateScreen.m
%   SEED=19 SCREEN=truthfree octave --quiet RUN/GA/runM24RolloutCandidateScreen.m
%   SEED=7 SCREEN=learned octave --quiet RUN/GA/runM24RolloutCandidateScreen.m
%   SEED=7 SCREEN=counterfactual BATCH_COUNT=4 BATCH_INDEX=1 \
%       octave --quiet RUN/GA/runM24RolloutCandidateScreen.m

addpath(genpath(pwd));

seed = str2double(strtrim(getenv('SEED')));
if ~isscalar(seed) || ~isfinite(seed) || seed < 1 || mod(seed, 1) ~= 0
    error('Set SEED to one positive integer before running this screen.');
end
screenName = lower(strtrim(getenv('SCREEN')));
if isempty(screenName)
    error(['Set SCREEN to ''risk'', ''hybrid'', ''diverse'', ', ...
        '''truthfree'', ''learned'', or ''counterfactual'' before ', ...
        'running this entry point.']);
end

options = struct();
options.maxTimeSteps = 77;
options.continuationStartTime = 75;
options.behaviorCacheDirectory = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'cache');
generateCacheToken = lower(strtrim(getenv('GENERATE_CACHE')));
options.generateMissingBehaviorCache = ...
    any(strcmp(generateCacheToken, {'1', 'true', 'yes'}));
options.primaryAttributionFamily = 'rolling-safe';

switch screenName
    case 'risk'
        options.armNames = { ...
            'directed-rolling-burst-r4-dccw-p2-w70', ...
            'oracle-rolling-safe-minimax-w70'};
        outputLeaf = 'risk';
    case 'hybrid'
        masks = { ...
            '000', '001', '010', '011', ...
            '100', '101', '110', '111'};
        options.armNames = cellfun(@(mask) sprintf([ ...
            'oracle-rolling-safe-hybrid-t75-m%s-', ...
            'r4-dccw-p2-w70'], mask), ...
            masks, 'UniformOutput', false);
        outputLeaf = 'hybrid';
    case 'diverse'
        options.armNames = { ...
            'directed-rolling-burst-r4-dccw-p2-w70', ...
            'oracle-rolling-safe-sequence-t75-s000000-w70', ...
            'oracle-rolling-safe-sequence-t75-s900000-w70', ...
            'oracle-rolling-safe-sequence-t75-s910000-w70', ...
            'oracle-rolling-safe-sequence-t75-s920000-w70'};
        outputLeaf = 'diverse';
    case 'truthfree'
        options.armNames = { ...
            'directed-rolling-burst-r4-dccw-p2-w70', ...
            'rolling-safe-sequence-t75-s242480-w70', ...
            'rolling-safe-sequence-t75-s248080-w70', ...
            'rolling-safe-sequence-t75-s242481-w70', ...
            'rolling-safe-sequence-t75-s248181-w70'};
        outputLeaf = 'truthfree';
    case 'learned'
        options.allowRejectedLearnedArtifact = true;
        options.evidenceSplit = 'development';
        options.armNames = { ...
            'directed-rolling-burst-r4-dccw-p2-w70', ...
            'learned-rolling-safe-rollout-w70'};
        outputLeaf = 'learned';
    case 'counterfactual'
        actionCodes = [1:24, 80:87];
        batchCount = str2double(strtrim(getenv('BATCH_COUNT')));
        batchIndex = str2double(strtrim(getenv('BATCH_INDEX')));
        if ~isfinite(batchCount)
            batchCount = 1;
        end
        if ~isfinite(batchIndex)
            batchIndex = 1;
        end
        batchCount = round(batchCount);
        batchIndex = round(batchIndex);
        if batchCount < 1 || batchIndex < 1 || ...
                batchIndex > batchCount
            error(['BATCH_INDEX must lie in 1:BATCH_COUNT for the ', ...
                'counterfactual screen.']);
        end
        selectedCodes = actionCodes( ...
            batchIndex:batchCount:numel(actionCodes));
        options.armNames = arrayfun(@(code) sprintf( ...
            'rolling-safe-sequence-t75-s%02d2424-w70', code), ...
            selectedCodes, 'UniformOutput', false);
        outputLeaf = sprintf( ...
            'counterfactual_b%02dof%02d', ...
            batchIndex, batchCount);
    otherwise
        error('Unknown SCREEN value: %s', screenName);
end

options.outputDirectory = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'rollout_dataset', sprintf('seed%d_%s', seed, outputLeaf));
runDynamicTopologyOracleGapScreen('m24-hard', seed, options);
