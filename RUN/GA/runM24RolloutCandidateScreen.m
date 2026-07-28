% RUNM24ROLLOUTCANDIDATESCREEN
% Reproducible M24 continuation screens used to expand the
% finite-horizon rolling-safe rollout candidate set.
%
% Invoke with, for example:
%   SEED=19 SCREEN=risk octave --quiet RUN/GA/runM24RolloutCandidateScreen.m
%   SEED=19 SCREEN=hybrid octave --quiet RUN/GA/runM24RolloutCandidateScreen.m
%   SEED=19 SCREEN=truthfree octave --quiet RUN/GA/runM24RolloutCandidateScreen.m

addpath(genpath(pwd));

seed = str2double(strtrim(getenv('SEED')));
if ~isscalar(seed) || ~isfinite(seed) || seed < 1 || mod(seed, 1) ~= 0
    error('Set SEED to one positive integer before running this screen.');
end
screenName = lower(strtrim(getenv('SCREEN')));
if isempty(screenName)
    error(['Set SCREEN to ''risk'', ''hybrid'', or ''truthfree'' ', ...
        'before running this experiment entry point.']);
end

options = struct();
options.maxTimeSteps = 77;
options.continuationStartTime = 75;
options.behaviorCacheDirectory = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'cache');
options.generateMissingBehaviorCache = false;
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
    case 'truthfree'
        options.armNames = { ...
            'directed-rolling-burst-r4-dccw-p2-w70', ...
            'rolling-safe-sequence-t75-s242480-w70', ...
            'rolling-safe-sequence-t75-s248080-w70', ...
            'rolling-safe-sequence-t75-s242481-w70', ...
            'rolling-safe-sequence-t75-s248181-w70'};
        outputLeaf = 'truthfree';
    otherwise
        error('Unknown SCREEN value: %s', screenName);
end

options.outputDirectory = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'rollout_dataset', sprintf('seed%d_%s', seed, outputLeaf));
runDynamicTopologyOracleGapScreen('m24-hard', seed, options);
