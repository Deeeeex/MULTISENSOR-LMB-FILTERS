function [shardPath, shard] = ...
    runSafeGraphOptionDwellV154ArmShard( ...
        presetName, seed, rankIndex, options)
% RUNSAFEGRAPHOPTIONDWELLV154ARMSHARD Execute one parallel-safe option arm.

if nargin < 4 || isempty(options)
    options = struct();
end
protocol = getSafeGraphOptionDwellV154Protocol();
presetIdx = find(strcmp(protocol.presetNames, presetName));
rankIndex = round(rankIndex);
if numel(presetIdx) ~= 1 || ...
        ~ismember(seed, protocol.openedDevelopmentSeeds) || ...
        ~ismember(rankIndex, protocol.optionRanks)
    error('V154 arm shard is outside the frozen option bank.');
end
armMode = sprintf('%s%d', protocol.optionArmPrefix, rankIndex);
cacheDirectory = getField(options, 'cacheDirectory', '');
if isempty(cacheDirectory) || exist(cacheDirectory, 'dir') ~= 7
    error('V154 arm shard requires an explicit cacheDirectory.');
end
outputRoot = getField(options, 'outputRoot', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v154', 'safe_graph_option_dwell', ...
    strrep(presetName, '-', '_'), sprintf('seed%d', seed), 'shards'));
armDirectory = fullfile(outputRoot, strrep(armMode, '-', '_'));
if exist(armDirectory, 'dir') ~= 7
    mkdir(armDirectory);
end
shardPath = fullfile(armDirectory, ...
    'safe_graph_option_dwell_v154_arm_shard.mat');
overwrite = logical(getField(options, 'overwrite', false));
if exist(shardPath, 'file') == 2 && ~overwrite
    error('V154 arm shard already exists; set overwrite=true intentionally.');
end

runnerOptions = struct( ...
    'armNames', {{armMode}}, ...
    'maxTimeSteps', protocol.anchorTimes(presetIdx) + ...
        protocol.horizon - 1, ...
    'continuationStartTime', protocol.anchorTimes(presetIdx), ...
    'behaviorCacheDirectory', cacheDirectory, ...
    'generateMissingBehaviorCache', false, ...
    'filterSeedOffset', protocol.filterSeedOffset, ...
    'evidenceSplit', protocol.evidenceSplit, ...
    'v154ExecutionAuthorized', true, ...
    'writeReport', false);
[~, raw] = runDynamicTopologyOracleGapScreen( ...
    presetName, seed, runnerOptions);
if numel(raw.records) ~= 1 || ...
        ~strcmp(raw.records.armMode, armMode)
    error('V154 arm runner returned the wrong option.');
end
shard = struct();
shard.protocolId = protocol.id;
shard.contractVersion = ...
    'safe-graph-option-dwell-v154-arm-shard-v1';
shard.generatedAt = datestr(now, 31);
shard.generationGitCommit = raw.generationGitCommit;
shard.generationTrackedWorktreeDirty = ...
    raw.generationTrackedWorktreeDirty;
shard.generationUntrackedSourceFiles = ...
    raw.generationUntrackedSourceFiles;
shard.presetName = presetName;
shard.seed = seed;
shard.rankIndex = rankIndex;
shard.armMode = armMode;
shard.record = raw.records;
shard.scenarioConfigSnapshot = raw.scenarioConfigSnapshot;
shard.claimBoundary = protocol.claimBoundary;
save('-mat7-binary', shardPath, 'shard');
fprintf('V154 arm shard: %s\n', shardPath);
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
