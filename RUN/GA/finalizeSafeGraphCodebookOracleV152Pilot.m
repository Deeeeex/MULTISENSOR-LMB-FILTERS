function [reportPath, summary] = ...
    finalizeSafeGraphCodebookOracleV152Pilot( ...
        presetName, seed, options)
% FINALIZESAFEGRAPHCODEBOOKORACLEV152PILOT Join exact arm shards.

if nargin < 3 || isempty(options)
    options = struct();
end
protocol = getSafeGraphCodebookOracleV152Protocol();
rankArms = arrayfun(@(rankIndex) sprintf('%s%d', ...
    protocol.proposalArmPrefix, rankIndex), ...
    protocol.proposalRanks, 'UniformOutput', false);
allArms = [protocol.staticArmModes, rankArms];
outputRoot = getField(options, 'outputRoot', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v152', 'safe_graph_codebook_oracle', ...
    strrep(presetName, '-', '_'), sprintf('seed%d', seed), 'shards'));
records = repmat(struct(), 1, numel(allArms));
shardPaths = cell(1, numel(allArms));
generationCommit = '';
scenarioConfig = struct();
for armIdx = 1:numel(allArms)
    armMode = allArms{armIdx};
    shardPath = fullfile(outputRoot, strrep(armMode, '-', '_'), ...
        'safe_graph_codebook_oracle_v152_arm_shard.mat');
    loaded = load(shardPath, 'shard');
    if ~isfield(loaded, 'shard')
        error('V152 arm shard is missing: %s', shardPath);
    end
    shard = loaded.shard;
    if ~strcmp(shard.protocolId, protocol.id) || ...
            ~strcmp(shard.contractVersion, ...
                'safe-graph-codebook-oracle-v152-arm-shard-v1') || ...
            ~strcmp(shard.presetName, presetName) || ...
            shard.seed ~= seed || ...
            ~strcmp(shard.armMode, armMode) || ...
            ~strcmp(shard.record.armMode, armMode)
        error('V152 arm shard violates the frozen identity contract.');
    end
    if armIdx == 1
        generationCommit = shard.generationGitCommit;
        scenarioConfig = shard.scenarioConfigSnapshot;
        records = repmat(shard.record, 1, numel(allArms));
    elseif ~strcmp(shard.generationGitCommit, generationCommit) || ...
            ~isequaln(shard.scenarioConfigSnapshot, scenarioConfig)
        error('V152 arm shards do not share one code/scenario boundary.');
    end
    records(armIdx) = shard.record;
    shardPaths{armIdx} = shardPath;
end
raw = struct( ...
    'presetName', presetName, ...
    'seeds', seed, ...
    'records', records, ...
    'scenarioConfigSnapshot', scenarioConfig);
summary = summarizeSafeGraphCodebookOracleV152RawSummary(raw);
summary.generationGitCommit = generationCommit;
summary.shardPaths = shardPaths;
aggregateDirectory = fileparts(outputRoot);
reportPath = fullfile(aggregateDirectory, ...
    'SAFE_GRAPH_CODEBOOK_ORACLE_V152_PILOT.md');
matPath = fullfile(aggregateDirectory, ...
    'safe_graph_codebook_oracle_v152_pilot.mat');
summary.reportPath = reportPath;
summary.matPath = matPath;
save('-mat7-binary', matPath, 'summary');
writeSafeGraphCodebookOracleV152PilotReport(reportPath, summary);
fprintf('V152 finalized pilot: %s\n', reportPath);
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
