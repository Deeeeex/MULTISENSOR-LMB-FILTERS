function [reportPath, summary] = ...
    finalizeSafeGraphParetoOracleV153Pilot( ...
        presetName, seed, options)
% FINALIZESAFEGRAPHPARETOORACLEV153PILOT Re-score unchanged V152 shards.

if nargin < 3 || isempty(options)
    options = struct();
end
protocol = getSafeGraphParetoOracleV153Protocol();
generator = getSafeGraphCodebookOracleV152Protocol();
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
        error('V153 source arm shard is missing: %s', shardPath);
    end
    shard = loaded.shard;
    if ~strcmp(shard.protocolId, generator.id) || ...
            ~strcmp(shard.presetName, presetName) || ...
            shard.seed ~= seed || ...
            ~strcmp(shard.armMode, armMode) || ...
            ~strcmp(shard.record.armMode, armMode)
        error('V153 source shard violates the frozen V152 identity.');
    end
    if armIdx == 1
        generationCommit = shard.generationGitCommit;
        scenarioConfig = shard.scenarioConfigSnapshot;
        records = repmat(shard.record, 1, numel(allArms));
    elseif ~strcmp(shard.generationGitCommit, generationCommit) || ...
            ~isequaln(shard.scenarioConfigSnapshot, scenarioConfig)
        error('V153 source shards do not share one code/scenario boundary.');
    end
    records(armIdx) = shard.record;
    shardPaths{armIdx} = shardPath;
end
raw = struct( ...
    'presetName', presetName, ...
    'seeds', seed, ...
    'records', records, ...
    'scenarioConfigSnapshot', scenarioConfig);
summary = summarizeSafeGraphParetoOracleV153RawSummary(raw);
summary.generationGitCommit = generationCommit;
summary.shardPaths = shardPaths;
aggregateDirectory = fileparts(outputRoot);
reportPath = fullfile(aggregateDirectory, ...
    'SAFE_GRAPH_PARETO_ORACLE_V153_PILOT.md');
matPath = fullfile(aggregateDirectory, ...
    'safe_graph_pareto_oracle_v153_pilot.mat');
summary.reportPath = reportPath;
summary.matPath = matPath;
save('-mat7-binary', matPath, 'summary');
writeSafeGraphParetoOracleV153PilotReport(reportPath, summary);
fprintf('V153 finalized pilot: %s\n', reportPath);
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
