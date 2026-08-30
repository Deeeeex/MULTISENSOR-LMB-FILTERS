function [reportPath, summary] = ...
    finalizeOutputAlignedSafeGraphNeighborhoodV155Pilot( ...
        presetName, seed, options)
% FINALIZEOUTPUTALIGNEDSAFEGRAPHNEIGHBORHOODV155PILOT Join Stage-A shards.

if nargin < 3 || isempty(options)
    options = struct();
end
protocol = getOutputAlignedSafeGraphNeighborhoodV155Protocol();
generator = getSafeGraphCodebookOracleV152Protocol();
presetIdx = find(strcmp(protocol.presetNames, presetName));
if numel(presetIdx) ~= 1 || ...
        ~strcmp(presetName, protocol.stageAPresetName) || ...
        seed ~= protocol.stageASeed
    error('V155 finalizer is outside the frozen Stage-A case.');
end
staticRoot = getField(options, 'staticRoot', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v152', 'safe_graph_codebook_oracle', ...
    strrep(presetName, '-', '_'), sprintf('seed%d', seed), 'shards'));
candidateRoot = getField(options, 'candidateRoot', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v155', 'safe_graph_neighborhood', ...
    strrep(presetName, '-', '_'), sprintf('seed%d', seed), 'shards'));
candidateOrdinals = 1:protocol.candidateCounts(presetIdx);
candidateArms = arrayfun(@(ordinal) sprintf('%s%d', ...
    protocol.candidateArmPrefix, ordinal), ...
    candidateOrdinals, 'UniformOutput', false);
allArms = [protocol.staticArmModes, candidateArms];
records = repmat(struct(), 1, numel(allArms));
staticShardPaths = cell(1, numel(protocol.staticArmModes));
candidateShardPaths = cell(1, numel(candidateArms));
scenarioConfig = struct();
staticCommit = '';
candidateCommit = '';

for armIdx = 1:numel(protocol.staticArmModes)
    armMode = protocol.staticArmModes{armIdx};
    path = fullfile(staticRoot, strrep(armMode, '-', '_'), ...
        'safe_graph_codebook_oracle_v152_arm_shard.mat');
    loaded = load(path, 'shard');
    if ~isfield(loaded, 'shard') || ...
            ~strcmp(loaded.shard.protocolId, generator.id) || ...
            ~strcmp(loaded.shard.presetName, presetName) || ...
            loaded.shard.seed ~= seed || ...
            ~strcmp(loaded.shard.armMode, armMode)
        error('V155 static source shard violates its V152 identity.');
    end
    if armIdx == 1
        staticCommit = loaded.shard.generationGitCommit;
        scenarioConfig = loaded.shard.scenarioConfigSnapshot;
        records = repmat(loaded.shard.record, 1, numel(allArms));
    elseif ~strcmp(loaded.shard.generationGitCommit, staticCommit) || ...
            ~isequaln(loaded.shard.scenarioConfigSnapshot, scenarioConfig)
        error('V155 static source shards are not paired.');
    end
    records(armIdx) = loaded.shard.record;
    staticShardPaths{armIdx} = path;
end

for candidateIdx = 1:numel(candidateArms)
    armMode = candidateArms{candidateIdx};
    path = fullfile(candidateRoot, strrep(armMode, '-', '_'), ...
        'safe_graph_neighborhood_v155_arm_shard.mat');
    loaded = load(path, 'shard');
    if ~isfield(loaded, 'shard') || ...
            ~strcmp(loaded.shard.protocolId, protocol.id) || ...
            ~strcmp(loaded.shard.contractVersion, ...
                'safe-graph-neighborhood-v155-arm-shard-v1') || ...
            ~strcmp(loaded.shard.presetName, presetName) || ...
            loaded.shard.seed ~= seed || ...
            loaded.shard.candidateOrdinal ~= candidateIdx || ...
            ~strcmp(loaded.shard.armMode, armMode) || ...
            ~strcmp(loaded.shard.record.armMode, armMode)
        error('V155 candidate shard violates the frozen identity.');
    end
    if candidateIdx == 1
        candidateCommit = loaded.shard.generationGitCommit;
    elseif ~strcmp(loaded.shard.generationGitCommit, candidateCommit)
        error('V155 candidate shards do not share one code boundary.');
    end
    if ~isequaln(loaded.shard.scenarioConfigSnapshot, scenarioConfig)
        error('V155 static and candidate shards do not share one scene.');
    end
    records(numel(protocol.staticArmModes) + candidateIdx) = ...
        loaded.shard.record;
    candidateShardPaths{candidateIdx} = path;
end

raw = struct( ...
    'presetName', presetName, ...
    'seeds', seed, ...
    'records', records, ...
    'scenarioConfigSnapshot', scenarioConfig);
summary = summarizeOutputAlignedSafeGraphNeighborhoodV155RawSummary(raw);
summary.staticGenerationGitCommit = staticCommit;
summary.candidateGenerationGitCommit = candidateCommit;
summary.staticShardPaths = staticShardPaths;
summary.candidateShardPaths = candidateShardPaths;
aggregateDirectory = fileparts(candidateRoot);
reportPath = fullfile(aggregateDirectory, ...
    'OUTPUT_ALIGNED_SAFE_GRAPH_NEIGHBORHOOD_V155_PILOT.md');
matPath = fullfile(aggregateDirectory, ...
    'output_aligned_safe_graph_neighborhood_v155_pilot.mat');
summary.reportPath = reportPath;
summary.matPath = matPath;
save('-mat7-binary', matPath, 'summary');
writeOutputAlignedSafeGraphNeighborhoodV155Report( ...
    reportPath, summary);
fprintf('V155 finalized pilot: %s\n', reportPath);
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
