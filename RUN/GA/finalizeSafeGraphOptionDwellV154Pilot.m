function [reportPath, summary] = ...
    finalizeSafeGraphOptionDwellV154Pilot( ...
        presetName, seed, options)
% FINALIZESAFEGRAPHOPTIONDWELLV154PILOT Join saved static and option shards.

if nargin < 3 || isempty(options)
    options = struct();
end
protocol = getSafeGraphOptionDwellV154Protocol();
generator = getSafeGraphCodebookOracleV152Protocol();
staticRoot = getField(options, 'staticRoot', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v152', 'safe_graph_codebook_oracle', ...
    strrep(presetName, '-', '_'), sprintf('seed%d', seed), 'shards'));
optionRoot = getField(options, 'optionRoot', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v154', 'safe_graph_option_dwell', ...
    strrep(presetName, '-', '_'), sprintf('seed%d', seed), 'shards'));
optionArms = arrayfun(@(rankIndex) sprintf('%s%d', ...
    protocol.optionArmPrefix, rankIndex), ...
    protocol.optionRanks, 'UniformOutput', false);
allArms = [protocol.staticArmModes, optionArms];
records = repmat(struct(), 1, numel(allArms));
staticShardPaths = cell(1, numel(protocol.staticArmModes));
optionShardPaths = cell(1, numel(optionArms));
scenarioConfig = struct();
staticCommit = '';
optionCommit = '';

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
        error('V154 static source shard violates its V152 identity.');
    end
    if armIdx == 1
        staticCommit = loaded.shard.generationGitCommit;
        scenarioConfig = loaded.shard.scenarioConfigSnapshot;
        records = repmat(loaded.shard.record, 1, numel(allArms));
    elseif ~strcmp(loaded.shard.generationGitCommit, staticCommit) || ...
            ~isequaln(loaded.shard.scenarioConfigSnapshot, scenarioConfig)
        error('V154 static source shards are not paired.');
    end
    records(armIdx) = loaded.shard.record;
    staticShardPaths{armIdx} = path;
end

for optionIdx = 1:numel(optionArms)
    armMode = optionArms{optionIdx};
    path = fullfile(optionRoot, strrep(armMode, '-', '_'), ...
        'safe_graph_option_dwell_v154_arm_shard.mat');
    loaded = load(path, 'shard');
    if ~isfield(loaded, 'shard') || ...
            ~strcmp(loaded.shard.protocolId, protocol.id) || ...
            ~strcmp(loaded.shard.contractVersion, ...
                'safe-graph-option-dwell-v154-arm-shard-v1') || ...
            ~strcmp(loaded.shard.presetName, presetName) || ...
            loaded.shard.seed ~= seed || ...
            loaded.shard.rankIndex ~= protocol.optionRanks(optionIdx) || ...
            ~strcmp(loaded.shard.armMode, armMode) || ...
            ~strcmp(loaded.shard.record.armMode, armMode)
        error('V154 option shard violates the frozen identity.');
    end
    if optionIdx == 1
        optionCommit = loaded.shard.generationGitCommit;
    elseif ~strcmp(loaded.shard.generationGitCommit, optionCommit)
        error('V154 option shards do not share one code boundary.');
    end
    if ~isequaln(loaded.shard.scenarioConfigSnapshot, scenarioConfig)
        error('V154 static and option shards do not share one scene.');
    end
    records(numel(protocol.staticArmModes) + optionIdx) = ...
        loaded.shard.record;
    optionShardPaths{optionIdx} = path;
end

raw = struct( ...
    'presetName', presetName, ...
    'seeds', seed, ...
    'records', records, ...
    'scenarioConfigSnapshot', scenarioConfig);
summary = summarizeSafeGraphOptionDwellV154RawSummary(raw);
summary.staticGenerationGitCommit = staticCommit;
summary.optionGenerationGitCommit = optionCommit;
summary.staticShardPaths = staticShardPaths;
summary.optionShardPaths = optionShardPaths;
aggregateDirectory = fileparts(optionRoot);
reportPath = fullfile(aggregateDirectory, ...
    'SAFE_GRAPH_OPTION_DWELL_V154_PILOT.md');
matPath = fullfile(aggregateDirectory, ...
    'safe_graph_option_dwell_v154_pilot.mat');
summary.reportPath = reportPath;
summary.matPath = matPath;
save('-mat7-binary', matPath, 'summary');
writeSafeGraphOptionDwellV154PilotReport(reportPath, summary);
fprintf('V154 finalized pilot: %s\n', reportPath);
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
