% GENERATEBACKBONERESIDUALRETURNPROPOSALBANK
% Freeze the M24 t=75 candidate set before paired H=3 return replay.

addpath(genpath(pwd));
protocol = getBackboneResidualReturnHeadroomProtocol();
[~, context, sourceState] = ...
    loadBackboneResidualReturnHeadroomState();
[candidateAdjacency, candidateRecords, metadata] = ...
    buildBackboneResidualReturnProposalBank(context);
gitState = resolveResearchGitState();

dataset = struct();
dataset.contractVersion = ...
    'backbone-residual-return-proposal-dataset-v1';
dataset.protocolId = protocol.id;
dataset.generatedAt = datestr(now, 31);
dataset.presetName = protocol.presetName;
dataset.seed = protocol.seed;
dataset.currentTime = protocol.currentTime;
dataset.candidateAdjacency = candidateAdjacency;
dataset.candidateRecords = candidateRecords;
dataset.metadata = metadata;
dataset.sourceStateContractVersion = ...
    sourceState.contractVersion;
dataset.sourceCachePath = sourceState.cachePath;
dataset.sourceCacheSha256 = sourceState.cacheSha256;
dataset.previousAdjacencyHistoryTimes = ...
    sourceState.previousAdjacencyHistoryTimes;
dataset.generationGitCommit = gitState.commit;
dataset.generationTrackedWorktreeDirty = ...
    gitState.trackedWorktreeDirty;
dataset.generationUntrackedSourceFiles = ...
    gitState.untrackedSourceFiles;
dataset.futureOutcomeUsed = false;
dataset.candidateRankingPerformed = false;
dataset.deployable = false;

outputPath = strtrim(getenv('OUTPUT_PATH'));
if isempty(outputPath)
    outputPath = fullfile( ...
        'RUN', 'GA', 'dynamic_topology', 'proposals', ...
        'backbone_residual_return_m24_t75_seed7_v1.mat');
end
outputDirectory = fileparts(outputPath);
if ~exist(outputDirectory, 'dir')
    mkdir(outputDirectory);
end
save('-mat7-binary', outputPath, 'dataset');
fprintf('Proposal dataset: %s\n', outputPath);
fprintf('SHA-256: %s\n', computeFileSha256(outputPath));
fprintf('Distinct candidates: %d\n', ...
    size(candidateAdjacency, 3));
