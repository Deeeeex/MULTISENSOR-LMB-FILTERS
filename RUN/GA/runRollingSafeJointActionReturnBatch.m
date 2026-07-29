% RUNROLLINGSAFEJOINTACTIONRETURNBATCH Environment entry point.
%
% Example:
%   SEED=7 TIME=75 BATCH_INDEX=1 BATCH_COUNT=4 \
%     octave --quiet RUN/GA/runRollingSafeJointActionReturnBatch.m
%   PROPOSAL_ROLE=privileged selects the frozen offline target bank.

addpath(genpath(pwd));

environmentNames = {'SEED', 'TIME', 'BATCH_INDEX', 'BATCH_COUNT'};
environmentValues = nan(1, numel(environmentNames));
for environmentIdx = 1:numel(environmentNames)
    environmentName = environmentNames{environmentIdx};
    token = strtrim(getenv(environmentName));
    value = str2double(token);
    if isempty(token) || ...
            ~isscalar(value) || ~isfinite(value) || ...
            value < 1 || mod(value, 1) ~= 0
        error('Set %s to one positive integer.', environmentName);
    end
    environmentValues(environmentIdx) = round(value);
end
seed = environmentValues(1);
currentTime = environmentValues(2);
batchIndex = environmentValues(3);
batchCount = environmentValues(4);
options = struct();
proposalRole = lower(strtrim(getenv('PROPOSAL_ROLE')));
if ~isempty(proposalRole)
    options.proposalRole = proposalRole;
end
maximumCandidatesToken = strtrim( ...
    getenv('MAXIMUM_CANDIDATES'));
if ~isempty(maximumCandidatesToken)
    options.maximumCandidatesToRun = ...
        str2double(maximumCandidatesToken);
end
overwriteToken = lower(strtrim(getenv('OVERWRITE')));
options.overwrite = any(strcmp( ...
    overwriteToken, {'1', 'true', 'yes'}));
outputDirectory = strtrim(getenv('OUTPUT_DIRECTORY'));
if ~isempty(outputDirectory)
    options.outputDirectory = outputDirectory;
end

runRollingSafeJointActionReturnShard( ...
    seed, currentTime, batchIndex, batchCount, options);
