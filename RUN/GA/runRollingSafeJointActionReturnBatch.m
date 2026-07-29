% RUNROLLINGSAFEJOINTACTIONRETURNBATCH Environment entry point.
%
% Example:
%   SEED=7 TIME=75 BATCH_INDEX=1 BATCH_COUNT=4 \
%     octave --quiet RUN/GA/runRollingSafeJointActionReturnBatch.m

addpath(genpath(pwd));

seed = parseRequiredInteger('SEED');
currentTime = parseRequiredInteger('TIME');
batchIndex = parseRequiredInteger('BATCH_INDEX');
batchCount = parseRequiredInteger('BATCH_COUNT');
options = struct();
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

function value = parseRequiredInteger(name)
token = strtrim(getenv(name));
value = str2double(token);
if isempty(token) || ...
        ~isscalar(value) || ~isfinite(value) || ...
        value < 1 || mod(value, 1) ~= 0
    error('Set %s to one positive integer.', name);
end
value = round(value);
end
