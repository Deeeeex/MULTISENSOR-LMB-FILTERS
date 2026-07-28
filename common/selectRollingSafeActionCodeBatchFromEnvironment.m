function [selectedCodes, batchCount, batchIndex] = ...
    selectRollingSafeActionCodeBatchFromEnvironment(actionCodes)
% SELECTROLLINGSAFEACTIONCODEBATCHFROMENVIRONMENT Deterministic screen shard.

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
    error('BATCH_INDEX must lie in 1:BATCH_COUNT.');
end
selectedCodes = actionCodes( ...
    batchIndex:batchCount:numel(actionCodes));
end
