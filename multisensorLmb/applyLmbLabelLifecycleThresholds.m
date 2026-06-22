function [outputObjects, recursionObjects, discardedObjects, thresholds] = ...
    applyLmbLabelLifecycleThresholds(objects, model, currentTime)
% APPLYLMBLABELLIFECYCLETHRESHOLDS Separate estimate extraction from label pruning.
%
% LMB estimate extraction and Bernoulli hypothesis recursion should not have
% to share the same existence threshold.  A high output threshold can suppress
% noisy estimates without deleting labels that may be recovered by later
% measurements.

thresholds.output = resolveOutputThreshold(model);
thresholds.pruning = resolvePruningThreshold(model, thresholds.output);
thresholds.pruningMinTrajectoryLength = resolvePruningMinTrajectoryLength( ...
    model, thresholds.pruning < thresholds.output);
thresholds.protectionMode = resolvePruningProtectionMode(model);
thresholds.maxOutputGap = resolvePruningMaxOutputGap(model);

if nargin < 3 || isempty(currentTime)
    currentTime = NaN;
end

if isempty(objects)
    outputObjects = objects;
    recursionObjects = objects;
    discardedObjects = objects;
    return;
end

valid = [objects.numberOfGmComponents] > 0;
outputMask = valid & [objects.r] > thresholds.output;
if strcmpi(thresholds.protectionMode, 'last-output')
    protectedMask = computeLastOutputProtectionMask( ...
        objects, currentTime, thresholds.maxOutputGap);
else
    protectedMask = ...
        [objects.trajectoryLength] >= thresholds.pruningMinTrajectoryLength;
end
protectedLowConfidenceMask = valid & ...
    [objects.r] > thresholds.pruning & ...
    [objects.r] <= thresholds.output & ...
    protectedMask;
recursionMask = outputMask | protectedLowConfidenceMask;
discardedMask = ~recursionMask & ...
    [objects.trajectoryLength] > getField(model, 'minimumTrajectoryLength', 0);

outputObjects = objects(outputMask);
recursionObjects = objects(recursionMask);
discardedObjects = objects(discardedMask);
end

function threshold = resolveOutputThreshold(model)
threshold = getField(model, 'existenceThreshold', 0);
threshold = sanitizeThreshold(threshold, 0);
end

function threshold = resolvePruningThreshold(model, outputThreshold)
threshold = getField(model, 'labelPruningThreshold', outputThreshold);
if isfield(model, 'adaptiveFusion') && isstruct(model.adaptiveFusion)
    threshold = getField(model.adaptiveFusion, 'labelPruningThreshold', threshold);
end
threshold = sanitizeThreshold(threshold, outputThreshold);
threshold = min(threshold, outputThreshold);
end

function minLength = resolvePruningMinTrajectoryLength(model, hasDecoupledPruning)
if hasDecoupledPruning
    defaultValue = 1;
else
    defaultValue = 0;
end
minLength = getField(model, 'labelPruningMinTrajectoryLength', defaultValue);
if isfield(model, 'adaptiveFusion') && isstruct(model.adaptiveFusion)
    minLength = getField(model.adaptiveFusion, ...
        'labelPruningMinTrajectoryLength', minLength);
end
if isempty(minLength) || ~isnumeric(minLength) || ~isfinite(minLength)
    minLength = defaultValue;
end
minLength = max(0, round(minLength));
end

function mode = resolvePruningProtectionMode(model)
mode = getField(model, 'labelPruningProtectionMode', 'trajectory-age');
if isfield(model, 'adaptiveFusion') && isstruct(model.adaptiveFusion)
    mode = getField(model.adaptiveFusion, 'labelPruningProtectionMode', mode);
end
if ~(ischar(mode) || isstring(mode))
    mode = 'trajectory-age';
end
mode = lower(strtrim(char(mode)));
if ~any(strcmp(mode, {'trajectory-age', 'trajectory_age', 'age', ...
        'last-output', 'last_output', 'output-history', 'output_history'}))
    mode = 'trajectory-age';
end
if any(strcmp(mode, {'trajectory_age', 'age'}))
    mode = 'trajectory-age';
elseif any(strcmp(mode, {'last_output', 'output-history', 'output_history'}))
    mode = 'last-output';
end
end

function maxGap = resolvePruningMaxOutputGap(model)
maxGap = getField(model, 'labelPruningMaxOutputGap', 1);
if isfield(model, 'adaptiveFusion') && isstruct(model.adaptiveFusion)
    maxGap = getField(model.adaptiveFusion, 'labelPruningMaxOutputGap', maxGap);
end
if isempty(maxGap) || ~isnumeric(maxGap) || ~isfinite(maxGap)
    maxGap = 1;
end
maxGap = max(0, round(maxGap));
end

function mask = computeLastOutputProtectionMask(objects, currentTime, maxGap)
mask = false(1, numel(objects));
if ~isfinite(currentTime)
    return;
end
for idx = 1:numel(objects)
    if isfield(objects, 'lastOutputTime')
        lastOutputTime = objects(idx).lastOutputTime;
    else
        lastOutputTime = -Inf;
    end
    mask(idx) = isfinite(lastOutputTime) && ...
        (currentTime - lastOutputTime) <= maxGap;
end
end

function threshold = sanitizeThreshold(value, fallback)
threshold = value;
if isempty(threshold) || ~isnumeric(threshold) || ~isfinite(threshold)
    threshold = fallback;
end
threshold = min(max(threshold, 0), 1);
end

function value = getField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end
