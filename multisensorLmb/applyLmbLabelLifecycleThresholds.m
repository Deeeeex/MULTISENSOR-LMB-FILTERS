function [outputObjects, recursionObjects, discardedObjects, thresholds] = ...
    applyLmbLabelLifecycleThresholds(objects, model)
% APPLYLMBLABELLIFECYCLETHRESHOLDS Separate estimate extraction from label pruning.
%
% LMB estimate extraction and Bernoulli hypothesis recursion should not have
% to share the same existence threshold.  A high output threshold can suppress
% noisy estimates without deleting labels that may be recovered by later
% measurements.

thresholds.output = resolveOutputThreshold(model);
thresholds.pruning = resolvePruningThreshold(model, thresholds.output);

if isempty(objects)
    outputObjects = objects;
    recursionObjects = objects;
    discardedObjects = objects;
    return;
end

valid = [objects.numberOfGmComponents] > 0;
outputMask = valid & [objects.r] > thresholds.output;
recursionMask = valid & [objects.r] > thresholds.pruning;
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
