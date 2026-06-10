function payload = buildMixedLmbPayload( ...
    objects, model, triggerConfig, utilityDetails, diagnostics)
% BUILDMIXEDLMBPAYLOAD Pack high-utility labels as GM-LMB and others as light.
%
% The returned value is still an ordinary LMB object array. Labels selected
% for heavy payload keep their Gaussian mixtures, while light labels are
% moment matched to one Gaussian. No extra object fields are introduced.

if nargin < 4 || isempty(utilityDetails)
    utilityDetails = struct();
end
if nargin < 5 || isempty(diagnostics)
    diagnostics = struct();
end
existenceThreshold = getField( ...
    triggerConfig, 'payloadExistenceThreshold', ...
    getField(model, 'existenceThreshold', 0));
activeObjects = selectActiveObjects(objects, existenceThreshold);
if isempty(activeObjects)
    payload = activeObjects;
    return;
end

thresholdHigh = getField(triggerConfig, ...
    'mixedPayloadHeavyThreshold', ...
    getField(triggerConfig, 'thresholdHigh', 0.60));
thresholdLow = getField(triggerConfig, ...
    'mixedPayloadLightThreshold', ...
    getField(triggerConfig, 'thresholdLow', 0.25));
includeAllActiveAsLight = getField( ...
    triggerConfig, 'mixedPayloadLightForAllActiveLabels', true);

payload = activeObjects([]);
for objectIdx = 1:numel(activeObjects)
    object = activeObjects(objectIdx);
    labelUtility = resolveLabelUtility(object, utilityDetails);
    if labelUtility >= thresholdHigh
        payload(end+1) = object; %#ok<AGROW>
    elseif includeAllActiveAsLight || labelUtility >= thresholdLow
        lightObject = compressLmbPosterior( ...
            object, model, existenceThreshold, triggerConfig, diagnostics);
        if ~isempty(lightObject)
            payload(end+1) = lightObject; %#ok<AGROW>
        end
    end
end
end

function selected = selectActiveObjects(objects, threshold)
if isempty(objects)
    selected = objects;
    return;
end
active = [objects.r] > threshold & ...
    [objects.numberOfGmComponents] > 0;
selected = objects(active);
end

function value = resolveLabelUtility(object, utilityDetails)
value = 0;
if ~isstruct(utilityDetails) || ...
        ~isfield(utilityDetails, 'labels') || ...
        ~isfield(utilityDetails, 'perLabelUtility')
    return;
end
label = [object.birthTime; object.birthLocation];
labels = utilityDetails.labels;
if isempty(labels)
    return;
end
labelIdx = find(all(labels == label, 1), 1);
if isempty(labelIdx) || numel(utilityDetails.perLabelUtility) < labelIdx
    return;
end
value = utilityDetails.perLabelUtility(labelIdx);
if ~isfinite(value)
    value = 0;
end
end

function value = getField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end
