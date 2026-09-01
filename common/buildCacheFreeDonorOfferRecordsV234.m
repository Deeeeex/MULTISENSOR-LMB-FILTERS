function records = buildCacheFreeDonorOfferRecordsV234( ...
        sourcePosterior, model, sourceSensorId, options)
% BUILDCACHEFREEDONOROFFERRECORDSV234 Serialize compact source summaries.
%
% Every record follows the frozen 24-byte V229 layout.  The source sensor
% identity is carried by the response envelope and is therefore metadata,
% not part of the per-label byte charge or learned feature vector.

if nargin < 4 || isempty(options)
    options = struct();
end
protocol = getCacheFreeDonorOfferCoverageV233Protocol();
minimumExistence = getField(options, ...
    'minimumActiveExistence', ...
    protocol.coverage.minimumActiveExistence);
if ~isstruct(sourcePosterior) || ~isstruct(model) || ...
        ~isscalar(model) || ~isfield(model, 'xDimension') || ...
        ~isscalar(model.xDimension) || model.xDimension < 2 || ...
        ~isscalar(sourceSensorId) || ~isfinite(sourceSensorId) || ...
        sourceSensorId ~= round(sourceSensorId) || sourceSensorId < 1 || ...
        ~isscalar(minimumExistence) || ~isfinite(minimumExistence) || ...
        minimumExistence < 0 || minimumExistence >= 1
    error('CacheFreeDonorOfferV234:InvalidRecordInput', ...
        'The compact donor-offer request is malformed.');
end

records = repmat(emptyRecord(), 1, 0);
for objectIndex = 1:numel(sourcePosterior)
    object = sourcePosterior(objectIndex);
    if object.numberOfGmComponents < 1 || object.r < minimumExistence
        continue;
    end
    label = [object.birthTime; object.birthLocation];
    if any(~isfinite(label)) || any(label ~= round(label)) || ...
            any(label < 0) || any(label > double(intmax('uint16')))
        error('CacheFreeDonorOfferV234:LabelKeyOverflow', ...
            'A semantic label key does not fit the four-byte layout.');
    end
    [meanVector, covariance] = objectMoments( ...
        object, model.xDimension);
    evidence = 0.5 * (boundedScalar( ...
        object, 'associationConfidence') + boundedScalar( ...
        object, 'detectionAssociationMass'));
    payload = estimateLmbPayloadSize(object, model, 2, struct());
    record = emptyRecord();
    record.sourceSensorId = sourceSensorId;
    record.label = double(uint16(label));
    record.existence = quantizeUnit(object.r);
    record.evidenceQuality = quantizeUnit(evidence);
    record.positionMean = double(single(meanVector(1:2)));
    record.positionTrace = double(single(max(real(trace( ...
        covariance(1:2, 1:2))), 0)));
    payloadBytes = min(max(round(payload.estimatedBytes), 0), ...
        double(intmax('uint32')));
    record.payloadBytesPerReceiver = double(uint32(payloadBytes));
    records(end + 1) = record; %#ok<AGROW>
end
if isempty(records)
    return;
end
labels = reshape([records.label], 2, [])';
if size(unique(labels, 'rows'), 1) ~= size(labels, 1)
    error('CacheFreeDonorOfferV234:DuplicateLabel', ...
        'Every source response must contain unique label keys.');
end
[~, order] = sortrows(labels, [1, 2]);
records = records(order);
end

function [meanVector, covariance] = objectMoments(object, dimension)
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if numel(weights) ~= object.numberOfGmComponents || sum(weights) <= 0
    weights = ones(1, object.numberOfGmComponents);
end
weights = weights / sum(weights);
meanVector = zeros(dimension, 1);
for componentIndex = 1:object.numberOfGmComponents
    meanVector = meanVector + ...
        weights(componentIndex) * object.mu{componentIndex};
end
covariance = zeros(dimension);
for componentIndex = 1:object.numberOfGmComponents
    delta = object.mu{componentIndex} - meanVector;
    covariance = covariance + weights(componentIndex) * ...
        (object.Sigma{componentIndex} + delta * delta');
end
covariance = (covariance + covariance') / 2;
end

function value = boundedScalar(data, name)
value = 0;
if isfield(data, name) && isscalar(data.(name)) && ...
        isfinite(data.(name))
    value = data.(name);
end
value = min(max(value, 0), 1);
end

function value = quantizeUnit(value)
value = min(max(value, 0), 1);
value = double(uint16(round( ...
    value * double(intmax('uint16'))))) / ...
    double(intmax('uint16'));
end

function record = emptyRecord()
record = struct( ...
    'sourceSensorId', 0, ...
    'label', zeros(2, 1), ...
    'existence', 0, ...
    'evidenceQuality', 0, ...
    'positionMean', zeros(2, 1), ...
    'positionTrace', 0, ...
    'payloadBytesPerReceiver', 0);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
