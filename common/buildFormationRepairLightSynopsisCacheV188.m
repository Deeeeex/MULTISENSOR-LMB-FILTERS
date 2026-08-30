function cache = buildFormationRepairLightSynopsisCacheV188( ...
        fusedPosteriorBySensor, localPosteriorBySensor, ...
        model, currentTime, options)
% BUILDFORMATIONREPAIRLIGHTSYNOPSISCACHEV188 Quantized online metadata.
%
% Each sensor serializes its local and fused active labels under one header.
% A 24-byte label record contains a four-byte semantic label key, quantized
% existence/evidence/support/opportunity scalars, two float32 position
% coordinates, and a float32 position-covariance trace.  Full covariance
% and GM components remain local until a shortlisted 64-byte rich synopsis
% or complete-label response is explicitly requested.

if nargin < 5 || isempty(options)
    options = struct();
end
protocol = getBudgetRecycledFormationRepairV188Protocol();
sensorCount = numel(localPosteriorBySensor);
activeExistenceThreshold = getField( ...
    options, 'activeExistenceThreshold', 1e-2);
if ~iscell(fusedPosteriorBySensor) || ...
        numel(fusedPosteriorBySensor) ~= sensorCount || ...
        sensorCount < 1 || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime < 1 || currentTime ~= round(currentTime) || ...
        ~isscalar(activeExistenceThreshold) || ...
        ~isfinite(activeExistenceThreshold) || ...
        activeExistenceThreshold < 0 || ...
        activeExistenceThreshold >= 1
    error('FormationRepairSynopsisV188:InvalidInput', ...
        'The posterior cache inputs are malformed.');
end
positionCutoff = resolvePositionCutoff(model);
localBySensor = cell(1, sensorCount);
fusedBySensor = cell(1, sensorCount);
attemptedBytesBySensor = zeros(1, sensorCount);
for sensorIdx = 1:sensorCount
    localBySensor{sensorIdx} = summarizePosterior( ...
        localPosteriorBySensor{sensorIdx}, model, sensorIdx, ...
        currentTime, positionCutoff, activeExistenceThreshold);
    fusedBySensor{sensorIdx} = summarizePosterior( ...
        fusedPosteriorBySensor{sensorIdx}, model, sensorIdx, ...
        currentTime, positionCutoff, activeExistenceThreshold);
    attemptedBytesBySensor(sensorIdx) = ...
        protocol.lightSynopsisHeaderBytes + ...
        protocol.lightSynopsisBytesPerLabel * ( ...
        localBySensor{sensorIdx}.labelCount + ...
        fusedBySensor{sensorIdx}.labelCount);
end

cache = struct();
cache.contractVersion = ...
    'formation-repair-light-synopsis-cache-v188-v1';
cache.currentTime = currentTime;
cache.sensorCount = sensorCount;
cache.activeExistenceThreshold = activeExistenceThreshold;
cache.positionCutoff = positionCutoff;
cache.localBySensor = localBySensor;
cache.fusedBySensor = fusedBySensor;
cache.headerBytesPerSensor = protocol.lightSynopsisHeaderBytes;
cache.bytesPerLabelRecord = protocol.lightSynopsisBytesPerLabel;
cache.attemptedBytesBySensor = attemptedBytesBySensor;
cache.totalAttemptedBytes = sum(attemptedBytesBySensor);
cache.labelRecordLayout = [ ...
    'uint16 birth-time, uint16 birth-location, uint16 existence, ', ...
    'uint16 evidence, uint16 support, float32 x, float32 y, ', ...
    'float32 position-trace, uint16 observation-opportunity'];
cache.fullCovarianceIncluded = false;
cache.gmComponentsIncluded = false;
cache.truthUsed = false;
cache.futureInformationUsed = false;
cache.numericLabelIdentifiersUsedAsFeatures = false;
end

function summary = summarizePosterior( ...
        objects, model, sensorIdx, currentTime, ...
        positionCutoff, activeExistenceThreshold)
objects = reshape(objects, 1, []);
labels = zeros(2, 0);
existence = zeros(1, 0);
positionMean = zeros(2, 0);
positionTrace = zeros(1, 0);
evidenceQuality = zeros(1, 0);
associationSupport = zeros(1, 0);
observationOpportunity = zeros(1, 0);
bayesRisk = zeros(1, 0);
for objectIdx = 1:numel(objects)
    object = objects(objectIdx);
    if object.numberOfGmComponents < 1 || ...
            object.r < activeExistenceThreshold
        continue;
    end
    label = [object.birthTime; object.birthLocation];
    if any(~isfinite(label)) || any(label ~= round(label)) || ...
            any(label < 0) || any(label > double(intmax('uint16')))
        error('FormationRepairSynopsisV188:LabelKeyOverflow', ...
            'A semantic label key does not fit the frozen four-byte layout.');
    end
    [meanVector, covariance] = objectMoments(object);
    traceValue = max(real(trace(covariance(1:2, 1:2))), 0);
    association = boundedScalar(object, 'associationConfidence');
    detection = boundedScalar(object, 'detectionAssociationMass');
    evidence = 0.5 * (association + detection);
    support = max(association, detection);
    opportunity = advertisedOpportunity(object);
    if ~isfinite(opportunity)
        [opportunity, ~] = evaluateSensorQuality( ...
            model, sensorIdx, meanVector, currentTime);
    end

    labels(:, end + 1) = double(uint16(label)); %#ok<AGROW>
    existence(end + 1) = quantizeUnit(object.r); %#ok<AGROW>
    positionMean(:, end + 1) = ...
        double(single(meanVector(1:2))); %#ok<AGROW>
    positionTrace(end + 1) = ...
        double(single(traceValue)); %#ok<AGROW>
    evidenceQuality(end + 1) = quantizeUnit(evidence); %#ok<AGROW>
    associationSupport(end + 1) = quantizeUnit(support); %#ok<AGROW>
    observationOpportunity(end + 1) = ...
        quantizeUnit(opportunity); %#ok<AGROW>
    existenceRisk = min(existence(end), 1 - existence(end));
    localizationRisk = existence(end) * min( ...
        positionTrace(end) / max(positionCutoff^2, eps), 1);
    bayesRisk(end + 1) = ...
        0.5 * existenceRisk + 0.5 * localizationRisk; %#ok<AGROW>
end
if size(unique(labels', 'rows'), 1) ~= size(labels, 2)
    error('FormationRepairSynopsisV188:DuplicateLabel', ...
        'Every sensor synopsis must contain unique semantic labels.');
end
summary = struct( ...
    'labels', labels, ...
    'existence', existence, ...
    'positionMean', positionMean, ...
    'positionTrace', positionTrace, ...
    'evidenceQuality', evidenceQuality, ...
    'associationSupport', associationSupport, ...
    'observationOpportunity', observationOpportunity, ...
    'bayesRisk', bayesRisk, ...
    'labelCount', size(labels, 2));
end

function [meanVector, covariance] = objectMoments(object)
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if numel(weights) ~= object.numberOfGmComponents || sum(weights) <= 0
    weights = ones(1, object.numberOfGmComponents);
end
weights = weights / sum(weights);
stateDimension = numel(object.mu{1});
meanVector = zeros(stateDimension, 1);
for componentIdx = 1:object.numberOfGmComponents
    meanVector = meanVector + ...
        weights(componentIdx) * object.mu{componentIdx};
end
covariance = zeros(stateDimension);
for componentIdx = 1:object.numberOfGmComponents
    delta = object.mu{componentIdx} - meanVector;
    covariance = covariance + weights(componentIdx) * ...
        (object.Sigma{componentIdx} + delta * delta');
end
covariance = (covariance + covariance') / 2;
end

function value = advertisedOpportunity(object)
value = NaN;
if isfield(object, 'advertisedObservationOpportunity') && ...
        isscalar(object.advertisedObservationOpportunity) && ...
        isfinite(object.advertisedObservationOpportunity)
    value = object.advertisedObservationOpportunity;
end
end

function value = boundedScalar(data, name)
value = 0;
if isstruct(data) && isfield(data, name) && ...
        isscalar(data.(name)) && isfinite(data.(name))
    value = data.(name);
end
value = min(max(value, 0), 1);
end

function value = quantizeUnit(value)
if ~isscalar(value) || ~isfinite(value)
    value = 0;
end
value = min(max(value, 0), 1);
value = double(uint16(round(value * double(intmax('uint16'))))) / ...
    double(intmax('uint16'));
end

function value = resolvePositionCutoff(model)
value = NaN;
if isfield(model, 'ospaParameters') && ...
        isstruct(model.ospaParameters) && ...
        isfield(model.ospaParameters, 'eC')
    value = model.ospaParameters.eC;
end
if ~isscalar(value) || ~isfinite(value) || value <= 0
    error('FormationRepairSynopsisV188:MissingCutoff', ...
        'A positive E-OSPA position cutoff is required.');
end
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
