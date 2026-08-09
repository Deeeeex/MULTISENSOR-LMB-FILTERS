function synopsis = buildLmbLabelControlSynopsis( ...
        posteriorObjects, labelEvidence, model, options)
% BUILDLMBLABELCONTROLSYNOPSIS Fixed-shape observable V54 control metadata.
%
% The synopsis is used to predict label-subset utility before any selected
% full Gaussian-mixture label payload is transmitted. It contains posterior
% moments and current local evidence, but not Gaussian-mixture components.

if nargin < 4 || isempty(options)
    options = struct();
end
if ~isstruct(posteriorObjects) || ~isstruct(labelEvidence) || ...
        ~isstruct(model) || ~isfield(model, 'xDimension')
    error('LmbControlSynopsis:InvalidInput', ...
        'Posterior objects, label evidence, and a model are required.');
end
stateDimension = model.xDimension;
existenceThreshold = getField( ...
    options, 'existenceThreshold', getField(model, 'existenceThreshold', 0));
headerScalars = getField(options, 'headerScalars', 4);
if ~isscalar(headerScalars) || ~isfinite(headerScalars) || ...
        headerScalars < 0 || headerScalars ~= round(headerScalars)
    error('LmbControlSynopsis:InvalidHeaderCost', ...
        'headerScalars must be a nonnegative integer.');
end

active = false(1, numel(posteriorObjects));
for objectIdx = 1:numel(posteriorObjects)
    active(objectIdx) = posteriorObjects(objectIdx).r > ...
        existenceThreshold && ...
        posteriorObjects(objectIdx).numberOfGmComponents > 0;
end
activeObjects = posteriorObjects(active);
template = makeLabelSynopsis(stateDimension);
labels = repmat(template, 1, numel(activeObjects));
for objectIdx = 1:numel(activeObjects)
    object = activeObjects(objectIdx);
    evidenceIdx = findEvidenceIndex(labelEvidence, object);
    if evidenceIdx == 0
        error('LmbControlSynopsis:MissingEvidence', ...
            'Current-step evidence is missing for label (%g, %g).', ...
            object.birthTime, object.birthLocation);
    end
    metadata = labelEvidence(evidenceIdx);
    [mean, covariance, mixtureEntropy] = momentMatchObject( ...
        object, stateDimension);
    labels(objectIdx).birthTime = object.birthTime;
    labels(objectIdx).birthLocation = object.birthLocation;
    labels(objectIdx).existenceProbability = object.r;
    positionDimension = min(2, stateDimension);
    velocityIndices = (positionDimension + 1):stateDimension;
    positionCovariance = covariance( ...
        1:positionDimension, 1:positionDimension);
    labels(objectIdx).positionMean = mean(1:positionDimension);
    labels(objectIdx).velocityMean = mean(velocityIndices);
    labels(objectIdx).upperPositionCovariance = ...
        positionCovariance(triu(true(positionDimension)))';
    labels(objectIdx).velocityVariance = ...
        diag(covariance(velocityIndices, velocityIndices))';
    labels(objectIdx).componentCount = ...
        object.numberOfGmComponents;
    labels(objectIdx).normalizedMixtureEntropy = mixtureEntropy;
    labels(objectIdx).expectedDetectionProbability = ...
        metadata.opportunity.expectedDetectionProbability;
    labels(objectIdx).detectionAssociationMass = ...
        metadata.evidence.detectionAssociationMass;
    labels(objectIdx).predictedExistence = ...
        metadata.evidence.predictedExistence;
    labels(objectIdx).evidenceCode = evidenceTypeCode(metadata.type);
end

positionDimension = min(2, stateDimension);
velocityDimension = stateDimension - positionDimension;
positionCovarianceScalars = ...
    positionDimension * (positionDimension + 1) / 2;
% Numeric learning features kept in memory.  The wire contract quantizes
% continuous features to float32, labels to int16, and the component/evidence
% codes to uint8/int8.  Records are padded to a four-byte boundary.
perLabelScalars = 9 + stateDimension + ...
    positionCovarianceScalars + velocityDimension;
featureScalarCount = headerScalars + numel(labels) * perLabelScalars;
headerBytes = 4 * headerScalars;
perLabelFloat32Count = 5 + stateDimension + ...
    positionCovarianceScalars + velocityDimension;
rawPerLabelBytes = 4 + 4 * perLabelFloat32Count + 2;
perLabelBytes = 4 * ceil(rawPerLabelBytes / 4);
synopsis = struct();
synopsis.contractVersion = 'lmb-label-control-synopsis-v2-compact';
synopsis.labels = labels;
synopsis.labelCount = numel(labels);
synopsis.headerScalars = headerScalars;
synopsis.perLabelScalars = perLabelScalars;
synopsis.headerBytes = headerBytes;
synopsis.perLabelBytes = perLabelBytes;
synopsis.estimatedBytes = headerBytes + numel(labels) * perLabelBytes;
synopsis.scalarCount = synopsis.estimatedBytes / 8;
synopsis.featureScalarCount = featureScalarCount;
synopsis.wireEncoding = ...
    'uint32-header-int16-label-float32-features-uint8-codes-padded4';
synopsis.quantizationCalibrationRequired = true;
synopsis.containsGaussianMixtureComponents = false;
synopsis.isPosteriorFusionPayload = false;
synopsis.usesTargetTruth = false;
synopsis.usesFutureMeasurements = false;
end

function [mean, covariance, entropy] = momentMatchObject(object, dimension)
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if numel(weights) ~= object.numberOfGmComponents || sum(weights) <= 0
    error('LmbControlSynopsis:InvalidMixture', ...
        'Every synopsis label must contain a valid Gaussian mixture.');
end
weights = weights / sum(weights);
mean = zeros(dimension, 1);
for componentIdx = 1:numel(weights)
    componentMean = reshape(object.mu{componentIdx}, [], 1);
    if numel(componentMean) ~= dimension
        error('LmbControlSynopsis:DimensionMismatch', ...
            'A component state dimension does not match the model.');
    end
    mean = mean + weights(componentIdx) * componentMean;
end
covariance = zeros(dimension);
for componentIdx = 1:numel(weights)
    residual = reshape(object.mu{componentIdx}, [], 1) - mean;
    componentCovariance = object.Sigma{componentIdx};
    if ~isequal(size(componentCovariance), [dimension, dimension])
        error('LmbControlSynopsis:DimensionMismatch', ...
            'A component covariance dimension does not match the model.');
    end
    covariance = covariance + weights(componentIdx) * ...
        (componentCovariance + residual * residual');
end
covariance = (covariance + covariance') / 2;
if numel(weights) <= 1
    entropy = 0;
else
    positiveWeights = weights(weights > 0);
    entropy = -sum(positiveWeights .* log(positiveWeights)) / ...
        log(numel(weights));
end
end

function index = findEvidenceIndex(labelEvidence, object)
index = 0;
for evidenceIdx = 1:numel(labelEvidence)
    if labelEvidence(evidenceIdx).birthTime == object.birthTime && ...
            labelEvidence(evidenceIdx).birthLocation == ...
                object.birthLocation
        index = evidenceIdx;
        return;
    end
end
end

function code = evidenceTypeCode(type)
switch lower(type)
    case 'positive-support'
        code = 1;
    case 'credible-negative'
        code = -1;
    case 'unsupported-absence'
        code = -2;
    case 'ambiguous'
        code = 0;
    otherwise
        error('LmbControlSynopsis:UnknownEvidenceType', ...
            'Unknown evidence type: %s.', type);
end
end

function template = makeLabelSynopsis(dimension)
positionDimension = min(2, dimension);
velocityDimension = dimension - positionDimension;
template = struct( ...
    'birthTime', 0, ...
    'birthLocation', 0, ...
    'existenceProbability', 0, ...
    'positionMean', zeros(positionDimension, 1), ...
    'velocityMean', zeros(velocityDimension, 1), ...
    'upperPositionCovariance', zeros( ...
        1, positionDimension * (positionDimension + 1) / 2), ...
    'velocityVariance', zeros(1, velocityDimension), ...
    'componentCount', 0, ...
    'normalizedMixtureEntropy', 0, ...
    'expectedDetectionProbability', 0, ...
    'detectionAssociationMass', 0, ...
    'predictedExistence', 0, ...
    'evidenceCode', 0);
end

function value = getField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end
