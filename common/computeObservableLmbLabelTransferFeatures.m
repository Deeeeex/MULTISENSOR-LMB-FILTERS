function features = computeObservableLmbLabelTransferFeatures( ...
        receiverObject, sourceObject, model, receiverIdx, sourceIdx, ...
        currentTime)
% COMPUTEOBSERVABLELMBLABELTRANSFERFEATURES Present-time label handover state.
%
% The source carries a complete Bernoulli GM density.  This helper exposes
% only truth-free scalar metadata used before a receiver requests that full
% payload.  It never reads target truth or future measurements.

if isempty(sourceObject) || ~isstruct(sourceObject) || ...
        ~isscalar(sourceObject) || ...
        sourceObject.numberOfGmComponents <= 0
    error('ObservableLabelTransfer:InvalidSource', ...
        'A nonempty source Bernoulli GM label is required.');
end
positionCutoff = resolvePositionCutoff(model);
receiver = summarizeObject(receiverObject, positionCutoff);
source = summarizeObject(sourceObject, positionCutoff);
receiverOpportunity = observationOpportunity( ...
    model, receiverIdx, receiverObject, currentTime);
sourceOpportunity = observationOpportunity( ...
    model, sourceIdx, sourceObject, currentTime);
[mahalanobis, compatibility] = compareObjects(receiver, source);

existenceGain = max(source.existence - receiver.existence, 0);
precisionGain = max(log((receiver.positionTrace + eps) / ...
    (source.positionTrace + eps)), 0);
precisionGain = 1 - exp(-min(precisionGain, 20));
evidenceGap = max(source.evidenceQuality - ...
    receiver.evidenceQuality, 0);
opportunityGap = max(sourceOpportunity - receiverOpportunity, 0);
disagreement = 1 - exp(-0.5 * min(mahalanobis, 100));
sourcePrecisionQuality = 1 / (1 + ...
    source.positionTrace / max(positionCutoff^2, eps));
sourceQuality = source.existence * ( ...
    0.40 * source.evidenceQuality + ...
    0.40 * sourceOpportunity + ...
    0.20 * sourcePrecisionQuality);
riskReduction = receiver.bayesRisk - source.bayesRisk;
confidenceDisagreement = ...
    0.55 * existenceGain + 0.45 * sourceQuality * disagreement;
handoverRescue = 0.30 * existenceGain + ...
    0.20 * precisionGain + ...
    0.20 * evidenceGap + ...
    0.15 * opportunityGap + ...
    0.15 * sourceQuality * disagreement + ...
    0.25 * max(riskReduction, 0);

features = struct();
features.contractVersion = ...
    'observable-lmb-label-transfer-features-v1';
features.receiverPresent = receiver.present;
features.receiverExistence = receiver.existence;
features.sourceExistence = source.existence;
features.receiverBayesRisk = receiver.bayesRisk;
features.sourceBayesRisk = source.bayesRisk;
features.riskReductionScore = riskReduction;
features.receiverPositionTrace = receiver.positionTrace;
features.sourcePositionTrace = source.positionTrace;
features.receiverEvidenceQuality = receiver.evidenceQuality;
features.sourceEvidenceQuality = source.evidenceQuality;
features.receiverObservationOpportunity = receiverOpportunity;
features.sourceObservationOpportunity = sourceOpportunity;
features.existenceGain = existenceGain;
features.precisionGain = precisionGain;
features.evidenceGap = evidenceGap;
features.opportunityGap = opportunityGap;
features.mahalanobisDisagreement = mahalanobis;
features.normalizedDisagreement = disagreement;
features.compatibility = compatibility;
features.sourceQuality = sourceQuality;
features.confidenceDisagreementScore = confidenceDisagreement;
features.handoverRescueScore = handoverRescue;
features.payloadBytes = labelPayloadBytes(sourceObject);
features.riskAdvertisementBytes = 8;
features.richSynopsisBytes = 64;
features.truthUsed = false;
features.futureInformationUsed = false;
end

function summary = summarizeObject(object, positionCutoff)
summary = struct( ...
    'present', false, ...
    'existence', 0, ...
    'mean', zeros(0, 1), ...
    'covariance', zeros(0), ...
    'positionTrace', positionCutoff^2, ...
    'evidenceQuality', 0, ...
    'bayesRisk', 0.5);
if isempty(object) || object.numberOfGmComponents <= 0
    return;
end
[meanVector, covariance] = momentMatch(object);
positionDimension = min(2, numel(meanVector));
positionTrace = max(trace(covariance( ...
    1:positionDimension, 1:positionDimension)), 0);
existence = clamp01(object.r);
association = clamp01(getScalarField( ...
    object, 'associationConfidence', 0));
detection = clamp01(getScalarField( ...
    object, 'detectionAssociationMass', 0));
existenceRisk = min(existence, 1 - existence);
localizationRisk = existence * min( ...
    positionTrace / max(positionCutoff^2, eps), 1);
summary.present = true;
summary.existence = existence;
summary.mean = meanVector;
summary.covariance = covariance;
summary.positionTrace = positionTrace;
summary.evidenceQuality = 0.5 * association + 0.5 * detection;
summary.bayesRisk = 0.5 * existenceRisk + 0.5 * localizationRisk;
end

function value = observationOpportunity(model, sensorIdx, object, currentTime)
if isempty(object) || object.numberOfGmComponents <= 0
    value = 0;
    return;
end
if isfield(object, 'advertisedObservationOpportunity') && ...
        isscalar(object.advertisedObservationOpportunity) && ...
        isfinite(object.advertisedObservationOpportunity)
    value = clamp01(object.advertisedObservationOpportunity);
    return;
end
opportunity = computeLmbLabelObservationOpportunity( ...
    model, sensorIdx, object, currentTime);
value = clamp01(opportunity.expectedDetectionProbability);
end

function [mahalanobis, compatibility] = compareObjects(receiver, source)
if ~receiver.present || ~source.present
    mahalanobis = 0;
    compatibility = 0;
    return;
end
dimension = min(numel(receiver.mean), numel(source.mean));
delta = receiver.mean(1:dimension) - source.mean(1:dimension);
covariance = receiver.covariance(1:dimension, 1:dimension) + ...
    source.covariance(1:dimension, 1:dimension);
covariance = regularizeCovariance(covariance);
mahalanobis = max(real(delta' * (covariance \ delta)), 0);
if ~isfinite(mahalanobis)
    mahalanobis = 1e6;
end
compatibility = exp(-0.5 * min(mahalanobis, 100));
end

function [meanVector, covariance] = momentMatch(object)
componentCount = object.numberOfGmComponents;
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if numel(weights) ~= componentCount || sum(weights) <= 0
    weights = ones(1, componentCount);
end
weights = weights / sum(weights);
stateDimension = numel(object.mu{1});
meanVector = zeros(stateDimension, 1);
for componentIdx = 1:componentCount
    meanVector = meanVector + weights(componentIdx) * ...
        object.mu{componentIdx};
end
covariance = zeros(stateDimension);
for componentIdx = 1:componentCount
    delta = object.mu{componentIdx} - meanVector;
    covariance = covariance + weights(componentIdx) * ...
        (object.Sigma{componentIdx} + delta * delta');
end
covariance = regularizeCovariance(covariance);
end

function covariance = regularizeCovariance(covariance)
covariance = (covariance + covariance') / 2;
jitter = 0;
for attempt = 1:8
    [~, flag] = chol(covariance + jitter * eye(size(covariance)));
    if flag == 0
        covariance = covariance + jitter * eye(size(covariance));
        return;
    end
    jitter = max(1e-12, 10 * max(jitter, 1e-12));
end
error('ObservableLabelTransfer:InvalidCovariance', ...
    'A label covariance cannot be regularized.');
end

function bytes = labelPayloadBytes(object)
if isfield(object, 'advertisedCompletePayloadBytes') && ...
        isscalar(object.advertisedCompletePayloadBytes) && ...
        isfinite(object.advertisedCompletePayloadBytes) && ...
        object.advertisedCompletePayloadBytes >= 0
    bytes = object.advertisedCompletePayloadBytes;
    return;
end
dimension = numel(object.mu{1});
bytes = 8 * (3 + object.numberOfGmComponents * ...
    (1 + dimension + dimension * dimension));
end

function value = resolvePositionCutoff(model)
value = NaN;
if isfield(model, 'ospaParameters') && ...
        isstruct(model.ospaParameters) && ...
        isfield(model.ospaParameters, 'eC')
    value = model.ospaParameters.eC;
end
if ~isscalar(value) || ~isfinite(value) || value <= 0
    error('ObservableLabelTransfer:MissingCutoff', ...
        'A positive E-OSPA position cutoff is required.');
end
end

function value = getScalarField(data, name, fallback)
value = fallback;
if isstruct(data) && isfield(data, name) && ...
        isscalar(data.(name)) && isfinite(data.(name))
    value = data.(name);
end
end

function value = clamp01(value)
if ~isfinite(value)
    value = 0;
end
value = min(max(value, 0), 1);
end
