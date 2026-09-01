function diagnostics = computeCompleteLabelKlaPairDiagnosticsV217( ...
        receiverObject, sourceObject, model, options)
% COMPUTECOMPLETELABELKLAPAIRDIAGNOSTICSV217 Inspect the executed KLA pair.
%
% This helper calls the same componentwise powered-GM label-fusion operator
% used by the recursive tracker and exposes its spatial normalizer eta.  It
% is an operator diagnostic, not an exact arbitrary-GM KLA certificate.

if nargin < 4 || isempty(options)
    options = struct();
end
sourceWeight = getField(options, 'sourceWeight', 0.5);
if ~isscalar(sourceWeight) || ~isnumeric(sourceWeight) || ...
        ~isfinite(sourceWeight) || sourceWeight <= 0 || sourceWeight >= 1
    error('CompleteLabelKlaDiagnosticsV217:InvalidSourceWeight', ...
        'The source weight must lie strictly between zero and one.');
end
validateObject(receiverObject, 'receiver');
validateObject(sourceObject, 'source');
receiverLabel = [receiverObject.birthTime; receiverObject.birthLocation];
sourceLabel = [sourceObject.birthTime; sourceObject.birthLocation];
if ~isequal(receiverLabel, sourceLabel)
    error('CompleteLabelKlaDiagnosticsV217:LabelMismatch', ...
        'Receiver and source objects must carry the same semantic label.');
end
if ~isstruct(model) || ~isscalar(model) || ...
        ~isfield(model, 'xDimension') || ...
        ~isscalar(model.xDimension) || ~isfinite(model.xDimension) || ...
        model.xDimension < 1 || model.xDimension ~= round(model.xDimension)
    error('CompleteLabelKlaDiagnosticsV217:InvalidModel', ...
        'The model must expose a positive integer state dimension.');
end

overrides = getField(options, 'triggerConfigOverrides', struct());
if ~isstruct(overrides) || ~isscalar(overrides)
    error('CompleteLabelKlaDiagnosticsV217:InvalidOverrides', ...
        'The trigger-config overrides must be one scalar structure.');
end
overrides.captureLabelKlaDiagnosticsEnabled = true;
triggerConfig = buildMixtureAwareKlaReferenceConfig(overrides);
weights = [1 - sourceWeight, sourceWeight];
fusionDetails = struct('eventType', [0, 2]);
[fusedObjects, fusionDiagnostics] = fuseLmbPosteriorsByLabel( ...
    {receiverObject, sourceObject}, weights, model, weights, ...
    fusionDetails, triggerConfig);
if numel(fusedObjects) ~= 1 || ...
        numel(fusionDiagnostics.labelKlaRecords) ~= 1
    error('CompleteLabelKlaDiagnosticsV217:FusionContractDrift', ...
        'The two-input complete-label fusion returned an invalid result.');
end
record = fusionDiagnostics.labelKlaRecords(1);
receiverLogOdds = probabilityLogOdds(receiverObject.r);
sourceLogOdds = probabilityLogOdds(sourceObject.r);
[receiverMean, receiverCovariance] = objectMoments(receiverObject);
[sourceMean, sourceCovariance] = objectMoments(sourceObject);
positionDimension = min(2, model.xDimension);
positionDelta = receiverMean(1:positionDimension) - ...
    sourceMean(1:positionDimension);
positionTraceSum = trace(receiverCovariance( ...
    1:positionDimension, 1:positionDimension)) + ...
    trace(sourceCovariance(1:positionDimension, 1:positionDimension));
positionIsotropicSquaredDistance = ...
    positionDimension * sum(positionDelta .^ 2) / ...
    max(positionTraceSum, eps);
positionMarginalLogNormalizer = gaussianChernoffLogNormalizer( ...
    receiverMean(1:positionDimension), ...
    receiverCovariance(1:positionDimension, 1:positionDimension), ...
    sourceMean(1:positionDimension), ...
    sourceCovariance(1:positionDimension, 1:positionDimension), ...
    sourceWeight);
positionChiSquare99Threshold = 9.21034037197618;

diagnostics = record;
diagnostics.contractVersion = ...
    'complete-label-kla-pair-diagnostics-v217-v1';
diagnostics.sourceWeight = sourceWeight;
diagnostics.receiverWeight = 1 - sourceWeight;
diagnostics.receiverExistence = receiverObject.r;
diagnostics.sourceExistence = sourceObject.r;
diagnostics.receiverLogOdds = receiverLogOdds;
diagnostics.sourceLogOdds = sourceLogOdds;
diagnostics.receiverLogOddsDelta = ...
    record.fusedLogOdds - receiverLogOdds;
diagnostics.sourceLogOddsDelta = ...
    record.fusedLogOdds - sourceLogOdds;
diagnostics.positionIsotropicSquaredDistance = ...
    positionIsotropicSquaredDistance;
diagnostics.positionIsotropicCompatibility = exp( ...
    -0.5 * min(positionIsotropicSquaredDistance, 100));
diagnostics.positionMarginalLogNormalizer = ...
    positionMarginalLogNormalizer;
diagnostics.positionMarginalNormalizer = exp( ...
    min(positionMarginalLogNormalizer, log(realmax)));
diagnostics.positionChiSquare99Threshold = ...
    positionChiSquare99Threshold;
diagnostics.positionSupport99Passed = ...
    positionIsotropicSquaredDistance <= ...
    positionChiSquare99Threshold + 1e-12;
diagnostics.fusedObject = fusedObjects(1);
diagnostics.operatorIdentityPassed = ...
    abs(record.normalizerIdentityResidual) <= 1e-9;
diagnostics.operatorQualification = ...
    'repository-componentwise-powered-gm-approximation';
diagnostics.exactArbitraryGmKlaClaimed = false;
diagnostics.truthUsed = false;
diagnostics.futureInformationUsed = false;
end

function [meanVector, covariance] = objectMoments(object)
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if sum(weights) <= 0
    weights = ones(size(weights));
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
covariance = regularizeCovariance(covariance);
end

function logNormalizer = gaussianChernoffLogNormalizer( ...
        receiverMean, receiverCovariance, sourceMean, sourceCovariance, ...
        sourceWeight)
receiverWeight = 1 - sourceWeight;
receiverCovariance = regularizeCovariance(receiverCovariance);
sourceCovariance = regularizeCovariance(sourceCovariance);
receiverPrecision = inv(receiverCovariance);
sourcePrecision = inv(sourceCovariance);
canonicalK = receiverWeight * receiverPrecision + ...
    sourceWeight * sourcePrecision;
canonicalH = receiverWeight * receiverPrecision * receiverMean + ...
    sourceWeight * sourcePrecision * sourceMean;
canonicalG = -0.5 * receiverWeight * ...
    (receiverMean' * receiverPrecision * receiverMean) - ...
    0.5 * receiverWeight * logDet(2 * pi * receiverCovariance) - ...
    0.5 * sourceWeight * ...
    (sourceMean' * sourcePrecision * sourceMean) - ...
    0.5 * sourceWeight * logDet(2 * pi * sourceCovariance);
fusedCovariance = regularizeCovariance(inv(canonicalK));
fusedMean = fusedCovariance * canonicalH;
logNormalizer = canonicalG + ...
    0.5 * fusedMean' * canonicalK * fusedMean + ...
    0.5 * logDet(2 * pi * fusedCovariance);
logNormalizer = real(logNormalizer);
end

function covariance = regularizeCovariance(covariance)
covariance = (covariance + covariance') / 2;
if rcond(covariance) < 1e-12
    covariance = covariance + 1e-9 * eye(size(covariance));
end
end

function value = logDet(matrix)
matrix = regularizeCovariance(matrix);
[factor, flag] = chol(matrix);
if flag == 0
    value = 2 * sum(log(diag(factor)));
else
    value = log(max(real(det(matrix)), realmin));
end
end

function validateObject(object, role)
required = {'numberOfGmComponents', 'w', 'mu', 'Sigma', 'r', ...
    'birthTime', 'birthLocation'};
if ~isstruct(object) || ~isscalar(object) || ...
        any(~isfield(object, required)) || ...
        ~isscalar(object.numberOfGmComponents) || ...
        ~isfinite(object.numberOfGmComponents) || ...
        object.numberOfGmComponents < 1 || ...
        object.numberOfGmComponents ~= round(object.numberOfGmComponents) || ...
        numel(object.w) ~= object.numberOfGmComponents || ...
        ~iscell(object.mu) || ~iscell(object.Sigma) || ...
        numel(object.mu) ~= object.numberOfGmComponents || ...
        numel(object.Sigma) ~= object.numberOfGmComponents || ...
        ~isscalar(object.r) || ~isfinite(object.r) || ...
        object.r <= 0 || object.r >= 1
    error('CompleteLabelKlaDiagnosticsV217:InvalidObject', ...
        'The %s complete-label object is malformed.', role);
end
end

function value = probabilityLogOdds(probability)
probability = min(max(probability, 1e-9), 1 - 1e-9);
value = log(probability) - log(1 - probability);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
