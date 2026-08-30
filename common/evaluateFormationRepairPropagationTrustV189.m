function trust = evaluateFormationRepairPropagationTrustV189( ...
        proposal, fusedPosteriorBySensor, model, options)
% EVALUATEFORMATIONREPAIRPROPAGATIONTRUSTV189 Receiver-local motion gate.
%
% The shortlisted source advertises the moment state already covered by the
% charged rich synopsis.  Each receiver compares that source moment with its
% own fused Bernoulli, propagates both through the known motion model, and
% returns one fixed-size vote.  No truth or future measurement is read.

if nargin < 4 || isempty(options)
    options = struct();
end
horizonSteps = getField(options, 'horizonSteps', 3);
positionCutoff = getField(options, 'positionCutoff', ...
    resolvePositionCutoff(model));
voteBytesPerReceiver = getField(options, ...
    'voteBytesPerReceiver', 8);
validateInputs( ...
    proposal, fusedPosteriorBySensor, model, horizonSteps, ...
    positionCutoff, voteBytesPerReceiver);

receiverIds = reshape(proposal.receiverIds, 1, []);
receiverCount = numel(receiverIds);
distances = inf(receiverCount, horizonSteps);
receiverPassed = false(1, receiverCount);
[sourceMean, sourceCovariance] = momentMatch( ...
    proposal.sourceObject, model.xDimension);
sourcePredicted = sourceMean;
sourcePredictedCovariance = sourceCovariance;
for receiverPosition = 1:receiverCount
    receiverIdx = receiverIds(receiverPosition);
    receiverObject = findLabelObject( ...
        fusedPosteriorBySensor{receiverIdx}, proposal.label);
    if isempty(receiverObject)
        continue;
    end
    [receiverPredicted, receiverPredictedCovariance] = ...
        momentMatch(receiverObject, model.xDimension);
    sourceState = sourcePredicted;
    sourceCovarianceState = sourcePredictedCovariance;
    receiverState = receiverPredicted;
    receiverCovarianceState = receiverPredictedCovariance;
    for horizonIdx = 1:horizonSteps
        distances(receiverPosition, horizonIdx) = norm( ...
            sourceState(1:2) - receiverState(1:2));
        if horizonIdx < horizonSteps
            sourceState = model.A * sourceState + model.u;
            receiverState = model.A * receiverState + model.u;
            sourceCovarianceState = model.A * ...
                sourceCovarianceState * model.A' + model.R;
            receiverCovarianceState = model.A * ...
                receiverCovarianceState * model.A' + model.R;
        end
    end
    receiverPassed(receiverPosition) = all( ...
        distances(receiverPosition, :) <= positionCutoff + 1e-12);
end

trust = struct();
trust.contractVersion = ...
    'formation-repair-propagation-trust-v189-v1';
trust.formationId = proposal.formationId;
trust.sourceId = proposal.sourceId;
trust.label = proposal.label;
trust.receiverIds = receiverIds;
trust.horizonSteps = horizonSteps;
trust.positionCutoff = positionCutoff;
trust.positionDistanceByReceiverHorizon = distances;
trust.maximumDistanceByReceiver = max(distances, [], 2)';
trust.maximumDistance = max(distances(:));
trust.maximumNormalizedDistance = ...
    trust.maximumDistance / positionCutoff;
trust.receiverPassed = receiverPassed;
trust.passedAllReceivers = all(receiverPassed);
trust.richSynopsisBytes = proposal.richSynopsisBytes;
trust.voteBytesPerReceiver = voteBytesPerReceiver;
trust.voteBytes = receiverCount * voteBytesPerReceiver;
trust.preflightAttemptedBytes = ...
    trust.richSynopsisBytes + trust.voteBytes;
trust.payloadAttemptedBytesIfAccepted = ...
    proposal.requestBytes + proposal.responseBytes;
trust.totalAttemptedBytesIfAccepted = ...
    trust.preflightAttemptedBytes + ...
    trust.payloadAttemptedBytesIfAccepted;
trust.receiverVoteRecordLayout = ...
    'float32 maximum-normalized-distance, uint8 decision, padding';
trust.sourceMomentCarriedByRichSynopsis = true;
trust.receiverMomentRemainsLocal = true;
trust.truthUsed = false;
trust.futureMeasurementsUsed = false;
trust.fixedFormationIdentifierUsed = false;
trust.trackingImprovementGuaranteed = false;
trust.evidenceBoundary = [ ...
    'Passing certifies only that every receiver and the source remain ', ...
    'inside the registered position-error trust region under open-loop ', ...
    'prediction. It is a causal necessary safety gate, not a sufficient ', ...
    'tracking-performance guarantee.'];
end

function validateInputs( ...
        proposal, fusedPosteriorBySensor, model, horizonSteps, ...
        positionCutoff, voteBytesPerReceiver)
requiredProposal = { ...
    'formationId', 'receiverIds', 'sourceId', 'label', ...
    'sourceObject', 'richSynopsisBytes', 'requestBytes', ...
    'responseBytes'};
if ~isstruct(proposal) || ~isscalar(proposal) || ...
        any(~isfield(proposal, requiredProposal)) || ...
        ~iscell(fusedPosteriorBySensor) || ...
        any(proposal.receiverIds < 1) || ...
        any(proposal.receiverIds > numel(fusedPosteriorBySensor)) || ...
        numel(unique(proposal.receiverIds)) ~= ...
            numel(proposal.receiverIds) || ...
        ~isstruct(model) || ~isscalar(model) || ...
        ~isfield(model, 'xDimension') || model.xDimension < 2 || ...
        ~isfield(model, 'A') || ...
        ~isequal(size(model.A), [model.xDimension, model.xDimension]) || ...
        ~isfield(model, 'u') || numel(model.u) ~= model.xDimension || ...
        ~isfield(model, 'R') || ...
        ~isequal(size(model.R), [model.xDimension, model.xDimension]) || ...
        ~isscalar(horizonSteps) || ~isfinite(horizonSteps) || ...
        horizonSteps < 1 || horizonSteps ~= round(horizonSteps) || ...
        ~isscalar(positionCutoff) || ~isfinite(positionCutoff) || ...
        positionCutoff <= 0 || ...
        ~isscalar(voteBytesPerReceiver) || ...
        ~isfinite(voteBytesPerReceiver) || ...
        voteBytesPerReceiver < 0 || ...
        voteBytesPerReceiver ~= round(voteBytesPerReceiver)
    error('FormationRepairPropagationTrustV189:InvalidInput', ...
        'The propagation-trust input is malformed.');
end
end

function object = findLabelObject(objects, label)
object = [];
objects = reshape(objects, 1, []);
for objectIdx = 1:numel(objects)
    candidate = objects(objectIdx);
    if candidate.birthTime == label(1) && ...
            candidate.birthLocation == label(2)
        object = candidate;
        return;
    end
end
end

function [meanVector, covariance] = momentMatch(object, stateDimension)
if isempty(object) || object.numberOfGmComponents < 1 || ...
        numel(object.w) ~= object.numberOfGmComponents
    error('FormationRepairPropagationTrustV189:InvalidBernoulli', ...
        'The propagation vote requires a valid Bernoulli GM density.');
end
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if sum(weights) <= 0
    error('FormationRepairPropagationTrustV189:InvalidBernoulli', ...
        'The Bernoulli GM weights have zero mass.');
end
weights = weights / sum(weights);
meanVector = zeros(stateDimension, 1);
for componentIdx = 1:numel(weights)
    componentMean = reshape(object.mu{componentIdx}, [], 1);
    if numel(componentMean) ~= stateDimension
        error('FormationRepairPropagationTrustV189:StateDimensionDrift', ...
            'A GM component has the wrong state dimension.');
    end
    meanVector = meanVector + weights(componentIdx) * componentMean;
end
covariance = zeros(stateDimension);
for componentIdx = 1:numel(weights)
    componentCovariance = object.Sigma{componentIdx};
    if ~isequal(size(componentCovariance), ...
            [stateDimension, stateDimension])
        error('FormationRepairPropagationTrustV189:StateDimensionDrift', ...
            'A GM covariance has the wrong state dimension.');
    end
    delta = reshape(object.mu{componentIdx}, [], 1) - meanVector;
    covariance = covariance + weights(componentIdx) * ...
        (componentCovariance + delta * delta');
end
covariance = (covariance + covariance') / 2;
end

function value = resolvePositionCutoff(model)
value = NaN;
if isstruct(model) && isfield(model, 'ospaParameters') && ...
        isstruct(model.ospaParameters) && ...
        isfield(model.ospaParameters, 'eC')
    value = model.ospaParameters.eC;
end
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
