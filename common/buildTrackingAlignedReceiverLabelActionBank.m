function bank = buildTrackingAlignedReceiverLabelActionBank(context, options)
% BUILDTRACKINGALIGNEDRECEIVERLABELACTIONBANK V61 observable label actions.
%
% The registered physical/reference route is unchanged.  Each nonreference
% action removes a bundle of sender labels from selected cross-formation
% residual messages.  Candidate ranking uses only current posterior fields
% and exact one-round counterfactual fusion under the implementation that
% will execute the action.

if nargin < 2 || isempty(options)
    options = struct();
end
if ~isstruct(context) || ...
        ~isfield(context, 'localPosteriorBySensor') || ...
        ~isfield(context, 'model')
    error('ReceiverLabelBank:InvalidContext', ...
        'A current observable LMB context is required.');
end
nodeCount = numel(context.localPosteriorBySensor);
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
if numel(groupIds) ~= nodeCount
    error('ReceiverLabelBank:InvalidFormationGroups', ...
        'The formation membership does not match the posterior count.');
end

supportThreshold = getField(options, ...
    'positiveSupportThreshold', 0.20);
activeThreshold = getField(options, ...
    'payloadExistenceThreshold', getField( ...
        buildMixtureAwareKlaReferenceConfig(), ...
        'payloadExistenceThreshold', 0));
minimumScore = getField(options, 'minimumCandidateScore', 1e-8);
protocol = getFormationIsolateReconnectProbeProtocol();
policyOptions = struct( ...
    'dominantWeight', protocol.dominantWeight, ...
    'residualWeight', protocol.residualWeight);
[referenceAdjacency, referenceDetails] = ...
    selectBackboneResidualSplicedCyclePolicy( ...
        context, 'fixed-counter-clockwise', policyOptions);
referenceWeights = referenceDetails.fusionWeightMatrix;
crossResidual = logical(referenceDetails.residualAdjacency) & ...
    (groupIds(:) ~= groupIds(:)');
if nnz(crossResidual) < 2
    error('ReceiverLabelBank:MissingCrossResidual', ...
        'The registered reference lacks cross-formation residual edges.');
end

fusionConfig = buildMixtureAwareKlaReferenceConfig();
candidates = repmat(emptyCandidate(), 1, 0);
candidateCursor = 0;
for receiverIdx = reshape(find(any(crossResidual, 2)), 1, [])
    incomingSenders = reshape(find( ...
        referenceAdjacency(receiverIdx, :)), 1, []);
    distributions = [{context.localPosteriorBySensor{receiverIdx}}, ...
        context.localPosteriorBySensor(incomingSenders)];
    weights = [referenceWeights(receiverIdx, receiverIdx), ...
        referenceWeights(receiverIdx, incomingSenders)];
    fusionDetails = struct('eventType', [0, 2 * ones( ...
        1, numel(incomingSenders))]);
    referenceFused = fuseLmbPosteriorsByLabel( ...
        distributions, weights, context.model, weights, ...
        fusionDetails, fusionConfig);
    crossSenders = reshape(find(crossResidual(receiverIdx, :)), 1, []);
    for senderIdx = crossSenders
        localSenderIdx = find(incomingSenders == senderIdx, 1);
        if isempty(localSenderIdx)
            error('ReceiverLabelBank:ReferenceEdgeMismatch', ...
                'A cross residual edge is absent from the reference route.');
        end
        senderObjects = activeObjects( ...
            context.localPosteriorBySensor{senderIdx}, activeThreshold);
        for senderObjectIdx = 1:numel(senderObjects)
            senderObject = senderObjects(senderObjectIdx);
            label = [senderObject.birthTime; senderObject.birthLocation];
            receiverObject = findObject( ...
                context.localPosteriorBySensor{receiverIdx}, label);
            if isempty(receiverObject)
                continue;
            end
            receiverSupport = clamp01(getField( ...
                receiverObject, 'detectionAssociationMass', 0));
            senderSupport = clamp01(getField( ...
                senderObject, 'detectionAssociationMass', 0));
            if receiverSupport < supportThreshold - 1e-12 || ...
                    senderSupport >= supportThreshold - 1e-12
                continue;
            end

            candidateDistributions = distributions;
            sourcePosition = 1 + localSenderIdx;
            candidateDistributions{sourcePosition} = removeLabel( ...
                candidateDistributions{sourcePosition}, label);
            candidateFused = fuseLmbPosteriorsByLabel( ...
                candidateDistributions, weights, context.model, weights, ...
                fusionDetails, fusionConfig);
            referenceObject = findObject(referenceFused, label);
            candidateObject = findObject(candidateFused, label);
            if isempty(referenceObject) || isempty(candidateObject)
                continue;
            end
            existenceGain = candidateObject.r - referenceObject.r;
            [receiverMean, ~] = momentMatch( ...
                receiverObject, context.model.xDimension);
            [referenceMean, ~] = momentMatch( ...
                referenceObject, context.model.xDimension);
            [candidateMean, ~] = momentMatch( ...
                candidateObject, context.model.xDimension);
            positionDimension = min(2, context.model.xDimension);
            positionScale = resolvePositionScale(context.model);
            referenceDistance = norm( ...
                referenceMean(1:positionDimension) - ...
                receiverMean(1:positionDimension)) / positionScale;
            candidateDistance = norm( ...
                candidateMean(1:positionDimension) - ...
                receiverMean(1:positionDimension)) / positionScale;
            anchorGain = referenceDistance - candidateDistance;
            upwardCrossing = referenceObject.r < 0.50 && ...
                candidateObject.r >= 0.50;
            existenceScore = receiverSupport * ( ...
                max(existenceGain, 0) + 0.25 * double(upwardCrossing));
            spatialScore = receiverSupport * ...
                min(receiverObject.r, candidateObject.r) * ...
                max(anchorGain, 0);
            combinedScore = existenceScore + spatialScore;
            if combinedScore <= minimumScore
                continue;
            end
            candidateCursor = candidateCursor + 1;
            candidate = emptyCandidate();
            candidate.index = candidateCursor;
            candidate.receiverIdx = receiverIdx;
            candidate.senderIdx = senderIdx;
            candidate.receiverFormationId = groupIds(receiverIdx);
            candidate.senderFormationId = groupIds(senderIdx);
            candidate.label = label;
            candidate.receiverSupport = receiverSupport;
            candidate.senderSupport = senderSupport;
            candidate.referenceExistence = referenceObject.r;
            candidate.candidateExistence = candidateObject.r;
            candidate.existenceGain = existenceGain;
            candidate.referenceAnchorDistance = referenceDistance;
            candidate.candidateAnchorDistance = candidateDistance;
            candidate.anchorGain = anchorGain;
            candidate.upwardDecisionCrossing = upwardCrossing;
            candidate.existenceScore = existenceScore;
            candidate.spatialScore = spatialScore;
            candidate.combinedScore = combinedScore;
            candidate.labelPayloadBytes = objectLabelBytes( ...
                senderObject, context.model.xDimension);
            candidates(candidateCursor) = candidate; %#ok<AGROW>
        end
    end
end

synopsisBytes = totalSynopsisBytes( ...
    context.localPosteriorBySensor, crossResidual, ...
    context.model.xDimension, activeThreshold);
[actionNames, actionCandidateIndices] = buildBundleSet( ...
    candidates, synopsisBytes);
actionCount = numel(actionNames);
dropPlans = cell(1, actionCount);
actionDroppedBytes = zeros(1, actionCount);
actionObjective = zeros(1, actionCount);
actionWithinPayload = true(1, actionCount);
for actionIdx = 1:actionCount
    dropPlan = cell(nodeCount);
    selected = actionCandidateIndices{actionIdx};
    for candidateIdx = reshape(selected, 1, [])
        candidate = candidates(candidateIdx);
        labels = dropPlan{candidate.receiverIdx, candidate.senderIdx};
        labels(:, end + 1) = candidate.label; %#ok<AGROW>
        dropPlan{candidate.receiverIdx, candidate.senderIdx} = labels;
        actionDroppedBytes(actionIdx) = ...
            actionDroppedBytes(actionIdx) + candidate.labelPayloadBytes;
        actionObjective(actionIdx) = ...
            actionObjective(actionIdx) + candidate.combinedScore;
    end
    dropPlans{actionIdx} = dropPlan;
    if actionIdx > 1
        actionWithinPayload(actionIdx) = ...
            actionDroppedBytes(actionIdx) >= synopsisBytes - 1e-9;
    end
end

actionAdjacency = repmat(logical(referenceAdjacency), 1, 1, actionCount);
actionWeights = repmat(referenceWeights, 1, 1, actionCount);
bank = struct();
bank.contractVersion = ...
    'tracking-aligned-receiver-label-action-bank-v61-v1';
bank.actionCount = actionCount;
bank.referenceActionIndex = 1;
bank.actionNames = actionNames;
bank.actionAdjacency = actionAdjacency;
bank.actionFusionWeights = actionWeights;
bank.actionDropLabelsByReceiverSender = dropPlans;
bank.actionCandidateIndices = actionCandidateIndices;
bank.actionPosteriorObjective = actionObjective;
bank.actionPosteriorProxyAllowed = actionWithinPayload;
bank.actionWithinReferencePayload = actionWithinPayload;
bank.actionDroppedPayloadBytes = actionDroppedBytes;
bank.actionControlSynopsisBytes = [0, ...
    synopsisBytes * ones(1, max(actionCount - 1, 0))];
bank.actionPredictedNetSavingBytes = actionDroppedBytes - ...
    bank.actionControlSynopsisBytes;
bank.candidates = candidates;
bank.referenceAdjacency = logical(referenceAdjacency);
bank.referenceFusionWeights = referenceWeights;
bank.referencePolicyDetails = referenceDetails;
bank.crossResidualAdjacency = crossResidual;
bank.positiveSupportThreshold = supportThreshold;
bank.payloadExistenceThreshold = activeThreshold;
bank.controlSynopsisBytes = synopsisBytes;
bank.truthUsed = false;
bank.futureMeasurementsUsed = false;
bank.futureOutcomesUsed = false;
end

function [names, selections] = buildBundleSet(candidates, synopsisBytes)
names = {'reference-full-payload'};
selections = {zeros(1, 0)};
if isempty(candidates)
    return;
end
families = { ...
    'existence', [candidates.existenceScore]; ...
    'spatial', [candidates.spatialScore]; ...
    'combined', [candidates.combinedScore]};
for familyIdx = 1:size(families, 1)
    familyName = families{familyIdx, 1};
    scores = families{familyIdx, 2};
    positive = find(scores > 1e-12);
    if isempty(positive)
        continue;
    end
    rankingTable = [-reshape(scores(positive), [], 1), ...
        reshape(positive, [], 1)];
    [~, order] = sortrows(rankingTable, [1, 2]);
    ordered = positive(order);
    saved = cumsum([candidates(ordered).labelPayloadBytes]);
    byteSafeIdx = find(saved >= synopsisBytes - 1e-9, 1);
    if isempty(byteSafeIdx)
        continue;
    end
    prefixes = unique([byteSafeIdx, min(byteSafeIdx + 2, numel(ordered)), ...
        numel(ordered)], 'stable');
    for prefix = prefixes
        selected = reshape(ordered(1:prefix), 1, []);
        if hasSelection(selections, selected)
            continue;
        end
        names{end + 1} = sprintf('%s-top-%d', ...
            familyName, prefix); %#ok<AGROW>
        selections{end + 1} = selected; %#ok<AGROW>
    end
end
end

function detected = hasSelection(selections, candidate)
detected = false;
candidate = sort(candidate);
for idx = 2:numel(selections)
    if isequal(sort(selections{idx}), candidate)
        detected = true;
        return;
    end
end
end

function bytes = totalSynopsisBytes( ...
        posteriors, crossResidual, stateDimension, activeThreshold)
bytes = 0;
for receiverIdx = reshape(find(any(crossResidual, 2)), 1, [])
    for senderIdx = reshape(find(crossResidual(receiverIdx, :)), 1, [])
        objects = activeObjects(posteriors{senderIdx}, activeThreshold);
        bytes = bytes + compactSynopsisBytes( ...
            numel(objects), stateDimension);
    end
end
end

function bytes = compactSynopsisBytes(labelCount, stateDimension)
positionDimension = min(2, stateDimension);
velocityDimension = stateDimension - positionDimension;
positionCovarianceScalars = ...
    positionDimension * (positionDimension + 1) / 2;
floatCount = 5 + stateDimension + ...
    positionCovarianceScalars + velocityDimension;
rawPerLabelBytes = 4 + 4 * floatCount + 2;
perLabelBytes = 4 * ceil(rawPerLabelBytes / 4);
bytes = 16 + labelCount * perLabelBytes;
end

function objects = activeObjects(objects, threshold)
if isempty(objects)
    return;
end
objects = objects([objects.r] > threshold & ...
    [objects.numberOfGmComponents] > 0);
end

function objects = removeLabel(objects, label)
keep = true(1, numel(objects));
for objectIdx = 1:numel(objects)
    keep(objectIdx) = ~( ...
        objects(objectIdx).birthTime == label(1) && ...
        objects(objectIdx).birthLocation == label(2));
end
objects = objects(keep);
end

function object = findObject(objects, label)
object = [];
for objectIdx = 1:numel(objects)
    if objects(objectIdx).numberOfGmComponents > 0 && ...
            objects(objectIdx).birthTime == label(1) && ...
            objects(objectIdx).birthLocation == label(2)
        object = objects(objectIdx);
        return;
    end
end
end

function [meanVector, covariance] = momentMatch(object, stateDimension)
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if numel(weights) ~= object.numberOfGmComponents || sum(weights) <= 0
    weights = ones(1, object.numberOfGmComponents);
end
weights = weights / sum(weights);
meanVector = zeros(stateDimension, 1);
for componentIdx = 1:numel(weights)
    meanVector = meanVector + ...
        weights(componentIdx) * object.mu{componentIdx};
end
covariance = zeros(stateDimension);
for componentIdx = 1:numel(weights)
    delta = object.mu{componentIdx} - meanVector;
    covariance = covariance + weights(componentIdx) * ...
        (object.Sigma{componentIdx} + delta * delta');
end
covariance = (covariance + covariance') / 2;
end

function bytes = objectLabelBytes(object, stateDimension)
scalarCount = 3 + object.numberOfGmComponents * ...
    (1 + stateDimension + stateDimension * stateDimension);
bytes = 8 * scalarCount;
end

function value = resolvePositionScale(model)
value = 100;
if isfield(model, 'ospaParameters') && ...
        isfield(model.ospaParameters, 'eC') && ...
        isscalar(model.ospaParameters.eC) && ...
        isfinite(model.ospaParameters.eC) && ...
        model.ospaParameters.eC > 0
    value = model.ospaParameters.eC;
end
end

function candidate = emptyCandidate()
candidate = struct( ...
    'index', 0, ...
    'receiverIdx', 0, ...
    'senderIdx', 0, ...
    'receiverFormationId', 0, ...
    'senderFormationId', 0, ...
    'label', zeros(2, 1), ...
    'receiverSupport', 0, ...
    'senderSupport', 0, ...
    'referenceExistence', 0, ...
    'candidateExistence', 0, ...
    'existenceGain', 0, ...
    'referenceAnchorDistance', 0, ...
    'candidateAnchorDistance', 0, ...
    'anchorGain', 0, ...
    'upwardDecisionCrossing', false, ...
    'existenceScore', 0, ...
    'spatialScore', 0, ...
    'combinedScore', 0, ...
    'labelPayloadBytes', 0);
end

function value = clamp01(value)
if ~isscalar(value) || ~isfinite(value)
    value = 0;
end
value = min(max(value, 0), 1);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
