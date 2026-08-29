function bank = buildOutputAlignedLabelEffectiveOracleActionBank(context, options)
% BUILDOUTPUTALIGNEDLABELEFFECTIVEORACLEACTIONBANK Bounded V150 actions.
%
% The bank is constructed from the opened current posterior only.  Each
% primitive action explicitly omits one complete Bernoulli mixture from one
% cross-formation sender-to-receiver KLA input.  Future tracking outcomes
% are deliberately not used here; the V150 wrapper uses them only to rank
% the resulting development-oracle actions after ordinary filter replay.

if nargin < 2 || isempty(options)
    options = struct();
end
if ~isstruct(context) || ~isscalar(context) || ...
        ~isfield(context, 'localPosteriorBySensor') || ...
        ~isfield(context, 'model')
    error('OutputAlignedLabelOracle:InvalidContext', ...
        'An opened observable LMB context is required.');
end

posteriors = context.localPosteriorBySensor;
nodeCount = numel(posteriors);
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
if numel(groupIds) ~= nodeCount
    error('OutputAlignedLabelOracle:InvalidFormationGroups', ...
        'Formation membership does not match the posterior count.');
end

maximumCandidates = round(getField(options, ...
    'maximumSingletonActions', 16));
activeThreshold = getField(options, ...
    'payloadExistenceThreshold', getField( ...
        buildMixtureAwareKlaReferenceConfig(), ...
        'payloadExistenceThreshold', 0));
if ~isscalar(maximumCandidates) || ~isfinite(maximumCandidates) || ...
        maximumCandidates < 1 || ...
        ~isscalar(activeThreshold) || ~isfinite(activeThreshold) || ...
        activeThreshold < 0 || activeThreshold >= 1
    error('OutputAlignedLabelOracle:InvalidOptions', ...
        'The bounded candidate options are invalid.');
end
maximumCandidates = max(1, maximumCandidates);

routeProtocol = getFormationIsolateReconnectProbeProtocol();
policyOptions = struct( ...
    'dominantWeight', routeProtocol.dominantWeight, ...
    'residualWeight', routeProtocol.residualWeight);
[referenceAdjacency, referenceDetails] = ...
    selectBackboneResidualSplicedCyclePolicy( ...
        context, 'fixed-counter-clockwise', policyOptions);
referenceWeights = referenceDetails.fusionWeightMatrix;
crossResidual = logical(referenceDetails.residualAdjacency) & ...
    (groupIds(:) ~= groupIds(:)');
if nnz(crossResidual) < 2
    error('OutputAlignedLabelOracle:MissingCrossResidual', ...
        'The registered reference lacks cross-formation residual edges.');
end

candidates = repmat(emptyCandidate(), 1, 0);
cursor = 0;
for receiverIdx = reshape(find(any(crossResidual, 2)), 1, [])
    senders = reshape(find(crossResidual(receiverIdx, :)), 1, []);
    for senderIdx = senders
        senderObjects = activeObjects( ...
            posteriors{senderIdx}, activeThreshold);
        for objectIdx = 1:numel(senderObjects)
            senderObject = senderObjects(objectIdx);
            label = [senderObject.birthTime; senderObject.birthLocation];
            receiverObject = findObject(posteriors{receiverIdx}, label);
            [score, details] = observableInfluenceScore( ...
                senderObject, receiverObject, ...
                referenceWeights(receiverIdx, senderIdx), ...
                context.model);
            cursor = cursor + 1;
            candidate = emptyCandidate();
            candidate.index = cursor;
            candidate.receiverIdx = receiverIdx;
            candidate.senderIdx = senderIdx;
            candidate.receiverFormationId = groupIds(receiverIdx);
            candidate.senderFormationId = groupIds(senderIdx);
            candidate.label = label;
            candidate.senderExistence = senderObject.r;
            candidate.receiverExistence = details.receiverExistence;
            candidate.senderSupport = details.senderSupport;
            candidate.receiverSupport = details.receiverSupport;
            candidate.sourceWeight = ...
                referenceWeights(receiverIdx, senderIdx);
            candidate.existenceDisagreement = ...
                details.existenceDisagreement;
            candidate.supportDisagreement = ...
                details.supportDisagreement;
            candidate.spatialDisagreement = ...
                details.spatialDisagreement;
            candidate.observableInfluenceScore = score;
            candidate.labelPayloadBytes = objectLabelBytes( ...
                senderObject, context.model.xDimension);
            candidates(cursor) = candidate; %#ok<AGROW>
        end
    end
end
if isempty(candidates)
    error('OutputAlignedLabelOracle:NoCandidateLabels', ...
        'No active sender labels exist on the cross-formation route.');
end

rankTable = [-reshape([candidates.observableInfluenceScore], [], 1), ...
    reshape([candidates.receiverIdx], [], 1), ...
    reshape([candidates.senderIdx], [], 1), ...
    reshape([candidates.index], [], 1)];
[~, rankOrder] = sortrows(rankTable, [1, 2, 3, 4]);
rankOrder = reshape(rankOrder( ...
    1:min(maximumCandidates, numel(rankOrder))), 1, []);
candidates = candidates(rankOrder);
for candidateIdx = 1:numel(candidates)
    candidates(candidateIdx).index = candidateIdx;
end

defaultSelections = [{zeros(1, 0)}, ...
    arrayfun(@(idx) idx, 1:numel(candidates), ...
        'UniformOutput', false)];
selections = getField(options, ...
    'actionCandidateSelections', defaultSelections);
selections = validateSelections(selections, numel(candidates));
defaultNames = cell(1, numel(selections));
defaultNames{1} = 'reference-full-payload';
for actionIdx = 2:numel(selections)
    selected = selections{actionIdx};
    if numel(selected) == 1
        candidate = candidates(selected);
        defaultNames{actionIdx} = sprintf( ...
            'omit-r%d-s%d-l%d-%d', ...
            candidate.receiverIdx, candidate.senderIdx, ...
            candidate.label(1), candidate.label(2));
    else
        defaultNames{actionIdx} = sprintf( ...
            'output-ranked-bundle-%d', numel(selected));
    end
end
actionNames = getField(options, 'actionNames', defaultNames);
if ~iscell(actionNames) || numel(actionNames) ~= numel(selections) || ...
        any(~cellfun(@(value) ischar(value) && ~isempty(value), actionNames))
    error('OutputAlignedLabelOracle:InvalidActionNames', ...
        'Action names must match the selected candidate bundles.');
end

actionCount = numel(selections);
dropPlans = cell(1, actionCount);
droppedBytes = zeros(1, actionCount);
controlBytes = zeros(1, actionCount);
objectives = zeros(1, actionCount);
for actionIdx = 1:actionCount
    dropPlan = cell(nodeCount);
    selected = selections{actionIdx};
    for candidateIdx = reshape(selected, 1, [])
        candidate = candidates(candidateIdx);
        labels = dropPlan{candidate.receiverIdx, candidate.senderIdx};
        labels(:, end + 1) = candidate.label; %#ok<AGROW>
        dropPlan{candidate.receiverIdx, candidate.senderIdx} = labels;
        droppedBytes(actionIdx) = droppedBytes(actionIdx) + ...
            candidate.labelPayloadBytes;
        objectives(actionIdx) = objectives(actionIdx) + ...
            candidate.observableInfluenceScore;
    end
    activeEdges = find(~cellfun(@isempty, dropPlan));
    for edgeIdx = reshape(activeEdges, 1, [])
        controlBytes(actionIdx) = controlBytes(actionIdx) + ...
            16 + 8 * size(dropPlan{edgeIdx}, 2);
    end
    dropPlans{actionIdx} = dropPlan;
end
withinPayload = droppedBytes + 1e-9 >= controlBytes;
withinPayload(1) = true;

bank = struct();
bank.contractVersion = ...
    'output-aligned-label-effective-oracle-action-bank-v150-v1';
bank.bankVariant = 'output-aligned-label-effective-oracle-v150';
bank.actionCount = actionCount;
bank.referenceActionIndex = 1;
bank.actionNames = actionNames;
bank.actionAdjacency = repmat( ...
    logical(referenceAdjacency), 1, 1, actionCount);
bank.actionFusionWeights = repmat( ...
    referenceWeights, 1, 1, actionCount);
bank.actionDropLabelsByReceiverSender = dropPlans;
bank.actionCandidateIndices = selections;
bank.actionPosteriorObjective = objectives;
bank.actionPosteriorProxyAllowed = withinPayload;
bank.actionWithinReferencePayload = withinPayload;
bank.actionDroppedPayloadBytes = droppedBytes;
bank.actionControlSynopsisBytes = controlBytes;
bank.actionPredictedNetSavingBytes = droppedBytes - controlBytes;
bank.candidates = candidates;
bank.referenceAdjacency = logical(referenceAdjacency);
bank.referenceFusionWeights = referenceWeights;
bank.referencePolicyDetails = referenceDetails;
bank.crossResidualAdjacency = crossResidual;
bank.positiveSupportThreshold = getField(options, ...
    'positiveSupportThreshold', 0.20);
bank.payloadExistenceThreshold = activeThreshold;
bank.controlSynopsisBytes = max(controlBytes);
bank.truthUsed = false;
bank.futureMeasurementsUsed = false;
bank.futureOutcomesUsed = false;
bank.outputAlignedScoringDeferred = true;
end

function selections = validateSelections(selections, candidateCount)
if ~iscell(selections) || isempty(selections) || ...
        ~isempty(selections{1})
    error('OutputAlignedLabelOracle:InvalidSelections', ...
        'The first action must be the empty reference selection.');
end
for actionIdx = 1:numel(selections)
    selected = reshape(selections{actionIdx}, 1, []);
    if any(~isfinite(selected)) || any(selected ~= round(selected)) || ...
            any(selected < 1) || any(selected > candidateCount) || ...
            numel(unique(selected)) ~= numel(selected)
        error('OutputAlignedLabelOracle:InvalidSelections', ...
            'Every action must contain unique valid candidate indices.');
    end
    selections{actionIdx} = selected;
end
end

function [score, details] = observableInfluenceScore( ...
        senderObject, receiverObject, sourceWeight, model)
senderExistence = clamp01(senderObject.r);
senderSupport = clamp01(getField( ...
    senderObject, 'detectionAssociationMass', 0));
receiverExistence = 0;
receiverSupport = 0;
spatialDisagreement = 0;
if ~isempty(receiverObject)
    receiverExistence = clamp01(receiverObject.r);
    receiverSupport = clamp01(getField( ...
        receiverObject, 'detectionAssociationMass', 0));
    [senderMean, ~] = momentMatch(senderObject, model.xDimension);
    [receiverMean, ~] = momentMatch(receiverObject, model.xDimension);
    positionDimension = min(2, model.xDimension);
    spatialDisagreement = min(norm( ...
        senderMean(1:positionDimension) - ...
        receiverMean(1:positionDimension)) / ...
        resolvePositionScale(model), 2);
end
existenceDisagreement = abs(senderExistence - receiverExistence);
supportDisagreement = abs(senderSupport - receiverSupport);
score = max(sourceWeight, 0) * max(senderExistence, 0.05) * ...
    (0.25 + existenceDisagreement + ...
     0.50 * supportDisagreement + 0.25 * spatialDisagreement);
details = struct( ...
    'receiverExistence', receiverExistence, ...
    'senderSupport', senderSupport, ...
    'receiverSupport', receiverSupport, ...
    'existenceDisagreement', existenceDisagreement, ...
    'supportDisagreement', supportDisagreement, ...
    'spatialDisagreement', spatialDisagreement);
end

function objects = activeObjects(objects, threshold)
if isempty(objects)
    return;
end
objects = objects([objects.r] > threshold & ...
    [objects.numberOfGmComponents] > 0);
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
    'senderExistence', 0, ...
    'receiverExistence', 0, ...
    'senderSupport', 0, ...
    'receiverSupport', 0, ...
    'sourceWeight', 0, ...
    'existenceDisagreement', 0, ...
    'supportDisagreement', 0, ...
    'spatialDisagreement', 0, ...
    'observableInfluenceScore', 0, ...
    'labelPayloadBytes', 0);
end

function value = clamp01(value)
if ~isscalar(value) || ~isfinite(value)
    value = 0;
else
    value = min(max(value, 0), 1);
end
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
