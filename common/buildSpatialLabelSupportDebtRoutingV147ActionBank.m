function bank = buildSpatialLabelSupportDebtRoutingV147ActionBank( ...
        context, referenceAdjacency, referenceWeights, options)
% BUILDSPATIALLABELSUPPORTDEBTROUTINGV147ACTIONBANK Current-only graph bank.

if nargin < 4 || isempty(options)
    options = struct();
end
protocol = getSpatialLabelSupportDebtRoutingV147Protocol();
required = {'localPosteriorBySensor', 'physicalAdjacency', ...
    'model', 'commConfig', 'currentTime'};
if ~isstruct(context) || ~isscalar(context) || ...
        ~all(isfield(context, required))
    error('SpatialDebtV147:InvalidContext', ...
        'V147 requires one current observable posterior context.');
end
posteriors = reshape(context.localPosteriorBySensor, 1, []);
nodeCount = numel(posteriors);
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
formationIds = unique(groupIds, 'stable');
formationCount = numel(formationIds);
referenceAdjacency = logical(referenceAdjacency);
physicalAdjacency = logical(context.physicalAdjacency);
if numel(groupIds) ~= nodeCount || ...
        ~isequal(size(referenceAdjacency), [nodeCount, nodeCount]) || ...
        ~isequal(size(referenceWeights), [nodeCount, nodeCount]) || ...
        ~isequal(size(physicalAdjacency), [nodeCount, nodeCount]) || ...
        any(referenceAdjacency(:) & ~physicalAdjacency(:)) || ...
        ~validWeights(referenceAdjacency, referenceWeights)
    error('SpatialDebtV147:InvalidReference', ...
        'The V147 reference graph or weights are invalid.');
end

senderPayloadBytes = estimateSenderPayloadBytes(posteriors, context.model);
referencePayloadBytes = topologyPayloadBytes( ...
    referenceAdjacency, senderPayloadBytes);
positionScale = resolvePositionScale(context.model);
fusionConfig = buildMixtureAwareKlaReferenceConfig(struct( ...
    'missingLabelFusionMode', 'fov-aware-censored', ...
    'missingNeighborWeightMode', ...
        protocol.primaryMissingNeighborWeightMode, ...
    'fusionWeightMode', protocol.nominalKlaWeightMode, ...
    'useStaleNeighborCache', false));
fusionContext = context;
fusionContext.model = materializeObservableSensorFusionModel( ...
    context.model, context.currentTime);

dominantCandidates = repmat(emptyCandidate(), 1, 0);
residualCandidates = repmat(emptyCandidate(), 1, 0);
screenedDominantCandidates = repmat(emptyCandidate(), 1, 0);
screenedResidualCandidates = repmat(emptyCandidate(), 1, 0);
for receiverIdx = 1:nodeCount
    incoming = reshape(find(referenceAdjacency(receiverIdx, :)), 1, []);
    dominant = incoming(abs(referenceWeights(receiverIdx, incoming) - ...
        protocol.routeWeights.dominantWeight) <= 1e-12);
    residual = incoming(abs(referenceWeights(receiverIdx, incoming) - ...
        protocol.routeWeights.residualWeight) <= 1e-12);
    if numel(dominant) ~= 1 || numel(residual) ~= 1
        continue;
    end
    dominant = dominant(1);
    residual = residual(1);
    alternatives = reshape(find( ...
        physicalAdjacency(receiverIdx, :) & ...
        ~referenceAdjacency(receiverIdx, :) & ...
        groupIds ~= groupIds(receiverIdx)), 1, []);
    alternatives(alternatives == receiverIdx) = [];
    if isempty(alternatives)
        continue;
    end
    rawDebt = zeros(1, numel(alternatives));
    for alternativeIdx = 1:numel(alternatives)
        senderIdx = alternatives(alternativeIdx);
        rawDebt(alternativeIdx) = candidateRawDebt( ...
            posteriors{receiverIdx}, posteriors{senderIdx}, ...
            context.model, positionScale, protocol) * ...
            linkReliability(context.commConfig, senderIdx, ...
                receiverIdx, context.currentTime);
    end
    eligible = find(rawDebt > protocol.minimumRawDebt);
    if isempty(eligible)
        continue;
    end
    ranking = [-rawDebt(eligible)', alternatives(eligible)'];
    [~, order] = sortrows(ranking, [1, 2]);
    eligible = eligible(order(1:min( ...
        protocol.maximumSourcesPerReceiver, numel(order))));
    referenceFused = fuseReceiverRow( ...
        posteriors, receiverIdx, incoming, ...
        referenceWeights(receiverIdx, incoming), ...
        fusionContext, fusionConfig);
    for alternativeIdx = reshape(eligible, 1, [])
        senderIdx = alternatives(alternativeIdx);
        dominantCandidate = evaluateCandidate( ...
            'dominant-promotion', receiverIdx, dominant, residual, ...
            senderIdx, referenceFused, posteriors, fusionContext, ...
            fusionConfig, positionScale, senderPayloadBytes, protocol);
        screenedDominantCandidates(end + 1) = ...
            dominantCandidate; %#ok<AGROW>
        if dominantCandidate.safe
            dominantCandidates(end + 1) = ...
                dominantCandidate; %#ok<AGROW>
        end
        residualCandidate = evaluateCandidate( ...
            'residual-replacement', receiverIdx, dominant, residual, ...
            senderIdx, referenceFused, posteriors, fusionContext, ...
            fusionConfig, positionScale, senderPayloadBytes, protocol);
        screenedResidualCandidates(end + 1) = ...
            residualCandidate; %#ok<AGROW>
        if residualCandidate.safe
            residualCandidates(end + 1) = ...
                residualCandidate; %#ok<AGROW>
        end
    end
end

dominantCandidates = rankAndLimitByFormation( ...
    dominantCandidates, formationIds, ...
    protocol.maximumCandidatesPerFormation);
residualCandidates = rankAndLimitByFormation( ...
    residualCandidates, formationIds, ...
    protocol.maximumCandidatesPerFormation);
dominantSolutions = solveCoverageModes( ...
    dominantCandidates, formationIds, referenceAdjacency, ...
    referenceWeights, physicalAdjacency, senderPayloadBytes, ...
    referencePayloadBytes, protocol);
residualSolutions = solveCoverageModes( ...
    residualCandidates, formationIds, referenceAdjacency, ...
    referenceWeights, physicalAdjacency, senderPayloadBytes, ...
    referencePayloadBytes, protocol);

patterns = protocol.actionPatterns;
actions = repmat(emptyAction(nodeCount), 1, numel(patterns));
for actionIdx = 1:numel(patterns)
    pattern = patterns(actionIdx);
    action = emptyAction(nodeCount);
    action.index = actionIdx;
    action.name = pattern.name;
    action.roleMode = pattern.roleMode;
    action.coverageMode = pattern.coverageMode;
    action.activeOffsets = reshape(pattern.activeOffsets, 1, []);
    action.interventionEndOffset = 0;
    action.adjacency = referenceAdjacency;
    action.fusionWeights = referenceWeights;
    action.referencePayloadBytes = referencePayloadBytes;
    action.candidatePayloadBytes = referencePayloadBytes;
    action.initialByteDeltaFraction = 0;
    action.instantStrongConnected = ...
        isStronglyConnected(referenceAdjacency);
    if strcmp(pattern.roleMode, 'reference')
        action.gateEligible = true;
        action.failureReason = '';
    else
        if strcmp(pattern.roleMode, 'dominant-promotion')
            solution = dominantSolutions.(pattern.coverageMode);
        else
            solution = residualSolutions.(pattern.coverageMode);
        end
        action = attachSolution(action, solution);
        if ~isempty(action.activeOffsets)
            action.interventionEndOffset = max(action.activeOffsets);
        end
    end
    actions(actionIdx) = action;
end

bank = struct();
bank.contractVersion = ...
    'v147-spatial-label-support-debt-routing-action-bank-v1';
bank.protocolId = protocol.id;
bank.nodeCount = nodeCount;
bank.formationCount = formationCount;
bank.groupIds = groupIds;
bank.referenceActionIndex = 1;
bank.actionCount = numel(actions);
bank.actions = actions;
bank.referenceAdjacency = referenceAdjacency;
bank.referenceFusionWeights = referenceWeights;
bank.referencePayloadBytes = referencePayloadBytes;
bank.senderPayloadBytes = senderPayloadBytes;
bank.dominantCandidates = dominantCandidates;
bank.residualCandidates = residualCandidates;
bank.candidateScreen = struct( ...
    'dominant', summarizeScreen(screenedDominantCandidates, protocol), ...
    'residual', summarizeScreen(screenedResidualCandidates, protocol));
bank.dominantSolutions = dominantSolutions;
bank.residualSolutions = residualSolutions;
bank.truthUsed = false;
bank.futureMeasurementUsed = false;
bank.futureOutcomeUsed = false;
bank.validationClaimAllowed = false;
bank.developmentEvidenceOnly = true;
end

function summary = summarizeScreen(candidates, protocol)
summary = struct( ...
    'evaluatedCount', numel(candidates), ...
    'positiveNetGainCount', 0, ...
    'noDownwardCrossingCount', 0, ...
    'harmSafeCount', 0, ...
    'safeCount', 0, ...
    'maximumNetGain', -inf, ...
    'maximumNoDownwardNetGain', -inf);
if isempty(candidates)
    return;
end
net = [candidates.netGain];
downward = [candidates.downwardCrossingCount];
harmSafe = [candidates.protectedHarm] <= ...
    protocol.maximumProtectedHarmToGainRatio * ...
        [candidates.supportGain] + 1e-12;
summary.positiveNetGainCount = nnz( ...
    net > protocol.minimumVirtualNetGain);
summary.noDownwardCrossingCount = nnz(downward == 0);
summary.harmSafeCount = nnz(harmSafe);
summary.safeCount = nnz([candidates.safe]);
summary.maximumNetGain = max(net);
if any(downward == 0)
    summary.maximumNoDownwardNetGain = max(net(downward == 0));
end
end

function candidate = evaluateCandidate( ...
        roleMode, receiverIdx, dominantIdx, residualIdx, senderIdx, ...
        referenceFused, posteriors, context, fusionConfig, ...
        positionScale, senderPayloadBytes, protocol)
incoming = [dominantIdx, residualIdx];
candidateSenders = incoming;
candidateWeights = [protocol.routeWeights.dominantWeight, ...
    protocol.routeWeights.residualWeight];
if strcmp(roleMode, 'dominant-promotion')
    candidateSenders = [dominantIdx, senderIdx];
    candidateWeights = [protocol.routeWeights.residualWeight, ...
        protocol.routeWeights.dominantWeight];
elseif strcmp(roleMode, 'residual-replacement')
    candidateSenders = [dominantIdx, senderIdx];
else
    error('SpatialDebtV147:UnknownRoleMode', ...
        'Unknown V147 row role mode.');
end
candidateFused = fuseReceiverRow( ...
    posteriors, receiverIdx, candidateSenders, candidateWeights, ...
    context, fusionConfig);
[gain, harm, downward, spatialGain, existenceGain] = ...
    compareFusedSupport( ...
        referenceFused, candidateFused, ...
        posteriors{receiverIdx}, posteriors{dominantIdx}, ...
        posteriors{residualIdx}, posteriors{senderIdx}, ...
        context.model, positionScale, protocol);
netGain = gain - harm;
candidate = emptyCandidate();
candidate.roleMode = roleMode;
candidate.receiverIdx = receiverIdx;
candidate.receiverFormationId = context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds(receiverIdx);
candidate.dominantSenderIdx = dominantIdx;
candidate.residualSenderIdx = residualIdx;
candidate.candidateSenderIdx = senderIdx;
candidate.candidateSenderFormationId = ...
    context.model.dynamicTopologyScenario.config.sensorGroupIds(senderIdx);
candidate.linkReliability = linkReliability( ...
    context.commConfig, senderIdx, receiverIdx, context.currentTime);
candidate.supportGain = gain;
candidate.protectedHarm = harm;
candidate.netGain = netGain;
candidate.spatialGain = spatialGain;
candidate.existenceGain = existenceGain;
candidate.downwardCrossingCount = downward;
candidate.byteDelta = ...
    senderPayloadBytes(senderIdx) - senderPayloadBytes(residualIdx);
candidate.safe = netGain > protocol.minimumVirtualNetGain && ...
    downward == 0 && ...
    harm <= protocol.maximumProtectedHarmToGainRatio * gain + 1e-12;
end

function [gain, harm, downward, spatialGain, existenceGain] = ...
        compareFusedSupport(reference, candidate, receiver, dominant, ...
        residual, source, model, positionScale, protocol)
labels = collectLabels({ ...
    reference, candidate, receiver, dominant, residual, source});
gain = 0;
harm = 0;
downward = 0;
spatialGain = 0;
existenceGain = 0;
for labelIdx = 1:size(labels, 2)
    label = labels(:, labelIdx);
    sourceObject = objectByLabel(source, label);
    referenceObject = objectByLabel(reference, label);
    candidateObject = objectByLabel(candidate, label);
    sourceStrength = supportStrength(sourceObject, model, protocol);
    if sourceStrength > 0
        referenceAlignment = spatialAlignment( ...
            referenceObject, sourceObject, model, positionScale);
        candidateAlignment = spatialAlignment( ...
            candidateObject, sourceObject, model, positionScale);
        localSpatialGain = sourceStrength * ...
            max(candidateAlignment - referenceAlignment, 0);
        localExistenceGain = sourceStrength * max( ...
            objectExistence(candidateObject) - ...
            objectExistence(referenceObject), 0);
        spatialGain = spatialGain + localSpatialGain;
        existenceGain = existenceGain + localExistenceGain;
        gain = gain + localSpatialGain + localExistenceGain;
    end
    anchors = { ...
        objectByLabel(receiver, label), ...
        objectByLabel(dominant, label), ...
        objectByLabel(residual, label)};
    [protectedStrength, protectedAnchor, protectedAssociation] = ...
        strongestAnchor(anchors, model, protocol);
    if protectedStrength > 0
        referenceProtection = spatialAlignment( ...
            referenceObject, protectedAnchor, model, positionScale);
        candidateProtection = spatialAlignment( ...
            candidateObject, protectedAnchor, model, positionScale);
        existenceProtection = max( ...
            objectExistence(referenceObject) - ...
            objectExistence(candidateObject), 0);
        harm = harm + protectedStrength * ( ...
            max(referenceProtection - candidateProtection, 0) + ...
            existenceProtection);
        downward = downward + double( ...
            protectedAssociation >= ...
                protocol.positiveAssociationThreshold - 1e-12 && ...
            objectExistence(referenceObject) >= ...
                protocol.decisionExistenceThreshold - 1e-12 && ...
            objectExistence(candidateObject) < ...
                protocol.decisionExistenceThreshold - 1e-12);
    end
end
end

function debt = candidateRawDebt(receiver, source, model, ...
        positionScale, protocol)
labels = collectLabels({receiver, source});
debt = 0;
for labelIdx = 1:size(labels, 2)
    label = labels(:, labelIdx);
    sourceObject = objectByLabel(source, label);
    receiverObject = objectByLabel(receiver, label);
    sourceStrength = supportStrength(sourceObject, model, protocol);
    if sourceStrength <= 0
        continue;
    end
    receiverStrength = supportStrength(receiverObject, model, protocol);
    existenceDebt = max(sourceStrength - receiverStrength, 0);
    spatialDebt = sourceStrength * objectExistence(receiverObject) * ...
        (1 - spatialAlignment( ...
            receiverObject, sourceObject, model, positionScale));
    debt = debt + existenceDebt + spatialDebt;
end
end

function fused = fuseReceiverRow(posteriors, receiverIdx, senders, ...
        neighborWeights, context, fusionConfig)
senders = reshape(senders, 1, []);
neighborWeights = reshape(neighborWeights, 1, []);
sources = [receiverIdx, senders];
selfWeight = 1 - sum(neighborWeights);
weights = [selfWeight, neighborWeights];
for sourceCursor = 2:numel(sources)
    senderIdx = sources(sourceCursor);
    weights(sourceCursor) = weights(sourceCursor) * ...
        linkReliability(context.commConfig, senderIdx, ...
            receiverIdx, context.currentTime);
end
positive = weights > eps;
sources = sources(positive);
weights = weights(positive);
weights = weights / sum(weights);
fusionDetails = struct( ...
    'eventType', [0, 2 * ones(1, numel(sources) - 1)], ...
    'sourceIndices', sources, ...
    'isStale', false(1, numel(sources)), ...
    'isSelf', sources == receiverIdx, ...
    'currentTime', context.currentTime);
fused = fuseLmbPosteriorsByLabel( ...
    posteriors(sources), weights, context.model, weights, ...
    fusionDetails, fusionConfig);
end

function candidates = rankAndLimitByFormation( ...
        candidates, formationIds, maximumCount)
if isempty(candidates)
    return;
end
kept = zeros(1, 0);
for formationId = formationIds
    indices = find([candidates.receiverFormationId] == formationId);
    if isempty(indices)
        continue;
    end
    ranking = [-[candidates(indices).netGain]', ...
        [candidates(indices).byteDelta]', ...
        [candidates(indices).receiverIdx]', ...
        [candidates(indices).candidateSenderIdx]'];
    [~, order] = sortrows(ranking, [1, 2, 3, 4]);
    kept = [kept, indices(order(1:min(maximumCount, ...
        numel(order))))]; %#ok<AGROW>
end
candidates = candidates(kept);
for candidateIdx = 1:numel(candidates)
    candidates(candidateIdx).index = candidateIdx;
end
end

function solutions = solveCoverageModes( ...
        candidates, formationIds, referenceAdjacency, ...
        referenceWeights, physicalAdjacency, senderPayloadBytes, ...
        referencePayloadBytes, protocol)
solutions = struct( ...
    'single', emptySolution(referenceAdjacency, referenceWeights), ...
    'half', emptySolution(referenceAdjacency, referenceWeights), ...
    'maximum', emptySolution(referenceAdjacency, referenceWeights));
if isempty(candidates)
    solutions.single.failureReason = 'no-safe-row-candidates';
    solutions.half.failureReason = 'no-safe-row-candidates';
    solutions.maximum.failureReason = 'no-safe-row-candidates';
    return;
end
optionsByFormation = cell(1, numel(formationIds));
for formationIdx = 1:numel(formationIds)
    optionsByFormation{formationIdx} = find( ...
        [candidates.receiverFormationId] == formationIds(formationIdx));
end
radix = cellfun(@numel, optionsByFormation) + 1;
combinationCount = prod(radix);
halfCount = ceil(numel(formationIds) / 2);
for code = 0:(combinationCount - 1)
    residualCode = code;
    selected = zeros(1, 0);
    for formationIdx = 1:numel(formationIds)
        digit = mod(residualCode, radix(formationIdx));
        residualCode = floor(residualCode / radix(formationIdx));
        if digit > 0
            selected(end + 1) = ...
                optionsByFormation{formationIdx}(digit); %#ok<AGROW>
        end
    end
    selectedCount = numel(selected);
    if selectedCount == 0
        continue;
    end
    [adjacency, weights] = applyCandidates( ...
        candidates(selected), referenceAdjacency, referenceWeights, ...
        protocol);
    payloadBytes = topologyPayloadBytes(adjacency, senderPayloadBytes);
    byteDeltaFraction = (payloadBytes - referencePayloadBytes) / ...
        max(referencePayloadBytes, eps);
    feasible = ~any(adjacency(:) & ~physicalAdjacency(:)) && ...
        nnz(adjacency) == nnz(referenceAdjacency) && ...
        validWeights(adjacency, weights) && ...
        (~protocol.requireInstantStrongConnectivity || ...
         isStronglyConnected(adjacency)) && ...
        byteDeltaFraction <= ...
            protocol.maximumInitialByteDeltaFraction + 1e-12;
    if ~feasible
        continue;
    end
    solution = emptySolution(referenceAdjacency, referenceWeights);
    solution.feasible = true;
    solution.failureReason = '';
    solution.selectedCandidateIndices = selected;
    solution.selectedFormationIds = unique( ...
        [candidates(selected).receiverFormationId], 'stable');
    solution.adjacency = adjacency;
    solution.fusionWeights = weights;
    solution.objective = sum([candidates(selected).netGain]);
    solution.supportGain = sum([candidates(selected).supportGain]);
    solution.protectedHarm = ...
        sum([candidates(selected).protectedHarm]);
    solution.referencePayloadBytes = referencePayloadBytes;
    solution.candidatePayloadBytes = payloadBytes;
    solution.initialByteDeltaFraction = byteDeltaFraction;
    solution.instantStrongConnected = ...
        isStronglyConnected(adjacency);
    if selectedCount == 1
        solutions.single = chooseHigherObjective( ...
            solutions.single, solution);
    end
    if selectedCount == halfCount
        solutions.half = chooseHigherObjective( ...
            solutions.half, solution);
    end
    solutions.maximum = chooseMaximumCoverage( ...
        solutions.maximum, solution);
end
solutions.single = attachFailureReason( ...
    solutions.single, 'no-byte-safe-single-formation-solution');
solutions.half = attachFailureReason( ...
    solutions.half, 'no-byte-safe-half-formation-solution');
solutions.maximum = attachFailureReason( ...
    solutions.maximum, 'no-byte-safe-maximum-coverage-solution');
end

function [adjacency, weights] = applyCandidates( ...
        candidates, referenceAdjacency, referenceWeights, protocol)
adjacency = referenceAdjacency;
weights = referenceWeights;
for candidate = candidates
    receiver = candidate.receiverIdx;
    residual = candidate.residualSenderIdx;
    dominant = candidate.dominantSenderIdx;
    source = candidate.candidateSenderIdx;
    adjacency(receiver, residual) = false;
    weights(receiver, residual) = 0;
    adjacency(receiver, source) = true;
    if strcmp(candidate.roleMode, 'dominant-promotion')
        weights(receiver, dominant) = ...
            protocol.routeWeights.residualWeight;
        weights(receiver, source) = ...
            protocol.routeWeights.dominantWeight;
    else
        weights(receiver, source) = ...
            protocol.routeWeights.residualWeight;
    end
end
end

function action = attachSolution(action, solution)
action.gateEligible = solution.feasible;
action.failureReason = solution.failureReason;
action.selectedCandidateIndices = ...
    solution.selectedCandidateIndices;
action.selectedFormationIds = solution.selectedFormationIds;
action.adjacency = solution.adjacency;
action.fusionWeights = solution.fusionWeights;
action.objective = solution.objective;
action.supportGain = solution.supportGain;
action.protectedHarm = solution.protectedHarm;
action.referencePayloadBytes = solution.referencePayloadBytes;
action.candidatePayloadBytes = solution.candidatePayloadBytes;
action.initialByteDeltaFraction = ...
    solution.initialByteDeltaFraction;
action.instantStrongConnected = ...
    solution.instantStrongConnected;
end

function output = chooseHigherObjective(current, candidate)
output = current;
if ~current.feasible || candidate.objective > current.objective + 1e-12
    output = candidate;
end
end

function output = chooseMaximumCoverage(current, candidate)
output = current;
currentCount = numel(current.selectedFormationIds);
candidateCount = numel(candidate.selectedFormationIds);
if ~current.feasible || candidateCount > currentCount || ...
        (candidateCount == currentCount && ...
         candidate.objective > current.objective + 1e-12)
    output = candidate;
end
end

function solution = attachFailureReason(solution, reason)
if ~solution.feasible
    solution.failureReason = reason;
end
end

function [strength, anchor, association] = ...
        strongestAnchor(objects, model, protocol)
strength = 0;
association = 0;
anchor = [];
for objectIdx = 1:numel(objects)
    object = objects{objectIdx};
    candidateStrength = supportStrength(object, model, protocol);
    if candidateStrength > strength
        strength = candidateStrength;
        association = objectAssociation(object);
        anchor = object;
    end
end
end

function strength = supportStrength(object, model, protocol)
strength = 0;
if isempty(object) || object.numberOfGmComponents <= 0 || ...
        object.r < protocol.sourceExistenceThreshold - 1e-12
    return;
end
association = objectAssociation(object);
[~, covariance] = momentMatch(object, model.xDimension);
positionDimension = min(2, model.xDimension);
positionScale = resolvePositionScale(model);
uncertainty = trace(covariance(1:positionDimension, ...
    1:positionDimension)) / max(positionScale ^ 2, eps);
spatialCredibility = 0.25 / (1 + max(uncertainty, 0));
strength = clamp01(object.r) * max(association, spatialCredibility);
end

function value = spatialAlignment(object, anchor, model, positionScale)
value = 0;
if isempty(object) || isempty(anchor) || ...
        object.numberOfGmComponents <= 0 || ...
        anchor.numberOfGmComponents <= 0
    return;
end
[objectMean, ~] = momentMatch(object, model.xDimension);
[anchorMean, ~] = momentMatch(anchor, model.xDimension);
positionDimension = min(2, model.xDimension);
distance = norm(objectMean(1:positionDimension) - ...
    anchorMean(1:positionDimension)) / max(positionScale, eps);
value = clamp01(object.r) * exp(-0.5 * distance ^ 2);
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

function labels = collectLabels(posteriors)
labels = zeros(2, 0);
for posteriorIdx = 1:numel(posteriors)
    objects = posteriors{posteriorIdx};
    for objectIdx = 1:numel(objects)
        if objects(objectIdx).numberOfGmComponents <= 0
            continue;
        end
        label = [objects(objectIdx).birthTime; ...
            objects(objectIdx).birthLocation];
        if isempty(labels) || ...
                ~any(all(bsxfun(@eq, labels, label), 1))
            labels(:, end + 1) = label; %#ok<AGROW>
        end
    end
end
end

function object = objectByLabel(objects, label)
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

function value = objectExistence(object)
value = 0;
if ~isempty(object)
    value = clamp01(object.r);
end
end

function value = objectAssociation(object)
value = 0;
if ~isempty(object) && isfield(object, 'detectionAssociationMass')
    value = clamp01(object.detectionAssociationMass);
end
end

function bytes = estimateSenderPayloadBytes(posteriors, model)
bytes = zeros(1, numel(posteriors));
for sensorIdx = 1:numel(posteriors)
    stats = estimateLmbPayloadSize(posteriors{sensorIdx}, model, 2, struct());
    bytes(sensorIdx) = stats.estimatedBytes;
end
end

function bytes = topologyPayloadBytes(adjacency, senderPayloadBytes)
bytes = sum(sum(logical(adjacency), 1) .* senderPayloadBytes);
end

function probability = linkReliability( ...
        config, senderIdx, receiverIdx, currentTime)
drop = 0;
if isfield(config, 'pDropByEdge') && ~isempty(config.pDropByEdge)
    if ndims(config.pDropByEdge) >= 3
        timeIdx = min(currentTime, size(config.pDropByEdge, 3));
        drop = config.pDropByEdge(senderIdx, receiverIdx, timeIdx);
    else
        drop = config.pDropByEdge(senderIdx, receiverIdx);
    end
elseif isfield(config, 'pDropBySensor') && ...
        numel(config.pDropBySensor) >= senderIdx
    drop = config.pDropBySensor(senderIdx);
elseif isfield(config, 'defaultDropProbability')
    drop = config.defaultDropProbability;
end
probability = 1 - clamp01(drop);
end

function valid = validWeights(adjacency, weights)
support = logical(adjacency) | logical(eye(size(adjacency, 1)));
valid = all(isfinite(weights(:))) && ...
    all(weights(:) >= -1e-12) && ...
    ~any(weights(:) > 1e-12 & ~support(:)) && ...
    all(abs(sum(weights, 2) - 1) <= 1e-12);
end

function connected = isStronglyConnected(adjacency)
senderAdjacency = logical(adjacency');
connected = reachesAll(senderAdjacency) && reachesAll(senderAdjacency');
end

function connected = reachesAll(adjacency)
visited = false(1, size(adjacency, 1));
frontier = 1;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if visited(node)
        continue;
    end
    visited(node) = true;
    frontier = [frontier, reshape(find( ...
        adjacency(node, :) & ~visited), 1, [])]; %#ok<AGROW>
end
connected = all(visited);
end

function value = resolvePositionScale(model)
value = 100;
if isfield(model, 'ospaParameters') && ...
        isfield(model.ospaParameters, 'eC') && ...
        isfinite(model.ospaParameters.eC) && model.ospaParameters.eC > 0
    value = model.ospaParameters.eC;
end
end

function value = clamp01(value)
if ~isscalar(value) || ~isfinite(value)
    value = 0;
end
value = min(max(value, 0), 1);
end

function candidate = emptyCandidate()
candidate = struct( ...
    'index', 0, 'roleMode', '', ...
    'receiverIdx', 0, 'receiverFormationId', 0, ...
    'dominantSenderIdx', 0, 'residualSenderIdx', 0, ...
    'candidateSenderIdx', 0, 'candidateSenderFormationId', 0, ...
    'linkReliability', 0, 'supportGain', 0, ...
    'protectedHarm', 0, 'netGain', 0, ...
    'spatialGain', 0, 'existenceGain', 0, ...
    'downwardCrossingCount', 0, 'byteDelta', 0, 'safe', false);
end

function solution = emptySolution(adjacency, weights)
solution = struct( ...
    'feasible', false, 'failureReason', '', ...
    'selectedCandidateIndices', zeros(1, 0), ...
    'selectedFormationIds', zeros(1, 0), ...
    'adjacency', logical(adjacency), 'fusionWeights', weights, ...
    'objective', -inf, 'supportGain', 0, 'protectedHarm', 0, ...
    'referencePayloadBytes', NaN, 'candidatePayloadBytes', NaN, ...
    'initialByteDeltaFraction', NaN, ...
    'instantStrongConnected', false);
end

function action = emptyAction(nodeCount)
action = struct( ...
    'index', 0, 'name', '', 'roleMode', '', 'coverageMode', '', ...
    'activeOffsets', zeros(1, 0), 'interventionEndOffset', 0, ...
    'selectedCandidateIndices', zeros(1, 0), ...
    'selectedFormationIds', zeros(1, 0), ...
    'adjacency', false(nodeCount), ...
    'fusionWeights', zeros(nodeCount), ...
    'objective', 0, 'supportGain', 0, 'protectedHarm', 0, ...
    'referencePayloadBytes', NaN, 'candidatePayloadBytes', NaN, ...
    'initialByteDeltaFraction', NaN, ...
    'instantStrongConnected', false, ...
    'gateEligible', false, 'failureReason', 'uninitialized');
end
