function [adjacency, details] = selectD12TopologyPolicy(context, mode)
% SELECTD12TOPOLOGYPOLICY Enumerated D12 analytic and oracle policies.
%
% mode:
%   reliability       - current link reliability and spectral tie-break
%   discrepancy       - deployable local-posterior discrepancy heuristic
%   oracle-consensus  - exact one-round fusion, minimum network disagreement
%   oracle-truth      - same oracle with an offline truth-labelled surrogate

if nargin < 2 || isempty(mode)
    mode = 'discrepancy';
end
mode = lower(char(mode));
ticId = tic;

scenario = context.model.dynamicTopologyScenario;
candidates = scenario.candidateAdjacency;
candidateCount = size(candidates, 3);
valid = false(1, candidateCount);
objectives = inf(1, candidateCount);
consensusTerms = nan(1, candidateCount);
truthTerms = nan(1, candidateCount);
switchTerms = nan(1, candidateCount);
byteMismatchTerms = nan(1, candidateCount);
fusionCache = initializeFusionCache(size(candidates, 1));
fusionCacheEnabled = getField( ...
    context.triggerConfig, 'topologyOracleFusionCacheEnabled', true);
nodePayloadBytes = estimateNodePayloadBytes( ...
    context.localPosteriorBySensor, context.model);
referenceBytes = topologyAttemptedBytes( ...
    scenario.staticAdjacency, nodePayloadBytes);
byteTolerance = getField( ...
    scenario.config, 'attemptedByteToleranceFraction', 0.02);

maxReplacements = scenario.config.maxEdgeReplacementsPerStep;
for candidateIdx = 1:candidateCount
    candidate = candidates(:, :, candidateIdx);
    if any(candidate(:) & ~context.physicalAdjacency(:)) || ...
            nnz(triu(candidate, 1)) > context.edgeBudget
        continue;
    end
    switchCost = countRemovedEdges( ...
        context.previousAdjacency, candidate);
    if any(context.previousAdjacency(:)) && ...
            switchCost > maxReplacements
        continue;
    end
    candidateBytes = topologyAttemptedBytes(candidate, nodePayloadBytes);
    byteMismatch = abs(candidateBytes - referenceBytes) / ...
        max(referenceBytes, 1);
    byteMismatchTerms(candidateIdx) = byteMismatch;
    if byteMismatch > byteTolerance + 1e-12
        continue;
    end
    valid(candidateIdx) = true;
    switchTerms(candidateIdx) = switchCost;
    switch mode
        case 'reliability'
            reliabilityValue = topologyReliability( ...
                candidate, context.commConfig, context.currentTime);
            connectivityValue = algebraicConnectivity(candidate);
            objectives(candidateIdx) = ...
                -reliabilityValue - 0.05 * connectivityValue + ...
                0.02 * switchCost;
        case 'discrepancy'
            taskValue = topologyPosteriorEdgeValue( ...
                candidate, context.localPosteriorBySensor, ...
                context.model, context.commConfig, context.currentTime);
            connectivityValue = algebraicConnectivity(candidate);
            objectives(candidateIdx) = ...
                -taskValue - 0.05 * connectivityValue + ...
                0.02 * switchCost;
        case {'oracle-consensus', 'oracle-truth'}
            [consensusValue, truthValue, fusionCache] = ...
                evaluateFusionCandidate( ...
                    candidate, context, fusionCache, fusionCacheEnabled);
            consensusTerms(candidateIdx) = consensusValue;
            truthTerms(candidateIdx) = truthValue;
            truthWeight = 0;
            if strcmp(mode, 'oracle-truth')
                truthWeight = 0.25;
            end
            objectives(candidateIdx) = consensusValue + ...
                truthWeight * truthValue + 0.02 * switchCost;
        otherwise
            error('Unknown D12 topology policy mode: %s', mode);
    end
end

if ~any(valid)
    % Fail closed: retain only currently physical base edges. The core
    % filter records this as infeasible instead of inventing links.
    adjacency = context.baseAdjacency & context.physicalAdjacency;
    details = struct( ...
        'objective', inf, ...
        'candidateIndex', NaN, ...
        'mode', mode, ...
        'validCandidateCount', 0, ...
        'selectionSeconds', toc(ticId));
    return;
end

[bestObjective, candidateIdx] = min(objectives);
adjacency = candidates(:, :, candidateIdx);
details = struct();
details.objective = bestObjective;
details.candidateIndex = candidateIdx;
details.mode = mode;
details.validCandidateCount = sum(valid);
details.consensusTerm = consensusTerms(candidateIdx);
details.truthTerm = truthTerms(candidateIdx);
details.switchTerm = switchTerms(candidateIdx);
details.attemptedByteMismatchFraction = ...
    byteMismatchTerms(candidateIdx);
details.selectionSeconds = toc(ticId);
end

function bytes = estimateNodePayloadBytes(posteriors, model)
bytes = zeros(1, numel(posteriors));
for sensorIdx = 1:numel(posteriors)
    stats = estimateLmbPayloadSize( ...
        posteriors{sensorIdx}, model, 2);
    bytes(sensorIdx) = stats.estimatedBytes;
end
end

function bytes = topologyAttemptedBytes(adjacency, nodePayloadBytes)
% Each undirected edge carries one directed message in each direction.
bytes = sum(sum(adjacency, 2)' .* nodePayloadBytes);
end

function value = topologyReliability(adjacency, commConfig, currentTime)
edges = find(triu(adjacency, 1));
value = 0;
for edgeCursor = 1:numel(edges)
    [leftIdx, rightIdx] = ind2sub(size(adjacency), edges(edgeCursor));
    pForward = edgeDrop(commConfig, leftIdx, rightIdx, currentTime);
    pReverse = edgeDrop(commConfig, rightIdx, leftIdx, currentTime);
    value = value + 1 - 0.5 * (pForward + pReverse);
end
end

function value = topologyPosteriorEdgeValue( ...
    adjacency, posteriors, model, commConfig, currentTime)
edges = find(triu(adjacency, 1));
value = 0;
for edgeCursor = 1:numel(edges)
    [leftIdx, rightIdx] = ind2sub(size(adjacency), edges(edgeCursor));
    reliability = 1 - 0.5 * ( ...
        edgeDrop(commConfig, leftIdx, rightIdx, currentTime) + ...
        edgeDrop(commConfig, rightIdx, leftIdx, currentTime));
    discrepancy = posteriorDisagreement( ...
        posteriors{leftIdx}, posteriors{rightIdx}, model);
    value = value + reliability * discrepancy;
end
end

function cache = initializeFusionCache(sensorCount)
cache = repmat(struct( ...
    'keys', {{}}, ...
    'values', {{}}), 1, sensorCount);
end

function [consensusValue, truthValue, cache] = evaluateFusionCandidate( ...
    adjacency, context, cache, cacheEnabled)
sensorCount = size(adjacency, 1);
weights = metropolisWeights(adjacency);
fused = cell(1, sensorCount);
for receiverIdx = 1:sensorCount
    neighbors = find(adjacency(receiverIdx, :));
    sources = [receiverIdx, neighbors];
    localWeights = weights(receiverIdx, sources);
    localWeights = localWeights / max(sum(localWeights), eps);
    % Metropolis weights depend on the neighbours' global degrees, so the
    % cache key must include both the local neighbourhood and its weights.
    cacheKey = [sprintf('%d_', neighbors), '|', ...
        sprintf('%.17g_', localWeights)];
    cachedIdx = [];
    if cacheEnabled
        cachedIdx = find(strcmp( ...
            cache(receiverIdx).keys, cacheKey), 1);
    end
    if ~isempty(cachedIdx)
        fused{receiverIdx} = ...
            cache(receiverIdx).values{cachedIdx};
        continue;
    end
    fusionInputs = context.localPosteriorBySensor(sources);
    fusionDetails = struct( ...
        'eventType', [0, 2 * ones(1, numel(neighbors))]);
    fused{receiverIdx} = fuseLmbPosteriorsByLabel( ...
        fusionInputs, localWeights, context.model, localWeights, ...
        fusionDetails, context.triggerConfig);
    if cacheEnabled
        cache(receiverIdx).keys{end+1} = cacheKey;
        cache(receiverIdx).values{end+1} = fused{receiverIdx};
    end
end

pairValues = [];
for leftIdx = 1:sensorCount-1
    for rightIdx = leftIdx+1:sensorCount
        pairValues(end+1) = posteriorDisagreement( ...
            fused{leftIdx}, fused{rightIdx}, context.model); %#ok<AGROW>
    end
end
if isempty(pairValues)
    consensusValue = 0;
else
    consensusValue = mean(pairValues);
end
truthValue = truthLabelSurrogate( ...
    fused, context.model, context.currentTime);
end

function value = posteriorDisagreement(leftObjects, rightObjects, model)
labels = collectLabels(leftObjects, rightObjects);
if isempty(labels)
    value = 0;
    return;
end
terms = zeros(1, size(labels, 2));
for labelIdx = 1:size(labels, 2)
    left = findLabel(leftObjects, labels(:, labelIdx));
    right = findLabel(rightObjects, labels(:, labelIdx));
    leftExistence = objectExistence(left);
    rightExistence = objectExistence(right);
    existenceTerm = abs(leftExistence - rightExistence);
    spatialTerm = 0;
    if ~isempty(left) && ~isempty(right)
        [leftMean, leftCovariance] = objectMoments(left, model.xDimension);
        [rightMean, rightCovariance] = objectMoments( ...
            right, model.xDimension);
        scale = sqrt(max(trace( ...
            leftCovariance(1:2, 1:2) + ...
            rightCovariance(1:2, 1:2)), 1));
        spatialTerm = min(norm( ...
            leftMean(1:2) - rightMean(1:2)) / scale, 5);
    end
    terms(labelIdx) = existenceTerm + ...
        min(leftExistence, rightExistence) * spatialTerm;
end
value = mean(terms);
end

function value = truthLabelSurrogate(fused, model, currentTime)
scenario = model.dynamicTopologyScenario;
birthTimes = scenario.target.birthTimes;
targetTrajectories = scenario.targetTrajectories;
sensorTerms = zeros(1, numel(fused));
for sensorIdx = 1:numel(fused)
    objects = fused{sensorIdx};
    targetTerms = [];
    activeTargetCount = 0;
    for targetIdx = 1:numel(targetTrajectories)
        truth = targetTrajectories{targetIdx}(:, currentTime);
        if any(~isfinite(truth))
            continue;
        end
        activeTargetCount = activeTargetCount + 1;
        object = findLabel(objects, [birthTimes(targetIdx); targetIdx]);
        if isempty(object)
            targetTerms(end+1) = 1; %#ok<AGROW>
        else
            [meanVector, ~] = objectMoments(object, model.xDimension);
            positionTerm = min(norm( ...
                meanVector(1:2) - truth(1:2)) / 50, 5);
            targetTerms(end+1) = ...
                (1 - object.r) + object.r * positionTerm; %#ok<AGROW>
        end
    end
    falseExistence = 0;
    for objectIdx = 1:numel(objects)
        birthLocation = objects(objectIdx).birthLocation;
        if birthLocation < 1 || birthLocation > numel(targetTrajectories)
            falseExistence = falseExistence + objects(objectIdx).r;
            continue;
        end
        truth = targetTrajectories{birthLocation}(:, currentTime);
        expectedBirth = birthTimes(birthLocation);
        if any(~isfinite(truth)) || ...
                objects(objectIdx).birthTime ~= expectedBirth
            falseExistence = falseExistence + objects(objectIdx).r;
        end
    end
    sensorTerms(sensorIdx) = ...
        meanOrZero(targetTerms) + falseExistence / ...
        max(activeTargetCount, 1);
end
value = meanOrZero(sensorTerms);
end

function probability = edgeDrop(config, senderIdx, receiverIdx, currentTime)
if isfield(config, 'pDropByEdge') && ~isempty(config.pDropByEdge)
    if ndims(config.pDropByEdge) >= 3
        probability = config.pDropByEdge( ...
            senderIdx, receiverIdx, currentTime);
    else
        probability = config.pDropByEdge(senderIdx, receiverIdx);
    end
elseif isfield(config, 'pDropBySensor') && ...
        numel(config.pDropBySensor) >= senderIdx
    probability = config.pDropBySensor(senderIdx);
else
    probability = 0;
end
probability = min(max(probability, 0), 1);
end

function weights = metropolisWeights(adjacency)
nodeCount = size(adjacency, 1);
degreesIncludingSelf = sum(adjacency, 2)' + 1;
weights = zeros(nodeCount);
for receiverIdx = 1:nodeCount
    neighbors = find(adjacency(receiverIdx, :));
    for senderIdx = neighbors
        weights(receiverIdx, senderIdx) = 1 / (1 + max( ...
            degreesIncludingSelf(receiverIdx), ...
            degreesIncludingSelf(senderIdx)));
    end
    weights(receiverIdx, receiverIdx) = ...
        1 - sum(weights(receiverIdx, :));
end
end

function value = algebraicConnectivity(adjacency)
degree = sum(adjacency, 2);
eigenvalues = sort(real(eig(diag(degree) - double(adjacency))));
if numel(eigenvalues) < 2
    value = 0;
else
    value = max(eigenvalues(2), 0);
end
end

function count = countRemovedEdges(previous, current)
if isempty(previous) || ~any(previous(:))
    count = 0;
else
    count = nnz(triu(previous & ~current, 1));
end
end

function labels = collectLabels(leftObjects, rightObjects)
labels = zeros(2, 0);
collections = {leftObjects, rightObjects};
for collectionIdx = 1:2
    objects = collections{collectionIdx};
    for objectIdx = 1:numel(objects)
        if objects(objectIdx).numberOfGmComponents < 1
            continue;
        end
        label = [objects(objectIdx).birthTime; ...
            objects(objectIdx).birthLocation];
        if isempty(labels) || ~any(all(labels == label, 1))
            labels(:, end+1) = label; %#ok<AGROW>
        end
    end
end
end

function object = findLabel(objects, label)
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
if isempty(object)
    value = 0;
else
    value = min(max(object.r, 0), 1);
end
end

function [meanVector, covariance] = objectMoments(object, stateDimension)
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if numel(weights) ~= object.numberOfGmComponents || sum(weights) <= 0
    weights = ones(1, object.numberOfGmComponents);
end
weights = weights / sum(weights);
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

function value = meanOrZero(values)
if isempty(values)
    value = 0;
else
    value = mean(values);
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
