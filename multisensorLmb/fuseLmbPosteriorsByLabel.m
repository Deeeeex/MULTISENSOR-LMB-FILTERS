function fusedObjects = fuseLmbPosteriorsByLabel( ...
    posteriorDistributions, spatialWeights, model, existenceWeights, ...
    fusionDetails, triggerConfig)
% FUSELMBPOSTERIORSBYLABEL GA/KLA fusion for possibly different label sets.
%
% Sources missing a label do not participate in that label's fusion. The
% remaining source weights are renormalized, avoiding a zero-existence veto
% when independently pruned nodes have different active label sets.

if isempty(posteriorDistributions)
    fusedObjects = model.object;
    return;
end
if nargin < 4 || isempty(existenceWeights)
    existenceWeights = spatialWeights;
end
if nargin < 5 || isempty(fusionDetails)
    fusionDetails = struct();
end
if nargin < 6 || isempty(triggerConfig)
    triggerConfig = struct();
end
spatialWeights = normalizeWeights( ...
    spatialWeights, numel(posteriorDistributions));
existenceWeights = normalizeWeights( ...
    existenceWeights, numel(posteriorDistributions));
sourceModes = resolveSourceModes(fusionDetails, numel(posteriorDistributions));

labels = collectLabels(posteriorDistributions);
if isempty(labels)
    fusedObjects = posteriorDistributions{1};
    return;
end

fusedObjects = posteriorDistributions{1}([]);
for labelIdx = 1:size(labels, 2)
    label = labels(:, labelIdx);
    localObjects = cell(1, numel(posteriorDistributions));
    present = false(1, numel(posteriorDistributions));
    for sourceIdx = 1:numel(posteriorDistributions)
        localObjects{sourceIdx} = findObject( ...
            posteriorDistributions{sourceIdx}, label);
        present(sourceIdx) = ~isempty(localObjects{sourceIdx});
    end
    activeSpatialWeights = spatialWeights .* present;
    activeExistenceWeights = existenceWeights .* present;
    if sum(activeSpatialWeights) <= 0 || sum(activeExistenceWeights) <= 0
        continue;
    end
    activeSpatialWeights = activeSpatialWeights / sum(activeSpatialWeights);
    activeExistenceWeights = ...
        activeExistenceWeights / sum(activeExistenceWeights);
    templateIdx = find(present, 1, 'first');
    fusedObject = localObjects{templateIdx};

    if shouldUseMixtureAwareFusion( ...
            localObjects, present, sourceModes, triggerConfig)
        [fusedObject, logEta] = fuseSpatialMixtureAware( ...
            fusedObject, localObjects, present, activeSpatialWeights, ...
            model, triggerConfig);
        if strcmpi(getField(triggerConfig, ...
                'mixtureAwareExistenceEtaMode', 'momentMatched'), ...
                'momentMatched')
            [~, logEta] = fuseSpatialMomentMatched( ...
                fusedObject, localObjects, present, ...
                activeSpatialWeights, model);
        end
    else
        [fusedObject, logEta] = fuseSpatialMomentMatched( ...
            fusedObject, localObjects, present, activeSpatialWeights, model);
    end
    fusedObject.r = fuseExistenceProbability( ...
        localObjects, present, activeExistenceWeights, logEta);
    fusedObjects(end+1) = fusedObject; %#ok<AGROW>
end

function weights = normalizeWeights(weights, sourceCount)
weights = reshape(weights, 1, []);
if numel(weights) ~= sourceCount
    weights = ones(1, sourceCount);
end
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if sum(weights) <= 0
    weights = ones(1, sourceCount);
end
weights = weights / sum(weights);
end
end

function sourceModes = resolveSourceModes(fusionDetails, sourceCount)
sourceModes = zeros(1, sourceCount);
if ~isstruct(fusionDetails) || ~isfield(fusionDetails, 'eventType')
    return;
end
eventTypes = reshape(fusionDetails.eventType, 1, []);
if numel(eventTypes) ~= sourceCount
    return;
end
eventTypes(~isfinite(eventTypes)) = 0;
sourceModes = eventTypes;
end

function enabled = shouldUseMixtureAwareFusion( ...
    localObjects, present, sourceModes, triggerConfig)
enabled = false;
if ~getField(triggerConfig, 'mixtureAwareHeavyFusionEnabled', false)
    return;
end
activeSources = find(present);
if numel(activeSources) < 2
    return;
end
if getField(triggerConfig, 'mixtureAwareRequireHeavyInput', true)
    % The first fusion input is the receiver's local posterior. Heavy payload
    % semantics should be activated by a delivered heavy neighbor message.
    if ~any(sourceModes(activeSources(activeSources > 1)) >= 2)
        return;
    end
end
minimumEntropy = getField(triggerConfig, 'mixtureAwareMinEntropy', 0.35);
minimumComponents = max(2, round(getField( ...
    triggerConfig, 'mixtureAwareMinComponentCount', 2)));
minimumExistence = getField(triggerConfig, 'mixtureAwareMinExistence', 0.7);
minimumAssociationAmbiguity = getField( ...
    triggerConfig, 'mixtureAwareMinAssociationAmbiguity', 0);
minimumDetectionAssociationMass = getField( ...
    triggerConfig, 'mixtureAwareMinDetectionAssociationMass', 0);
for sourceIdx = activeSources
    if sourceQualifiesForMixtureAwareFusion( ...
            localObjects{sourceIdx}, minimumExistence, ...
            minimumComponents, minimumEntropy, ...
            minimumAssociationAmbiguity, ...
            minimumDetectionAssociationMass)
        enabled = true;
        return;
    end
end
end

function qualifies = sourceQualifiesForMixtureAwareFusion( ...
    object, minimumExistence, minimumComponents, minimumEntropy, ...
    minimumAssociationAmbiguity, minimumDetectionAssociationMass)
qualifies = object.r >= minimumExistence && ...
    object.numberOfGmComponents >= minimumComponents && ...
    normalizedMixtureEntropy(object) >= minimumEntropy && ...
    labelAssociationAmbiguity(object) >= minimumAssociationAmbiguity && ...
    labelDetectionAssociationMass(object) >= ...
    minimumDetectionAssociationMass;
end

function [fusedObject, logEta] = fuseSpatialMomentMatched( ...
    fusedObject, localObjects, present, activeSpatialWeights, model)
canonicalK = zeros(model.xDimension);
canonicalH = zeros(model.xDimension, 1);
canonicalG = 0;
for sourceIdx = find(present)
    [mu, covariance] = momentMatch( ...
        localObjects{sourceIdx}, model.xDimension);
    covariance = regularizeCovariance(covariance);
    precision = activeSpatialWeights(sourceIdx) * inv(covariance);
    canonicalK = canonicalK + precision;
    canonicalH = canonicalH + precision * mu;
    canonicalG = canonicalG - ...
        0.5 * mu' * precision * mu - ...
        0.5 * activeSpatialWeights(sourceIdx) * ...
        logDet(2 * pi * covariance);
end
fusedCovariance = regularizeCovariance(inv(canonicalK));
fusedMean = fusedCovariance * canonicalH;
logEta = canonicalG + ...
    0.5 * fusedMean' * canonicalK * fusedMean + ...
    0.5 * logDet(2 * pi * fusedCovariance);
fusedObject.numberOfGmComponents = 1;
fusedObject.w = 1;
fusedObject.mu = {fusedMean};
fusedObject.Sigma = {fusedCovariance};
end

function [fusedObject, logEta] = fuseSpatialMixtureAware( ...
    fusedObject, localObjects, present, activeSpatialWeights, model, ...
    triggerConfig)
activeSources = find(present);
topComponentCount = max(1, round(getField( ...
    triggerConfig, 'mixtureAwareTopComponents', 2)));
maxOutputComponents = max(1, round(getField( ...
    triggerConfig, 'mixtureAwareMaxFusedComponents', 6)));
maxComponentTuples = max(maxOutputComponents, round(getField( ...
    triggerConfig, 'mixtureAwareMaxComponentTuples', 512)));

partialK = zeros(model.xDimension, model.xDimension, 0);
partialH = zeros(model.xDimension, 0);
partialG = zeros(1, 0);
partialLogMixture = zeros(1, 0);

for activeIdx = 1:numel(activeSources)
    sourceIdx = activeSources(activeIdx);
    object = localObjects{sourceIdx};
    [object, componentIdxs] = selectSourceFusionComponents( ...
        object, topComponentCount, model.xDimension, triggerConfig);
    sourceWeight = activeSpatialWeights(sourceIdx);
    if activeIdx == 1
        [partialK, partialH, partialG, partialLogMixture] = ...
            initializeMixturePartials( ...
                object, componentIdxs, sourceWeight, model.xDimension);
    else
        [partialK, partialH, partialG, partialLogMixture] = ...
            extendMixturePartials( ...
                partialK, partialH, partialG, partialLogMixture, ...
                object, componentIdxs, sourceWeight);
    end
    [partialK, partialH, partialG, partialLogMixture] = ...
        trimMixturePartials( ...
            partialK, partialH, partialG, partialLogMixture, ...
            maxComponentTuples);
end

componentCount = numel(partialG);
if componentCount == 0
    [fusedObject, logEta] = fuseSpatialMomentMatched( ...
        fusedObject, localObjects, present, activeSpatialWeights, model);
    return;
end

means = cell(1, componentCount);
covariances = cell(1, componentCount);
componentLogWeights = zeros(1, componentCount);
for componentIdx = 1:componentCount
    precision = regularizeCovariance(partialK(:, :, componentIdx));
    covariance = regularizeCovariance(inv(precision));
    mean = covariance * partialH(:, componentIdx);
    componentLogEta = partialG(componentIdx) + ...
        0.5 * mean' * precision * mean + ...
        0.5 * logDet(2 * pi * covariance);
    means{componentIdx} = mean;
    covariances{componentIdx} = covariance;
    componentLogWeights(componentIdx) = ...
        partialLogMixture(componentIdx) + componentLogEta;
end

[componentLogWeights, sortedIndices] = sort(componentLogWeights, 'descend');
keepCount = min(maxOutputComponents, numel(sortedIndices));
sortedIndices = sortedIndices(1:keepCount);
componentLogWeights = componentLogWeights(1:keepCount);
logEta = logSumExp(componentLogWeights);
normalizedWeights = exp(componentLogWeights - logEta);
normalizedWeights = normalizedWeights / max(sum(normalizedWeights), eps);

maxFusedEntropy = getField(triggerConfig, 'mixtureAwareMaxFusedEntropy', 1);
minFusedDominance = getField(triggerConfig, 'mixtureAwareMinFusedDominance', 0);
if normalizedWeightEntropy(normalizedWeights) > maxFusedEntropy || ...
        max(normalizedWeights) < minFusedDominance
    [fusedObject, logEta] = fuseSpatialMomentMatched( ...
        fusedObject, localObjects, present, activeSpatialWeights, model);
    return;
end

fusedObject.numberOfGmComponents = keepCount;
fusedObject.w = normalizedWeights;
fusedObject.mu = means(sortedIndices);
fusedObject.Sigma = covariances(sortedIndices);
end

function [sourceObject, componentIdxs] = selectSourceFusionComponents( ...
    object, topComponentCount, stateDimension, triggerConfig)
minimumEntropy = getField(triggerConfig, 'mixtureAwareMinEntropy', 0.35);
minimumComponents = max(2, round(getField( ...
    triggerConfig, 'mixtureAwareMinComponentCount', 2)));
minimumExistence = getField(triggerConfig, 'mixtureAwareMinExistence', 0.7);
minimumAssociationAmbiguity = getField( ...
    triggerConfig, 'mixtureAwareMinAssociationAmbiguity', 0);
minimumDetectionAssociationMass = getField( ...
    triggerConfig, 'mixtureAwareMinDetectionAssociationMass', 0);
if sourceQualifiesForMixtureAwareFusion( ...
        object, minimumExistence, minimumComponents, minimumEntropy, ...
        minimumAssociationAmbiguity, minimumDetectionAssociationMass)
    sourceObject = object;
    componentIdxs = selectTopComponents(sourceObject, topComponentCount);
    return;
end

[mu, covariance] = momentMatch(object, stateDimension);
sourceObject = object;
sourceObject.numberOfGmComponents = 1;
sourceObject.w = 1;
sourceObject.mu = {mu};
sourceObject.Sigma = {covariance};
componentIdxs = 1;
end

function [partialK, partialH, partialG, partialLogMixture] = ...
    initializeMixturePartials(object, componentIdxs, sourceWeight, ...
        stateDimension)
componentCount = numel(componentIdxs);
partialK = zeros(stateDimension, stateDimension, componentCount);
partialH = zeros(stateDimension, componentCount);
partialG = zeros(1, componentCount);
partialLogMixture = zeros(1, componentCount);
weights = normalizedComponentWeights(object);
for outputIdx = 1:componentCount
    componentIdx = componentIdxs(outputIdx);
    [K, h, g] = weightedGaussianCanonical( ...
        object.mu{componentIdx}, object.Sigma{componentIdx}, sourceWeight);
    partialK(:, :, outputIdx) = K;
    partialH(:, outputIdx) = h;
    partialG(outputIdx) = g;
    partialLogMixture(outputIdx) = ...
        sourceWeight * log(max(weights(componentIdx), realmin));
end
end

function [nextK, nextH, nextG, nextLogMixture] = extendMixturePartials( ...
    partialK, partialH, partialG, partialLogMixture, object, ...
    componentIdxs, sourceWeight)
inputCount = numel(partialG);
componentCount = numel(componentIdxs);
stateDimension = size(partialH, 1);
nextCount = inputCount * componentCount;
nextK = zeros(stateDimension, stateDimension, nextCount);
nextH = zeros(stateDimension, nextCount);
nextG = zeros(1, nextCount);
nextLogMixture = zeros(1, nextCount);
weights = normalizedComponentWeights(object);
cursor = 0;
for inputIdx = 1:inputCount
    for outputIdx = 1:componentCount
        cursor = cursor + 1;
        componentIdx = componentIdxs(outputIdx);
        [K, h, g] = weightedGaussianCanonical( ...
            object.mu{componentIdx}, object.Sigma{componentIdx}, ...
            sourceWeight);
        nextK(:, :, cursor) = partialK(:, :, inputIdx) + K;
        nextH(:, cursor) = partialH(:, inputIdx) + h;
        nextG(cursor) = partialG(inputIdx) + g;
        nextLogMixture(cursor) = partialLogMixture(inputIdx) + ...
            sourceWeight * log(max(weights(componentIdx), realmin));
    end
end
end

function [partialK, partialH, partialG, partialLogMixture] = ...
    trimMixturePartials(partialK, partialH, partialG, ...
        partialLogMixture, maxComponentTuples)
if numel(partialG) <= maxComponentTuples
    return;
end
scores = partialLogMixture + partialG;
[~, order] = sort(scores, 'descend');
keep = order(1:maxComponentTuples);
partialK = partialK(:, :, keep);
partialH = partialH(:, keep);
partialG = partialG(keep);
partialLogMixture = partialLogMixture(keep);
end

function [K, h, g] = weightedGaussianCanonical(mu, covariance, weight)
covariance = regularizeCovariance(covariance);
KBase = inv(covariance);
K = weight * KBase;
h = K * mu;
g = -0.5 * mu' * K * mu - ...
    0.5 * weight * logDet(2 * pi * covariance);
end

function componentIdxs = selectTopComponents(object, topComponentCount)
weights = normalizedComponentWeights(object);
[~, order] = sort(weights, 'descend');
componentIdxs = order(1:min(topComponentCount, numel(order)));
end

function weights = normalizedComponentWeights(object)
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
componentCount = max(1, object.numberOfGmComponents);
if numel(weights) ~= componentCount
    weights = ones(1, componentCount);
end
if sum(weights) <= 0
    weights = ones(1, componentCount) / componentCount;
else
    weights = weights / sum(weights);
end
end

function value = normalizedMixtureEntropy(object)
componentCount = max(1, object.numberOfGmComponents);
if componentCount <= 1
    value = 0;
    return;
end
weights = normalizedComponentWeights(object);
weights = weights(weights > 0);
entropy = -sum(weights .* log(weights));
value = min(max(entropy / log(componentCount), 0), 1);
end

function value = normalizedWeightEntropy(weights)
weights = reshape(weights, 1, []);
weights = weights(isfinite(weights) & weights > 0);
componentCount = numel(weights);
if componentCount <= 1
    value = 0;
    return;
end
weights = weights / sum(weights);
value = -sum(weights .* log(weights)) / log(componentCount);
value = min(max(value, 0), 1);
end

function value = labelAssociationAmbiguity(object)
value = getField(object, 'associationAmbiguity', 0);
if isempty(value) || ~isfinite(value)
    value = 0;
end
value = min(max(value, 0), 1);
end

function value = labelDetectionAssociationMass(object)
value = getField(object, 'detectionAssociationMass', 0);
if isempty(value) || ~isfinite(value)
    value = 0;
end
value = min(max(value, 0), 1);
end

function probability = fuseExistenceProbability( ...
    localObjects, present, activeExistenceWeights, logEta)
logNumerator = logEta;
logAbsent = 0;
for sourceIdx = find(present)
    probabilitySource = clampProbability(localObjects{sourceIdx}.r);
    logNumerator = logNumerator + ...
        activeExistenceWeights(sourceIdx) * log(probabilitySource);
    logAbsent = logAbsent + activeExistenceWeights(sourceIdx) * ...
        log(1 - probabilitySource);
end
probability = logistic(logNumerator - logAbsent);
end

function value = logistic(logOdds)
if logOdds >= 0
    value = 1 / (1 + exp(-logOdds));
else
    expValue = exp(logOdds);
    value = expValue / (1 + expValue);
end
end

function value = logSumExp(values)
values = reshape(values(isfinite(values)), 1, []);
if isempty(values)
    value = -inf;
    return;
end
maxValue = max(values);
value = maxValue + log(sum(exp(values - maxValue)));
end

function labels = collectLabels(distributions)
labels = zeros(2, 0);
for sourceIdx = 1:numel(distributions)
    objects = distributions{sourceIdx};
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
if ~isempty(labels)
    labels = sortrows(labels')';
end
end

function object = findObject(objects, label)
object = [];
for idx = 1:numel(objects)
    if objects(idx).numberOfGmComponents > 0 && ...
            objects(idx).birthTime == label(1) && ...
            objects(idx).birthLocation == label(2)
        object = objects(idx);
        return;
    end
end
end

function [mu, covariance] = momentMatch(object, stateDimension)
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
weights = weights / max(sum(weights), eps);
mu = zeros(stateDimension, 1);
for componentIdx = 1:object.numberOfGmComponents
    mu = mu + weights(componentIdx) * object.mu{componentIdx};
end
covariance = zeros(stateDimension);
for componentIdx = 1:object.numberOfGmComponents
    delta = object.mu{componentIdx} - mu;
    covariance = covariance + weights(componentIdx) * ...
        (object.Sigma{componentIdx} + delta * delta');
end
covariance = regularizeCovariance(covariance);
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
    value = log(max(det(matrix), realmin));
end
end

function probability = clampProbability(probability)
if ~isfinite(probability)
    probability = 0.5;
end
probability = min(max(probability, 1e-9), 1 - 1e-9);
end

function value = getField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end
