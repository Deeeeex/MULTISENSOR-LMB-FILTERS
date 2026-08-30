function [posteriorBySensor, detailsByReceiver] = ...
        selectFormationCoordinatedPosteriorRepair( ...
            posteriorBySensor, localPosteriorBySensor, ...
            physicalAdjacency, directedLinkDelivered, config, ...
            receiverIds, currentTime, model)
% SELECTFORMATIONCOORDINATEDPOSTERIORREPAIR Choose one shared label action.
%
% Every receiver first builds the ordinary truth-free one-hop candidate
% shortlist.  The formation then admits only an exact (label, source) pair
% shared by every receiver and satisfying the frozen safety certificate.
% The same complete Bernoulli GM posterior is requested independently by
% every receiver, so communication remains charged on the physical links.

sensorCount = numel(localPosteriorBySensor);
receiverIds = reshape(receiverIds, 1, []);
detailsByReceiver = cell(1, sensorCount);
for receiverIdx = receiverIds
    detailsByReceiver{receiverIdx} = emptyDetails(sensorCount);
end
if isempty(receiverIds)
    return;
end
if ~iscell(posteriorBySensor) || numel(posteriorBySensor) ~= sensorCount || ...
        ~isequal(size(physicalAdjacency), [sensorCount, sensorCount]) || ...
        ~isequal(size(directedLinkDelivered), [sensorCount, sensorCount]) || ...
        numel(unique(receiverIds)) ~= numel(receiverIds) || ...
        any(receiverIds < 1) || any(receiverIds > sensorCount)
    error('FormationPosteriorRepairV185:InvalidInput', ...
        'The formation-level selector inputs are malformed.');
end
policyPath = getField(config, ...
    'formationCoordinatedPosteriorRepairPolicyPath', ...
    config.learnedOneHopSafeLabelPolicyPath);
modelPath = getField(config, ...
    'formationCoordinatedPosteriorRepairModelPath', ...
    config.learnedOneHopSafeLabelModelPath);
formationTimes = reshape(getField(config, ...
    'formationCoordinatedPosteriorRepairTimes', ...
    config.learnedOneHopSafeLabelTimes), 1, []);
commonNeighborPrefilterEnabled = logical(getField(config, ...
    'formationCoordinatedPosteriorRepairCommonNeighborPrefilterEnabled', ...
    false));
maximumAdvertisedLabelsPerSource = getField(config, ...
    'formationCoordinatedPosteriorRepairMaximumAdvertisedLabelsPerSource', ...
    inf);
policy = loadPolicy(policyPath);
classifier = loadClassifier(modelPath, policy);
if config.learnedOneHopSafeLabelMaximumEdits ~= 1 || ...
        config.learnedOneHopSafeLabelSynopsisBytesPerLabel ~= ...
            policy.synopsisBytesPerLabel
    error('FormationPosteriorRepairV185:ConfigurationDrift', ...
        'The runtime configuration differs from the frozen V185 policy.');
end
groupIds = reshape( ...
    model.dynamicTopologyScenario.config.sensorGroupIds, 1, []);
if numel(groupIds) ~= sensorCount || ...
        numel(unique(groupIds(receiverIds))) ~= 1
    error('FormationPosteriorRepairV185:FormationDrift', ...
        'All coordinated receivers must belong to one formation.');
end
commonSourceIds = zeros(1, 0);
if commonNeighborPrefilterEnabled
    commonSourceIds = 1:sensorCount;
    for receiverIdx = receiverIds
        physicalNeighbors = reshape(find(logical( ...
            physicalAdjacency(receiverIdx, :))), 1, []);
        commonSourceIds = intersect( ...
            commonSourceIds, physicalNeighbors, 'stable');
    end
end

candidateByReceiver = cell(1, numel(receiverIds));
for receiverPosition = 1:numel(receiverIds)
    receiverIdx = receiverIds(receiverPosition);
    [candidateByReceiver{receiverPosition}, ...
     detailsByReceiver{receiverIdx}] = buildReceiverCandidates( ...
        posteriorBySensor{receiverIdx}, localPosteriorBySensor, ...
        physicalAdjacency, directedLinkDelivered, config, receiverIdx, ...
        currentTime, model, classifier, detailsByReceiver{receiverIdx}, ...
        commonNeighborPrefilterEnabled, commonSourceIds, formationTimes, ...
        maximumAdvertisedLabelsPerSource);
end

common = commonCandidateReadouts( ...
    candidateByReceiver, receiverIds, policy);
if isempty(common)
    detailsByReceiver = finalizeDetails(detailsByReceiver, receiverIds);
    return;
end
ranking = [[common.rescueScore]', [common.minimumSafetyProbability]', ...
    [common.minimumRiskReduction]', ...
    -reshape([common.label], 2, [])', -[common.source]'];
[~, order] = sortrows(ranking, [-1, -2, -3, -4, -5, -6]);
selected = common(order(1));

for receiverPosition = 1:numel(receiverIds)
    receiverIdx = receiverIds(receiverPosition);
    candidate = selected.candidates(receiverPosition);
    details = detailsByReceiver{receiverIdx};
    details.selectedLabelCount = 1;
    details.selectedLabels = candidate.label;
    details.selectedSources = candidate.source;
    details.selectedRiskReduction = candidate.riskReduction;
    details.selectedSafetyScore = selected.minimumSafetyProbability;
    requestBytes = config.learnedOneHopSafeLabelRequestHeaderBytes + ...
        config.learnedOneHopSafeLabelRequestBytesPerLabel;
    sourceIdx = candidate.source;
    details.attemptedRequestBytesBySource(sourceIdx) = requestBytes;
    details.attemptedRequestMessageBySource(sourceIdx) = true;
    if directedLinkDelivered(receiverIdx, sourceIdx)
        details.deliveredRequestBytesBySource(sourceIdx) = requestBytes;
        details.deliveredRequestMessageBySource(sourceIdx) = true;
        responseStats = estimateLmbPayloadSize( ...
            candidate.sourceObject, model, 2, struct());
        if responseStats.objectCount ~= 1
            error('FormationPosteriorRepairV185:PayloadThresholdDrift', ...
                'The selected complete label was omitted from its response.');
        end
        details.attemptedResponseBytesBySource(sourceIdx) = ...
            responseStats.estimatedBytes;
        details.attemptedResponseMessageBySource(sourceIdx) = true;
        if directedLinkDelivered(sourceIdx, receiverIdx)
            details.deliveredResponseBytesBySource(sourceIdx) = ...
                responseStats.estimatedBytes;
            details.deliveredResponseMessageBySource(sourceIdx) = true;
            posteriorBySensor{receiverIdx} = replaceLabelObject( ...
                posteriorBySensor{receiverIdx}, candidate.sourceObject);
            details.applied = true;
            details.appliedLabelCount = 1;
            details.appliedLabels = candidate.label;
            details.appliedSources = sourceIdx;
        end
    end
    detailsByReceiver{receiverIdx} = details;
end
detailsByReceiver = finalizeDetails(detailsByReceiver, receiverIds);
end

function [candidates, details] = buildReceiverCandidates( ...
        receiverPosterior, localPosteriorBySensor, physicalAdjacency, ...
        directedLinkDelivered, config, receiverIdx, currentTime, ...
        model, classifier, details, restrictSources, allowedSourceIds, ...
        formationTimes, maximumAdvertisedLabelsPerSource)
sensorCount = numel(localPosteriorBySensor);
physicalNeighbors = reshape(find(logical( ...
    physicalAdjacency(receiverIdx, :))), 1, []);
physicalNeighbors = physicalNeighbors(physicalNeighbors ~= receiverIdx);
neighbors = physicalNeighbors;
if restrictSources
    neighbors = intersect(neighbors, allowedSourceIds, 'stable');
end
details.triggered = true;
details.pageIndex = find( ...
    formationTimes == currentTime, 1);
details.selectorMode = 'formation-coordinated-posterior-repair';
details.physicalNeighborCount = numel(physicalNeighbors);
activeThreshold = config.observableOneHopRiskLabelExistenceThreshold;
advertisedBySource = cell(1, sensorCount);
for sourceIdx = neighbors
    active = selectActiveLabels( ...
        localPosteriorBySensor{sourceIdx}, activeThreshold);
    active = limitAdvertisedLabels( ...
        active, maximumAdvertisedLabelsPerSource, model);
    if isempty(active)
        continue;
    end
    bytes = config.learnedOneHopSafeLabelSynopsisHeaderBytes + ...
        config.learnedOneHopSafeLabelSynopsisBytesPerLabel * numel(active);
    details.attemptedSynopsisBytesBySource(sourceIdx) = bytes;
    details.attemptedSynopsisMessageBySource(sourceIdx) = true;
    if directedLinkDelivered(sourceIdx, receiverIdx)
        advertisedBySource{sourceIdx} = active;
        details.deliveredSynopsisBytesBySource(sourceIdx) = bytes;
        details.deliveredSynopsisMessageBySource(sourceIdx) = true;
    end
end

groupIds = reshape( ...
    model.dynamicTopologyScenario.config.sensorGroupIds, 1, []);
receiverActiveCount = numel(selectActiveLabels( ...
    receiverPosterior, activeThreshold));
raw = repmat(emptyCandidate(), 1, 0);
featureNames = cell(1, 0);
for sourceIdx = neighbors
    sourceObjects = advertisedBySource{sourceIdx};
    for objectIdx = 1:numel(sourceObjects)
        sourceObject = sourceObjects(objectIdx);
        label = objectLabel(sourceObject);
        peers = objectsForLabel( ...
            advertisedBySource, neighbors, label, sourceIdx);
        receiverObject = findLabelObject(receiverPosterior, label);
        featureContext = struct( ...
            'receiverFormation', groupIds(receiverIdx), ...
            'sourceFormation', groupIds(sourceIdx), ...
            'nodeCount', sensorCount, ...
            'physicalNeighborCount', numel(physicalNeighbors), ...
            'receiverActiveLabelCount', receiverActiveCount, ...
            'sourceActiveLabelCount', numel(sourceObjects), ...
            'formationSize', nnz(groupIds == groupIds(receiverIdx)));
        [features, currentNames, featureDetails] = ...
            computeObservableOneHopLabelActionFeatures( ...
                receiverObject, sourceObject, peers, model, ...
                receiverIdx, sourceIdx, currentTime, featureContext);
        expectedNames = classifier.featureNames;
        if isfield(classifier, 'sourceFeatureNames')
            expectedNames = classifier.sourceFeatureNames;
        end
        if ~isequal(currentNames, expectedNames)
            error('FormationPosteriorRepairV185:FeatureContractDrift', ...
                'Runtime feature names differ from the V181 contract.');
        end
        if isempty(featureNames)
            featureNames = currentNames;
        elseif ~isequal(featureNames, currentNames)
            error('FormationPosteriorRepairV185:FeatureContractDrift', ...
                'Feature names changed within one receiver cell.');
        end
        [lowerProbability, inSupport] = ...
            scoreClassifier(classifier, features);
        candidate = emptyCandidate();
        candidate.label = label;
        candidate.receiver = receiverIdx;
        candidate.source = sourceIdx;
        candidate.sourceObject = sourceObject;
        candidate.features = features;
        candidate.payloadBytes = featureDetails.payloadBytes;
        candidate.riskReduction = featureValue( ...
            features, currentNames, 'risk_reduction');
        candidate.receiverPresent = featureValue( ...
            features, currentNames, 'receiver_present');
        candidate.compatibility = featureValue( ...
            features, currentNames, 'receiver_source_compatibility');
        candidate.peerConsensus = featureValue( ...
            features, currentNames, 'peer_consensus_mean');
        candidate.sourceEvidence = featureValue( ...
            features, currentNames, 'source_evidence_quality');
        candidate.minimumSafetyProbability = min(lowerProbability);
        candidate.inSupport = inSupport;
        candidate.rescueScore = candidate.peerConsensus * ...
            max(1 - candidate.compatibility, 0) * ...
            max(candidate.sourceEvidence, 0);
        raw(end + 1) = candidate; %#ok<AGROW>
    end
end
details.rawCandidateLabelCount = numel(raw);
candidates = shortlistCandidates(raw, featureNames, 1);
details.candidateLabelCount = numel(candidates);
end

function common = commonCandidateReadouts( ...
        candidateByReceiver, receiverIds, policy)
common = repmat(emptyCommonCandidate(), 1, 0);
if any(cellfun(@isempty, candidateByReceiver))
    return;
end
pairs = uniquePairs(candidateByReceiver{1});
for pairIdx = 1:size(pairs, 2)
    label = pairs(1:2, pairIdx);
    source = pairs(3, pairIdx);
    selected = repmat(emptyCandidate(), 1, numel(receiverIds));
    complete = true;
    for receiverPosition = 1:numel(receiverIds)
        idx = findPair(candidateByReceiver{receiverPosition}, label, source);
        if idx == 0
            complete = false;
            break;
        end
        selected(receiverPosition) = ...
            candidateByReceiver{receiverPosition}(idx);
    end
    if ~complete
        continue;
    end
    minimumSafety = min([selected.minimumSafetyProbability]);
    minimumRisk = min([selected.riskReduction]);
    minimumPresence = min([selected.receiverPresent]);
    supportCount = nnz([selected.inSupport]);
    if minimumSafety < policy.minimumSafetyProbability || ...
            minimumRisk < policy.minimumRiskReduction || ...
            minimumPresence < policy.minimumReceiverPresence || ...
            supportCount < policy.minimumSupportCount
        continue;
    end
    item = emptyCommonCandidate();
    item.label = label;
    item.source = source;
    item.candidates = selected;
    item.minimumSafetyProbability = minimumSafety;
    item.minimumRiskReduction = minimumRisk;
    item.supportCount = supportCount;
    item.rescueScore = median([selected.rescueScore]);
    common(end + 1) = item; %#ok<AGROW>
end
end

function shortlisted = shortlistCandidates(candidates, featureNames, count)
shortlisted = repmat(emptyCandidate(), 1, 0);
if isempty(candidates)
    return;
end
labels = unique(reshape([candidates.label], 2, [])', ...
    'rows', 'stable')';
for labelIdx = 1:size(labels, 2)
    label = labels(:, labelIdx);
    local = candidates(arrayfun(@(item) ...
        isequal(item.label, label), candidates));
    risk = candidateFeatureColumn(local, featureNames, 'risk_reduction');
    compatibility = candidateFeatureColumn( ...
        local, featureNames, 'receiver_source_compatibility');
    consensus = candidateFeatureColumn( ...
        local, featureNames, 'peer_consensus_mean');
    evidence = candidateFeatureColumn( ...
        local, featureNames, 'source_evidence_quality');
    opportunity = candidateFeatureColumn( ...
        local, featureNames, 'source_observation_opportunity');
    credibility = candidateFeatureColumn( ...
        local, featureNames, 'spatial_credibility');
    lowRisk = candidateFeatureColumn( ...
        local, featureNames, 'source_low_risk_percentile');
    criteria = [risk; compatibility; consensus; evidence; opportunity; ...
        risk .* sqrt(max(credibility, 0)); lowRisk];
    chosen = zeros(1, 0);
    for criterionIdx = 1:size(criteria, 1)
        ranking = [-criteria(criterionIdx, :)', [local.source]'];
        [~, order] = sortrows(ranking, [1, 2]);
        chosen = unique([chosen, ...
            order(1:min(count, numel(order)))'], 'stable'); %#ok<AGROW>
    end
    shortlisted = [shortlisted, local(chosen)]; %#ok<AGROW>
end
end

function policy = loadPolicy(path)
loaded = load(path, 'policy');
required = {'contractVersion', 'modelPath', 'minimumSupportCount', ...
    'minimumSafetyProbability', 'minimumRiskReduction', ...
    'minimumReceiverPresence', 'synopsisBytesPerLabel', ...
    'featuresUseTruth', 'numericLabelIdentifiersUsedAsFeatures', ...
    'preflightGatePassed'};
if ~isfield(loaded, 'policy') || ~isstruct(loaded.policy) || ...
        ~isscalar(loaded.policy) || ~all(isfield(loaded.policy, required)) || ...
        ~strcmp(loaded.policy.contractVersion, ...
            'formation-coordinated-posterior-repair-policy-v185-v1') || ...
        loaded.policy.featuresUseTruth || ...
        loaded.policy.numericLabelIdentifiersUsedAsFeatures || ...
        ~loaded.policy.preflightGatePassed || ...
        exist(loaded.policy.modelPath, 'file') ~= 2
    error('FormationPosteriorRepairV185:InvalidPolicy', ...
        'The registered V185 policy is invalid.');
end
policy = loaded.policy;
end

function classifier = loadClassifier(path, policy)
loaded = load(path, 'model');
if ~strcmp(path, policy.modelPath) || ~isfield(loaded, 'model') || ...
        ~isstruct(loaded.model) || ...
        ~strcmp(loaded.model.contractVersion, ...
            'nonlinear-safe-one-hop-label-action-model-v168-v1') || ...
        loaded.model.featuresUseTruth || ...
        loaded.model.numericLabelIdentifiersUsedAsFeatures || ...
        ~loaded.model.heldoutSafetyGatePassed
    error('FormationPosteriorRepairV185:InvalidClassifier', ...
        'The registered V181 safety classifier is invalid.');
end
classifier = loaded.model;
end

function [lowerProbability, inSupport] = scoreClassifier(classifier, features)
if isfield(classifier, 'featureMask')
    features = features(logical(classifier.featureMask));
end
z = (features - classifier.featureMean) ./ classifier.featureScale;
inSupport = all(z >= classifier.supportLower & z <= classifier.supportUpper);
probability = zeros(numel(classifier.members), 3);
for memberIdx = 1:numel(classifier.members)
    parameters = classifier.members{memberIdx}.parameters;
    hidden = tanh(z * parameters{1} + parameters{2});
    probability(memberIdx, :) = stableSigmoid( ...
        hidden * parameters{3} + parameters{4});
end
lowerProbability = mean(probability, 1) - ...
    classifier.uncertaintyPenalty * std(probability, 0, 1);
end

function detailsByReceiver = finalizeDetails(detailsByReceiver, receiverIds)
for receiverIdx = receiverIds
    details = detailsByReceiver{receiverIdx};
    details.attemptedSynopsisBytes = sum( ...
        details.attemptedSynopsisBytesBySource);
    details.deliveredSynopsisBytes = sum( ...
        details.deliveredSynopsisBytesBySource);
    details.attemptedRequestBytes = sum( ...
        details.attemptedRequestBytesBySource);
    details.deliveredRequestBytes = sum( ...
        details.deliveredRequestBytesBySource);
    details.attemptedResponseBytes = sum( ...
        details.attemptedResponseBytesBySource);
    details.deliveredResponseBytes = sum( ...
        details.deliveredResponseBytesBySource);
    details.attemptedTotalBytes = details.attemptedSynopsisBytes + ...
        details.attemptedRequestBytes + details.attemptedResponseBytes;
    details.deliveredTotalBytes = details.deliveredSynopsisBytes + ...
        details.deliveredRequestBytes + details.deliveredResponseBytes;
    detailsByReceiver{receiverIdx} = details;
end
end

function values = candidateFeatureColumn(candidates, names, name)
idx = find(strcmp(names, name), 1);
if isempty(idx)
    error('FormationPosteriorRepairV185:MissingShortlistFeature', ...
        'Required shortlist feature is missing: %s.', name);
end
matrix = vertcat(candidates.features);
values = matrix(:, idx)';
end

function value = featureValue(features, names, name)
idx = find(strcmp(names, name), 1);
if isempty(idx)
    error('FormationPosteriorRepairV185:MissingPolicyFeature', ...
        'Required policy feature is missing: %s.', name);
end
value = features(idx);
end

function objects = objectsForLabel( ...
        advertisedBySource, neighbors, label, excludedSource)
objects = [];
for sourceIdx = neighbors
    if sourceIdx == excludedSource
        continue;
    end
    object = findLabelObject(advertisedBySource{sourceIdx}, label);
    if ~isempty(object)
        if isempty(objects)
            objects = object;
        else
            objects(end + 1) = object; %#ok<AGROW>
        end
    end
end
objects = reshape(objects, 1, []);
end

function pairs = uniquePairs(candidates)
pairs = [[candidates.label]; [candidates.source]];
pairs = unique(pairs', 'rows', 'stable')';
end

function idx = findPair(candidates, label, source)
idx = find([candidates.source] == source & arrayfun(@(item) ...
    isequal(item.label, label), candidates), 1);
if isempty(idx)
    idx = 0;
end
end

function objects = selectActiveLabels(objects, threshold)
objects = reshape(objects, 1, []);
if isempty(objects)
    return;
end
objects = objects([objects.r] > threshold & ...
    [objects.numberOfGmComponents] > 0);
end

function objects = limitAdvertisedLabels(objects, maximumCount, model)
objects = reshape(objects, 1, []);
if isinf(maximumCount) || numel(objects) <= maximumCount
    return;
end
positionTrace = zeros(numel(objects), 1);
existence = reshape([objects.r], [], 1);
for objectIdx = 1:numel(objects)
    [~, riskDetails] = computeObservableLmbLabelBayesRisk( ...
        objects(objectIdx), model);
    positionTrace(objectIdx) = riskDetails.positionTrace;
end
originalOrder = (1:numel(objects))';
[~, order] = sortrows( ...
    [positionTrace, -existence, originalOrder], [1, 2, 3]);
objects = objects(order(1:maximumCount));
end

function posterior = replaceLabelObject(posterior, object)
posterior = reshape(posterior, 1, []);
idx = findLabelIndex(posterior, objectLabel(object));
if idx == 0
    posterior(end + 1) = object;
else
    posterior(idx) = object;
end
end

function object = findLabelObject(objects, label)
idx = findLabelIndex(objects, label);
if idx == 0
    object = [];
else
    object = objects(idx);
end
end

function idx = findLabelIndex(objects, label)
idx = 0;
for objectIdx = 1:numel(objects)
    if objects(objectIdx).birthTime == label(1) && ...
            objects(objectIdx).birthLocation == label(2)
        idx = objectIdx;
        return;
    end
end
end

function label = objectLabel(object)
label = [object.birthTime; object.birthLocation];
end

function value = stableSigmoid(value)
positive = value >= 0;
negative = ~positive;
result = zeros(size(value));
result(positive) = 1 ./ (1 + exp(-value(positive)));
exponential = exp(value(negative));
result(negative) = exponential ./ (1 + exponential);
value = result;
end

function candidate = emptyCandidate()
candidate = struct( ...
    'label', zeros(2, 1), 'receiver', 0, 'source', 0, ...
    'sourceObject', [], 'features', zeros(1, 0), ...
    'payloadBytes', 0, 'riskReduction', -inf, ...
    'receiverPresent', 0, 'compatibility', 0, 'peerConsensus', 0, ...
    'sourceEvidence', 0, 'minimumSafetyProbability', -inf, ...
    'inSupport', false, 'rescueScore', -inf);
end

function item = emptyCommonCandidate()
item = struct( ...
    'label', zeros(2, 1), 'source', 0, ...
    'candidates', repmat(emptyCandidate(), 1, 0), ...
    'minimumSafetyProbability', -inf, ...
    'minimumRiskReduction', -inf, 'supportCount', 0, ...
    'rescueScore', -inf);
end

function details = emptyDetails(sensorCount)
details = struct( ...
    'contractVersion', 'observable-one-hop-risk-label-transfer-v162-v1', ...
    'sourceRule', 'formation-common-exact-label-source', ...
    'labelRule', 'safe-consensus-rescue-ranking', ...
    'selectorMode', 'formation-coordinated-posterior-repair', ...
    'triggered', false, 'applied', false, 'pageIndex', 0, ...
    'physicalNeighborCount', 0, 'rawCandidateLabelCount', 0, ...
    'candidateLabelCount', 0, 'conditionalCandidateLabelCount', 0, ...
    'selectedLabelCount', 0, 'appliedLabelCount', 0, ...
    'selectedLabels', zeros(2, 0), ...
    'selectedSources', zeros(1, 0), ...
    'selectedRiskReduction', zeros(1, 0), ...
    'selectedSafetyScore', zeros(1, 0), ...
    'cardinalityPressureExpectedFraction', NaN, ...
    'cardinalityPressureMapProbability', NaN, ...
    'cardinalityPressureGatePassed', false, ...
    'appliedLabels', zeros(2, 0), 'appliedSources', zeros(1, 0), ...
    'attemptedSynopsisMessageBySource', false(1, sensorCount), ...
    'deliveredSynopsisMessageBySource', false(1, sensorCount), ...
    'attemptedRequestMessageBySource', false(1, sensorCount), ...
    'deliveredRequestMessageBySource', false(1, sensorCount), ...
    'attemptedResponseMessageBySource', false(1, sensorCount), ...
    'deliveredResponseMessageBySource', false(1, sensorCount), ...
    'attemptedSynopsisBytesBySource', zeros(1, sensorCount), ...
    'deliveredSynopsisBytesBySource', zeros(1, sensorCount), ...
    'attemptedRequestBytesBySource', zeros(1, sensorCount), ...
    'deliveredRequestBytesBySource', zeros(1, sensorCount), ...
    'attemptedResponseBytesBySource', zeros(1, sensorCount), ...
    'deliveredResponseBytesBySource', zeros(1, sensorCount), ...
    'attemptedSynopsisBytes', 0, 'deliveredSynopsisBytes', 0, ...
    'attemptedRequestBytes', 0, 'deliveredRequestBytes', 0, ...
    'attemptedResponseBytes', 0, 'deliveredResponseBytes', 0, ...
    'attemptedTotalBytes', 0, 'deliveredTotalBytes', 0, ...
    'truthUsed', false, 'futureInformationUsed', false);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
