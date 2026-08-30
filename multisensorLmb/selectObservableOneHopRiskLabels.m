function [posterior, details] = selectObservableOneHopRiskLabels( ...
        currentPosterior, localPosteriorBySensor, physicalAdjacency, ...
        directedLinkDelivered, config, receiverIdx, currentTime, model)
% SELECTOBSERVABLEONEHOPRISKLABELS Request bounded complete labels by risk.
%
% Every physical neighbor advertises its active label keys and scalar
% posterior Bayes risks.  The receiver chooses the minimum-risk source for
% each label, ranks labels by receiver-risk minus source-risk, and requests
% at most K positive-risk-reduction complete Bernoulli GM densities.  The
% routine reads no truth, future measurements, or alternative-arm state.

sensorCount = numel(localPosteriorBySensor);
posterior = reshape(currentPosterior, 1, []);
details = emptyDetails(sensorCount);
if ~config.observableOneHopRiskLabelEnabled
    return;
end

pageIdx = find(config.observableOneHopRiskLabelTimes == currentTime, 1);
analyticTriggered = ~isempty(pageIdx) && ismember(receiverIdx, ...
    config.observableOneHopRiskLabelSensorIdsByTime{pageIdx});
learnedPageIdx = zeros(1, 0);
learnedTriggered = false;
learnedEnabled = isfield(config, 'learnedOneHopSafeLabelEnabled') && ...
    logical(config.learnedOneHopSafeLabelEnabled);
if learnedEnabled
    learnedPageIdx = find( ...
        config.learnedOneHopSafeLabelTimes == currentTime, 1);
    learnedTriggered = ~isempty(learnedPageIdx) && ...
        ismember(receiverIdx, ...
            config.learnedOneHopSafeLabelSensorIdsByTime{learnedPageIdx});
end
if ~analyticTriggered && ~learnedTriggered
    return;
end
if analyticTriggered && learnedTriggered
    error('LearnedOneHopSafeLabelV169:OverlappingSelectorSchedule', ...
        'A receiver-time cell cannot run both one-hop selectors.');
end
if ~isequal(size(physicalAdjacency), [sensorCount, sensorCount]) || ...
        ~isequal(size(directedLinkDelivered), [sensorCount, sensorCount])
    error('ObservableOneHopRiskLabel:InvalidLinkState', ...
        'Physical adjacency and directed delivery state must be S-by-S.');
end

details.triggered = true;
if learnedTriggered
    details.pageIndex = learnedPageIdx;
    details.selectorMode = 'sequential-cardinality-pressure-relief';
    synopsisHeaderBytes = ...
        config.learnedOneHopSafeLabelSynopsisHeaderBytes;
    synopsisBytesPerLabel = ...
        config.learnedOneHopSafeLabelSynopsisBytesPerLabel;
    requestHeaderBytes = ...
        config.learnedOneHopSafeLabelRequestHeaderBytes;
    requestBytesPerLabel = ...
        config.learnedOneHopSafeLabelRequestBytesPerLabel;
    maximumEdits = config.learnedOneHopSafeLabelMaximumEdits;
else
    details.pageIndex = pageIdx;
    synopsisHeaderBytes = ...
        config.observableOneHopRiskLabelSynopsisHeaderBytes;
    synopsisBytesPerLabel = ...
        config.observableOneHopRiskLabelSynopsisBytesPerLabel;
    requestHeaderBytes = ...
        config.observableOneHopRiskLabelRequestHeaderBytes;
    requestBytesPerLabel = ...
        config.observableOneHopRiskLabelRequestBytesPerLabel;
    maximumEdits = config.observableOneHopRiskLabelMaximumEdits;
end
neighbors = reshape(find(logical(physicalAdjacency(receiverIdx, :))), 1, []);
neighbors = neighbors(neighbors ~= receiverIdx);
details.physicalNeighborCount = numel(neighbors);
activeThreshold = config.observableOneHopRiskLabelExistenceThreshold;

advertisedBySource = cell(1, sensorCount);
for sourceIdx = neighbors
    active = selectActiveLabels( ...
        localPosteriorBySensor{sourceIdx}, activeThreshold);
    if isempty(active)
        continue;
    end
    bytes = synopsisHeaderBytes + synopsisBytesPerLabel * numel(active);
    details.attemptedSynopsisBytesBySource(sourceIdx) = bytes;
    details.attemptedSynopsisMessageBySource(sourceIdx) = true;
    if directedLinkDelivered(sourceIdx, receiverIdx)
        advertisedBySource{sourceIdx} = active;
        details.deliveredSynopsisBytesBySource(sourceIdx) = bytes;
        details.deliveredSynopsisMessageBySource(sourceIdx) = true;
    end
end

if learnedTriggered
    policy = loadLearnedSequentialPolicy( ...
        config.learnedOneHopSafeLabelPolicyPath);
    if maximumEdits ~= policy.maximumSequentialEdits || ...
            ~strcmp(config.learnedOneHopSafeLabelModelPath, ...
                policy.firstModelPath)
        error('LearnedOneHopSafeLabelV169:PolicyConfigurationDrift', ...
            'The registered sequential policy and runtime config differ.');
    end
    firstClassifier = loadLearnedClassifier( ...
        policy.firstModelPath, true);
    [rawCandidates, sourceFeatureNames] = buildLearnedCandidates( ...
        posterior, advertisedBySource, neighbors, receiverIdx, ...
        currentTime, model, config, physicalAdjacency, firstClassifier);
    candidates = shortlistLearnedCandidates( ...
        rawCandidates, sourceFeatureNames, 1);
    details.rawCandidateLabelCount = numel(rawCandidates);
    details.candidateLabelCount = numel(candidates);
    first = selectFirstLearnedCandidate(candidates);
    if isempty(first)
        finalizeByteDetails();
        return;
    end

    beforeFirst = posterior;
    appendSelectedCandidate(first);
    [posterior, firstApplied] = requestAndApplySingle(first);
    if firstApplied && maximumEdits >= 2
        firstAction = struct( ...
            'label', first.label, ...
            'source', first.source, ...
            'sourceObject', first.sourceObject, ...
            'safetyScore', first.selectionScore, ...
            'payloadBytes', first.payloadBytes);
        secondClassifier = loadLearnedClassifier( ...
            policy.secondModelPath, false);
        [conditional, pressure] = buildConditionalLearnedCandidates( ...
            beforeFirst, posterior, firstAction, candidates, ...
            advertisedBySource, neighbors, receiverIdx, currentTime, ...
            model, config, physicalAdjacency, secondClassifier, policy);
        details.conditionalCandidateLabelCount = numel(conditional);
        details.cardinalityPressureExpectedFraction = ...
            pressure.expectedCardinalityFraction;
        details.cardinalityPressureMapProbability = pressure.mapProbability;
        details.cardinalityPressureGatePassed = pressure.passed;
        if pressure.passed
            second = selectSecondLearnedCandidate(conditional);
            if ~isempty(second)
                appendSelectedCandidate(second);
                [posterior, ~] = requestAndApplySingle(second);
            end
        end
    end
    details.appliedLabelCount = size(details.appliedLabels, 2);
    details.applied = details.appliedLabelCount > 0;
    finalizeByteDetails();
    return;
end

candidates = buildCandidates( ...
    posterior, advertisedBySource, neighbors, receiverIdx, ...
    currentTime, model);
details.candidateLabelCount = numel(candidates);
if isempty(candidates)
    finalizeByteDetails();
    return;
end
score = [candidates.riskReduction]';
eligibleOrder = find(score > ...
    config.observableOneHopRiskLabelMinimumRiskReduction + 1e-12);
labelRows = reshape([candidates.label], 2, [])';
sourceIds = [candidates.source]';
ranking = [-score, labelRows, sourceIds];
[~, order] = sortrows(ranking, 1:size(ranking, 2));
order = order(ismember(order, eligibleOrder));
selectedIndices = zeros(1, 0);
selectedLabels = zeros(2, 0);
for candidateIdx = reshape(order, 1, [])
    label = candidates(candidateIdx).label;
    if ~isempty(selectedLabels) && ...
            any(all(bsxfun(@eq, selectedLabels, label), 1))
        continue;
    end
    selectedIndices(end + 1) = candidateIdx; %#ok<AGROW>
    selectedLabels(:, end + 1) = label; %#ok<AGROW>
    if numel(selectedIndices) >= maximumEdits
        break;
    end
end
selected = candidates(selectedIndices);
details.selectedLabelCount = numel(selected);
if isempty(selected)
    finalizeByteDetails();
    return;
end
details.selectedLabels = reshape([selected.label], 2, []);
details.selectedSources = reshape([selected.source], 1, []);
details.selectedRiskReduction = reshape( ...
    [selected.riskReduction], 1, []);
details.selectedSafetyScore = reshape( ...
    [selected.selectionScore], 1, []);

sources = unique(details.selectedSources, 'stable');
for sourceIdx = sources
    selectedMask = details.selectedSources == sourceIdx;
    requested = selected(selectedMask);
    requestBytes = requestHeaderBytes + ...
        requestBytesPerLabel * numel(requested);
    details.attemptedRequestBytesBySource(sourceIdx) = requestBytes;
    details.attemptedRequestMessageBySource(sourceIdx) = true;
    if ~directedLinkDelivered(receiverIdx, sourceIdx)
        continue;
    end
    details.deliveredRequestBytesBySource(sourceIdx) = requestBytes;
    details.deliveredRequestMessageBySource(sourceIdx) = true;

    responseObjects = reshape([requested.sourceObject], 1, []);
    responseStats = estimateLmbPayloadSize( ...
        responseObjects, model, 2, struct());
    if responseStats.objectCount ~= numel(responseObjects)
        error('ObservableOneHopRiskLabel:PayloadThresholdDrift', ...
            'A selected complete label was omitted from its response.');
    end
    details.attemptedResponseBytesBySource(sourceIdx) = ...
        responseStats.estimatedBytes;
    details.attemptedResponseMessageBySource(sourceIdx) = true;
    if ~directedLinkDelivered(sourceIdx, receiverIdx)
        continue;
    end
    details.deliveredResponseBytesBySource(sourceIdx) = ...
        responseStats.estimatedBytes;
    details.deliveredResponseMessageBySource(sourceIdx) = true;
    for objectIdx = 1:numel(responseObjects)
        posterior = replaceLabelObject(posterior, ...
            responseObjects(objectIdx));
        details.appliedLabels(:, end + 1) = [ ...
            responseObjects(objectIdx).birthTime; ...
            responseObjects(objectIdx).birthLocation]; %#ok<AGROW>
        details.appliedSources(end + 1) = sourceIdx; %#ok<AGROW>
    end
end
details.appliedLabelCount = size(details.appliedLabels, 2);
details.applied = details.appliedLabelCount > 0;
finalizeByteDetails();

    function appendSelectedCandidate(candidate)
        details.selectedLabels(:, end + 1) = candidate.label;
        details.selectedSources(end + 1) = candidate.source;
        details.selectedRiskReduction(end + 1) = ...
            candidate.riskReduction;
        details.selectedSafetyScore(end + 1) = ...
            candidate.selectionScore;
        details.selectedLabelCount = size(details.selectedLabels, 2);
    end

    function [updatedPosterior, applied] = ...
            requestAndApplySingle(candidate)
        updatedPosterior = posterior;
        applied = false;
        sourceIdx = candidate.source;
        requestBytes = requestHeaderBytes + requestBytesPerLabel;
        details.attemptedRequestBytesBySource(sourceIdx) = ...
            details.attemptedRequestBytesBySource(sourceIdx) + requestBytes;
        details.attemptedRequestMessageBySource(sourceIdx) = true;
        if ~directedLinkDelivered(receiverIdx, sourceIdx)
            return;
        end
        details.deliveredRequestBytesBySource(sourceIdx) = ...
            details.deliveredRequestBytesBySource(sourceIdx) + requestBytes;
        details.deliveredRequestMessageBySource(sourceIdx) = true;
        responseStats = estimateLmbPayloadSize( ...
            candidate.sourceObject, model, 2, struct());
        if responseStats.objectCount ~= 1
            error('ObservableOneHopRiskLabel:PayloadThresholdDrift', ...
                'A selected complete label was omitted from its response.');
        end
        details.attemptedResponseBytesBySource(sourceIdx) = ...
            details.attemptedResponseBytesBySource(sourceIdx) + ...
            responseStats.estimatedBytes;
        details.attemptedResponseMessageBySource(sourceIdx) = true;
        if ~directedLinkDelivered(sourceIdx, receiverIdx)
            return;
        end
        details.deliveredResponseBytesBySource(sourceIdx) = ...
            details.deliveredResponseBytesBySource(sourceIdx) + ...
            responseStats.estimatedBytes;
        details.deliveredResponseMessageBySource(sourceIdx) = true;
        updatedPosterior = replaceLabelObject( ...
            updatedPosterior, candidate.sourceObject);
        details.appliedLabels(:, end + 1) = candidate.label;
        details.appliedSources(end + 1) = sourceIdx;
        applied = true;
    end

    function finalizeByteDetails()
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
    end
end

function candidates = buildCandidates( ...
        receiverPosterior, advertisedBySource, neighbors, receiverIdx, ...
        currentTime, model)
labels = zeros(2, 0);
for sourceIdx = neighbors
    objects = advertisedBySource{sourceIdx};
    for objectIdx = 1:numel(objects)
        label = objectLabel(objects(objectIdx));
        if isempty(labels) || ~any(all(bsxfun(@eq, labels, label), 1))
            labels(:, end + 1) = label; %#ok<AGROW>
        end
    end
end
if ~isempty(labels)
    labels = sortrows(labels')';
end

candidates = repmat(emptyCandidate(), 1, 0);
for labelIdx = 1:size(labels, 2)
    label = labels(:, labelIdx);
    best = emptyCandidate();
    for sourceIdx = neighbors
        object = findLabelObject(advertisedBySource{sourceIdx}, label);
        if isempty(object)
            continue;
        end
        sourceRisk = computeObservableLmbLabelBayesRisk(object, model);
        if isempty(best.sourceObject) || ...
                sourceRisk < best.sourceRisk - 1e-12 || ...
                (abs(sourceRisk - best.sourceRisk) <= 1e-12 && ...
                 sourceIdx < best.source)
            best.label = label;
            best.source = sourceIdx;
            best.sourceObject = object;
            best.sourceRisk = sourceRisk;
        end
    end
    if isempty(best.sourceObject)
        continue;
    end
    receiverObject = findLabelObject(receiverPosterior, label);
    receiverRisk = computeObservableLmbLabelBayesRisk( ...
        receiverObject, model);
    best.receiverRisk = receiverRisk;
    best.riskReduction = receiverRisk - best.sourceRisk;
    best.receiver = receiverIdx;
    best.currentTime = currentTime;
    candidates(end + 1) = best; %#ok<AGROW>
end
end

function [candidates, featureNames] = buildLearnedCandidates( ...
        receiverPosterior, advertisedBySource, neighbors, receiverIdx, ...
        currentTime, model, config, physicalAdjacency, classifier)
if ~isfield(model, 'dynamicTopologyScenario') || ...
        ~isstruct(model.dynamicTopologyScenario) || ...
        ~isfield(model.dynamicTopologyScenario, 'config') || ...
        ~isfield(model.dynamicTopologyScenario.config, 'sensorGroupIds')
    error('LearnedOneHopSafeLabelV169:MissingFormationContext', ...
        'The learned selector requires registered sensor formation IDs.');
end
groupIds = reshape( ...
    model.dynamicTopologyScenario.config.sensorGroupIds, 1, []);
nodeCount = numel(groupIds);
if receiverIdx > nodeCount || ...
        ~isequal(size(physicalAdjacency), [nodeCount, nodeCount])
    error('LearnedOneHopSafeLabelV169:FormationContextDrift', ...
        'The learned selector formation context changed shape.');
end
receiverActiveCount = numel(selectActiveLabels( ...
    receiverPosterior, config.observableOneHopRiskLabelExistenceThreshold));
candidates = repmat(emptyCandidate(), 1, 0);
featureNames = cell(1, 0);
for sourceIdx = neighbors
    sourceObjects = advertisedBySource{sourceIdx};
    for objectIdx = 1:numel(sourceObjects)
        sourceObject = sourceObjects(objectIdx);
        label = objectLabel(sourceObject);
        peerObjects = objectsForLabel( ...
            advertisedBySource, neighbors, label, sourceIdx);
        receiverObject = findLabelObject(receiverPosterior, label);
        featureContext = struct( ...
            'receiverFormation', groupIds(receiverIdx), ...
            'sourceFormation', groupIds(sourceIdx), ...
            'nodeCount', nodeCount, ...
            'physicalNeighborCount', nnz( ...
                physicalAdjacency(receiverIdx, :)), ...
            'receiverActiveLabelCount', receiverActiveCount, ...
            'sourceActiveLabelCount', numel(sourceObjects), ...
            'formationSize', nnz(groupIds == groupIds(receiverIdx)));
        [features, currentFeatureNames, featureDetails] = ...
            computeObservableOneHopLabelActionFeatures( ...
                receiverObject, sourceObject, peerObjects, model, ...
                receiverIdx, sourceIdx, currentTime, featureContext);
        expectedSourceNames = classifier.featureNames;
        if isfield(classifier, 'sourceFeatureNames')
            expectedSourceNames = classifier.sourceFeatureNames;
        end
        if ~isequal(currentFeatureNames, expectedSourceNames)
            error('LearnedOneHopSafeLabelV169:FeatureContractDrift', ...
                'The runtime action feature names differ from V168.');
        end
        if isempty(featureNames)
            featureNames = currentFeatureNames;
        elseif ~isequal(featureNames, currentFeatureNames)
            error('LearnedOneHopSafeLabelV169:FeatureContractDrift', ...
                'The action feature names changed within a receiver cell.');
        end
        [lowerProbability, inSupport] = ...
            scoreLearnedClassifier(classifier, features);
        candidate = emptyCandidate();
        candidate.label = label;
        candidate.receiver = receiverIdx;
        candidate.source = sourceIdx;
        candidate.currentTime = currentTime;
        candidate.sourceObject = sourceObject;
        candidate.sourceRisk = ...
            computeObservableLmbLabelBayesRisk(sourceObject, model);
        candidate.receiverRisk = ...
            computeObservableLmbLabelBayesRisk(receiverObject, model);
        candidate.riskReduction = ...
            candidate.receiverRisk - candidate.sourceRisk;
        candidate.features = features;
        candidate.payloadBytes = featureDetails.payloadBytes;
        candidate.sourceEvidenceQuality = featureValue( ...
            features, currentFeatureNames, 'source_evidence_quality');
        candidate.safetyProbability = lowerProbability;
        if inSupport && all(lowerProbability > ...
                classifier.probabilityThreshold)
            candidate.selectionScore = min(lowerProbability);
        end
        candidates(end + 1) = candidate; %#ok<AGROW>
    end
end
end

function shortlisted = shortlistLearnedCandidates( ...
        candidates, featureNames, count)
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

function selected = selectFirstLearnedCandidate(candidates)
selected = repmat(emptyCandidate(), 1, 0);
if isempty(candidates)
    return;
end
score = [candidates.selectionScore]';
eligible = find(isfinite(score));
if isempty(eligible)
    return;
end
labels = reshape([candidates(eligible).label], 2, [])';
sources = [candidates(eligible).source]';
[~, order] = sortrows( ...
    [-score(eligible), labels, sources], [1, 2, 3, 4]);
selected = candidates(eligible(order(1)));
end

function selected = selectSecondLearnedCandidate(candidates)
selected = repmat(emptyCandidate(), 1, 0);
if isempty(candidates)
    return;
end
score = [candidates.selectionScore]';
eligible = find(isfinite(score));
if isempty(eligible)
    return;
end
evidence = [candidates(eligible).sourceEvidenceQuality]';
labels = reshape([candidates(eligible).label], 2, [])';
sources = [candidates(eligible).source]';
[~, order] = sortrows( ...
    [evidence, -score(eligible), labels, sources], [1, 2, 3, 4, 5]);
selected = candidates(eligible(order(1)));
end

function [candidates, pressure] = buildConditionalLearnedCandidates( ...
        receiverBeforeFirst, receiverAfterFirst, firstAction, ...
        firstShortlist, advertisedBySource, neighbors, receiverIdx, ...
        currentTime, model, config, physicalAdjacency, classifier, policy)
groupIds = reshape( ...
    model.dynamicTopologyScenario.config.sensorGroupIds, 1, []);
nodeCount = numel(groupIds);
activeThreshold = config.observableOneHopRiskLabelExistenceThreshold;
receiverActiveCount = numel(selectActiveLabels( ...
    receiverAfterFirst, activeThreshold));
candidates = repmat(emptyCandidate(), 1, 0);
pressure = emptyPressureReadout();
for firstCandidate = reshape(firstShortlist, 1, [])
    if isequal(firstCandidate.label, firstAction.label)
        continue;
    end
    sourceIdx = firstCandidate.source;
    sourceObject = findLabelObject( ...
        advertisedBySource{sourceIdx}, firstCandidate.label);
    if isempty(sourceObject)
        continue;
    end
    peerObjects = objectsForLabel( ...
        advertisedBySource, neighbors, firstCandidate.label, sourceIdx);
    receiverObject = findLabelObject( ...
        receiverAfterFirst, firstCandidate.label);
    sourceActiveCount = numel(selectActiveLabels( ...
        advertisedBySource{sourceIdx}, activeThreshold));
    featureContext = struct( ...
        'receiverFormation', groupIds(receiverIdx), ...
        'sourceFormation', groupIds(sourceIdx), ...
        'nodeCount', nodeCount, ...
        'physicalNeighborCount', nnz(physicalAdjacency(receiverIdx, :)), ...
        'receiverActiveLabelCount', receiverActiveCount, ...
        'sourceActiveLabelCount', sourceActiveCount, ...
        'formationSize', nnz(groupIds == groupIds(receiverIdx)), ...
        'activeExistenceThreshold', activeThreshold);
    [features, featureNames, featureDetails] = ...
        computeObservableConditionalOneHopLabelActionFeatures( ...
            receiverBeforeFirst, receiverAfterFirst, firstAction, ...
            sourceObject, peerObjects, model, receiverIdx, sourceIdx, ...
            currentTime, featureContext);
    expectedSourceNames = classifier.featureNames;
    if isfield(classifier, 'sourceFeatureNames')
        expectedSourceNames = classifier.sourceFeatureNames;
    end
    if ~isequal(featureNames, expectedSourceNames)
        error('LearnedOneHopSafeLabelV169:ConditionalFeatureContractDrift', ...
            'The runtime conditional features differ from V174.');
    end
    if isnan(pressure.expectedCardinalityFraction)
        pressure.expectedCardinalityFraction = ...
            featureDetails.expectedCardinalityFraction;
        pressure.mapProbability = featureDetails.mapProbability;
        pressure.passed = ...
            pressure.expectedCardinalityFraction >= ...
                policy.thresholds.minimumExpectedCardinalityFraction && ...
            pressure.mapProbability >= ...
                policy.thresholds.minimumMapProbability;
    end
    [lowerProbability, ~] = ...
        scoreLearnedClassifier(classifier, features);
    candidate = emptyCandidate();
    candidate.label = firstCandidate.label;
    candidate.receiver = receiverIdx;
    candidate.source = sourceIdx;
    candidate.currentTime = currentTime;
    candidate.sourceObject = sourceObject;
    candidate.sourceRisk = ...
        computeObservableLmbLabelBayesRisk(sourceObject, model);
    candidate.receiverRisk = ...
        computeObservableLmbLabelBayesRisk(receiverObject, model);
    candidate.riskReduction = ...
        candidate.receiverRisk - candidate.sourceRisk;
    candidate.features = features;
    candidate.payloadBytes = featureDetails.payloadBytes;
    candidate.sourceEvidenceQuality = featureValue( ...
        features, featureNames, 'source_evidence_quality');
    candidate.safetyProbability = lowerProbability;
    if all(lowerProbability > classifier.probabilityThreshold)
        candidate.selectionScore = min(lowerProbability);
    end
    candidates(end + 1) = candidate; %#ok<AGROW>
end
end

function values = candidateFeatureColumn(candidates, names, name)
idx = find(strcmp(names, name), 1);
if isempty(idx)
    error('LearnedOneHopSafeLabelV169:MissingShortlistFeature', ...
        'Missing learned shortlist feature: %s.', name);
end
matrix = vertcat(candidates.features);
values = matrix(:, idx)';
end

function value = featureValue(features, names, name)
idx = find(strcmp(names, name), 1);
if isempty(idx)
    error('LearnedOneHopSafeLabelV169:MissingPolicyFeature', ...
        'Missing learned policy feature: %s.', name);
end
value = features(idx);
end

function objects = objectsForLabel( ...
        advertisedBySource, neighbors, label, excludedSource)
if nargin < 4
    excludedSource = 0;
end
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

function policy = loadLearnedSequentialPolicy(path)
persistent cachedPath cachedPolicy
if isempty(cachedPath) || ~strcmp(cachedPath, path)
    loaded = load(path, 'policy');
    required = {'contractVersion', 'firstModelPath', ...
        'secondModelPath', 'thresholds', 'maximumSequentialEdits', ...
        'synopsisBytesPerLabel', 'featuresUseTruth', ...
        'numericLabelIdentifiersUsedAsFeatures', 'preflightGatePassed'};
    if ~isfield(loaded, 'policy') || ~isstruct(loaded.policy) || ...
            ~isscalar(loaded.policy) || ...
            ~all(isfield(loaded.policy, required)) || ...
            ~strcmp(loaded.policy.contractVersion, ...
                'cardinality-pressure-relief-policy-v175-v1') || ...
            loaded.policy.featuresUseTruth || ...
            loaded.policy.numericLabelIdentifiersUsedAsFeatures || ...
            ~loaded.policy.preflightGatePassed || ...
            loaded.policy.maximumSequentialEdits ~= 2 || ...
            ~isfield(loaded.policy.thresholds, ...
                'minimumExpectedCardinalityFraction') || ...
            ~isfield(loaded.policy.thresholds, 'minimumMapProbability') || ...
            exist(loaded.policy.firstModelPath, 'file') ~= 2 || ...
            exist(loaded.policy.secondModelPath, 'file') ~= 2
        error('LearnedOneHopSafeLabelV169:InvalidSequentialPolicy', ...
            'The registered V175 sequential policy contract is invalid.');
    end
    cachedPath = path;
    cachedPolicy = loaded.policy;
end
policy = cachedPolicy;
end

function classifier = loadLearnedClassifier(path, requireSafetyGate)
if nargin < 2
    requireSafetyGate = true;
end
persistent cachedPaths cachedModels
if isempty(cachedPaths)
    cachedPaths = cell(1, 0);
    cachedModels = cell(1, 0);
end
cachedIdx = find(strcmp(cachedPaths, path), 1);
if isempty(cachedIdx)
    loaded = load(path, 'model');
    if ~isfield(loaded, 'model') || ...
            ~isstruct(loaded.model) || ...
            ~isfield(loaded.model, 'contractVersion') || ...
            ~strcmp(loaded.model.contractVersion, ...
                'nonlinear-safe-one-hop-label-action-model-v168-v1') || ...
            ~isfield(loaded.model, 'featuresUseTruth') || ...
            loaded.model.featuresUseTruth || ...
            ~isfield(loaded.model, ...
                'numericLabelIdentifiersUsedAsFeatures') || ...
            loaded.model.numericLabelIdentifiersUsedAsFeatures || ...
            ~isfield(loaded.model, 'heldoutSafetyGatePassed') || ...
            ~isfield(loaded.model, 'maximumEditsPerReceiver') || ...
            loaded.model.maximumEditsPerReceiver < 1
        error('LearnedOneHopSafeLabelV169:InvalidModel', ...
            'The registered V168 classifier contract is invalid.');
    end
    cachedPaths{end + 1} = path;
    cachedModels{end + 1} = loaded.model;
    cachedIdx = numel(cachedPaths);
end
classifier = cachedModels{cachedIdx};
if requireSafetyGate && ~classifier.heldoutSafetyGatePassed
    error('LearnedOneHopSafeLabelV169:UnsafeFirstModel', ...
        'The first-action classifier did not pass its heldout safety gate.');
end
end

function [lowerProbability, inSupport] = ...
        scoreLearnedClassifier(classifier, features)
if isfield(classifier, 'featureMask')
    features = features(logical(classifier.featureMask));
end
z = (features - classifier.featureMean) ./ classifier.featureScale;
inSupport = all(z >= classifier.supportLower & ...
    z <= classifier.supportUpper);
memberCount = numel(classifier.members);
probability = zeros(memberCount, 3);
for memberIdx = 1:memberCount
    parameters = classifier.members{memberIdx}.parameters;
    hidden = tanh(z * parameters{1} + parameters{2});
    logits = hidden * parameters{3} + parameters{4};
    probability(memberIdx, :) = stableSigmoid(logits);
end
lowerProbability = mean(probability, 1) - ...
    classifier.uncertaintyPenalty * std(probability, 0, 1);
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

function objects = selectActiveLabels(objects, threshold)
objects = reshape(objects, 1, []);
if isempty(objects)
    return;
end
active = [objects.r] > threshold & ...
    [objects.numberOfGmComponents] > 0;
objects = objects(active);
end

function posterior = replaceLabelObject(posterior, object)
idx = findLabelIndex(posterior, objectLabel(object));
if idx == 0
    posterior(end + 1) = object; %#ok<AGROW>
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

function candidate = emptyCandidate()
candidate = struct( ...
    'label', zeros(2, 1), ...
    'receiver', 0, ...
    'source', 0, ...
    'currentTime', 0, ...
    'sourceObject', [], ...
    'sourceRisk', inf, ...
    'receiverRisk', inf, ...
    'riskReduction', -inf, ...
    'features', zeros(1, 0), ...
    'payloadBytes', 0, ...
    'sourceEvidenceQuality', inf, ...
    'safetyProbability', zeros(1, 0), ...
    'selectionScore', -inf);
end

function pressure = emptyPressureReadout()
pressure = struct( ...
    'expectedCardinalityFraction', NaN, ...
    'mapProbability', NaN, ...
    'passed', false);
end

function details = emptyDetails(sensorCount)
details = struct( ...
    'contractVersion', 'observable-one-hop-risk-label-transfer-v162-v1', ...
    'sourceRule', 'minimum-current-label-posterior-bayes-risk', ...
    'labelRule', 'positive-receiver-minus-source-risk-top-k', ...
    'selectorMode', 'analytic-risk-reduction', ...
    'triggered', false, ...
    'applied', false, ...
    'pageIndex', 0, ...
    'physicalNeighborCount', 0, ...
    'rawCandidateLabelCount', 0, ...
    'candidateLabelCount', 0, ...
    'conditionalCandidateLabelCount', 0, ...
    'selectedLabelCount', 0, ...
    'appliedLabelCount', 0, ...
    'selectedLabels', zeros(2, 0), ...
    'selectedSources', zeros(1, 0), ...
    'selectedRiskReduction', zeros(1, 0), ...
    'selectedSafetyScore', zeros(1, 0), ...
    'cardinalityPressureExpectedFraction', NaN, ...
    'cardinalityPressureMapProbability', NaN, ...
    'cardinalityPressureGatePassed', false, ...
    'appliedLabels', zeros(2, 0), ...
    'appliedSources', zeros(1, 0), ...
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
    'attemptedSynopsisBytes', 0, ...
    'deliveredSynopsisBytes', 0, ...
    'attemptedRequestBytes', 0, ...
    'deliveredRequestBytes', 0, ...
    'attemptedResponseBytes', 0, ...
    'deliveredResponseBytes', 0, ...
    'attemptedTotalBytes', 0, ...
    'deliveredTotalBytes', 0, ...
    'truthUsed', false, ...
    'futureInformationUsed', false);
end
