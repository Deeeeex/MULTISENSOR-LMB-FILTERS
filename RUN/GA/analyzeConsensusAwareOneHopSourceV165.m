function [reportPath, summary] = ...
        analyzeConsensusAwareOneHopSourceV165(options)
% ANALYZECONSENSUSAWAREONEHOPSOURCEV165 Observable source-ranking screen.
%
% Enumerate complete one-hop source-label candidates at the opened V163
% F3/F5 cells.  Source rankers use only present receiver/source posteriors
% and agreement among current physical neighbors.  Truth evaluates the
% selected Top-4 complete-label replacements but is never a ranker input.

if nargin < 1 || isempty(options)
    options = struct();
end
v157 = getPositiveValueReferenceLabelV157Protocol();
v162 = getObservableOneHopRiskLabelV162Protocol();
snapshotPath = getField(options, 'snapshotScreenPath', fullfile( ...
    v157.headroomOutputRoot, 'x36_t72_h8', 'screen', ...
    ['TRACKING_ALIGNED_X36_SCHEDULE_H8_', ...
     'X36_FORMATION_FOV_SEED211_T72_D8.mat']));
recursivePath = getField(options, 'recursiveScreenPath', fullfile( ...
    v162.headroomOutputRoot, 'x36_t72_h8', 'screen', ...
    ['TRACKING_ALIGNED_X36_SCHEDULE_H8_', ...
     'X36_FORMATION_FOV_SEED211_T72_D8.mat']));
outputRoot = getField(options, 'outputRoot', fullfile( ...
    v162.headroomOutputRoot, 'consensus_source_preflight'));
if exist(snapshotPath, 'file') ~= 2 || exist(recursivePath, 'file') ~= 2
    error('ConsensusSourceV165:MissingInput', ...
        'V157 snapshots and the V162 recursive screen are required.');
end

snapshotLoaded = load(snapshotPath, 'screen');
recursiveLoaded = load(recursivePath, 'screen');
snapshotScreen = snapshotLoaded.screen;
recursiveScreen = recursiveLoaded.screen;
snapshotOutcome = outcomeByAction(snapshotScreen, ...
    v157.candidateActionName);
reference = outcomeByAction(recursiveScreen, ...
    'reference-full-payload');
base = outcomeByAction(recursiveScreen, v162.candidateActionName);
inputs = generateDynamicTopologyScenarioInputs( ...
    snapshotScreen.presetName, snapshotScreen.seed);
model = inputs.model;
model.dynamicTopologyScenario.targetTrajectories = ...
    inputs.targetTrajectories;
groupIds = reshape(snapshotScreen.sensorGroupIds, 1, []);
nodeCount = numel(groupIds);
groups = unique(groupIds, 'stable');
policies = buildPolicies();
repairPages = [5, 7, 8];
repairFormationsByPage = {[3], [3, 5], [3, 5]};
rows = repmat(emptyRow(numel(policies)), 1, 0);

for scheduleIdx = 1:numel(repairPages)
    pageIdx = repairPages(scheduleIdx);
    currentTime = snapshotScreen.returnTimes(pageIdx);
    receivers = find(ismember( ...
        groupIds, repairFormationsByPage{scheduleIdx}));
    physical = logical(inputs.graphData. ...
        physicalAdjacency(:, :, currentTime));
    physical = physical | physical';
    physical(1:nodeCount+1:end) = false;
    for receiverIdx = reshape(receivers, 1, [])
        baseline = snapshotOutcome. ...
            fusedPosteriorSnapshotsByTime{pageIdx}{receiverIdx};
        locals = snapshotOutcome. ...
            localPosteriorSnapshotsByTime{pageIdx};
        neighbors = reshape(find(physical(receiverIdx, :)), 1, []);
        config = selectorConfig(v162, currentTime, receiverIdx);
        [v162Trial, v162Details] = selectObservableOneHopRiskLabels( ...
            baseline, locals, physical, physical, config, ...
            receiverIdx, currentTime, model);
        candidates = buildCandidates( ...
            baseline, locals, neighbors, receiverIdx, currentTime, ...
            model, v162.activeExistenceThreshold);
        row = emptyRow(numel(policies));
        row.page = pageIdx;
        row.time = currentTime;
        row.formation = groupIds(receiverIdx);
        row.receiver = receiverIdx;
        row.candidateCount = numel(candidates);
        row.baselineEospa = evaluateLmbTopologyCurrentEospa( ...
            baseline, model, currentTime, struct());
        row.baselineRmse = currentPosteriorRmse( ...
            baseline, model, currentTime);
        for policyIdx = 1:numel(policies)
            selected = selectCandidates( ...
                candidates, policies(policyIdx), ...
                min(v162.maximumLabelEdits, ...
                    policies(policyIdx).maximumEdits), ...
                v162.minimumRiskReduction);
            trial = applyCandidates(baseline, selected);
            row.selectedCount(policyIdx) = numel(selected);
            row.attemptedBytes(policyIdx) = ...
                v162Details.attemptedSynopsisBytes + ...
                selectedTransportBytes(selected, v162, model);
            row.eospaGain(policyIdx) = row.baselineEospa - ...
                evaluateLmbTopologyCurrentEospa( ...
                    trial, model, currentTime, struct());
            row.rmseGain(policyIdx) = row.baselineRmse - ...
                currentPosteriorRmse(trial, model, currentTime);
            row.selectedSources{policyIdx} = [selected.source];
            if isempty(selected)
                row.selectedLabels{policyIdx} = zeros(2, 0);
            else
                row.selectedLabels{policyIdx} = ...
                    reshape([selected.label], 2, []);
            end
            if policyIdx == 1
                selectorEospa = evaluateLmbTopologyCurrentEospa( ...
                    v162Trial, model, currentTime, struct());
                selectorRmse = currentPosteriorRmse( ...
                    v162Trial, model, currentTime);
                if abs(selectorEospa - (row.baselineEospa - ...
                        row.eospaGain(policyIdx))) > 1e-9 || ...
                        abs(selectorRmse - (row.baselineRmse - ...
                        row.rmseGain(policyIdx))) > 1e-9 || ...
                        row.attemptedBytes(policyIdx) ~= ...
                        v162Details.attemptedTotalBytes
                    error('ConsensusSourceV165:V162ReplayDrift', ...
                        'The minimum-risk policy differs from V162.');
                end
            end
        end
        rows(end + 1) = row; %#ok<AGROW>
    end
end

policyMetrics = repmat(emptyPolicyMetric(), 1, numel(policies));
for policyIdx = 1:numel(policies)
    eospaGain = arrayfun(@(row) row.eospaGain(policyIdx), rows);
    rmseGain = arrayfun(@(row) row.rmseGain(policyIdx), rows);
    extraBytes = sum(arrayfun(@(row) ...
        row.attemptedBytes(policyIdx), rows));
    projectedFormationEospa = base.formationMeanEospa;
    projectedFormationRmse = base.formationMeanRmse;
    for groupIdx = 1:numel(groups)
        mask = [rows.formation] == groups(groupIdx);
        denominator = nnz(groupIds == groups(groupIdx)) * ...
            numel(snapshotScreen.returnTimes);
        projectedFormationEospa(groupIdx) = ...
            projectedFormationEospa(groupIdx) - sum(eospaGain(mask)) / ...
            denominator;
        projectedFormationRmse(groupIdx) = ...
            projectedFormationRmse(groupIdx) - sum(rmseGain(mask)) / ...
            denominator;
    end
    projectedMeanEospa = base.meanEospa - ...
        sum(eospaGain) / (nodeCount * numel(snapshotScreen.returnTimes));
    projectedMeanRmse = base.meanRmse - ...
        sum(rmseGain) / (nodeCount * numel(snapshotScreen.returnTimes));
    projectedBytes = base.attemptedBytes + extraBytes;
    metric = emptyPolicyMetric();
    metric.name = policies(policyIdx).name;
    metric.totalEospaGain = sum(eospaGain);
    metric.totalRmseGain = sum(rmseGain);
    metric.harmfulEospaCellCount = nnz(eospaGain < -1e-9);
    metric.harmfulRmseCellCount = nnz(rmseGain < -1e-9);
    metric.minimumCellEospaGain = min(eospaGain);
    metric.minimumCellRmseGain = min(rmseGain);
    metric.selectedCount = sum(arrayfun(@(row) ...
        row.selectedCount(policyIdx), rows));
    metric.additionalAttemptedBytes = extraBytes;
    metric.projectedMeanEospaGainPercent = relativeGain( ...
        reference.meanEospa, projectedMeanEospa);
    metric.projectedMeanRmseGainPercent = relativeGain( ...
        reference.meanRmse, projectedMeanRmse);
    metric.projectedFormationEospaGainPercent = 100 * ( ...
        reference.formationMeanEospa - projectedFormationEospa) ./ ...
        max(abs(reference.formationMeanEospa), eps);
    metric.projectedFormationRmseGainPercent = 100 * ( ...
        reference.formationMeanRmse - projectedFormationRmse) ./ ...
        max(abs(reference.formationMeanRmse), eps);
    metric.projectedByteSavingPercent = 100 * ( ...
        reference.attemptedBytes - projectedBytes) / ...
        max(abs(reference.attemptedBytes), eps);
    metric.mechanismGatePassed = ...
        metric.totalEospaGain > 0 && ...
        metric.totalRmseGain > 0 && ...
        metric.harmfulEospaCellCount == 0 && ...
        metric.projectedMeanEospaGainPercent >= 5 - 1e-12 && ...
        metric.projectedMeanRmseGainPercent >= 2 - 1e-12 && ...
        all(metric.projectedFormationEospaGainPercent >= -1e-12) && ...
        all(metric.projectedFormationRmseGainPercent >= -1e-12) && ...
        metric.projectedByteSavingPercent >= -1e-12;
    policyMetrics(policyIdx) = metric;
end

summary = struct();
summary.contractVersion = 'consensus-aware-one-hop-source-v165-v1';
summary.presetName = snapshotScreen.presetName;
summary.seed = snapshotScreen.seed;
summary.repairPages = repairPages;
summary.repairTimes = snapshotScreen.returnTimes(repairPages);
summary.repairFormationsByPage = repairFormationsByPage;
summary.cellCount = numel(rows);
summary.policies = policies;
summary.rows = rows;
summary.policyMetrics = policyMetrics;
summary.truthUsedForEvaluation = true;
summary.truthUsedByPolicies = false;
summary.futureInformationUsedByPolicies = false;
summary.recursiveResultClaimAllowed = false;
summary.evidenceBoundary = [ ...
    'V165 is an opened, nonrecursive source-ranking screen on the ', ...
    'privileged V163 F3/F5 cells. Every ranker enumerates current one-hop ', ...
    'complete Bernoulli GM labels and uses only present receiver/source ', ...
    'Bayes risk, receiver-source position compatibility, and source ', ...
    'agreement across current physical neighbors. Truth evaluates the ', ...
    'resulting complete-label Top-4 replacements through immediate E-OSPA ', ...
    'and matched-position RMSE; it is not a feature. Projected recursive ', ...
    'metrics add these snapshot deltas to V162. Synopsis, request, and ', ...
    'complete response bytes are charged conservatively. Passing only ', ...
    'authorizes an actual recursive source-policy probe and cannot support ', ...
    'an online, validation, or generalization claim.'];

if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
matPath = fullfile(outputRoot, ...
    'CONSENSUS_AWARE_ONE_HOP_SOURCE_V165_PREFLIGHT.mat');
reportPath = fullfile(outputRoot, ...
    'CONSENSUS_AWARE_ONE_HOP_SOURCE_V165_PREFLIGHT.md');
save('-mat7-binary', matPath, 'summary');
writeReport(reportPath, summary);
fprintf('V165 consensus-source preflight: %s\n', reportPath);
end

function policies = buildPolicies()
template = struct( ...
    'name', '', 'sourceRule', '', 'labelRule', '', ...
    'mahalanobisThreshold', inf, 'maximumEdits', 4);
policies = repmat(template, 1, 10);
policies(1) = setPolicy(template, ...
    'minimum-risk-k4', 'risk', 'risk', inf, 4);
policies(2) = setPolicy(template, ...
    'receiver-compatible-source-k4', 'receiver', 'risk', inf, 4);
policies(3) = setPolicy(template, ...
    'neighbor-medoid-source-k4', 'consensus', 'risk', inf, 4);
policies(4) = setPolicy(template, ...
    'maximum-local-support-source-k4', 'credibility', 'risk', inf, 4);
policies(5) = setPolicy(template, ...
    'credibility-weighted-risk-k4', 'product', 'product', inf, 4);
policies(6) = setPolicy(template, ...
    'chi2-95-gated-risk-k4', 'gated-risk', 'risk', 5.991, 4);
policies(7) = setPolicy(template, ...
    'chi2-99-gated-risk-k4', 'gated-risk', 'risk', 9.210, 4);
policies(8) = setPolicy(template, ...
    'credibility-weighted-risk-k1', 'product', 'product', inf, 1);
policies(9) = setPolicy(template, ...
    'credibility-weighted-risk-k2', 'product', 'product', inf, 2);
policies(10) = setPolicy(template, ...
    'credibility-weighted-risk-k3', 'product', 'product', inf, 3);
end

function policy = setPolicy( ...
        template, name, sourceRule, labelRule, threshold, maximumEdits)
policy = template;
policy.name = name;
policy.sourceRule = sourceRule;
policy.labelRule = labelRule;
policy.mahalanobisThreshold = threshold;
policy.maximumEdits = maximumEdits;
end

function candidates = buildCandidates( ...
        receiverPosterior, localPosteriors, neighbors, receiverIdx, ...
        currentTime, model, threshold)
labels = zeros(2, 0);
for sourceIdx = neighbors
    objects = activeObjects(localPosteriors{sourceIdx}, threshold);
    for object = reshape(objects, 1, [])
        label = [object.birthTime; object.birthLocation];
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
    receiver = findLabelObject(receiverPosterior, label);
    receiverRisk = computeObservableLmbLabelBayesRisk(receiver, model);
    receiverMean = zeros(0, 1);
    receiverCovariance = zeros(0);
    if ~isempty(receiver)
        [receiverMean, receiverCovariance] = momentMatch(receiver);
        receiverMean = receiverMean(1:min(2, numel(receiverMean)));
        receiverCovariance = receiverCovariance( ...
            1:numel(receiverMean), 1:numel(receiverMean));
    end
    firstIdx = numel(candidates) + 1;
    for sourceIdx = neighbors
        source = findLabelObject(localPosteriors{sourceIdx}, label);
        if isempty(source) || source.r <= threshold || ...
                source.numberOfGmComponents <= 0
            continue;
        end
        sourceRisk = computeObservableLmbLabelBayesRisk(source, model);
        candidate = emptyCandidate();
        candidate.label = label;
        candidate.source = sourceIdx;
        candidate.object = source;
        candidate.receiverPresent = ~isempty(receiver);
        candidate.riskReduction = receiverRisk - sourceRisk;
        [sourceMean, sourceCovariance] = momentMatch(source);
        candidate.positionMean = sourceMean(1:min(2, numel(sourceMean)));
        candidate.positionCovariance = sourceCovariance( ...
            1:numel(candidate.positionMean), ...
            1:numel(candidate.positionMean));
        if candidate.receiverPresent
            candidate.receiverMahalanobis = ...
                summaryMahalanobis(receiverMean, receiverCovariance, ...
                    candidate.positionMean, ...
                    candidate.positionCovariance);
            candidate.receiverCompatibility = exp(-0.5 * min( ...
                candidate.receiverMahalanobis, 100));
        end
        candidates(end + 1) = candidate; %#ok<AGROW>
    end
    lastIdx = numel(candidates);
    for candidateIdx = firstIdx:lastIdx
        others = setdiff(firstIdx:lastIdx, candidateIdx);
        if isempty(others)
            consensusCompatibility = 0;
            consensusSupport = 0;
        else
            compatibility = zeros(1, numel(others));
            support = false(1, numel(others));
            for otherIdx = 1:numel(others)
                distance = summaryMahalanobis( ...
                    candidates(candidateIdx).positionMean, ...
                    candidates(candidateIdx).positionCovariance, ...
                    candidates(others(otherIdx)).positionMean, ...
                    candidates(others(otherIdx)).positionCovariance);
                compatibility(otherIdx) = exp(-0.5 * min(distance, 100));
                support(otherIdx) = distance <= 9.210 + 1e-12;
            end
            consensusCompatibility = mean(compatibility);
            consensusSupport = mean(support);
        end
        candidates(candidateIdx).consensusCompatibility = ...
            consensusCompatibility;
        candidates(candidateIdx).consensusSupport = consensusSupport;
        if candidates(candidateIdx).receiverPresent
            candidates(candidateIdx).spatialCredibility = max( ...
                candidates(candidateIdx).receiverCompatibility, ...
                consensusCompatibility);
        else
            candidates(candidateIdx).spatialCredibility = ...
                consensusCompatibility;
        end
    end
end
end

function selected = selectCandidates( ...
        candidates, policy, maximumEdits, minimumRiskReduction)
selected = repmat(emptyCandidate(), 1, 0);
if isempty(candidates)
    return;
end
labels = unique(reshape([candidates.label], 2, [])', 'rows', 'stable')';
perLabel = repmat(emptyCandidate(), 1, 0);
labelScores = zeros(1, 0);
for labelIdx = 1:size(labels, 2)
    label = labels(:, labelIdx);
    mask = arrayfun(@(item) isequal(item.label, label), candidates) & ...
        [candidates.riskReduction] > minimumRiskReduction + 1e-12;
    local = candidates(mask);
    if isempty(local)
        continue;
    end
    switch policy.sourceRule
        case 'risk'
            primary = [local.riskReduction];
        case 'receiver'
            primary = arrayfun(@receiverOrConsensus, local);
        case 'consensus'
            primary = [local.consensusCompatibility];
        case 'credibility'
            primary = [local.spatialCredibility];
        case 'product'
            primary = [local.riskReduction] .* ...
                sqrt(max([local.spatialCredibility], 0));
        case 'gated-risk'
            eligible = arrayfun(@(item) gated(item, ...
                policy.mahalanobisThreshold), local);
            local = local(eligible);
            if isempty(local)
                continue;
            end
            primary = [local.riskReduction];
        otherwise
            error('ConsensusSourceV165:UnknownRule', ...
                'Unknown source rule: %s', policy.sourceRule);
    end
    selectedIdx = stableMaximum( ...
        primary, [local.riskReduction], [local.source]);
    item = local(selectedIdx);
    perLabel(end + 1) = item; %#ok<AGROW>
    if strcmp(policy.labelRule, 'product')
        labelScores(end + 1) = item.riskReduction * ...
            sqrt(max(item.spatialCredibility, 0)); %#ok<AGROW>
    else
        labelScores(end + 1) = item.riskReduction; %#ok<AGROW>
    end
end
if isempty(perLabel)
    return;
end
labelRows = reshape([perLabel.label], 2, [])';
sourceIds = [perLabel.source]';
ranking = [-labelScores', labelRows, sourceIds];
[~, order] = sortrows(ranking, 1:size(ranking, 2));
selected = perLabel(order(1:min(maximumEdits, numel(order))));
end

function value = receiverOrConsensus(item)
if item.receiverPresent
    value = item.receiverCompatibility;
else
    value = item.consensusCompatibility;
end
end

function value = gated(item, threshold)
if item.receiverPresent
    value = item.receiverMahalanobis <= threshold + 1e-12;
else
    value = item.consensusSupport >= 0.5 - 1e-12;
end
end

function idx = stableMaximum(primary, riskReduction, sourceIds)
ranking = [-reshape(primary, [], 1), ...
    -reshape(riskReduction, [], 1), reshape(sourceIds, [], 1)];
[~, order] = sortrows(ranking, 1:size(ranking, 2));
idx = order(1);
end

function posterior = applyCandidates(posterior, selected)
for item = reshape(selected, 1, [])
    posterior = replaceLabelObject(posterior, item.object);
end
end

function bytes = selectedTransportBytes(selected, protocol, model)
bytes = 0;
if isempty(selected)
    return;
end
sources = unique([selected.source], 'stable');
for sourceIdx = sources
    items = selected([selected.source] == sourceIdx);
    bytes = bytes + protocol.requestHeaderBytes + ...
        protocol.requestBytesPerLabel * numel(items);
    objects = reshape([items.object], 1, []);
    stats = estimateLmbPayloadSize(objects, model, 2, struct());
    bytes = bytes + stats.estimatedBytes;
end
end

function objects = activeObjects(objects, threshold)
objects = reshape(objects, 1, []);
if isempty(objects)
    return;
end
objects = objects([objects.r] > threshold & ...
    [objects.numberOfGmComponents] > 0);
end

function value = summaryMahalanobis( ...
        leftMean, leftCovariance, rightMean, rightCovariance)
dimension = min([2, numel(leftMean), numel(rightMean)]);
delta = leftMean(1:dimension) - rightMean(1:dimension);
covariance = leftCovariance(1:dimension, 1:dimension) + ...
    rightCovariance(1:dimension, 1:dimension);
covariance = regularizeCovariance(covariance);
value = max(real(delta' * (covariance \ delta)), 0);
if ~isfinite(value)
    value = inf;
end
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
    meanVector = meanVector + weights(componentIdx) * object.mu{componentIdx};
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
error('ConsensusSourceV165:InvalidCovariance', ...
    'A Bernoulli covariance cannot be regularized.');
end

function config = selectorConfig(protocol, currentTime, receiverIdx)
config = struct( ...
    'observableOneHopRiskLabelEnabled', true, ...
    'observableOneHopRiskLabelTimes', currentTime, ...
    'observableOneHopRiskLabelSensorIdsByTime', {{receiverIdx}}, ...
    'observableOneHopRiskLabelMaximumEdits', protocol.maximumLabelEdits, ...
    'observableOneHopRiskLabelExistenceThreshold', ...
        protocol.activeExistenceThreshold, ...
    'observableOneHopRiskLabelMinimumRiskReduction', ...
        protocol.minimumRiskReduction, ...
    'observableOneHopRiskLabelSynopsisHeaderBytes', ...
        protocol.synopsisHeaderBytes, ...
    'observableOneHopRiskLabelSynopsisBytesPerLabel', ...
        protocol.synopsisBytesPerLabel, ...
    'observableOneHopRiskLabelRequestHeaderBytes', ...
        protocol.requestHeaderBytes, ...
    'observableOneHopRiskLabelRequestBytesPerLabel', ...
        protocol.requestBytesPerLabel);
end

function value = currentPosteriorRmse(posterior, model, currentTime)
objects = reshape(posterior, 1, []);
if ~isempty(objects)
    objects = objects([objects.r] > model.existenceThreshold);
end
estimate = zeros(2, 0);
if ~isempty(objects)
    [cardinality, indices] = lmbMapCardinalityEstimate([objects.r]);
    estimate = zeros(2, cardinality);
    for idx = 1:cardinality
        object = objects(indices(idx));
        weights = reshape(object.w, 1, []);
        weights(~isfinite(weights)) = -inf;
        [~, componentIdx] = max(weights);
        if isempty(componentIdx) || ~isfinite(weights(componentIdx))
            componentIdx = 1;
        end
        estimate(:, idx) = object.mu{componentIdx}(1:2);
    end
end
truth = zeros(2, 0);
trajectories = model.dynamicTopologyScenario.targetTrajectories;
for targetIdx = 1:numel(trajectories)
    if currentTime <= size(trajectories{targetIdx}, 2)
        state = trajectories{targetIdx}(:, currentTime);
        if all(isfinite(state))
            truth(:, end + 1) = state(1:2); %#ok<AGROW>
        end
    end
end
if isempty(truth) || isempty(estimate)
    value = NaN;
    return;
end
distances = zeros(size(truth, 2), size(estimate, 2));
for truthIdx = 1:size(truth, 2)
    delta = bsxfun(@minus, estimate, truth(:, truthIdx));
    distances(truthIdx, :) = sqrt(sum(delta .^ 2, 1));
end
[matching, ~] = Hungarian(distances);
matched = distances(matching == 1);
if isempty(matched)
    value = NaN;
else
    value = sqrt(mean(matched .^ 2));
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

function posterior = replaceLabelObject(posterior, object)
posterior = reshape(posterior, 1, []);
idx = findLabelIndex(posterior, ...
    [object.birthTime; object.birthLocation]);
if idx == 0
    posterior(end + 1) = object;
else
    posterior(idx) = object;
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

function outcome = outcomeByAction(screen, actionName)
idx = find(strcmp({screen.records.actionName}, actionName), 1);
if isempty(idx)
    error('ConsensusSourceV165:MissingAction', ...
        'The requested action is missing: %s', actionName);
end
outcome = screen.outcomes(idx);
end

function value = relativeGain(reference, candidate)
value = 100 * (reference - candidate) / max(abs(reference), eps);
end

function candidate = emptyCandidate()
candidate = struct( ...
    'label', zeros(2, 1), 'source', 0, 'object', [], ...
    'positionMean', zeros(0, 1), 'positionCovariance', zeros(0), ...
    'receiverPresent', false, 'riskReduction', -inf, ...
    'receiverCompatibility', 0, 'receiverMahalanobis', inf, ...
    'consensusCompatibility', 0, 'consensusSupport', 0, ...
    'spatialCredibility', 0, 'sourceEvidence', 0, ...
    'sourceOpportunity', 0);
end

function row = emptyRow(policyCount)
row = struct( ...
    'page', 0, 'time', 0, 'formation', 0, 'receiver', 0, ...
    'candidateCount', 0, 'baselineEospa', NaN, 'baselineRmse', NaN, ...
    'selectedCount', zeros(1, policyCount), ...
    'attemptedBytes', zeros(1, policyCount), ...
    'eospaGain', nan(1, policyCount), ...
    'rmseGain', nan(1, policyCount), ...
    'selectedSources', {cell(1, policyCount)}, ...
    'selectedLabels', {cell(1, policyCount)});
end

function metric = emptyPolicyMetric()
metric = struct( ...
    'name', '', 'selectedCount', 0, 'totalEospaGain', NaN, ...
    'totalRmseGain', NaN, 'harmfulEospaCellCount', 0, ...
    'harmfulRmseCellCount', 0, 'minimumCellEospaGain', NaN, ...
    'minimumCellRmseGain', NaN, 'additionalAttemptedBytes', NaN, ...
    'projectedMeanEospaGainPercent', NaN, ...
    'projectedMeanRmseGainPercent', NaN, ...
    'projectedFormationEospaGainPercent', [], ...
    'projectedFormationRmseGainPercent', [], ...
    'projectedByteSavingPercent', NaN, ...
    'mechanismGatePassed', false);
end

function writeReport(path, summary)
fid = fopen(path, 'w');
if fid < 0
    error('ConsensusSourceV165:ReportOpenFailed', ...
        'Could not open the V165 report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V165 consensus-aware one-hop source preflight\n\n');
fprintf(fid, '- Preset / seed: `%s / %d`\n', ...
    summary.presetName, summary.seed);
fprintf(fid, '- Privileged repair pages / times: `%s / %s`\n', ...
    mat2str(summary.repairPages), mat2str(summary.repairTimes));
fprintf(fid, '- Receiver-time cells: `%d`\n\n', summary.cellCount);
fprintf(fid, ['| Policy | Actions | Immediate E-OSPA / RMSE gain | ', ...
    'Harmful cells E/R | Projected mean gain E/R | ', ...
    'Min formation gain E/R | Bytes | Saving | Gate |\n']);
fprintf(fid, '|:--|--:|:--|:--|:--|:--|--:|--:|:--:|\n');
for metric = summary.policyMetrics
    fprintf(fid, ['| %s | %d | %+.4f / %+.4f | %d / %d | ', ...
        '%+.3f%% / %+.3f%% | %+.3f%% / %+.3f%% | %d | ', ...
        '%+.3f%% | %d |\n'], ...
        metric.name, metric.selectedCount, metric.totalEospaGain, ...
        metric.totalRmseGain, metric.harmfulEospaCellCount, ...
        metric.harmfulRmseCellCount, ...
        metric.projectedMeanEospaGainPercent, ...
        metric.projectedMeanRmseGainPercent, ...
        min(metric.projectedFormationEospaGainPercent), ...
        min(metric.projectedFormationRmseGainPercent), ...
        metric.additionalAttemptedBytes, ...
        metric.projectedByteSavingPercent, metric.mechanismGatePassed);
end
fprintf(fid, '\n## Formation projections\n\n');
for metric = summary.policyMetrics
    fprintf(fid, '- `%s` E-OSPA: `%s%%`; RMSE: `%s%%`\n', ...
        metric.name, ...
        mat2str(metric.projectedFormationEospaGainPercent, 5), ...
        mat2str(metric.projectedFormationRmseGainPercent, 5));
end
fprintf(fid, '\n## Evidence boundary\n\n%s\n', ...
    summary.evidenceBoundary);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
