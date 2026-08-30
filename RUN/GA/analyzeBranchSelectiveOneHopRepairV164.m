function [reportPath, summary] = ...
        analyzeBranchSelectiveOneHopRepairV164(options)
% ANALYZEBRANCHSELECTIVEONEHOPREPAIRV164 Bernoulli-branch safety screen.
%
% Replays the opened F3/F5 V163 cells and keeps the V162 truth-free source
% and label selector fixed.  For a shared label, policies either replace the
% complete Bernoulli GM density or retain the receiver spatial density while
% copying only source existence evidence.  Chi-square policies allow a full
% spatial replacement only when present-time posterior means are compatible.
% Truth evaluates immediate effects and never enters a policy decision.


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
    v162.headroomOutputRoot, 'branch_selective_preflight'));
if exist(snapshotPath, 'file') ~= 2 || exist(recursivePath, 'file') ~= 2
    error('BranchSelectiveV164:MissingInput', ...
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
        config = selectorConfig(v162, currentTime, receiverIdx);
        [fullTrial, details] = selectObservableOneHopRiskLabels( ...
            baseline, locals, physical, physical, config, ...
            receiverIdx, currentTime, model);
        row = emptyRow(numel(policies));
        row.page = pageIdx;
        row.time = currentTime;
        row.formation = groupIds(receiverIdx);
        row.receiver = receiverIdx;
        row.selectedLabelCount = details.selectedLabelCount;
        row.attemptedBytes = details.attemptedTotalBytes;
        row.baselineEospa = evaluateLmbTopologyCurrentEospa( ...
            baseline, model, currentTime, struct());
        row.baselineRmse = currentPosteriorRmse( ...
            baseline, model, currentTime);
        for policyIdx = 1:numel(policies)
            [trial, counts] = applyPolicy( ...
                baseline, locals, details, policies(policyIdx));
            trialEospa = evaluateLmbTopologyCurrentEospa( ...
                trial, model, currentTime, struct());
            trialRmse = currentPosteriorRmse( ...
                trial, model, currentTime);
            row.eospaGain(policyIdx) = row.baselineEospa - trialEospa;
            row.rmseGain(policyIdx) = row.baselineRmse - trialRmse;
            row.fullSpatialCount(policyIdx) = counts.fullSpatial;
            row.existenceOnlyCount(policyIdx) = counts.existenceOnly;
            row.missingReceiverCount(policyIdx) = counts.missingReceiver;
            if policyIdx == 1
                fullEospa = evaluateLmbTopologyCurrentEospa( ...
                    fullTrial, model, currentTime, struct());
                fullRmse = currentPosteriorRmse( ...
                    fullTrial, model, currentTime);
                if abs(trialEospa - fullEospa) > 1e-9 || ...
                        abs(trialRmse - fullRmse) > 1e-9
                    error('BranchSelectiveV164:FullReplayDrift', ...
                        'The complete-label replay differs from V162.');
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
    attemptedBytes = base.attemptedBytes + sum([rows.attemptedBytes]);
    metric = emptyPolicyMetric();
    metric.name = policies(policyIdx).name;
    metric.totalEospaGain = sum(eospaGain);
    metric.totalRmseGain = sum(rmseGain);
    metric.harmfulEospaCellCount = nnz(eospaGain < -1e-9);
    metric.harmfulRmseCellCount = nnz(rmseGain < -1e-9);
    metric.minimumCellEospaGain = min(eospaGain);
    metric.minimumCellRmseGain = min(rmseGain);
    metric.fullSpatialCount = sum(arrayfun(@(row) ...
        row.fullSpatialCount(policyIdx), rows));
    metric.existenceOnlyCount = sum(arrayfun(@(row) ...
        row.existenceOnlyCount(policyIdx), rows));
    metric.missingReceiverCount = sum(arrayfun(@(row) ...
        row.missingReceiverCount(policyIdx), rows));
    metric.projectedMeanEospa = projectedMeanEospa;
    metric.projectedMeanRmse = projectedMeanRmse;
    metric.projectedMeanEospaGainPercent = relativeGain( ...
        reference.meanEospa, projectedMeanEospa);
    metric.projectedMeanRmseGainPercent = relativeGain( ...
        reference.meanRmse, projectedMeanRmse);
    metric.projectedFormationEospa = projectedFormationEospa;
    metric.projectedFormationRmse = projectedFormationRmse;
    metric.projectedFormationEospaGainPercent = 100 * ( ...
        reference.formationMeanEospa - projectedFormationEospa) ./ ...
        max(abs(reference.formationMeanEospa), eps);
    metric.projectedFormationRmseGainPercent = 100 * ( ...
        reference.formationMeanRmse - projectedFormationRmse) ./ ...
        max(abs(reference.formationMeanRmse), eps);
    metric.projectedAttemptedBytes = attemptedBytes;
    metric.projectedByteSavingPercent = 100 * ( ...
        reference.attemptedBytes - attemptedBytes) / ...
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
summary.contractVersion = 'branch-selective-one-hop-repair-v164-v1';
summary.presetName = snapshotScreen.presetName;
summary.seed = snapshotScreen.seed;
summary.repairPages = repairPages;
summary.repairTimes = snapshotScreen.returnTimes(repairPages);
summary.repairFormationsByPage = repairFormationsByPage;
summary.cellCount = numel(rows);
summary.policies = policies;
summary.rows = rows;
summary.policyMetrics = policyMetrics;
summary.referenceMeanEospa = reference.meanEospa;
summary.referenceMeanRmse = reference.meanRmse;
summary.baseMeanEospa = base.meanEospa;
summary.baseMeanRmse = base.meanRmse;
summary.truthUsedForEvaluation = true;
summary.truthUsedByPolicies = false;
summary.futureInformationUsedByPolicies = false;
summary.recursiveResultClaimAllowed = false;
summary.evidenceBoundary = [ ...
    'V164 is an opened, nonrecursive Bernoulli-branch mechanism screen. ', ...
    'It reuses the privileged F3/F5 V163 cell schedule and the frozen ', ...
    'V162 minimum-risk source and positive-risk Top-4 label selector. ', ...
    'Policies use only current receiver/source Bernoulli GM densities. ', ...
    'For shared labels they either copy the complete density or copy only ', ...
    'existence evidence while retaining receiver spatial content; the ', ...
    'chi-square rules gate a complete spatial copy by present-time ', ...
    'two-dimensional Mahalanobis compatibility. Missing receiver labels ', ...
    'always require a complete source density. Truth scores immediate ', ...
    'E-OSPA and matched-position RMSE only. Projected recursive metrics ', ...
    'add snapshot deltas to V162 and conservatively charge every policy ', ...
    'the original full-response V162 bytes. Passing authorizes an actual ', ...
    'recursive branch-selective probe, not an online or validation claim.'];

if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
matPath = fullfile(outputRoot, ...
    'BRANCH_SELECTIVE_ONE_HOP_REPAIR_V164_PREFLIGHT.mat');
reportPath = fullfile(outputRoot, ...
    'BRANCH_SELECTIVE_ONE_HOP_REPAIR_V164_PREFLIGHT.md');
save('-mat7-binary', matPath, 'summary');
writeReport(reportPath, summary);
fprintf('V164 branch-selective preflight: %s\n', reportPath);
end

function policies = buildPolicies()
template = struct('name', '', 'mode', '', 'threshold', NaN);
policies = repmat(template, 1, 5);
policies(1) = setPolicy(template, 'full-label', 'full', inf);
policies(2) = setPolicy(template, ...
    'existence-only-when-shared', 'existence-only', -inf);
policies(3) = setPolicy(template, ...
    'chi2-95-spatial-else-existence', 'compatibility', 5.991);
policies(4) = setPolicy(template, ...
    'chi2-99-spatial-else-existence', 'compatibility', 9.210);
policies(5) = setPolicy(template, ...
    'chi2-999-spatial-else-existence', 'compatibility', 13.816);
end

function policy = setPolicy(template, name, mode, threshold)
policy = template;
policy.name = name;
policy.mode = mode;
policy.threshold = threshold;
end

function [posterior, counts] = applyPolicy( ...
        posterior, localPosteriors, details, policy)
counts = struct('fullSpatial', 0, 'existenceOnly', 0, ...
    'missingReceiver', 0);
for labelIdx = 1:size(details.selectedLabels, 2)
    label = details.selectedLabels(:, labelIdx);
    sourceIdx = details.selectedSources(labelIdx);
    source = findLabelObject(localPosteriors{sourceIdx}, label);
    receiver = findLabelObject(posterior, label);
    if isempty(source)
        error('BranchSelectiveV164:MissingSelectedSource', ...
            'A selected source label disappeared from the snapshot.');
    end
    if isempty(receiver)
        posterior = replaceLabelObject(posterior, source);
        counts.fullSpatial = counts.fullSpatial + 1;
        counts.missingReceiver = counts.missingReceiver + 1;
        continue;
    end
    copyFull = strcmp(policy.mode, 'full');
    if strcmp(policy.mode, 'compatibility')
        copyFull = positionMahalanobis(receiver, source) <= ...
            policy.threshold + 1e-12;
    end
    if copyFull
        posterior = replaceLabelObject(posterior, source);
        counts.fullSpatial = counts.fullSpatial + 1;
    else
        receiver.r = source.r;
        receiver = copyEvidenceMetadata(receiver, source);
        posterior = replaceLabelObject(posterior, receiver);
        counts.existenceOnly = counts.existenceOnly + 1;
    end
end
end

function object = copyEvidenceMetadata(object, source)
names = {'associationConfidence', 'detectionAssociationMass'};
for idx = 1:numel(names)
    if isfield(source, names{idx})
        object.(names{idx}) = source.(names{idx});
    end
end
end

function value = positionMahalanobis(left, right)
[leftMean, leftCovariance] = momentMatch(left);
[rightMean, rightCovariance] = momentMatch(right);
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
error('BranchSelectiveV164:InvalidCovariance', ...
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
    error('BranchSelectiveV164:MissingAction', ...
        'The requested action is missing: %s', actionName);
end
outcome = screen.outcomes(idx);
end

function value = relativeGain(reference, candidate)
value = 100 * (reference - candidate) / max(abs(reference), eps);
end

function row = emptyRow(policyCount)
row = struct( ...
    'page', 0, 'time', 0, 'formation', 0, 'receiver', 0, ...
    'selectedLabelCount', 0, 'attemptedBytes', 0, ...
    'baselineEospa', NaN, 'baselineRmse', NaN, ...
    'eospaGain', nan(1, policyCount), ...
    'rmseGain', nan(1, policyCount), ...
    'fullSpatialCount', zeros(1, policyCount), ...
    'existenceOnlyCount', zeros(1, policyCount), ...
    'missingReceiverCount', zeros(1, policyCount));
end

function metric = emptyPolicyMetric()
metric = struct( ...
    'name', '', 'totalEospaGain', NaN, 'totalRmseGain', NaN, ...
    'harmfulEospaCellCount', 0, 'harmfulRmseCellCount', 0, ...
    'minimumCellEospaGain', NaN, 'minimumCellRmseGain', NaN, ...
    'fullSpatialCount', 0, 'existenceOnlyCount', 0, ...
    'missingReceiverCount', 0, 'projectedMeanEospa', NaN, ...
    'projectedMeanRmse', NaN, 'projectedMeanEospaGainPercent', NaN, ...
    'projectedMeanRmseGainPercent', NaN, ...
    'projectedFormationEospa', [], 'projectedFormationRmse', [], ...
    'projectedFormationEospaGainPercent', [], ...
    'projectedFormationRmseGainPercent', [], ...
    'projectedAttemptedBytes', NaN, ...
    'projectedByteSavingPercent', NaN, ...
    'mechanismGatePassed', false);
end

function writeReport(path, summary)
fid = fopen(path, 'w');
if fid < 0
    error('BranchSelectiveV164:ReportOpenFailed', ...
        'Could not open the V164 report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V164 branch-selective one-hop repair preflight\n\n');
fprintf(fid, '- Preset / seed: `%s / %d`\n', ...
    summary.presetName, summary.seed);
fprintf(fid, '- Privileged repair pages / times: `%s / %s`\n', ...
    mat2str(summary.repairPages), mat2str(summary.repairTimes));
fprintf(fid, '- Receiver-time cells: `%d`\n', summary.cellCount);
fprintf(fid, ['- Response accounting: `conservative full-label bytes for ', ...
    'every policy`\n\n']);
fprintf(fid, ['| Policy | Full / existence-only | Immediate E-OSPA / RMSE ', ...
    'gain | Harmful cells E/R | Projected mean gain E/R | ', ...
    'Min formation gain E/R | Byte saving | Gate |\n']);
fprintf(fid, '|:--|--:|:--|:--|:--|:--|--:|:--:|\n');
for metric = summary.policyMetrics
    fprintf(fid, ['| %s | %d / %d | %+.4f / %+.4f | %d / %d | ', ...
        '%+.3f%% / %+.3f%% | %+.3f%% / %+.3f%% | %+.3f%% | %d |\n'], ...
        metric.name, metric.fullSpatialCount, ...
        metric.existenceOnlyCount, metric.totalEospaGain, ...
        metric.totalRmseGain, metric.harmfulEospaCellCount, ...
        metric.harmfulRmseCellCount, ...
        metric.projectedMeanEospaGainPercent, ...
        metric.projectedMeanRmseGainPercent, ...
        min(metric.projectedFormationEospaGainPercent), ...
        min(metric.projectedFormationRmseGainPercent), ...
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
