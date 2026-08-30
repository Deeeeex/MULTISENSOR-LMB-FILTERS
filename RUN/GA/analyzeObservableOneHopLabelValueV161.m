function [reportPath, summary] = ...
        analyzeObservableOneHopLabelValueV161(options)
% ANALYZEOBSERVABLEONEHOPLABELVALUEV161 Truth-free label-ranking gate.
%
% Freeze the V160 minimum-Bayes-risk one-hop source selector, enumerate all
% available labels at the 36 registered receiver-time cells, and compare a
% small predeclared family of observable label-value policies. Truth scores
% selected actions offline and defines a same-source-set positive oracle.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getPositiveValueReferenceLabelV157Protocol();
candidateScreenPath = getField(options, 'candidateScreenPath', fullfile( ...
    protocol.headroomOutputRoot, 'x36_t72_h8', 'screen', ...
    ['TRACKING_ALIGNED_X36_SCHEDULE_H8_', ...
     'X36_FORMATION_FOV_SEED211_T72_D8.mat']));
selectionPath = getField(options, 'selectionPath', fullfile( ...
    protocol.headroomOutputRoot, 'selection_replay', ...
    'POSITIVE_VALUE_REFERENCE_LABEL_SELECTION_V158.mat'));
outputRoot = getField(options, 'outputRoot', fullfile( ...
    protocol.headroomOutputRoot, 'observable_label_value'));
reportPath = fullfile(outputRoot, ...
    'OBSERVABLE_ONE_HOP_LABEL_VALUE_V161.md');
matPath = fullfile(outputRoot, ...
    'OBSERVABLE_ONE_HOP_LABEL_VALUE_V161.mat');
if exist(candidateScreenPath, 'file') ~= 2 || ...
        exist(selectionPath, 'file') ~= 2
    error('ObservableLabelValueV161:MissingInput', ...
        'The V157 screen and V158 cell registry are required.');
end

candidateLoaded = load(candidateScreenPath, 'screen');
selectionLoaded = load(selectionPath, 'summary');
candidateScreen = candidateLoaded.screen;
selection = selectionLoaded.summary;
candidateOutcome = outcomeByAction(candidateScreen, ...
    protocol.candidateActionName);
referenceOutcome = outcomeByAction(candidateScreen, ...
    'reference-full-payload');
workingPages = candidateOutcome.fusedPosteriorSnapshotsByTime;
localPages = candidateOutcome.localPosteriorSnapshotsByTime;
refinalizeOnly = logical(getField(options, 'refinalizeOnly', false));
if refinalizeOnly
    if exist(matPath, 'file') ~= 2
        error('ObservableLabelValueV161:MissingRefinalizeInput', ...
            'The V161 MAT artifact does not exist.');
    end
    loaded = load(matPath, 'summary');
    summary = loaded.summary;
    summary.referenceAttemptedBytes = referenceOutcome.attemptedBytes;
    summary.baseCandidateAttemptedBytes = candidateOutcome.attemptedBytes;
    summary.nominalByteHeadroom = ...
        referenceOutcome.attemptedBytes - candidateOutcome.attemptedBytes;
    summary = refinalizeSummary(summary);
    save('-mat7-binary', matPath, 'summary');
    writeReport(reportPath, summary);
    fprintf('V161 observable one-hop label-value gate: %s\n', reportPath);
    return;
end
inputs = generateDynamicTopologyScenarioInputs( ...
    candidateScreen.presetName, candidateScreen.seed);
oracleModel = inputs.model;
if ~isfield(oracleModel, 'dynamicTopologyScenario') || ...
        ~isstruct(oracleModel.dynamicTopologyScenario)
    oracleModel.dynamicTopologyScenario = struct();
end
oracleModel.dynamicTopologyScenario.targetTrajectories = ...
    inputs.targetTrajectories;
groupIds = reshape(candidateScreen.sensorGroupIds, 1, []);
nodeCount = numel(groupIds);
activeThreshold = getField(options, 'activeExistenceThreshold', 0.01);
policies = buildPolicies();

cells = repmat(emptyCellResult(numel(policies)), 1, 0);
oracleActions = repmat(emptyAction(), 1, 0);
policyActions = cell(1, numel(policies));
for policyIdx = 1:numel(policies)
    policyActions{policyIdx} = repmat(emptyAction(), 1, 0);
end
riskSynopsisBytes = 0;
richSynopsisBytes = 0;
for cellIdx = 1:numel(selection.rows)
    selectionRow = selection.rows(cellIdx);
    pageIdx = selectionRow.page;
    receiverIdx = selectionRow.sensor;
    currentTime = selectionRow.time;
    baseline = workingPages{pageIdx}{receiverIdx};
    physical = logical(inputs.graphData. ...
        physicalAdjacency(:, :, currentTime));
    physical = physical | physical';
    physical(1:nodeCount+1:end) = false;
    neighborIds = reshape(find(physical(receiverIdx, :)), 1, []);
    fprintf('  V161 cell %d/%d: t=%d receiver=%d, %d neighbors\n', ...
        cellIdx, numel(selection.rows), currentTime, receiverIdx, ...
        numel(neighborIds));
    [cellRiskBytes, cellRichBytes] = synopsisCost( ...
        localPages{pageIdx}, neighborIds, activeThreshold);
    riskSynopsisBytes = riskSynopsisBytes + cellRiskBytes;
    richSynopsisBytes = richSynopsisBytes + cellRichBytes;

    cellResult = emptyCellResult(numel(policies));
    cellResult.page = pageIdx;
    cellResult.time = currentTime;
    cellResult.formation = groupIds(receiverIdx);
    cellResult.receiver = receiverIdx;
    candidates = buildCandidates( ...
        baseline, localPages{pageIdx}, neighborIds, receiverIdx, ...
        currentTime, oracleModel, groupIds, activeThreshold, ...
        zeros(2, 0));
    [cellResult.oracleGain, cellResult.oracleActionCount, ...
        newOracleActions] = runTruthOracle( ...
            baseline, candidates, receiverIdx, currentTime, ...
            oracleModel, 4);
    oracleActions = [oracleActions, newOracleActions]; %#ok<AGROW>
    for policyIdx = 1:numel(policies)
        [cellResult.policyGain(policyIdx), ...
            cellResult.policyActionCount(policyIdx), ...
            cellResult.policyPositiveCount(policyIdx), ...
            cellResult.policyNegativeCount(policyIdx), ...
            cellResult.policyPayloadBytes(policyIdx), ...
            newPolicyActions] = runObservablePolicy( ...
                baseline, candidates, receiverIdx, currentTime, ...
                oracleModel, policies(policyIdx));
        policyActions{policyIdx} = [ ...
            policyActions{policyIdx}, newPolicyActions]; %#ok<AGROW>
    end
    cells(end + 1) = cellResult; %#ok<AGROW>
end

oracleTotalGain = sum([cells.oracleGain]);
policyMetrics = repmat(emptyPolicyMetric(), 1, numel(policies));
for policyIdx = 1:numel(policies)
    actions = policyActions{policyIdx};
    gains = [actions.truthGain];
    cellGains = arrayfun(@(item) ...
        item.policyGain(policyIdx), cells);
    metric = emptyPolicyMetric();
    metric.name = policies(policyIdx).name;
    metric.maxEdits = policies(policyIdx).maxEdits;
    metric.threshold = policies(policyIdx).threshold;
    metric.selectedActionCount = numel(actions);
    metric.positiveActionCount = nnz(gains > 1e-9);
    metric.negativeActionCount = nnz(gains < -1e-9);
    metric.strongPositiveActionCount = nnz(gains > 0.1);
    metric.totalTruthGain = sum(gains);
    metric.positiveTruthGain = sum(max(gains, 0));
    metric.oracleGainCapturePercent = 100 * ...
        metric.totalTruthGain / max(oracleTotalGain, eps);
    metric.nonnegativeCellCount = nnz(cellGains >= -1e-9);
    metric.harmfulCellCount = nnz(cellGains < -1e-9);
    metric.minimumCellGain = min(cellGains);
    metric.payloadBytes = sum([actions.payloadBytes]);
    metric.riskSynopsisBytes = riskSynopsisBytes;
    metric.richSynopsisBytes = richSynopsisBytes;
    metric.riskControlTotalBytes = ...
        metric.payloadBytes + riskSynopsisBytes;
    metric.richControlTotalBytes = ...
        metric.payloadBytes + richSynopsisBytes;
    policyMetrics(policyIdx) = metric;
end

summary = struct();
summary.contractVersion = ...
    'observable-one-hop-label-value-v161-v1';
summary.presetName = candidateScreen.presetName;
summary.seed = candidateScreen.seed;
summary.cellCount = numel(cells);
summary.activeExistenceThreshold = activeThreshold;
summary.sourceRule = 'minimum-current-per-label-posterior-bayes-risk';
summary.policies = policies;
summary.policyMetrics = policyMetrics;
summary.oracleActionCount = numel(oracleActions);
summary.oraclePositiveGain = oracleTotalGain;
summary.oraclePayloadBytes = sum([oracleActions.payloadBytes]);
summary.riskSynopsisBytes = riskSynopsisBytes;
summary.richSynopsisBytes = richSynopsisBytes;
summary.referenceAttemptedBytes = referenceOutcome.attemptedBytes;
summary.baseCandidateAttemptedBytes = candidateOutcome.attemptedBytes;
summary.nominalByteHeadroom = ...
    referenceOutcome.attemptedBytes - candidateOutcome.attemptedBytes;
summary.cells = cells;
summary.oracleActions = oracleActions;
summary.policyActions = policyActions;
summary.truthUsedForEvaluation = true;
summary.truthUsedByObservablePolicies = false;
summary.futureInformationUsedByObservablePolicies = false;
summary.deployable = false;
summary.evidenceBoundary = [ ...
    'V161 reuses the privileged 36 V157 receiver-time cells but does ', ...
    'not reuse their selected labels. Every observable policy ranks all ', ...
    'currently available one-hop labels with present-time posterior, ', ...
    'evidence and FoV metadata; truth only evaluates chosen actions. ', ...
    'The positive oracle also uses truth and is an offline mechanism ', ...
    'upper bound. Reported gains are sums of immediate cell-wise E-OSPA ', ...
    'marginals, not recursive tracking outcomes. Synopsis byte estimates ', ...
    'are conservative directed-message charges but are not yet inserted ', ...
    'into the full filter accounting; request and response message headers ', ...
    'are also deferred to that actual ledger.'];
summary = refinalizeSummary(summary);

if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
save('-mat7-binary', matPath, 'summary');
writeReport(reportPath, summary);
fprintf('V161 observable one-hop label-value gate: %s\n', reportPath);
end

function policies = buildPolicies()
template = struct( ...
    'name', '', ...
    'scoreField', '', ...
    'maxEdits', 4, ...
    'threshold', 0, ...
    'requireNonnegativeRiskReduction', false, ...
    'controlSynopsisContract', 'rich');
policies = repmat(template, 1, 6);
policies(1) = setPolicy(template, ...
    'risk-reduction-k4', 'riskReductionScore', 4, 0, false, 'risk');
policies(2) = setPolicy(template, ...
    'risk-reduction-k2', 'riskReductionScore', 2, 0, false, 'risk');
policies(3) = setPolicy(template, ...
    'confidence-disagreement-k4', ...
    'confidenceDisagreementScore', 4, 0.05, false, 'rich');
policies(4) = setPolicy(template, ...
    'handover-rescue-k4', 'handoverRescueScore', 4, 0.05, false, 'rich');
policies(5) = setPolicy(template, ...
    'handover-rescue-k2', 'handoverRescueScore', 2, 0.05, false, 'rich');
policies(6) = setPolicy(template, ...
    'risk-gated-handover-k4', 'handoverRescueScore', 4, 0.05, true, 'rich');
end

function policy = setPolicy( ...
        template, name, scoreField, maxEdits, threshold, riskGate, ...
        synopsisContract)
policy = template;
policy.name = name;
policy.scoreField = scoreField;
policy.maxEdits = maxEdits;
policy.threshold = threshold;
policy.requireNonnegativeRiskReduction = riskGate;
policy.controlSynopsisContract = synopsisContract;
end

function [totalGain, actionCount, actions] = runTruthOracle( ...
        baseline, candidates, receiverIdx, currentTime, model, maximumEdits)
working = baseline;
actions = repmat(emptyAction(), 1, 0);
totalGain = 0;
available = true(1, numel(candidates));
for rankIdx = 1:maximumEdits
    remaining = find(available);
    if isempty(remaining)
        break;
    end
    currentRisk = evaluateRisk(working, model, currentTime);
    gains = -inf(1, numel(candidates));
    for candidateIdx = remaining
        trial = replaceLabelObject( ...
            working, candidates(candidateIdx).sourceObject);
        gains(candidateIdx) = currentRisk - ...
            evaluateRisk(trial, model, currentTime);
    end
    [bestGain, bestIdx] = max(gains);
    if bestGain <= 1e-9
        break;
    end
    candidate = candidates(bestIdx);
    action = makeAction(candidate, receiverIdx, currentTime, rankIdx, ...
        bestGain, 'truth-positive-oracle');
    actions(end + 1) = action; %#ok<AGROW>
    totalGain = totalGain + bestGain;
    working = replaceLabelObject(working, candidate.sourceObject);
    available(bestIdx) = false;
end
actionCount = numel(actions);
end

function [totalGain, actionCount, positiveCount, negativeCount, ...
        payloadBytes, actions] = runObservablePolicy( ...
            baseline, candidates, receiverIdx, currentTime, model, policy)
working = baseline;
actions = repmat(emptyAction(), 1, 0);
totalGain = 0;
scores = -inf(1, numel(candidates));
for candidateIdx = 1:numel(candidates)
    feature = candidates(candidateIdx).features;
    if policy.requireNonnegativeRiskReduction && ...
            feature.riskReductionScore < 0
        continue;
    end
    scores(candidateIdx) = feature.(policy.scoreField);
end
for rankIdx = 1:policy.maxEdits
    if isempty(candidates)
        break;
    end
    [bestScore, bestIdx] = max(scores);
    if ~isfinite(bestScore) || bestScore <= policy.threshold
        break;
    end
    candidate = candidates(bestIdx);
    currentRisk = evaluateRisk(working, model, currentTime);
    trial = replaceLabelObject(working, candidate.sourceObject);
    truthGain = currentRisk - evaluateRisk(trial, model, currentTime);
    action = makeAction(candidate, receiverIdx, currentTime, rankIdx, ...
        truthGain, policy.name);
    action.observableScore = bestScore;
    actions(end + 1) = action; %#ok<AGROW>
    totalGain = totalGain + truthGain;
    working = trial;
    scores(bestIdx) = -inf;
end
actionCount = numel(actions);
if isempty(actions)
    positiveCount = 0;
    negativeCount = 0;
    payloadBytes = 0;
else
    gains = [actions.truthGain];
    positiveCount = nnz(gains > 1e-9);
    negativeCount = nnz(gains < -1e-9);
    payloadBytes = sum([actions.payloadBytes]);
end
end

function candidates = buildCandidates( ...
        working, localPosteriors, neighborIds, receiverIdx, currentTime, ...
        model, groupIds, activeThreshold, selectedLabels)
labels = collectAvailableLabels( ...
    localPosteriors, neighborIds, activeThreshold);
candidates = repmat(emptyCandidate(), 1, 0);
for labelIdx = 1:size(labels, 2)
    label = labels(:, labelIdx);
    if ~isempty(selectedLabels) && ...
            any(all(bsxfun(@eq, selectedLabels, label), 1))
        continue;
    end
    receiverObject = findLabelObject(working, label);
    best = emptyCandidate();
    for sourceIdx = neighborIds
        sourceObject = findLabelObject(localPosteriors{sourceIdx}, label);
        if isempty(sourceObject) || sourceObject.r < activeThreshold || ...
                sourceObject.numberOfGmComponents <= 0
            continue;
        end
        sourceRisk = computeObservableLmbLabelBayesRisk( ...
            sourceObject, model);
        if isempty(best.sourceObject) || ...
                sourceRisk < best.sourceRisk - 1e-12 || ...
                (abs(sourceRisk - best.sourceRisk) <= 1e-12 && ...
                 sourceIdx < best.source)
            best.label = label;
            best.labelText = sprintf('(%d,%d)', label(1), label(2));
            best.receiver = receiverIdx;
            best.receiverFormation = groupIds(receiverIdx);
            best.source = sourceIdx;
            best.sourceFormation = groupIds(sourceIdx);
            best.sourceObject = sourceObject;
            best.sourceRisk = sourceRisk;
        end
    end
    if ~isempty(best.sourceObject)
        best.features = computeObservableLmbLabelTransferFeatures( ...
            receiverObject, best.sourceObject, model, receiverIdx, ...
            best.source, currentTime);
        candidates(end + 1) = best; %#ok<AGROW>
    end
end
end

function labels = collectAvailableLabels( ...
        localPosteriors, neighborIds, activeThreshold)
labels = zeros(2, 0);
for sourceIdx = neighborIds
    objects = localPosteriors{sourceIdx};
    for objectIdx = 1:numel(objects)
        object = objects(objectIdx);
        if object.r < activeThreshold || ...
                object.numberOfGmComponents <= 0
            continue;
        end
        label = [object.birthTime; object.birthLocation];
        if isempty(labels) || ...
                ~any(all(bsxfun(@eq, labels, label), 1))
            labels(:, end + 1) = label; %#ok<AGROW>
        end
    end
end
if ~isempty(labels)
    labels = sortrows(labels')';
end
end

function [riskBytes, richBytes] = synopsisCost( ...
        localPosteriors, neighborIds, activeThreshold)
riskBytes = 0;
richBytes = 0;
headerBytes = 16;
for sourceIdx = neighborIds
    objects = localPosteriors{sourceIdx};
    activeCount = nnz([objects.r] >= activeThreshold & ...
        [objects.numberOfGmComponents] > 0);
    if activeCount > 0
        riskBytes = riskBytes + headerBytes + 8 * activeCount;
        richBytes = richBytes + headerBytes + 64 * activeCount;
    end
end
end

function action = makeAction( ...
        candidate, receiverIdx, currentTime, rankIdx, truthGain, policyName)
action = emptyAction();
action.time = currentTime;
action.receiver = receiverIdx;
action.receiverFormation = candidate.receiverFormation;
action.source = candidate.source;
action.sourceFormation = candidate.sourceFormation;
action.rank = rankIdx;
action.label = candidate.label;
action.labelText = candidate.labelText;
action.policyName = policyName;
action.truthGain = truthGain;
action.observableScore = NaN;
action.payloadBytes = candidate.features.payloadBytes;
action.sourceBayesRisk = candidate.features.sourceBayesRisk;
action.receiverBayesRisk = candidate.features.receiverBayesRisk;
action.riskReductionScore = candidate.features.riskReductionScore;
action.confidenceDisagreementScore = ...
    candidate.features.confidenceDisagreementScore;
action.handoverRescueScore = candidate.features.handoverRescueScore;
end

function summary = refinalizeSummary(summary)
if ~isfield(summary.policies, 'controlSynopsisContract')
    contracts = repmat({'rich'}, 1, numel(summary.policies));
    for policyIdx = 1:numel(summary.policies)
        if strcmp(summary.policies(policyIdx).scoreField, ...
                'riskReductionScore')
            contracts{policyIdx} = 'risk';
        end
    end
    [summary.policies.controlSynopsisContract] = contracts{:};
end
if ~isfield(summary.policyMetrics, 'controlSynopsisContract')
    [summary.policyMetrics.controlSynopsisContract] = deal('');
    [summary.policyMetrics.controlSynopsisBytes] = deal(0);
    [summary.policyMetrics.controlTotalBytes] = deal(0);
    [summary.policyMetrics.adjustedByteSavingPercent] = deal(NaN);
end
for policyIdx = 1:numel(summary.policies)
    policy = summary.policies(policyIdx);
    metric = summary.policyMetrics(policyIdx);
    if strcmp(policy.controlSynopsisContract, 'risk')
        metric.controlSynopsisBytes = summary.riskSynopsisBytes;
    else
        metric.controlSynopsisBytes = summary.richSynopsisBytes;
    end
    metric.controlSynopsisContract = policy.controlSynopsisContract;
    metric.controlTotalBytes = ...
        metric.payloadBytes + metric.controlSynopsisBytes;
    metric.adjustedByteSavingPercent = 100 * ...
        (summary.nominalByteHeadroom - metric.controlTotalBytes) / ...
        max(summary.referenceAttemptedBytes, eps);
    summary.policyMetrics(policyIdx) = metric;
end
selectedIdx = selectDeployableDevelopmentPolicy( ...
    summary.policyMetrics);
summary.selectedDevelopmentPolicy = ...
    summary.policies(selectedIdx).name;
summary.selectedDevelopmentPolicyIndex = selectedIdx;
end

function idx = selectDeployableDevelopmentPolicy(metrics)
eligible = [metrics.harmfulCellCount] == 0 & ...
    [metrics.adjustedByteSavingPercent] >= 0;
indices = find(eligible);
if isempty(indices)
    indices = 1:numel(metrics);
end
keys = [ ...
    [metrics(indices).oracleGainCapturePercent]', ...
    [metrics(indices).minimumCellGain]', ...
    [metrics(indices).adjustedByteSavingPercent]', ...
    -[metrics(indices).negativeActionCount]'];
[~, order] = sortrows(keys, [-1, -2, -3, -4]);
idx = indices(order(1));
end

function risk = evaluateRisk(posterior, model, currentTime)
[risk, ~] = evaluateLmbTopologyCurrentEospa( ...
    posterior, model, currentTime, struct());
end

function posterior = replaceLabelObject(posterior, object)
posterior = reshape(posterior, 1, []);
label = [object.birthTime; object.birthLocation];
idx = findLabelIndex(posterior, label);
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

function outcome = outcomeByAction(screen, actionName)
idx = find(strcmp({screen.records.actionName}, actionName), 1);
if isempty(idx)
    error('ObservableLabelValueV161:MissingAction', ...
        'The requested action is missing: %s', actionName);
end
outcome = screen.outcomes(idx);
end

function writeReport(path, summary)
fileId = fopen(path, 'w');
if fileId < 0
    error('ObservableLabelValueV161:ReportOpenFailed', ...
        'Could not open the V161 report.');
end
cleanup = onCleanup(@() fclose(fileId)); %#ok<NASGU>
fprintf(fileId, '# V161 observable one-hop label-value gate\n\n');
fprintf(fileId, '- Preset / seed: `%s / %d`\n', ...
    summary.presetName, summary.seed);
fprintf(fileId, '- Registered receiver-time cells: `%d`\n', ...
    summary.cellCount);
fprintf(fileId, '- Frozen source rule: `%s`\n', summary.sourceRule);
fprintf(fileId, '- Same-source-set positive oracle: `%d` actions / `%.6f` gain\n', ...
    summary.oracleActionCount, summary.oraclePositiveGain);
fprintf(fileId, '- Conservative risk/rich synopsis bytes: `%d / %d`\n\n', ...
    summary.riskSynopsisBytes, summary.richSynopsisBytes);
fprintf(fileId, '- Nominal base-route byte headroom: `%d`\n\n', ...
    summary.nominalByteHeadroom);
fprintf(fileId, ['| Observable label policy | Selected | Positive / negative | ', ...
    'Strong positive | Truth gain | Oracle capture | Harmful cells | ', ...
    'Minimum cell gain | Synopsis | Control total B | Adjusted byte saving |\n']);
fprintf(fileId, '|:--|--:|:--|--:|--:|--:|--:|--:|:--|--:|--:|\n');
for metric = summary.policyMetrics
    fprintf(fileId, ['| %s | %d | %d / %d | %d | %.6f | %.3f%% | ', ...
        '%d | %.6f | %s | %d | %+.3f%% |\n'], ...
        metric.name, metric.selectedActionCount, ...
        metric.positiveActionCount, metric.negativeActionCount, ...
        metric.strongPositiveActionCount, metric.totalTruthGain, ...
        metric.oracleGainCapturePercent, metric.harmfulCellCount, ...
        metric.minimumCellGain, metric.controlSynopsisContract, ...
        metric.controlTotalBytes, metric.adjustedByteSavingPercent);
end
fprintf(fileId, '\n- Safety-first development policy: `%s`\n\n', ...
    summary.selectedDevelopmentPolicy);
fprintf(fileId, ['| t | F | Receiver | Oracle gain/actions | ', ...
    'Selected-policy gain/actions |\n']);
fprintf(fileId, '|--:|--:|--:|:--|:--|\n');
policyIdx = summary.selectedDevelopmentPolicyIndex;
for cellResult = summary.cells
    fprintf(fileId, '| %d | %d | %d | %.4f/%d | %.4f/%d |\n', ...
        cellResult.time, cellResult.formation, cellResult.receiver, ...
        cellResult.oracleGain, cellResult.oracleActionCount, ...
        cellResult.policyGain(policyIdx), ...
        cellResult.policyActionCount(policyIdx));
end
fprintf(fileId, '\n## Evidence boundary\n\n%s\n', ...
    summary.evidenceBoundary);
end

function result = emptyCellResult(policyCount)
result = struct( ...
    'page', 0, ...
    'time', 0, ...
    'formation', 0, ...
    'receiver', 0, ...
    'oracleGain', 0, ...
    'oracleActionCount', 0, ...
    'policyGain', zeros(1, policyCount), ...
    'policyActionCount', zeros(1, policyCount), ...
    'policyPositiveCount', zeros(1, policyCount), ...
    'policyNegativeCount', zeros(1, policyCount), ...
    'policyPayloadBytes', zeros(1, policyCount));
end

function candidate = emptyCandidate()
candidate = struct( ...
    'label', zeros(2, 1), ...
    'labelText', '', ...
    'receiver', 0, ...
    'receiverFormation', 0, ...
    'source', 0, ...
    'sourceFormation', 0, ...
    'sourceObject', [], ...
    'sourceRisk', inf, ...
    'features', struct());
end

function action = emptyAction()
action = struct( ...
    'time', 0, ...
    'receiver', 0, ...
    'receiverFormation', 0, ...
    'source', 0, ...
    'sourceFormation', 0, ...
    'rank', 0, ...
    'label', zeros(2, 1), ...
    'labelText', '', ...
    'policyName', '', ...
    'truthGain', 0, ...
    'observableScore', NaN, ...
    'payloadBytes', 0, ...
    'sourceBayesRisk', 0, ...
    'receiverBayesRisk', 0, ...
    'riskReductionScore', 0, ...
    'confidenceDisagreementScore', 0, ...
    'handoverRescueScore', 0);
end

function metric = emptyPolicyMetric()
metric = struct( ...
    'name', '', ...
    'maxEdits', 0, ...
    'threshold', 0, ...
    'selectedActionCount', 0, ...
    'positiveActionCount', 0, ...
    'negativeActionCount', 0, ...
    'strongPositiveActionCount', 0, ...
    'totalTruthGain', 0, ...
    'positiveTruthGain', 0, ...
    'oracleGainCapturePercent', 0, ...
    'nonnegativeCellCount', 0, ...
    'harmfulCellCount', 0, ...
    'minimumCellGain', 0, ...
    'payloadBytes', 0, ...
    'riskSynopsisBytes', 0, ...
    'richSynopsisBytes', 0, ...
    'riskControlTotalBytes', 0, ...
    'richControlTotalBytes', 0);
metric.controlSynopsisContract = '';
metric.controlSynopsisBytes = 0;
metric.controlTotalBytes = 0;
metric.adjustedByteSavingPercent = NaN;
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
