function [reportPath, analysis] = ...
        analyzeBudgetRecycledRepairImmediateHeadroomV188(options)
% ANALYZEBUDGETRECYCLEDREPAIRIMMEDIATEHEADROOMV188 Paired action-bank test.
%
% Proposal construction is truth-free.  Truth is read only after each
% projected action has been frozen, to determine whether the executable,
% byte-safe action bank contains immediate E-OSPA/RMSE/consensus headroom.
% This is not a recursive or finite-horizon performance claim.

if nargin < 1 || isempty(options)
    options = struct();
end
screenPath = char(getField(options, 'screenPath', ''));
pageIndex = getField(options, 'pageIndex', 1);
baseActionPrefix = char(getField( ...
    options, 'baseActionPrefix', 'v99-online-positive-net-'));
if exist(screenPath, 'file') ~= 2 || ...
        ~isscalar(pageIndex) || ~isfinite(pageIndex) || ...
        pageIndex < 1 || pageIndex ~= round(pageIndex)
    error('BudgetRecycledRepairHeadroomV188:InvalidRequest', ...
        'A captured paired screen and positive page index are required.');
end
loaded = load(screenPath, 'screen');
screen = loaded.screen;
if pageIndex > numel(screen.returnTimes)
    error('BudgetRecycledRepairHeadroomV188:PageOutOfRange', ...
        'The requested page is outside the captured horizon.');
end
names = {screen.records.actionName};
baseIdx = find(cellfun(@(name) strncmp( ...
    name, baseActionPrefix, numel(baseActionPrefix)), names), 1);
referenceIdx = screen.referenceSubsetIndex;
if isempty(baseIdx)
    error('BudgetRecycledRepairHeadroomV188:MissingBaseArm', ...
        'The requested causal base-admission arm is absent.');
end
baseOutcome = screen.outcomes(baseIdx);
referenceOutcome = screen.outcomes(referenceIdx);
requiredSnapshots = {'localPosteriorSnapshotsByTime', ...
    'fusedPosteriorSnapshotsByTime'};
if any(~isfield(baseOutcome, requiredSnapshots)) || ...
        numel(baseOutcome.localPosteriorSnapshotsByTime) < pageIndex || ...
        numel(baseOutcome.fusedPosteriorSnapshotsByTime) < pageIndex
    error('BudgetRecycledRepairHeadroomV188:MissingSnapshots', ...
        'The causal base arm lacks captured posterior pages.');
end
localPosterior = ...
    baseOutcome.localPosteriorSnapshotsByTime{pageIndex};
basePosterior = ...
    baseOutcome.fusedPosteriorSnapshotsByTime{pageIndex};
if isempty(localPosterior) || isempty(basePosterior)
    error('BudgetRecycledRepairHeadroomV188:EmptySnapshots', ...
        'The requested captured posterior page is empty.');
end

currentTime = screen.returnTimes(pageIndex);
inputs = generateDynamicTopologyScenarioInputs( ...
    screen.presetName, screen.seed);
model = inputs.model;
model.dynamicTopologyScenario.targetTrajectories = ...
    inputs.targetTrajectories;
groupIds = reshape(screen.sensorGroupIds, 1, []);
sensorCount = numel(groupIds);
physical = logical( ...
    inputs.graphData.physicalAdjacency(:, :, currentTime));
physical = physical | physical';
physical(1:sensorCount+1:end) = false;

cache = buildFormationRepairLightSynopsisCacheV188( ...
    basePosterior, localPosterior, model, currentTime, struct());
[features, featureNames, featureDetails] = ...
    buildFormationRepairValueFeaturesV188( ...
        basePosterior, localPosterior, physical, groupIds, model, ...
        currentTime, struct(), struct('lightSynopsisCache', cache));
[proposals, proposalDetails] = ...
    buildFormationCommonLabelRepairProposalsV188( ...
        cache, localPosterior, physical, groupIds, model, struct());

referencePageBytes = ...
    referenceOutcome.attemptedBytesByTime(pageIndex);
basePageBytes = baseOutcome.attemptedBytesByTime(pageIndex);
admissionNetSavingBytes = max( ...
    referencePageBytes - basePageBytes, 0);
[creditAfterSynopsis, synopsisDecision] = ...
    preflightBudgetRecycledRepairSynopsisV188( ...
        [], admissionNetSavingBytes, cache.totalAttemptedBytes, ...
        referencePageBytes);
if synopsisDecision.lightSynopsisAuthorized
    projection = projectBudgetRecycledRepairActionsV188( ...
        proposals, creditAfterSynopsis);
else
    projection = projectBudgetRecycledRepairActionsV188( ...
        proposals, creditAfterSynopsis, struct('maximumActions', 0));
end

baseline = evaluateNetwork( ...
    basePosterior, model, currentTime, groupIds, [], []);
rows = repmat(emptyRow(), 1, numel(proposals));
for proposalIdx = 1:numel(proposals)
    row = emptyRow();
    proposal = proposals(proposalIdx);
    row.proposalIndex = proposalIdx;
    row.formationId = proposal.formationId;
    row.receiverIds = proposal.receiverIds;
    row.sourceId = proposal.sourceId;
    row.label = proposal.label;
    row.proxyUtilityLowerBound = ...
        proposal.estimatedUtilityLowerBound;
    row.rankingOpportunity = proposal.rankingOpportunity;
    row.actionBytes = proposal.attemptedBytes;
    if synopsisDecision.lightSynopsisAuthorized
        oneProjection = projectBudgetRecycledRepairActionsV188( ...
            proposal, creditAfterSynopsis);
        row.budgetFeasible = any(oneProjection.selectedMask);
    end
    if row.budgetFeasible
        [trialPosterior, ~] = ...
            applyFormationCommonLabelRepairV188( ...
                basePosterior, proposal, 1);
        trial = evaluateNetwork( ...
            trialPosterior, model, currentTime, groupIds, ...
            baseline, proposal.receiverIds);
        row = fillGains(row, baseline, trial, proposal.receiverIds, ...
            find(baseline.formationIds == proposal.formationId, 1));
        row.chargedNetByteSaving = referencePageBytes - ...
            basePageBytes - cache.totalAttemptedBytes - ...
            proposal.attemptedBytes;
        row.chargedNetByteSavingPercent = 100 * ...
            row.chargedNetByteSaving / max(referencePageBytes, eps);
        row.jointPositive = row.meanEospaGainPercent > 0 && ...
            row.meanRmseGainPercent > 0 && ...
            row.minimumAffectedEospaGainPercent >= -1e-12 && ...
            row.minimumAffectedRmseGainPercent >= -1e-12 && ...
            row.consensusGainPercent >= -1e-12 && ...
            row.chargedNetByteSaving > 0;
    end
    rows(proposalIdx) = row;
end

[projectedPosterior, applyDetails] = ...
    applyFormationCommonLabelRepairV188( ...
        basePosterior, proposals, ...
        projection.selectedProposalIndices);
projected = evaluateNetwork( ...
    projectedPosterior, model, currentTime, groupIds, baseline, ...
    unique(applyDetails.appliedReceiverIds));
projectedRow = emptyRow();
if ~isempty(projection.selectedProposalIndices)
    selectedIdx = projection.selectedProposalIndices(1);
    selected = proposals(selectedIdx);
    projectedRow = rows(selectedIdx);
    projectedRow.chargedNetByteSaving = referencePageBytes - ...
        basePageBytes - cache.totalAttemptedBytes - ...
        projection.selectedActionBytes;
    projectedRow.chargedNetByteSavingPercent = 100 * ...
        projectedRow.chargedNetByteSaving / ...
        max(referencePageBytes, eps);
else
    projectedRow = fillGains(projectedRow, baseline, projected, ...
        zeros(1, 0), []);
    projectedRow.chargedNetByteSaving = referencePageBytes - ...
        basePageBytes - synopsisDecision.chargedSynopsisBytes;
    projectedRow.chargedNetByteSavingPercent = 100 * ...
        projectedRow.chargedNetByteSaving / ...
        max(referencePageBytes, eps);
end

analysis = struct();
analysis.contractVersion = ...
    'budget-recycled-repair-immediate-headroom-v188-v1';
analysis.generatedAt = datestr(now, 31);
analysis.screenPath = screenPath;
analysis.presetName = screen.presetName;
analysis.seed = screen.seed;
analysis.currentTime = currentTime;
analysis.pageIndex = pageIndex;
analysis.referenceActionName = names{referenceIdx};
analysis.baseActionName = names{baseIdx};
analysis.referencePageBytes = referencePageBytes;
analysis.basePageBytes = basePageBytes;
analysis.admissionNetSavingBytes = ...
    referencePageBytes - basePageBytes;
analysis.lightSynopsisBytes = cache.totalAttemptedBytes;
analysis.synopsisDecision = synopsisDecision;
analysis.features = features;
analysis.featureNames = featureNames;
analysis.featureDetails = featureDetails;
analysis.proposalDetails = proposalDetails;
analysis.proposals = proposals;
analysis.proposalRows = rows;
analysis.projection = projection;
analysis.applyDetails = applyDetails;
analysis.baselineMetrics = stripInternalMetrics(baseline);
analysis.projectedMetrics = stripInternalMetrics(projected);
analysis.projectedRow = projectedRow;
analysis.budgetFeasibleProposalCount = ...
    nnz([rows.budgetFeasible]);
analysis.jointPositiveProposalCount = nnz([rows.jointPositive]);
analysis.actionHeadroomExists = ...
    analysis.jointPositiveProposalCount > 0;
analysis.projectedActionJointPositive = ...
    projectedRow.jointPositive;
analysis.truthUsedForProposalConstruction = false;
analysis.truthUsedForPostSelectionReadout = true;
analysis.recursiveEvaluationRun = false;
analysis.validationClaimAllowed = false;
analysis.evidenceBoundary = [ ...
    'The light synopsis, common source, label, payload and deterministic ', ...
    'byte projection use the captured current posterior and physical ', ...
    'graph only. Truth scores each already-frozen action afterward. ', ...
    'This single-page paired diagnostic tests executable action-bank ', ...
    'headroom only; it does not establish finite-horizon, recursive, ', ...
    'multi-seed or cross-scene performance.'];

defaultOutputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v188', 'immediate_headroom', ...
    sprintf('%s_seed%d_t%d', ...
        strrep(screen.presetName, '-', '_'), screen.seed, currentTime));
outputRoot = char(getField(options, 'outputRoot', defaultOutputRoot));
if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
stem = sprintf('BUDGET_RECYCLED_REPAIR_V188_%s_SEED%d_T%d', ...
    upper(strrep(screen.presetName, '-', '_')), ...
    screen.seed, currentTime);
matPath = fullfile(outputRoot, [stem, '.mat']);
reportPath = fullfile(outputRoot, [stem, '.md']);
analysis.matPath = matPath;
analysis.reportPath = reportPath;
save('-mat7-binary', matPath, 'analysis');
writeReport(reportPath, analysis);
fprintf('V188 immediate action headroom: %s\n', reportPath);
end

function row = fillGains( ...
        row, baseline, trial, receiverIds, formationIdx)
row.meanEospaGainPercent = relativeGain( ...
    baseline.meanEospa, trial.meanEospa);
row.meanRmseGainPercent = relativeGain( ...
    baseline.meanRmse, trial.meanRmse);
row.consensusGainPercent = relativeGain( ...
    baseline.consensusOspa, trial.consensusOspa);
if isempty(receiverIds)
    row.minimumAffectedEospaGainPercent = 0;
    row.minimumAffectedRmseGainPercent = 0;
else
    eospaGain = 100 * (baseline.eospaBySensor(receiverIds) - ...
        trial.eospaBySensor(receiverIds)) ./ ...
        max(abs(baseline.eospaBySensor(receiverIds)), eps);
    rmseGain = 100 * (baseline.rmseBySensor(receiverIds) - ...
        trial.rmseBySensor(receiverIds)) ./ ...
        max(abs(baseline.rmseBySensor(receiverIds)), eps);
    row.minimumAffectedEospaGainPercent = ...
        minimumFinite(eospaGain);
    row.minimumAffectedRmseGainPercent = ...
        minimumFinite(rmseGain);
end
if isempty(formationIdx)
    row.formationEospaGainPercent = 0;
    row.formationRmseGainPercent = 0;
else
    row.formationEospaGainPercent = relativeGain( ...
        baseline.formationMeanEospa(formationIdx), ...
        trial.formationMeanEospa(formationIdx));
    row.formationRmseGainPercent = relativeGain( ...
        baseline.formationMeanRmse(formationIdx), ...
        trial.formationMeanRmse(formationIdx));
end
end

function metrics = evaluateNetwork( ...
        posteriorBySensor, model, currentTime, groupIds, ...
        baseline, affectedReceiverIds)
sensorCount = numel(posteriorBySensor);
if nargin < 5 || isempty(baseline)
    eospa = nan(1, sensorCount);
    rmse = nan(1, sensorCount);
    estimates = cell(1, sensorCount);
    consensusMatrix = nan(sensorCount);
    evaluationIds = 1:sensorCount;
else
    eospa = baseline.eospaBySensor;
    rmse = baseline.rmseBySensor;
    estimates = baseline.internalStateEstimateBySensor;
    consensusMatrix = baseline.internalConsensusOspaMatrix;
    evaluationIds = reshape(unique(affectedReceiverIds), 1, []);
end
for sensorIdx = evaluationIds
    eospa(sensorIdx) = evaluateLmbTopologyCurrentEospa( ...
        posteriorBySensor{sensorIdx}, model, currentTime, struct());
    rmse(sensorIdx) = currentPosteriorRmse( ...
        posteriorBySensor{sensorIdx}, model, currentTime);
    estimates{sensorIdx} = posteriorStateEstimate( ...
        posteriorBySensor{sensorIdx}, model);
end
if nargin < 5 || isempty(baseline)
    for leftIdx = 1:sensorCount-1
        for rightIdx = leftIdx+1:sensorCount
            value = estimateStateSetOspa( ...
                estimates{leftIdx}, estimates{rightIdx}, model);
            consensusMatrix(leftIdx, rightIdx) = value;
            consensusMatrix(rightIdx, leftIdx) = value;
        end
    end
else
    for receiverIdx = evaluationIds
        peers = setdiff(1:sensorCount, receiverIdx);
        for peerIdx = reshape(peers, 1, [])
            value = estimateStateSetOspa( ...
                estimates{receiverIdx}, estimates{peerIdx}, model);
            consensusMatrix(receiverIdx, peerIdx) = value;
            consensusMatrix(peerIdx, receiverIdx) = value;
        end
    end
end
upperMask = triu(true(sensorCount), 1);
consensusOspa = finiteMean(consensusMatrix(upperMask));
formationIds = unique(groupIds, 'stable');
formationEospa = nan(1, numel(formationIds));
formationRmse = nan(1, numel(formationIds));
for formationIdx = 1:numel(formationIds)
    members = groupIds == formationIds(formationIdx);
    formationEospa(formationIdx) = finiteMean(eospa(members));
    formationRmse(formationIdx) = finiteMean(rmse(members));
end
metrics = struct( ...
    'meanEospa', finiteMean(eospa), ...
    'meanRmse', finiteMean(rmse), ...
    'worstSensorEospa', maximumFinite(eospa), ...
    'worstSensorRmse', maximumFinite(rmse), ...
    'eospaBySensor', eospa, ...
    'rmseBySensor', rmse, ...
    'formationIds', formationIds, ...
    'formationMeanEospa', formationEospa, ...
    'formationMeanRmse', formationRmse, ...
    'consensusOspa', consensusOspa, ...
    'internalStateEstimateBySensor', {estimates}, ...
    'internalConsensusOspaMatrix', consensusMatrix);
end

function estimate = posteriorStateEstimate(posterior, model)
objects = reshape(posterior, 1, []);
if ~isempty(objects)
    objects = objects([objects.r] > model.existenceThreshold);
end
estimate = struct('labels', {{zeros(2, 0)}}, ...
    'mu', {{cell(1, 0)}}, 'Sigma', {{cell(1, 0)}}, ...
    'objects', []);
if isempty(objects)
    return;
end
[cardinality, indices] = lmbMapCardinalityEstimate([objects.r]);
estimate.labels{1} = zeros(2, cardinality);
estimate.mu{1} = cell(1, cardinality);
estimate.Sigma{1} = cell(1, cardinality);
for idx = 1:cardinality
    object = objects(indices(idx));
    weights = reshape(object.w, 1, []);
    weights(~isfinite(weights)) = -inf;
    [~, componentIdx] = max(weights);
    if isempty(componentIdx) || ~isfinite(weights(componentIdx))
        componentIdx = 1;
    end
    estimate.labels{1}(:, idx) = ...
        [object.birthTime; object.birthLocation];
    estimate.mu{1}{idx} = object.mu{componentIdx};
    estimate.Sigma{1}{idx} = object.Sigma{componentIdx};
end
end

function distance = estimateStateSetOspa(left, right, model)
leftMu = left.mu{1};
leftSigma = left.Sigma{1};
rightMu = right.mu{1};
rightSigma = right.Sigma{1};
if isempty(leftMu) && isempty(rightMu)
    distance = 0;
    return;
elseif isempty(leftMu) || isempty(rightMu)
    distance = model.ospaParameters.eC;
    return;
end
[leftToRight, ~] = ospa( ...
    leftMu, leftMu, leftSigma, rightMu, rightSigma, ...
    model.ospaParameters);
[rightToLeft, ~] = ospa( ...
    rightMu, rightMu, rightSigma, leftMu, leftSigma, ...
    model.ospaParameters);
distance = 0.5 * (leftToRight(1) + rightToLeft(1));
end

function metrics = stripInternalMetrics(metrics)
fields = {'internalStateEstimateBySensor', ...
    'internalConsensusOspaMatrix'};
for fieldIdx = 1:numel(fields)
    if isfield(metrics, fields{fieldIdx})
        metrics = rmfield(metrics, fields{fieldIdx});
    end
end
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
if isempty(truth) && isempty(estimate)
    value = 0;
    return;
elseif isempty(truth) || isempty(estimate)
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

function writeReport(path, analysis)
fid = fopen(path, 'w');
if fid < 0
    error('BudgetRecycledRepairHeadroomV188:WriteFailed', ...
        'Could not create the V188 headroom report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V188 immediate executable-action headroom\n\n');
fprintf(fid, '- Scenario: `%s`, seed `%d`, time `%d`\n', ...
    analysis.presetName, analysis.seed, analysis.currentTime);
fprintf(fid, '- Base arm: `%s`\n', analysis.baseActionName);
fprintf(fid, '- Reference / base page bytes: `%d / %d`\n', ...
    analysis.referencePageBytes, analysis.basePageBytes);
fprintf(fid, '- Earned admission credit: `%d B`\n', ...
    analysis.admissionNetSavingBytes);
fprintf(fid, '- Charged light synopsis: `%d B` (authorized `%d`)\n', ...
    analysis.lightSynopsisBytes, ...
    analysis.synopsisDecision.lightSynopsisAuthorized);
fprintf(fid, '- Executable / joint-positive proposals: `%d / %d`\n', ...
    analysis.budgetFeasibleProposalCount, ...
    analysis.jointPositiveProposalCount);
fprintf(fid, '- Projected online action jointly positive: `%d`\n\n', ...
    analysis.projectedActionJointPositive);
fprintf(fid, ['| Proposal | Formation | Source | Label | Bytes | ', ...
    'E-OSPA gain | RMSE gain | Consensus gain | Net saving | Joint |\n']);
fprintf(fid, '|--:|--:|--:|:--|--:|--:|--:|--:|--:|:--:|\n');
for idx = 1:numel(analysis.proposalRows)
    row = analysis.proposalRows(idx);
    fprintf(fid, ['| %d | %d | %d | [%d,%d] | %d | ', ...
        '%+.3f%% | %+.3f%% | %+.3f%% | %+.3f%% | %d |\n'], ...
        row.proposalIndex, row.formationId, row.sourceId, ...
        row.label(1), row.label(2), row.actionBytes, ...
        row.meanEospaGainPercent, row.meanRmseGainPercent, ...
        row.consensusGainPercent, ...
        row.chargedNetByteSavingPercent, row.jointPositive);
end
fprintf(fid, '\n## Evidence boundary\n\n%s\n', ...
    analysis.evidenceBoundary);
end

function row = emptyRow()
row = struct( ...
    'proposalIndex', 0, 'formationId', 0, ...
    'receiverIds', zeros(1, 0), 'sourceId', 0, ...
    'label', zeros(2, 1), 'proxyUtilityLowerBound', 0, ...
    'rankingOpportunity', 0, 'actionBytes', 0, ...
    'budgetFeasible', false, ...
    'meanEospaGainPercent', 0, 'meanRmseGainPercent', 0, ...
    'minimumAffectedEospaGainPercent', 0, ...
    'minimumAffectedRmseGainPercent', 0, ...
    'formationEospaGainPercent', 0, ...
    'formationRmseGainPercent', 0, ...
    'consensusGainPercent', 0, ...
    'chargedNetByteSaving', 0, ...
    'chargedNetByteSavingPercent', 0, ...
    'jointPositive', false);
end

function value = relativeGain(reference, candidate)
if ~isfinite(reference) || ~isfinite(candidate)
    value = NaN;
else
    value = 100 * (reference - candidate) / max(abs(reference), eps);
end
end

function value = finiteMean(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = mean(values);
end
end

function value = maximumFinite(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = max(values);
end
end

function value = minimumFinite(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = min(values);
end
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
