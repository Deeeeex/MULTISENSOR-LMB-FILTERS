function [reportPath, result] = ...
        analyzeBeneficiaryOnlyQueryKeyRecallV229(options)
% ANALYZEBENEFICIARYONLYQUERYKEYRECALLV229 Test causal top-three discovery.
%
% The selector sees only the current local posteriors of the beneficiary
% formation.  It nominates one label for each of three receiver-side modes:
% existence disagreement, normalized spatial disagreement and position
% uncertainty.  Teacher source-label rows are consulted only after selection
% to measure recall; remote label inventories, truth and future outcomes are
% not selector inputs.

if nargin < 1 || isempty(options)
    options = struct();
end
candidateBankPath = char(getField(options, 'candidateBankPath', ''));
recordIndices = reshape(getField(options, ...
    'recordIndices', zeros(1, 0)), 1, []);
maximumQueriedLabels = getField(options, 'maximumQueriedLabels', 3);
minimumSupportFraction = getField(options, ...
    'minimumSupportFraction', 2 / 3);
minimumMeanExistence = getField(options, ...
    'minimumMeanExistence', 0.10);
minimumPresenceExistence = getField(options, ...
    'minimumPresenceExistence', 0.05);
if isempty(candidateBankPath) || exist(candidateBankPath, 'file') ~= 2 || ...
        maximumQueriedLabels ~= 3 || ...
        minimumSupportFraction <= 0 || minimumSupportFraction > 1 || ...
        minimumMeanExistence < 0 || minimumMeanExistence > 1 || ...
        minimumPresenceExistence < 0 || minimumPresenceExistence > 1
    error('BeneficiaryOnlyQueryKeyRecallV229:InvalidRequest', ...
        'A candidate bank and the frozen causal selector options are required.');
end

loaded = load(candidateBankPath, 'result');
bank = loaded.result;
required = {'contractVersion', 'presetName', 'seed', 'currentTime', ...
    'sourceScreenPath', 'records'};
if ~isstruct(bank) || ~isscalar(bank) || ...
        any(~isfield(bank, required)) || ...
        ~strcmp(bank.contractVersion, ...
            'decoupled-semantic-shortcut-candidate-bank-v220-v1') || ...
        getField(bank, 'truthUsedForCandidateFeatures', true) || ...
        getField(bank, 'futureInformationUsedForCandidateFeatures', true)
    error('BeneficiaryOnlyQueryKeyRecallV229:CandidateBankContract', ...
        'The causal V220 candidate-bank contract is required.');
end
if isempty(recordIndices)
    selectedRows = 1:numel(bank.records);
else
    [present, selectedRows] = ismember( ...
        recordIndices, [bank.records.recordIndex]);
    if ~all(present)
        error('BeneficiaryOnlyQueryKeyRecallV229:UnknownRecord', ...
            'A requested teacher record is absent from the bank.');
    end
end

screenLoaded = load(bank.sourceScreenPath, 'screen');
screen = screenLoaded.screen;
timeIndex = find(screen.returnTimes == bank.currentTime, 1);
if isempty(timeIndex) || ~screen.capturePosteriorSnapshotsEnabled || ...
        screen.referenceSubsetIndex < 1 || ...
        screen.referenceSubsetIndex > numel(screen.outcomes)
    error('BeneficiaryOnlyQueryKeyRecallV229:SnapshotContract', ...
        'The same-state reference local-posterior snapshot is unavailable.');
end
reference = screen.outcomes(screen.referenceSubsetIndex);
posteriors = reference.localPosteriorSnapshotsByTime{timeIndex};
if ~iscell(posteriors) || isempty(posteriors)
    error('BeneficiaryOnlyQueryKeyRecallV229:SnapshotContract', ...
        'Local posteriors must be available as one cell per sensor.');
end

teacherRows = bank.records(selectedRows);
formationIds = unique([teacherRows.beneficiaryFormationId], 'stable');
formations = repmat(emptyFormation(), 1, numel(formationIds));
audits = repmat(emptyAudit(), 1, numel(teacherRows));
for formationIndex = 1:numel(formationIds)
    formationId = formationIds(formationIndex);
    formationTeacherMask = ...
        [teacherRows.beneficiaryFormationId] == formationId;
    firstTeacher = teacherRows(find(formationTeacherMask, 1));
    receiverIds = reshape(firstTeacher.candidate.receiverIds, 1, []);
    [labels, metrics] = summarizeFormation( ...
        posteriors, receiverIds, minimumPresenceExistence);
    minimumSupportCount = ceil( ...
        minimumSupportFraction * numel(receiverIds));
    eligible = metrics.supportCount >= minimumSupportCount & ...
        metrics.meanExistence >= minimumMeanExistence;
    modeNames = { ...
        'existence-disagreement', ...
        'spatial-disagreement', ...
        'position-uncertainty'};
    modeScores = [ ...
        metrics.existenceDisagreement(:), ...
        metrics.spatialDisagreement(:), ...
        metrics.positionUncertainty(:)];
    rankings = cell(1, numel(modeNames));
    selectedLabels = zeros(2, 0);
    for modeIndex = 1:numel(modeNames)
        rankings{modeIndex} = rankLabels( ...
            labels, modeScores(:, modeIndex), metrics, eligible);
        if ~isempty(rankings{modeIndex}) && ...
                size(selectedLabels, 2) < maximumQueriedLabels
            proposed = rankings{modeIndex}(:, 1);
            if isempty(selectedLabels) || ...
                    ~any(all(selectedLabels == proposed, 1))
                selectedLabels(:, end + 1) = proposed; %#ok<AGROW>
            end
        end
    end
    formations(formationIndex) = struct( ...
        'formationId', formationId, ...
        'receiverIds', receiverIds, ...
        'labels', labels, ...
        'metrics', metrics, ...
        'eligibleMask', reshape(eligible, 1, []), ...
        'modeNames', {modeNames}, ...
        'modeScores', modeScores, ...
        'rankings', {rankings}, ...
        'selectedLabels', selectedLabels, ...
        'maximumQueriedLabels', maximumQueriedLabels);

    auditPositions = find(formationTeacherMask);
    for auditPosition = reshape(auditPositions, 1, [])
        teacher = teacherRows(auditPosition);
        label = reshape(teacher.label, 2, 1);
        labelIndex = find(all(labels == label, 1), 1);
        modeRanks = inf(1, numel(modeNames));
        if ~isempty(labelIndex)
            for modeIndex = 1:numel(modeNames)
                ranking = rankings{modeIndex};
                match = find(all(ranking == label, 1), 1);
                if ~isempty(match)
                    modeRanks(modeIndex) = match;
                end
            end
        end
        recovered = ~isempty(selectedLabels) && ...
            any(all(selectedLabels == label, 1));
        audits(auditPosition) = struct( ...
            'recordIndex', teacher.recordIndex, ...
            'beneficiaryFormationId', formationId, ...
            'sourceId', teacher.sourceId, ...
            'label', label, ...
            'labelPresent', ~isempty(labelIndex), ...
            'modeRanks', modeRanks, ...
            'selectedByTopThree', recovered, ...
            'supportCount', valueAt(metrics.supportCount, labelIndex, 0), ...
            'meanExistence', valueAt(metrics.meanExistence, labelIndex, NaN), ...
            'existenceDisagreement', valueAt( ...
                metrics.existenceDisagreement, labelIndex, NaN), ...
            'spatialDisagreement', valueAt( ...
                metrics.spatialDisagreement, labelIndex, NaN), ...
            'positionUncertainty', valueAt( ...
                metrics.positionUncertainty, labelIndex, NaN));
    end
end

outputRoot = char(getField(options, 'outputRoot', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v229', 'beneficiary_only_query_key_recall', ...
    sprintf('%s_seed%d_t%d', strrep(bank.presetName, '-', '_'), ...
        bank.seed, bank.currentTime))));
if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
result = struct();
result.contractVersion = ...
    'beneficiary-only-query-key-recall-v229-v1';
result.generatedAt = datestr(now, 31);
gitState = resolveResearchGitState();
result.generationGitCommit = gitState.commit;
result.candidateBankPath = candidateBankPath;
result.sourceScreenPath = bank.sourceScreenPath;
result.presetName = bank.presetName;
result.seed = bank.seed;
result.currentTime = bank.currentTime;
result.maximumQueriedLabels = maximumQueriedLabels;
result.minimumSupportFraction = minimumSupportFraction;
result.minimumMeanExistence = minimumMeanExistence;
result.minimumPresenceExistence = minimumPresenceExistence;
result.formations = formations;
result.teacherAudits = audits;
result.teacherLabelCount = numel(audits);
result.recoveredTeacherLabelCount = nnz([audits.selectedByTopThree]);
result.teacherRecall = result.recoveredTeacherLabelCount / ...
    max(result.teacherLabelCount, 1);
result.allTeacherLabelsRecovered = ...
    result.recoveredTeacherLabelCount == result.teacherLabelCount;
result.truthUsedForSelection = false;
result.futureInformationUsedForSelection = false;
result.remoteLabelInventoryUsedForSelection = false;
result.teacherLabelsUsedOnlyForRecallAudit = true;
result.onlineQueryKeyRuleAuthorized = result.allTeacherLabelsRecovered;
result.sourceOfferDesignRecommended = ~result.allTeacherLabelsRecovered;
result.validationClaimAllowed = false;
result.evidenceBoundary = [ ...
    'This is a single opened X36 t=133 causal-discoverability diagnostic. ', ...
    'Failure rejects this simple beneficiary-only three-mode selector for ', ...
    'the current teacher rows; it does not prove that every beneficiary-only ', ...
    'model must fail and is not tracking, closed-loop or generalization evidence.'];
matPath = fullfile(outputRoot, ...
    'BENEFICIARY_ONLY_QUERY_KEY_RECALL_V229.mat');
reportPath = fullfile(outputRoot, ...
    'BENEFICIARY_ONLY_QUERY_KEY_RECALL_V229.md');
result.matPath = matPath;
result.reportPath = reportPath;
save('-mat7-binary', matPath, 'result');
writeReport(reportPath, result);
fprintf('V229 beneficiary-only query-key recall: %s\n', reportPath);
end

function [labels, metrics] = summarizeFormation( ...
        posteriors, receiverIds, minimumPresenceExistence)
stateDimension = inferStateDimension(posteriors, receiverIds);
model = struct('xDimension', stateDimension);
summaries = cell(1, numel(receiverIds));
labels = zeros(2, 0);
for receiverIndex = 1:numel(receiverIds)
    sensorId = receiverIds(receiverIndex);
    summaries{receiverIndex} = ...
        summarizeLmbPosteriorForDisagreement(posteriors{sensorId}, model);
    additions = summaries{receiverIndex}.labels;
    for labelIndex = 1:size(additions, 2)
        label = additions(:, labelIndex);
        if isempty(labels) || ~any(all(labels == label, 1))
            labels(:, end + 1) = label; %#ok<AGROW>
        end
    end
end

labelCount = size(labels, 2);
existenceDisagreement = zeros(1, labelCount);
spatialDisagreement = zeros(1, labelCount);
positionUncertainty = zeros(1, labelCount);
supportCount = zeros(1, labelCount);
meanExistence = zeros(1, labelCount);
for labelIndex = 1:labelCount
    existence = zeros(1, numel(receiverIds));
    means = nan(2, numel(receiverIds));
    covariance = nan(2, 2, numel(receiverIds));
    traces = nan(1, numel(receiverIds));
    for receiverIndex = 1:numel(receiverIds)
        summary = summaries{receiverIndex};
        match = find(all(summary.labels == labels(:, labelIndex), 1), 1);
        if isempty(match)
            continue;
        end
        existence(receiverIndex) = summary.existence(match);
        means(:, receiverIndex) = summary.positionMean(:, match);
        covariance(:, :, receiverIndex) = ...
            summary.positionCovariance(:, :, match);
        traces(receiverIndex) = trace(covariance(:, :, receiverIndex));
    end
    active = find(existence >= minimumPresenceExistence);
    supportCount(labelIndex) = numel(active);
    meanExistence(labelIndex) = mean(existence);
    existenceDisagreement(labelIndex) = ...
        max(existence) - min(existence);
    if ~isempty(active)
        positionUncertainty(labelIndex) = median(traces(active));
    end
    for leftIndex = 1:numel(active)
        for rightIndex = leftIndex + 1:numel(active)
            left = active(leftIndex);
            right = active(rightIndex);
            scale = sqrt(max(trace( ...
                covariance(:, :, left) + covariance(:, :, right)), 1));
            spatialDisagreement(labelIndex) = max( ...
                spatialDisagreement(labelIndex), ...
                norm(means(:, left) - means(:, right)) / scale);
        end
    end
end
metrics = struct( ...
    'existenceDisagreement', existenceDisagreement, ...
    'spatialDisagreement', spatialDisagreement, ...
    'positionUncertainty', positionUncertainty, ...
    'supportCount', supportCount, ...
    'meanExistence', meanExistence);
end

function ranking = rankLabels(labels, score, metrics, eligible)
indices = find(eligible & isfinite(score(:)'));
if isempty(indices)
    ranking = zeros(2, 0);
    return;
end
keys = [ ...
    -score(indices), ...
    -metrics.supportCount(indices)', ...
    -metrics.meanExistence(indices)', ...
    labels(1, indices)', labels(2, indices)'];
[~, order] = sortrows(keys, 1:size(keys, 2));
ranking = labels(:, indices(order));
end

function dimension = inferStateDimension(posteriors, receiverIds)
dimension = 0;
for receiverId = reshape(receiverIds, 1, [])
    objects = posteriors{receiverId};
    for objectIndex = 1:numel(objects)
        if objects(objectIndex).numberOfGmComponents > 0 && ...
                ~isempty(objects(objectIndex).mu)
            dimension = numel(objects(objectIndex).mu{1});
            break;
        end
    end
    if dimension > 0
        break;
    end
end
if dimension < 2
    error('BeneficiaryOnlyQueryKeyRecallV229:StateDimension', ...
        'A position-bearing Gaussian-mixture state is required.');
end
end

function writeReport(path, result)
fid = fopen(path, 'w');
if fid < 0
    error('BeneficiaryOnlyQueryKeyRecallV229:WriteReport', ...
        'Could not write the V229 recall report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V229 beneficiary-only query-key recall\n\n');
fprintf(fid, '- State: `%s / seed %d / t=%d`\n', ...
    result.presetName, result.seed, result.currentTime);
fprintf(fid, '- Query cap: `%d labels`\n', ...
    result.maximumQueriedLabels);
fprintf(fid, '- Teacher recall: `%d / %d = %.1f%%`\n', ...
    result.recoveredTeacherLabelCount, result.teacherLabelCount, ...
    100 * result.teacherRecall);
fprintf(fid, ['- Selection inputs: beneficiary current local posteriors ', ...
    'only; no remote inventory, truth or future outcome\n\n']);
fprintf(fid, ['| Row | Beneficiary | Source / teacher label | ', ...
    'Ranks: existence / spatial / uncertainty | Selected | ', ...
    'Support / mean r |\n']);
fprintf(fid, '|--:|--:|:--|:--|:--|:--|\n');
for audit = result.teacherAudits
    fprintf(fid, ['| %d | F%d | S%d / `[%d,%d]` | ', ...
        '`%s / %s / %s` | `%d` | `%d / %.3f` |\n'], ...
        audit.recordIndex, audit.beneficiaryFormationId, ...
        audit.sourceId, audit.label(1), audit.label(2), ...
        rankText(audit.modeRanks(1)), rankText(audit.modeRanks(2)), ...
        rankText(audit.modeRanks(3)), audit.selectedByTopThree, ...
        audit.supportCount, audit.meanExistence);
end
fprintf(fid, '\n## Selected keys by formation\n\n');
for formation = result.formations
    fprintf(fid, '- F%d: `%s`\n', formation.formationId, ...
        labelsText(formation.selectedLabels));
end
fprintf(fid, '\n## Decision\n\n');
if result.onlineQueryKeyRuleAuthorized
    fprintf(fid, ['The frozen three-mode beneficiary-only selector ', ...
        'recovers every audited teacher label. Expand recall testing to ', ...
        'grouped trajectories before online integration.\n']);
else
    fprintf(fid, ['The frozen three-mode beneficiary-only selector misses ', ...
        'at least one audited teacher label and is not authorized as the ', ...
        'V228 query-key rule. Preserve teacher headroom as a mechanism ', ...
        'test and design a source-offer or query/offer hybrid control plane ', ...
        'that can expose remote surprise evidence.\n']);
end
fprintf(fid, '\n## Evidence boundary\n\n%s\n', result.evidenceBoundary);
end

function text = labelsText(labels)
if isempty(labels)
    text = 'none';
    return;
end
parts = cell(1, size(labels, 2));
for labelIndex = 1:size(labels, 2)
    parts{labelIndex} = sprintf('[%d,%d]', ...
        labels(1, labelIndex), labels(2, labelIndex));
end
text = strjoin(parts, ', ');
end

function text = rankText(rank)
if isfinite(rank)
    text = sprintf('%d', rank);
else
    text = '-';
end
end

function value = valueAt(values, index, fallback)
if isempty(index)
    value = fallback;
else
    value = values(index);
end
end

function value = emptyFormation()
value = struct('formationId', 0, 'receiverIds', zeros(1, 0), ...
    'labels', zeros(2, 0), 'metrics', struct(), ...
    'eligibleMask', false(1, 0), 'modeNames', {{}}, ...
    'modeScores', zeros(0, 3), 'rankings', {{}}, ...
    'selectedLabels', zeros(2, 0), 'maximumQueriedLabels', 3);
end

function value = emptyAudit()
value = struct('recordIndex', 0, 'beneficiaryFormationId', 0, ...
    'sourceId', 0, 'label', zeros(2, 1), 'labelPresent', false, ...
    'modeRanks', inf(1, 3), 'selectedByTopThree', false, ...
    'supportCount', 0, 'meanExistence', NaN, ...
    'existenceDisagreement', NaN, 'spatialDisagreement', NaN, ...
    'positionUncertainty', NaN);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
