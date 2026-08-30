function [reportPath, summary] = ...
        analyzeCausalSourceLabelAvailabilityV159(options)
% ANALYZECAUSALSOURCELABELAVAILABILITYV159 Source-content preflight.
%
% For high-value V157 restores, test whether a same-label complete
% Bernoulli density already held in a current local or pre-rollback fused
% posterior can reproduce the reference edit's immediate truth E-OSPA gain.
% Truth selects and scores sources only for this offline mechanism oracle.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getPositiveValueReferenceLabelV157Protocol();
valueThreshold = getField(options, 'valueThreshold', 0.1);
if ~isscalar(valueThreshold) || ~isfinite(valueThreshold) || ...
        valueThreshold < 0
    error('CausalSourceV159:InvalidThreshold', ...
        'The V159 value threshold must be a nonnegative scalar.');
end
candidateScreenPath = getField(options, 'candidateScreenPath', fullfile( ...
    protocol.headroomOutputRoot, 'x36_t72_h8', 'screen', ...
    ['TRACKING_ALIGNED_X36_SCHEDULE_H8_', ...
     'X36_FORMATION_FOV_SEED211_T72_D8.mat']));
referenceScreenPath = getField(options, 'referenceScreenPath', ...
    protocol.shadowCaptureScreenPath);
selectionPath = getField(options, 'selectionPath', fullfile( ...
    protocol.headroomOutputRoot, 'selection_replay', ...
    'POSITIVE_VALUE_REFERENCE_LABEL_SELECTION_V158.mat'));
outputRoot = getField(options, 'outputRoot', fullfile( ...
    protocol.headroomOutputRoot, 'causal_source_preflight'));
reportPath = fullfile(outputRoot, ...
    'CAUSAL_SOURCE_LABEL_AVAILABILITY_V159.md');
matPath = fullfile(outputRoot, ...
    'CAUSAL_SOURCE_LABEL_AVAILABILITY_V159.mat');
if exist(candidateScreenPath, 'file') ~= 2 || ...
        exist(referenceScreenPath, 'file') ~= 2 || ...
        exist(selectionPath, 'file') ~= 2
    error('CausalSourceV159:MissingInput', ...
        'The V157 screen, static screen and V158 replay are required.');
end

candidateLoaded = load(candidateScreenPath, 'screen');
referenceLoaded = load(referenceScreenPath, 'screen');
selectionLoaded = load(selectionPath, 'summary');
candidateScreen = candidateLoaded.screen;
referenceScreen = referenceLoaded.screen;
selection = selectionLoaded.summary;
candidateOutcome = outcomeByAction(candidateScreen, ...
    protocol.candidateActionName);
referenceOutcome = outcomeByAction(referenceScreen, ...
    'reference-full-payload');
workingPages = candidateOutcome.fusedPosteriorSnapshotsByTime;
localPages = candidateOutcome.localPosteriorSnapshotsByTime;
referencePages = referenceOutcome.fusedPosteriorSnapshotsByTime;
if isempty(workingPages) || isempty(localPages) || ...
        isempty(referencePages)
    error('CausalSourceV159:MissingSnapshots', ...
        'Current local, working fused and reference snapshots are required.');
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

rows = repmat(emptyRow(), 1, 0);
for selectionRow = selection.rows
    pageIdx = selectionRow.page;
    receiverIdx = selectionRow.sensor;
    currentTime = selectionRow.time;
    working = workingPages{pageIdx}{receiverIdx};
    physical = logical(inputs.graphData. ...
        physicalAdjacency(:, :, currentTime));
    physical = physical | physical';
    physical(1:nodeCount+1:end) = false;
    for rankIdx = 1:selectionRow.selectedLabelCount
        label = selectionRow.selectedLabels(:, rankIdx);
        referenceObject = findLabelObject( ...
            referencePages{pageIdx}{receiverIdx}, label);
        if selectionRow.selectedTombstone(rankIdx) || ...
                isempty(referenceObject)
            error('CausalSourceV159:UnexpectedTombstone', ...
                'V159 is frozen to the V158 complete-label restores.');
        end
        currentRisk = evaluateRisk(working, oracleModel, currentTime);
        referenceTrial = replaceLabelObject(working, referenceObject);
        referenceRisk = evaluateRisk( ...
            referenceTrial, oracleModel, currentTime);
        referenceGain = currentRisk - referenceRisk;
        expectedGain = ...
            selectionRow.marginalEospaReduction(rankIdx);
        if abs(referenceGain - expectedGain) > 1e-6
            error('CausalSourceV159:ReplayDrift', ...
                'The V157 marginal gain did not replay exactly.');
        end
        if referenceGain > valueThreshold
            row = emptyRow();
            row.page = pageIdx;
            row.time = currentTime;
            row.formation = groupIds(receiverIdx);
            row.receiver = receiverIdx;
            row.rank = rankIdx;
            row.label = label;
            row.labelText = sprintf('(%d,%d)', label(1), label(2));
            row.referenceGain = referenceGain;
            row.local = findBestSource( ...
                working, localPages{pageIdx}, label, ...
                currentRisk, oracleModel, currentTime, physical, ...
                receiverIdx, groupIds);
            row.fused = findBestSource( ...
                working, workingPages{pageIdx}, label, ...
                currentRisk, oracleModel, currentTime, physical, ...
                receiverIdx, groupIds);
            row.combined = betterSource(row.local, row.fused);
            row.localCoveragePercent = cappedCoverage( ...
                row.local.gain, referenceGain);
            row.fusedCoveragePercent = cappedCoverage( ...
                row.fused.gain, referenceGain);
            row.combinedCoveragePercent = cappedCoverage( ...
                row.combined.gain, referenceGain);
            rows(end + 1) = row; %#ok<AGROW>
        end
        working = referenceTrial;
    end
end
if isempty(rows)
    error('CausalSourceV159:NoHighValueEdits', ...
        'No V157 edit exceeded the frozen value threshold.');
end

referenceGains = [rows.referenceGain];
localSources = [rows.local];
fusedSources = [rows.fused];
combinedSources = [rows.combined];
localGains = max([localSources.gain], 0);
fusedGains = max([fusedSources.gain], 0);
combinedGains = max([combinedSources.gain], 0);
summary = struct();
summary.contractVersion = 'causal-source-label-availability-v159-v1';
summary.presetName = candidateScreen.presetName;
summary.seed = candidateScreen.seed;
summary.valueThreshold = valueThreshold;
summary.highValueEditCount = numel(rows);
summary.highValueReferenceGain = sum(referenceGains);
summary.rows = rows;
summary.localPositiveSourceCount = nnz(localGains > 1e-9);
summary.fusedPositiveSourceCount = nnz(fusedGains > 1e-9);
summary.combinedPositiveSourceCount = nnz(combinedGains > 1e-9);
summary.localHalfValueSourceCount = nnz( ...
    localGains >= 0.5 * referenceGains);
summary.fusedHalfValueSourceCount = nnz( ...
    fusedGains >= 0.5 * referenceGains);
summary.combinedHalfValueSourceCount = nnz( ...
    combinedGains >= 0.5 * referenceGains);
summary.localCappedGainCoveragePercent = gainCoverage( ...
    localGains, referenceGains);
summary.fusedCappedGainCoveragePercent = gainCoverage( ...
    fusedGains, referenceGains);
summary.combinedCappedGainCoveragePercent = gainCoverage( ...
    combinedGains, referenceGains);
summary.localImmediateSourceCount = nnz( ...
    [localSources.gain] > 1e-9 & [localSources.hopCount] <= 1);
summary.fusedImmediateSourceCount = nnz( ...
    [fusedSources.gain] > 1e-9 & [fusedSources.hopCount] <= 1);
summary.combinedImmediateSourceCount = nnz( ...
    [combinedSources.gain] > 1e-9 & ...
    [combinedSources.hopCount] <= 1);
summary.localImmediateCappedGainCoveragePercent = ...
    immediateGainCoverage(localSources, referenceGains);
summary.fusedImmediateCappedGainCoveragePercent = ...
    immediateGainCoverage(fusedSources, referenceGains);
summary.combinedImmediateCappedGainCoveragePercent = ...
    immediateGainCoverage(combinedSources, referenceGains);
summary.sameFormationBestCombinedCount = nnz( ...
    [combinedSources.sourceFormation] == [rows.formation]);
summary.truthUsed = true;
summary.deployable = false;
summary.evidenceBoundary = [ ...
    'V159 uses current local and captured pre-rollback fused posteriors ', ...
    'as source content, but truth still chooses the best source and ', ...
    'scores immediate E-OSPA. Network-wide availability does not prove ', ...
    'that a multi-hop payload can arrive before its value expires. ', ...
    'Immediate coverage includes only the receiver itself or a current ', ...
    'one-hop physical neighbor.'];

if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
save('-mat7-binary', matPath, 'summary');
writeReport(reportPath, summary);
fprintf('V159 causal-source preflight: %s\n', reportPath);
end

function best = findBestSource( ...
        working, pages, label, currentRisk, model, currentTime, ...
        physical, receiverIdx, groupIds)
best = emptySource();
for sourceIdx = 1:numel(pages)
    sourceObject = findLabelObject(pages{sourceIdx}, label);
    if isempty(sourceObject)
        continue;
    end
    trial = replaceLabelObject(working, sourceObject);
    trialRisk = evaluateRisk(trial, model, currentTime);
    gain = currentRisk - trialRisk;
    if gain > best.gain + 1e-12 || ...
            (abs(gain - best.gain) <= 1e-12 && ...
             (best.source == 0 || sourceIdx < best.source))
        best.source = sourceIdx;
        best.sourceFormation = groupIds(sourceIdx);
        best.gain = gain;
        best.hopCount = shortestPathLength( ...
            physical, sourceIdx, receiverIdx);
        best.payloadBytes = labelPayloadBytes(sourceObject);
    end
end
end

function source = betterSource(left, right)
if right.gain > left.gain + 1e-12
    source = right;
    source.kind = 'fused';
else
    source = left;
    source.kind = 'local';
end
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

function hops = shortestPathLength(adjacency, source, target)
if source == target
    hops = 0;
    return;
end
visited = false(1, size(adjacency, 1));
visited(source) = true;
frontier = source;
hops = inf;
for distance = 1:(size(adjacency, 1) - 1)
    next = zeros(1, 0);
    for node = frontier
        neighbors = reshape(find(adjacency(node, :)), 1, []);
        neighbors = neighbors(~visited(neighbors));
        if any(neighbors == target)
            hops = distance;
            return;
        end
        visited(neighbors) = true;
        next = [next, neighbors]; %#ok<AGROW>
    end
    frontier = unique(next, 'stable');
    if isempty(frontier)
        return;
    end
end
end

function bytes = labelPayloadBytes(object)
dimension = numel(object.mu{1});
bytes = 8 * (3 + object.numberOfGmComponents * ...
    (1 + dimension + dimension * dimension));
end

function value = cappedCoverage(gain, referenceGain)
value = 100 * max(min(gain, referenceGain), 0) / ...
    max(referenceGain, eps);
end

function value = gainCoverage(gains, referenceGains)
value = 100 * sum(max(min(gains, referenceGains), 0)) / ...
    max(sum(referenceGains), eps);
end

function value = immediateGainCoverage(sources, referenceGains)
gains = [sources.gain];
hops = [sources.hopCount];
gains(hops > 1) = 0;
value = gainCoverage(gains, referenceGains);
end

function outcome = outcomeByAction(screen, actionName)
idx = find(strcmp({screen.records.actionName}, actionName), 1);
if isempty(idx)
    error('CausalSourceV159:MissingAction', ...
        'The requested action is missing: %s', actionName);
end
outcome = screen.outcomes(idx);
end

function writeReport(path, summary)
fileId = fopen(path, 'w');
if fileId < 0
    error('CausalSourceV159:ReportOpenFailed', ...
        'Could not open the V159 report.');
end
cleanup = onCleanup(@() fclose(fileId)); %#ok<NASGU>
fprintf(fileId, '# V159 causal source-label availability\n\n');
fprintf(fileId, '- Preset / seed: `%s / %d`\n', ...
    summary.presetName, summary.seed);
fprintf(fileId, '- Frozen reference-gain threshold: `> %.3f`\n', ...
    summary.valueThreshold);
fprintf(fileId, '- High-value restores: `%d`\n', ...
    summary.highValueEditCount);
fprintf(fileId, '- High-value reference gain: `%.6f`\n\n', ...
    summary.highValueReferenceGain);
fprintf(fileId, ['| Source content | Positive edits | At least half ', ...
    'reference value | Capped gain coverage | Immediate positive | ', ...
    'Immediate capped coverage |\n']);
fprintf(fileId, '|:--|--:|--:|--:|--:|--:|\n');
fprintf(fileId, '| Current local posterior | %d | %d | %.3f%% | %d | %.3f%% |\n', ...
    summary.localPositiveSourceCount, ...
    summary.localHalfValueSourceCount, ...
    summary.localCappedGainCoveragePercent, ...
    summary.localImmediateSourceCount, ...
    summary.localImmediateCappedGainCoveragePercent);
fprintf(fileId, '| Current fused posterior | %d | %d | %.3f%% | %d | %.3f%% |\n', ...
    summary.fusedPositiveSourceCount, ...
    summary.fusedHalfValueSourceCount, ...
    summary.fusedCappedGainCoveragePercent, ...
    summary.fusedImmediateSourceCount, ...
    summary.fusedImmediateCappedGainCoveragePercent);
fprintf(fileId, '| Best of local/fused | %d | %d | %.3f%% | %d | %.3f%% |\n\n', ...
    summary.combinedPositiveSourceCount, ...
    summary.combinedHalfValueSourceCount, ...
    summary.combinedCappedGainCoveragePercent, ...
    summary.combinedImmediateSourceCount, ...
    summary.combinedImmediateCappedGainCoveragePercent);
fprintf(fileId, '- Best combined source in receiver formation: `%d / %d`\n\n', ...
    summary.sameFormationBestCombinedCount, ...
    summary.highValueEditCount);
fprintf(fileId, ['| t | F | Receiver | Rank | Label | Ref. gain | ', ...
    'Best local gain/source/hops | Best fused gain/source/hops | ', ...
    'Combined coverage |\n']);
fprintf(fileId, '|--:|--:|--:|--:|:--|--:|:--|:--|--:|\n');
for row = summary.rows
    fprintf(fileId, ['| %d | %d | %d | %d | %s | %.4f | ', ...
        '%.4f/%d/%s | %.4f/%d/%s | %.2f%% |\n'], ...
        row.time, row.formation, row.receiver, row.rank, ...
        row.labelText, row.referenceGain, ...
        row.local.gain, row.local.source, formatHops(row.local.hopCount), ...
        row.fused.gain, row.fused.source, formatHops(row.fused.hopCount), ...
        row.combinedCoveragePercent);
end
fprintf(fileId, '\n## Evidence boundary\n\n%s\n', ...
    summary.evidenceBoundary);
end

function text = formatHops(hops)
if isfinite(hops)
    text = sprintf('%d', hops);
else
    text = 'inf';
end
end

function row = emptyRow()
row = struct( ...
    'page', 0, ...
    'time', 0, ...
    'formation', 0, ...
    'receiver', 0, ...
    'rank', 0, ...
    'label', zeros(2, 1), ...
    'labelText', '', ...
    'referenceGain', 0, ...
    'local', emptySource(), ...
    'fused', emptySource(), ...
    'combined', emptySource(), ...
    'localCoveragePercent', 0, ...
    'fusedCoveragePercent', 0, ...
    'combinedCoveragePercent', 0);
end

function source = emptySource()
source = struct( ...
    'kind', '', ...
    'source', 0, ...
    'sourceFormation', 0, ...
    'gain', -inf, ...
    'hopCount', inf, ...
    'payloadBytes', 0);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
