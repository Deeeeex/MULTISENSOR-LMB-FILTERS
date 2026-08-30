function summary = analyzeSparseReferenceLabelGapV156( ...
        candidateScreenPath, referenceScreenPath, outputPath)
% ANALYZESPARSEREFERENCELABELGAPV156 Measure the V126 rollback label gap.
%
% This is a representational preflight, not an outcome experiment.  It asks
% whether the complete static posterior used by the privileged V126 rollback
% differs from the working V105 posterior mainly through a small number of
% labels at the registered rollback node-time cells.

if nargin < 3
    outputPath = '';
end
candidate = load(candidateScreenPath);
reference = load(referenceScreenPath);
candidateOutcome = outcomeByAction( ...
    candidate.screen, 'v105-protection-only-h8');
referenceOutcome = outcomeByAction( ...
    reference.screen, 'reference-full-payload');
candidatePages = candidateOutcome.fusedPosteriorSnapshotsByTime;
referencePages = referenceOutcome.fusedPosteriorSnapshotsByTime;
protocol = getShadowStateRollbackV126Protocol();
if numel(candidatePages) ~= protocol.horizonSteps || ...
        numel(referencePages) ~= protocol.horizonSteps
    error('SparseReferenceV156:SnapshotHorizonMismatch', ...
        'Both snapshot captures must cover the registered V126 horizon.');
end

existenceThreshold = 1e-2;
formationSize = 6;
rows = repmat(emptyRow(), 1, 0);
for pageIdx = 1:protocol.horizonSteps
    formationIds = protocol.rollbackFormationIdsByTime{pageIdx};
    for formationId = formationIds
        sensorIds = (formationId - 1) * formationSize + (1:formationSize);
        for sensorId = sensorIds
            working = candidatePages{pageIdx}{sensorId};
            static = referencePages{pageIdx}{sensorId};
            row = comparePosteriorPair( ...
                working, static, existenceThreshold);
            row.page = pageIdx;
            row.time = protocol.rollbackTimes(pageIdx);
            row.formation = formationId;
            row.sensor = sensorId;
            rows(end + 1) = row; %#ok<AGROW>
        end
    end
end

summary = struct();
summary.contractVersion = 'sparse-reference-label-gap-v156-v1';
summary.candidateScreenPath = candidateScreenPath;
summary.referenceScreenPath = referenceScreenPath;
summary.rollbackCellCount = numel(rows);
summary.rows = rows;
summary.mapSymmetricDifference = [rows.mapSymmetricDifference];
summary.supportCrossingCount = [rows.supportCrossingCount];
summary.materialLabelGapCount = [rows.materialLabelGapCount];
summary.commonMapPositionGapMean = [rows.commonMapPositionGapMean];
summary.cellsWithExactMapLabelSet = ...
    sum(summary.mapSymmetricDifference == 0);
summary.cellsWithAtMostTwoMapLabelEdits = ...
    sum(summary.mapSymmetricDifference <= 2);
summary.maximumMapLabelEdits = ...
    max(summary.mapSymmetricDifference);
summary.medianMapLabelEdits = ...
    median(summary.mapSymmetricDifference);
summary.maximumMaterialLabelGap = ...
    max(summary.materialLabelGapCount);
summary.medianMaterialLabelGap = ...
    median(summary.materialLabelGapCount);

if ~isempty(outputPath)
    writeReport(summary, outputPath);
end
end

function outcome = outcomeByAction(screen, actionName)
idx = find(strcmp({screen.records.actionName}, actionName), 1);
if isempty(idx)
    error('SparseReferenceV156:MissingAction', ...
        'The requested captured action is unavailable: %s', actionName);
end
outcome = screen.outcomes(idx);
if isempty(outcome.fusedPosteriorSnapshotsByTime)
    error('SparseReferenceV156:MissingFusedSnapshots', ...
        'The requested captured action has no fused posterior snapshots.');
end
end

function row = comparePosteriorPair(working, static, threshold)
workingSummary = posteriorSummary(working, threshold);
staticSummary = posteriorSummary(static, threshold);
allKeys = unique([workingSummary.keys, staticSummary.keys]);
workingMap = workingSummary.mapKeys;
staticMap = staticSummary.mapKeys;
mapGap = setxor(workingMap, staticMap);
supportCrossingCount = 0;
materialLabelGapCount = 0;
existenceGapMaximum = 0;
positionGapMaximum = 0;
commonMapPositionGaps = zeros(1, 0);
for keyIdx = 1:numel(allKeys)
    key = allKeys{keyIdx};
    workingIdx = find(strcmp(workingSummary.keys, key), 1);
    staticIdx = find(strcmp(staticSummary.keys, key), 1);
    workingR = 0;
    staticR = 0;
    if ~isempty(workingIdx)
        workingR = workingSummary.existence(workingIdx);
    end
    if ~isempty(staticIdx)
        staticR = staticSummary.existence(staticIdx);
    end
    supportCrossing = (workingR > threshold) ~= (staticR > threshold);
    supportCrossingCount = supportCrossingCount + supportCrossing;
    existenceGap = abs(workingR - staticR);
    existenceGapMaximum = max(existenceGapMaximum, existenceGap);
    positionGap = 0;
    if ~isempty(workingIdx) && ~isempty(staticIdx)
        positionGap = norm( ...
            workingSummary.position(:, workingIdx) - ...
            staticSummary.position(:, staticIdx));
    elseif workingR > threshold || staticR > threshold
        positionGap = inf;
    end
    positionGapMaximum = max(positionGapMaximum, positionGap);
    mapMembershipMismatch = ...
        ismember(key, workingMap) ~= ismember(key, staticMap);
    material = mapMembershipMismatch || supportCrossing || ...
        existenceGap >= 0.05 || positionGap >= 25;
    materialLabelGapCount = materialLabelGapCount + material;
    if ismember(key, workingMap) && ismember(key, staticMap) && ...
            isfinite(positionGap)
        commonMapPositionGaps(end + 1) = positionGap; %#ok<AGROW>
    end
end
if isempty(commonMapPositionGaps)
    commonMapPositionGapMean = NaN;
else
    commonMapPositionGapMean = mean(commonMapPositionGaps);
end
row = emptyRow();
row.workingLabelCount = numel(workingSummary.keys);
row.staticLabelCount = numel(staticSummary.keys);
row.workingMapCardinality = numel(workingMap);
row.staticMapCardinality = numel(staticMap);
row.mapSymmetricDifference = numel(mapGap);
row.supportCrossingCount = supportCrossingCount;
row.materialLabelGapCount = materialLabelGapCount;
row.existenceGapMaximum = existenceGapMaximum;
row.positionGapMaximum = positionGapMaximum;
row.commonMapPositionGapMean = commonMapPositionGapMean;
row.workingMapLabels = strjoin(workingMap, ', ');
row.staticMapLabels = strjoin(staticMap, ', ');
end

function summary = posteriorSummary(objects, threshold)
objects = reshape(objects, 1, []);
keys = cell(1, numel(objects));
existence = zeros(1, numel(objects));
position = zeros(2, numel(objects));
for objectIdx = 1:numel(objects)
    object = objects(objectIdx);
    keys{objectIdx} = labelKey(object);
    existence(objectIdx) = object.r;
    if object.numberOfGmComponents > 0
        position(:, objectIdx) = object.mu{1}(1:2);
    else
        position(:, objectIdx) = [NaN; NaN];
    end
end
if numel(unique(keys)) ~= numel(keys)
    error('SparseReferenceV156:DuplicateLabel', ...
        'A captured LMB posterior contains duplicate labels.');
end
active = existence > threshold;
activeExistence = existence(active);
activeKeys = keys(active);
mapKeys = cell(1, 0);
if ~isempty(activeExistence)
    [~, mapIndices] = lmbMapCardinalityEstimate(activeExistence);
    mapKeys = activeKeys(mapIndices);
end
summary = struct( ...
    'keys', {keys}, ...
    'existence', existence, ...
    'position', position, ...
    'mapKeys', {mapKeys});
end

function key = labelKey(object)
key = sprintf('%d:%d', object.birthTime, object.birthLocation);
end

function row = emptyRow()
row = struct( ...
    'page', 0, ...
    'time', 0, ...
    'formation', 0, ...
    'sensor', 0, ...
    'workingLabelCount', 0, ...
    'staticLabelCount', 0, ...
    'workingMapCardinality', 0, ...
    'staticMapCardinality', 0, ...
    'mapSymmetricDifference', 0, ...
    'supportCrossingCount', 0, ...
    'materialLabelGapCount', 0, ...
    'existenceGapMaximum', 0, ...
    'positionGapMaximum', 0, ...
    'commonMapPositionGapMean', NaN, ...
    'workingMapLabels', '', ...
    'staticMapLabels', '');
end

function writeReport(summary, outputPath)
folder = fileparts(outputPath);
if ~isempty(folder) && ~exist(folder, 'dir')
    mkdir(folder);
end
fileId = fopen(outputPath, 'w');
if fileId < 0
    error('SparseReferenceV156:ReportOpenFailed', ...
        'Unable to open the V156 preflight report.');
end
cleanup = onCleanup(@() fclose(fileId)); %#ok<NASGU>
fprintf(fileId, '# V156 sparse reference-label preflight\n\n');
fprintf(fileId, ['This diagnostic compares the V105 working fused posterior ', ...
    'with the paired static fused posterior only at the 36 privileged V126 ', ...
    'rollback cells. It does not measure tracking gain and does not establish ', ...
    'a deployable policy.\n\n']);
fprintf(fileId, '- Rollback cells: `%d`\n', summary.rollbackCellCount);
fprintf(fileId, '- Exact MAP-label-set matches: `%d / %d`\n', ...
    summary.cellsWithExactMapLabelSet, summary.rollbackCellCount);
fprintf(fileId, '- Cells needing at most two MAP-label edits: `%d / %d`\n', ...
    summary.cellsWithAtMostTwoMapLabelEdits, summary.rollbackCellCount);
fprintf(fileId, '- Median / maximum MAP-label edits: `%.1f / %d`\n', ...
    summary.medianMapLabelEdits, summary.maximumMapLabelEdits);
fprintf(fileId, '- Median / maximum material label gaps: `%.1f / %d`\n\n', ...
    summary.medianMaterialLabelGap, summary.maximumMaterialLabelGap);
fprintf(fileId, ['A material label gap means a MAP-membership mismatch, an ', ...
    'existence-threshold crossing, |Delta r| >= 0.05, or a first-component ', ...
    'position gap >= 25 m.\n\n']);
fprintf(fileId, ['| t | Formation | Sensor | MAP cards W/R | MAP edits | ', ...
    'Support crossings | Material labels | Max |Delta r| | ', ...
    'Mean common-MAP position gap |\n']);
fprintf(fileId, '|--:|--:|--:|:--:|--:|--:|--:|--:|--:|\n');
for row = summary.rows
    fprintf(fileId, ...
        '| %d | %d | %d | %d/%d | %d | %d | %d | %.3f | %.2f |\n', ...
        row.time, row.formation, row.sensor, ...
        row.workingMapCardinality, row.staticMapCardinality, ...
        row.mapSymmetricDifference, row.supportCrossingCount, ...
        row.materialLabelGapCount, row.existenceGapMaximum, ...
        row.commonMapPositionGapMean);
end
end
