function [reportPath, dataset] = ...
        buildSecondRolloutAggregatedActionDatasetV181(options)
% BUILDSECONDROLLOUTAGGREGATEDACTIONDATASETV181 Second DAgger dataset.
%
% The previously opened V176 rollout heldout cells are promoted to
% training only after V179 has been evaluated.  V180 t=78 duplicates V176
% exactly and is excluded.  V180 t=79 is the sole new heldout group.

if nargin < 1 || isempty(options)
    options = struct();
end
firstAggregatePath = getField(options, 'firstAggregatePath', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v178', 'rollout_aggregated_action_value', ...
    'ROLLOUT_AGGREGATED_ACTION_VALUE_DATASET_V178.mat'));
secondRolloutPath = getField(options, 'secondRolloutPath', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v180', 'recursive_rollout_action_value', ...
    'RECURSIVE_V179_ROLLOUT_ACTION_VALUE_DATASET_V180.mat'));
outputRoot = getField(options, 'outputRoot', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v181', 'second_rollout_aggregated_action_value'));
if exist(firstAggregatePath, 'file') ~= 2 || ...
        exist(secondRolloutPath, 'file') ~= 2
    error('SecondRolloutAggregateV181:MissingInput', ...
        'The V178 aggregate and V180 rollout datasets are required.');
end
first = load(firstAggregatePath, 'dataset').dataset;
second = load(secondRolloutPath, 'dataset').dataset;
validateInputs(first, second);

rows = first.rows;
cells = first.cells;
provenance = first.cellProvenance;
promotedCellIds = [cells([cells.split] == 3).cellId];
for idx = 1:numel(rows)
    if ismember(rows(idx).cellId, promotedCellIds)
        rows(idx).split = 1;
    end
end
for idx = 1:numel(cells)
    if ismember(cells(idx).cellId, promotedCellIds)
        cells(idx).split = 1;
    end
end
for idx = 1:numel(provenance)
    if ismember(provenance(idx).cellId, promotedCellIds)
        provenance(idx).split = 1;
        provenance(idx).sourceName = 'V176-rollout-promoted';
    end
end

secondHeldout = second.cells([second.cells.split] == 3);
for sourceCell = reshape(secondHeldout, 1, [])
    originalCellId = sourceCell.cellId;
    sourceRows = second.rows([second.rows.cellId] == originalCellId);
    newCellId = numel(cells) + 1;
    for idx = 1:numel(sourceRows)
        sourceRows(idx).cellId = newCellId;
    end
    sourceCell.cellId = newCellId;
    rows = [rows, sourceRows]; %#ok<AGROW>
    cells(end + 1) = sourceCell; %#ok<AGROW>
    item = provenance(1);
    item.cellId = newCellId;
    item.sourceName = 'V180-rollout-heldout';
    item.sourceContract = second.contractVersion;
    item.sourceCellId = originalCellId;
    item.split = 3;
    item.time = sourceCell.time;
    item.formation = sourceCell.formation;
    item.receiver = sourceCell.receiver;
    provenance(end + 1) = item; %#ok<AGROW>
end

dataset = struct();
dataset.contractVersion = ...
    'second-rollout-aggregated-label-action-value-dataset-v181-v1';
dataset.featureContractVersion = first.featureContractVersion;
dataset.presetName = first.presetName;
dataset.seed = first.seed;
dataset.sourceDatasetPaths = {firstAggregatePath, secondRolloutPath};
dataset.sourceDatasetContracts = { ...
    first.contractVersion, second.contractVersion};
dataset.featureNames = first.featureNames;
dataset.featureCount = numel(dataset.featureNames);
dataset.rows = rows;
dataset.cells = cells;
dataset.cellProvenance = provenance;
dataset.promotedV176CellIds = promotedCellIds;
dataset.excludedDuplicateV180TrainingCells = ...
    [second.cells([second.cells.split] == 1).cellId];
dataset.splitNames = {'training', 'calibration', 'heldout'};
dataset.trainingCellIds = [cells([cells.split] == 1).cellId];
dataset.calibrationCellIds = [cells([cells.split] == 2).cellId];
dataset.heldoutCellIds = [cells([cells.split] == 3).cellId];
dataset.featuresUseTruth = false;
dataset.featuresUseFutureInformation = false;
dataset.numericLabelIdentifiersUsedAsFeatures = false;
dataset.targetsUseCurrentTruth = true;
dataset.targetsUseFutureOutcome = false;
dataset.policyTrained = false;
dataset.recursiveEvaluationRun = false;
dataset.deployable = false;
dataset.evidenceBoundary = [ ...
    'V181 is the second opened seed-211 policy-iteration dataset. It ', ...
    'retains the V178 static and V176 t=78 training cells, promotes the ', ...
    'V176 t=79 cells to training only after V179 evaluation, and keeps ', ...
    'the original V166 F3 calibration cells unchanged. V180 t=78 is ', ...
    'excluded because it is feature-identical to V176 t=78; only V180 ', ...
    't=79 is held out. Features remain present-time and truth-free, while ', ...
    'current truth supplies immediate action targets. This same-seed ', ...
    'DAgger-style update supports method development only.'];

if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
matPath = fullfile(outputRoot, ...
    'SECOND_ROLLOUT_AGGREGATED_ACTION_DATASET_V181.mat');
reportPath = fullfile(outputRoot, ...
    'SECOND_ROLLOUT_AGGREGATED_ACTION_DATASET_V181.md');
dataset.matPath = matPath;
dataset.reportPath = reportPath;
save('-mat7-binary', matPath, 'dataset');
writeReport(reportPath, dataset);
fprintf('V181 second-rollout aggregate: %s\n', reportPath);
end

function validateInputs(first, second)
if ~strcmp(first.contractVersion, ...
        'rollout-aggregated-label-action-value-dataset-v178-v1') || ...
        ~strcmp(second.contractVersion, ...
            'recursive-rollout-label-action-value-dataset-v180-v1') || ...
        ~strcmp(first.featureContractVersion, ...
            second.featureContractVersion) || ...
        ~isequal(first.featureNames, second.featureNames) || ...
        first.featuresUseTruth || second.featuresUseTruth || ...
        first.featuresUseFutureInformation || ...
        second.featuresUseFutureInformation || ...
        first.numericLabelIdentifiersUsedAsFeatures || ...
        second.numericLabelIdentifiersUsedAsFeatures || ...
        ~first.targetsUseCurrentTruth || ...
        ~second.targetsUseCurrentTruth || ...
        first.targetsUseFutureOutcome || second.targetsUseFutureOutcome || ...
        numel(second.trainingCellIds) ~= 6 || ...
        numel(second.heldoutCellIds) ~= 6
    error('SecondRolloutAggregateV181:Contract', ...
        'The V178/V180 dataset contracts are incompatible.');
end
end

function writeReport(path, dataset)
fid = fopen(path, 'w');
if fid < 0
    error('SecondRolloutAggregateV181:ReportOpenFailed', ...
        'Could not open the V181 report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V181 second rollout-aggregated action dataset\n\n');
fprintf(fid, '- Features / rows / cells: `%d / %d / %d`\n', ...
    dataset.featureCount, numel(dataset.rows), numel(dataset.cells));
fprintf(fid, '- Duplicate V180 t=78 cells excluded: `%d`\n\n', ...
    numel(dataset.excludedDuplicateV180TrainingCells));
fprintf(fid, '| Split | Cells | Rows | Joint-positive | Safe cells |\n');
fprintf(fid, '|:--|--:|--:|--:|--:|\n');
for split = 1:3
    selectedCells = dataset.cells([dataset.cells.split] == split);
    cellIds = [selectedCells.cellId];
    selectedRows = dataset.rows(ismember([dataset.rows.cellId], cellIds));
    fprintf(fid, '| %s | %d | %d | %d | %d |\n', ...
        dataset.splitNames{split}, numel(selectedCells), ...
        numel(selectedRows), ...
        nnz([selectedRows.eospaGain] > 0 & ...
            [selectedRows.rmseGain] > 0), ...
        nnz([selectedCells.safeCandidateCount] > 0));
end
fprintf(fid, '\n## Evidence boundary\n\n%s\n', ...
    dataset.evidenceBoundary);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
