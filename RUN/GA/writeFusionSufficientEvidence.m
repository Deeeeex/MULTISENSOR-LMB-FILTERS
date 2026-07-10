function [reportPath, csvPath, summaryPath] = ...
    writeFusionSufficientEvidence( ...
        summary, config, publishClaim, testAuthorization)
% WRITEFUSIONSUFFICIENTEVIDENCE Publish a claimed evidence bundle.

if nargin < 4
    testAuthorization = struct();
end
validateWriterInputs(summary, config, publishClaim, testAuthorization);
outputDirectory = char(config.outputDirectory);
if exist(outputDirectory, 'dir') ~= 7
    [ok, message] = mkdir(outputDirectory);
    if ~ok
        error('Unable to create evidence directory: %s', message);
    end
end
stem = char(config.artifactStem);
summaryPath = fullfile(outputDirectory, [stem, '.mat']);
csvPath = fullfile(outputDirectory, [stem, '.csv']);
reportPath = fullfile(outputDirectory, [stem, '.md']);
plan = buildFusionSufficientParallelPlan(config);
if ~isequal(plan.finalPaths, {summaryPath, csvPath, reportPath})
    error('Evidence final paths differ from the claimed batch plan.');
end
assertFusionSufficientPublishClaim(plan, config, publishClaim);
assertTargetsAbsent({summaryPath, csvPath, reportPath});

temporaryBase = tempname(outputDirectory);
temporarySummary = [temporaryBase, '.mat'];
temporaryCsv = [temporaryBase, '.csv'];
temporaryReport = [temporaryBase, '.md'];
cleanup = onCleanup(@() deleteTemporaryFiles( ...
    {temporarySummary, temporaryCsv, temporaryReport})); %#ok<NASGU>

save(temporarySummary, 'summary', 'config', '-v7');
rows = buildEvidenceRows(summary);
writeEvidenceCsv(temporaryCsv, rows);
summaryHash = sha256File(temporarySummary);
csvHash = sha256File(temporaryCsv);
aggregate = computeAggregate(rows, config);
reportText = renderFusionSufficientEvidenceReport( ...
    stableArtifactPath(summaryPath), stableArtifactPath(csvPath), ...
    summaryHash, csvHash, rows, aggregate, config);
writeTextFile(temporaryReport, reportText);

validateFusionSufficientEvidence( ...
    temporarySummary, temporaryCsv, temporaryReport, testAuthorization);
publishFusionSufficientEvidenceBundle( ...
    {temporarySummary, temporaryCsv, temporaryReport}, ...
    {summaryPath, csvPath, reportPath}, ...
    @() validateFusionSufficientEvidence( ...
    summaryPath, csvPath, reportPath, testAuthorization));
fprintf('Evidence MAT: %s\n', summaryPath);
fprintf('Evidence CSV: %s\n', csvPath);
fprintf('Evidence report: %s\n', reportPath);
end

function validateWriterInputs( ...
    summary, config, publishClaim, testAuthorization)
requiredConfig = {'evidenceSchema', 'workerSchema', 'executionProtocol', ...
    'testOnly', 'wireSchemaVersion', 'gitCommit', ...
    'requiredSourcesSha256', ...
    'configSha256', 'batchIdentity', 'maxWorkers', 'octaveVersion', ...
    'numberOfTrials', 'baseSeed', 'firstSeed', 'lastSeed', ...
    'armSelection', 'internalArmSelectors', 'simulationLength', ...
    'capturePosteriorSnapshots', ...
    'requiredMaxExistenceResidual', 'requiredMaxMeanResidual', ...
    'requiredMaxCovarianceResidual', 'bootstrapSeed', ...
    'bootstrapResamples', 'byteSemantics', 'changedVariable', ...
    'artifactStem', 'outputDirectory', 'regenerationCommand', 'isSmoke'};
for fieldIdx = 1:numel(requiredConfig)
    if ~isfield(config, requiredConfig{fieldIdx})
        error('Evidence config is missing %s.', requiredConfig{fieldIdx});
    end
end
testMode = isFusionSufficientInternalTestAuthorization(testAuthorization);
if testMode
    validSchema = isequal(config.testOnly, true) && ...
        strcmp(config.evidenceSchema, ...
        'fusion-sufficient-moment-exchange-test-v2') && ...
        strcmp(config.workerSchema, ...
        'fusion-sufficient-seed-worker-test-v2') && ...
        strncmp(config.artifactStem, 'TEST_', 5) && ...
        strncmp(config.batchIdentity, 'test-', 5);
else
    validSchema = isequal(config.testOnly, false) && ...
        strcmp(config.evidenceSchema, ...
        'fusion-sufficient-moment-exchange-v2') && ...
        strcmp(config.workerSchema, ...
        'fusion-sufficient-seed-worker-v2');
end
if ~validSchema
    error('Evidence writer rejects unauthorized production/test schema.');
end
if ~testMode
    assertFusionSufficientConfigProvenance(config);
end
requiredSummary = {'armNames', 'numberOfTrials', 'trialSeeds', ...
    'trials', 'arms', 'scenarioConfig', 'equivalence', 'execution'};
for fieldIdx = 1:numel(requiredSummary)
    if ~isfield(summary, requiredSummary{fieldIdx})
        error('Evidence summary is missing %s.', requiredSummary{fieldIdx});
    end
end
plan = buildFusionSufficientParallelPlan(config);
assertFusionSufficientPublishClaim(plan, config, publishClaim);
if ~isequal(summary.armNames, config.armSelection) || ...
        summary.numberOfTrials ~= config.numberOfTrials || ...
        ~isequal(summary.trialSeeds, config.firstSeed:config.lastSeed)
    error('Summary does not match the frozen evidence configuration.');
end
if isempty(regexp(config.gitCommit, '^[0-9a-f]{40}$', 'once'))
    error('Evidence Git commit must be a lowercase 40-hex hash.');
end
if ~islogical(config.isSmoke) || ~isscalar(config.isSmoke)
    error('Evidence isSmoke must be a scalar logical.');
end
validateExecutionAgainstCurrentBatch(summary, config);
if ~isnumeric(summary.trials.attemptedPayloadBytes) || ...
        ~isnumeric(summary.trials.deliveredPayloadBytes) || ...
        ~isreal(summary.trials.attemptedPayloadBytes) || ...
        ~isreal(summary.trials.deliveredPayloadBytes) || ...
        any(~isfinite(summary.trials.attemptedPayloadBytes(:))) || ...
        any(~isfinite(summary.trials.deliveredPayloadBytes(:))) || ...
        any(summary.trials.attemptedPayloadBytes(:) <= 0) || ...
        any(summary.trials.deliveredPayloadBytes(:) < 0) || ...
        any(summary.trials.attemptedPayloadBytes(:) ~= floor( ...
        summary.trials.attemptedPayloadBytes(:))) || ...
        any(summary.trials.deliveredPayloadBytes(:) ~= floor( ...
        summary.trials.deliveredPayloadBytes(:))) || ...
        any(summary.trials.deliveredPayloadBytes(:) > ...
        summary.trials.attemptedPayloadBytes(:))
    error(['Attempted/delivered bytes must be finite nonnegative ', ...
        'integers, both attempted arms must be positive, and delivered ', ...
        'must not exceed attempted.']);
end
end

function validateExecutionAgainstCurrentBatch(summary, config)
plan = buildFusionSufficientParallelPlan(config);
state = inspectFusionSufficientBatchState(plan);
if ~strcmp(state.name, 'COMPLETE_WORKERS') || ...
        ~isfield(summary, 'execution') || ...
        ~isequal(summary.execution.receiptValidated, true)
    error('Evidence writer requires a sealed COMPLETE_WORKERS batch.');
end
if ~strcmp(summary.execution.batchIdentity, config.batchIdentity) || ...
        ~strcmp(summary.execution.planSha256, ...
        fusionSufficientSha256File(plan.batchPlanPath)) || ...
        ~strcmp(summary.execution.exitLedgerSha256, ...
        fusionSufficientSha256File(plan.exitLedgerPath)) || ...
        ~strcmp(summary.execution.batchSuccessReceiptSha256, ...
        fusionSufficientSha256File(plan.workerSuccessReceiptPath)) || ...
        ~isequal(summary.execution.workerSeeds, plan.workerSeeds)
    error('Evidence execution seal differs from current batch ledger.');
end
for seedIdx = 1:numel(plan.workerSeeds)
    if ~strcmp(summary.execution.workerMatSha256{seedIdx}, ...
            fusionSufficientSha256File( ...
            plan.workerMatPaths{seedIdx})) || ...
            ~strcmp(summary.execution.workerLogSha256{seedIdx}, ...
            fusionSufficientSha256File(plan.workerLogPaths{seedIdx}))
        error('Evidence worker artifact hash differs for seed %d.', ...
            plan.workerSeeds(seedIdx));
    end
end
end

function rows = buildEvidenceRows(summary)
numberOfTrials = summary.numberOfTrials;
rows = zeros(numberOfTrials, 30);
for trialIdx = 1:numberOfTrials
    fullAttempted = summary.trials.attemptedPayloadBytes(trialIdx, 1);
    momentAttempted = summary.trials.attemptedPayloadBytes(trialIdx, 2);
    fullDelivered = summary.trials.deliveredPayloadBytes(trialIdx, 1);
    momentDelivered = summary.trials.deliveredPayloadBytes(trialIdx, 2);
    if ~(isfinite(fullAttempted) && fullAttempted > 0)
        error('Full arm attempted bytes must be positive for every seed.');
    end
    localFull = mean(summary.trials.localEOspa(trialIdx, :, 1));
    localMoment = mean(summary.trials.localEOspa(trialIdx, :, 2));
    consensusFull = summary.trials.consensusOspa(trialIdx, 1);
    consensusMoment = summary.trials.consensusOspa(trialIdx, 2);
    positionFull = summary.trials.consensusPosition(trialIdx, 1);
    positionMoment = summary.trials.consensusPosition(trialIdx, 2);
    cardinalityFull = summary.trials.consensusCardinality(trialIdx, 1);
    cardinalityMoment = ...
        summary.trials.consensusCardinality(trialIdx, 2);
    attemptedMasksEqual = isequal( ...
        summary.trials.attemptedMask{trialIdx, 1}, ...
        summary.trials.attemptedMask{trialIdx, 2});
    deliveredMasksEqual = isequal( ...
        summary.trials.deliveredMask{trialIdx, 1}, ...
        summary.trials.deliveredMask{trialIdx, 2});
    rows(trialIdx, :) = [ ...
        summary.trialSeeds(trialIdx), ...
        fullAttempted, momentAttempted, ...
        percentReduction(fullAttempted, momentAttempted), ...
        fullDelivered, momentDelivered, ...
        percentReduction(fullDelivered, momentDelivered), ...
        localFull, localMoment, localMoment - localFull, ...
        consensusFull, consensusMoment, ...
        consensusMoment - consensusFull, ...
        positionFull, positionMoment, positionMoment - positionFull, ...
        cardinalityFull, cardinalityMoment, ...
        cardinalityMoment - cardinalityFull, ...
        summary.trials.posteriorMissingSnapshotCount(trialIdx, 2), ...
        summary.trials.posteriorLabelSetMismatchCount(trialIdx, 2), ...
        summary.trials.posteriorMissingLabelCount(trialIdx, 2), ...
        summary.trials.posteriorComparisonCount(trialIdx, 2), ...
        summary.trials.posteriorSnapshotCount(trialIdx, 2), ...
        summary.trials.posteriorMaxAbsR(trialIdx, 2), ...
        summary.trials.posteriorMaxAbsMu(trialIdx, 2), ...
        summary.trials.posteriorMaxAbsSigma(trialIdx, 2), ...
        double(summary.trials.posteriorExactMatch(trialIdx, 2)), ...
        double(attemptedMasksEqual), double(deliveredMasksEqual)];
end
end

function value = percentReduction(fullValue, momentValue)
if fullValue == 0
    if momentValue == 0
        value = 0;
    else
        value = NaN;
    end
else
    value = 100 * (fullValue - momentValue) / fullValue;
end
end

function writeEvidenceCsv(path, rows)
headers = evidenceHeaders();
fid = fopen(path, 'w');
if fid < 0
    error('Unable to open evidence CSV for writing: %s', path);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s\n', strjoin(headers, ','));
format = [repmat('%.17g,', 1, size(rows, 2) - 1), '%.17g\n'];
for rowIdx = 1:size(rows, 1)
    fprintf(fid, format, rows(rowIdx, :));
end
end

function headers = evidenceHeaders()
headers = { ...
    'seed', ...
    'full_attempted_bytes', 'moment_attempted_bytes', ...
    'attempted_reduction_percent', ...
    'full_delivered_bytes', 'moment_delivered_bytes', ...
    'delivered_reduction_percent', ...
    'full_local_eospa', 'moment_local_eospa', 'local_eospa_delta', ...
    'full_consensus_ospa', 'moment_consensus_ospa', ...
    'consensus_ospa_delta', ...
    'full_consensus_position', 'moment_consensus_position', ...
    'consensus_position_delta', ...
    'full_consensus_cardinality', 'moment_consensus_cardinality', ...
    'consensus_cardinality_delta', ...
    'missing_snapshot_count', 'label_set_mismatch_count', ...
    'missing_label_count', ...
    'comparison_count', 'snapshot_count', ...
    'max_abs_r', 'max_abs_mu', 'max_abs_sigma', ...
    'exact_match', 'attempted_masks_equal', 'delivered_masks_equal'};
end

function aggregate = computeAggregate(rows, config)
attemptedReduction = rows(:, 4);
deliveredReduction = rows(:, 7);
[ciLow, ciHigh] = pairedBootstrapMeanInterval( ...
    attemptedReduction, config.bootstrapSeed, config.bootstrapResamples);
aggregate = struct();
aggregate.mean_attempted_reduction_percent = mean(attemptedReduction);
aggregate.min_attempted_reduction_percent = min(attemptedReduction);
aggregate.median_attempted_reduction_percent = ...
    median(attemptedReduction);
aggregate.max_attempted_reduction_percent = max(attemptedReduction);
aggregate.bootstrap_ci_low_percent = ciLow;
aggregate.bootstrap_ci_high_percent = ciHigh;
aggregate.mean_delivered_reduction_percent = ...
    mean(deliveredReduction);
aggregate.total_full_attempted_bytes = sum(rows(:, 2));
aggregate.total_moment_attempted_bytes = sum(rows(:, 3));
aggregate.total_full_delivered_bytes = sum(rows(:, 5));
aggregate.total_moment_delivered_bytes = sum(rows(:, 6));
aggregate.max_abs_local_eospa_delta = maxFiniteAbs(rows(:, 10));
aggregate.max_abs_consensus_ospa_delta = maxFiniteAbs(rows(:, 13));
aggregate.max_abs_consensus_position_delta = maxFiniteAbs(rows(:, 16));
aggregate.max_abs_consensus_cardinality_delta = ...
    maxFiniteAbs(rows(:, 19));
aggregate.total_missing_snapshot_count = sum(rows(:, 20));
aggregate.total_label_set_mismatch_count = sum(rows(:, 21));
aggregate.total_missing_label_count = sum(rows(:, 22));
aggregate.total_comparison_count = sum(rows(:, 23));
aggregate.total_snapshot_count = sum(rows(:, 24));
aggregate.max_abs_r = max(rows(:, 25));
aggregate.max_abs_mu = max(rows(:, 26));
aggregate.max_abs_sigma = max(rows(:, 27));
aggregate.all_exact_match = double(all(rows(:, 28) == 1));
aggregate.all_attempted_masks_equal = double(all(rows(:, 29) == 1));
aggregate.all_delivered_masks_equal = double(all(rows(:, 30) == 1));
end

function [low, high] = pairedBootstrapMeanInterval(values, seed, count)
values = reshape(values, [], 1);
if isempty(values) || any(~isfinite(values))
    error('Attempted-byte reductions must be finite for bootstrapping.');
end
previousState = rng;
cleanup = onCleanup(@() rng(previousState)); %#ok<NASGU>
rng(seed);
indices = randi(numel(values), numel(values), count);
bootstrapMeans = mean(values(indices), 1);
low = percentileScalar(bootstrapMeans, 0.025);
high = percentileScalar(bootstrapMeans, 0.975);
end

function value = percentileScalar(values, probability)
values = sort(reshape(values, 1, []));
position = 1 + probability * (numel(values) - 1);
lowerIdx = floor(position);
upperIdx = ceil(position);
if lowerIdx == upperIdx
    value = values(lowerIdx);
else
    fraction = position - lowerIdx;
    value = values(lowerIdx) * (1 - fraction) + ...
        values(upperIdx) * fraction;
end
end

function value = maxFiniteAbs(values)
values = abs(values(isfinite(values)));
if isempty(values)
    value = NaN;
else
    value = max(values);
end
end

function writeTextFile(path, text)
fid = fopen(path, 'wb');
if fid < 0
    error('Unable to open evidence text file for writing: %s', path);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
count = fwrite(fid, uint8(text), 'uint8');
if count ~= numel(uint8(text))
    error('Unable to write complete evidence text file: %s', path);
end
end

function hashValue = sha256File(path)
command = sprintf('shasum -a 256 %s', shellQuote(path));
[status, output] = system(command);
if status ~= 0
    error('Unable to hash evidence file: %s', path);
end
match = regexp(lower(strtrim(output)), '^[0-9a-f]{64}', 'match', 'once');
if isempty(match)
    error('Invalid SHA-256 output for %s.', path);
end
hashValue = match;
end

function quoted = shellQuote(value)
value = char(value);
quoted = ['''', strrep(value, '''', '''"''"'''), ''''];
end

function assertTargetsAbsent(paths)
for pathIdx = 1:numel(paths)
    if exist(paths{pathIdx}, 'file') == 2
        error('Refusing to overwrite evidence artifact: %s', ...
            paths{pathIdx});
    end
end
end

function displayPath = stableArtifactPath(path)
[~, name, extension] = fileparts(path);
displayPath = ['RUN/GA/', name, extension];
end

function deleteTemporaryFiles(paths)
for pathIdx = 1:numel(paths)
    if exist(paths{pathIdx}, 'file') == 2
        delete(paths{pathIdx});
    end
end
end
