function test_formation_h3_multiscale_value_audit()
% TEST_FORMATION_H3_MULTISCALE_VALUE_AUDIT Synthetic split smoke test.

protocol = getFormationH3MultiscaleTeacherProtocol();
blocks = repmat(syntheticBlock( ...
    protocol.presets{1}, 1, protocol.trainingSeeds(1), ...
    protocol.snapshotTimes(1)), 1, 18);
cursor = 0;
for presetIdx = 1:numel(protocol.presets)
    formationCount = protocol.expectedFormationCounts(presetIdx);
    for seed = protocol.allSeeds
        for currentTime = protocol.snapshotTimes
            cursor = cursor + 1;
            blocks(cursor) = syntheticBlock( ...
                protocol.presets{presetIdx}, formationCount, ...
                seed, currentTime);
        end
    end
end
dataset = struct();
dataset.contractVersion = ...
    'formation-h3-multiscale-sentinel-dataset-v1';
dataset.protocolId = protocol.id;
dataset.generationGitCommit = 'synthetic';
dataset.generationTrackedWorktreeDirty = false;
dataset.generationUntrackedSourceFiles = {};
dataset.teacherGenerationCommit = 'synthetic';
dataset.featureContractVersion = 'synthetic-features-v1';
dataset.featureNames = { ...
    'scale', 'time', 'trust', 'order', ...
    'formation_sum', 'formation_span'};
dataset.targetNames = { ...
    'mean_tracking_gain_percent', ...
    'minimum_formation_gain_percent', ...
    'worst_sensor_gain_percent', ...
    'consensus_gain_percent', ...
    'attempted_byte_saving_percent', ...
    'delivered_byte_saving_percent'};
dataset.blocks = blocks;
dataset.blockCount = 18;
dataset.trainingBlockCount = 12;
dataset.developmentBlockCount = 6;
dataset.scaleSummaries = repmat(struct('gatePassed', true), 1, 3);
dataset.oracleHeadroomGatePassed = true;
dataset.featuresUseTruth = false;
dataset.featuresUseFutureMeasurements = false;
dataset.featuresUseFutureOutcome = false;
dataset.targetsUseTruth = true;
dataset.targetsUseFutureMeasurements = true;
dataset.openedSentinelDevelopmentOnly = true;
dataset.sharedRidgeAuditAuthorized = true;
dataset.finalModelTrainingAuthorized = false;
dataset.messagePassingModelAuthorized = false;
dataset.validationClaimAllowed = false;
dataset.reservedValidationSeeds = ...
    protocol.finalValidationSeedsReserved;

temporaryRoot = tempname();
mkdir(temporaryRoot);
cleanup = onCleanup(@() removeTemporaryRoot(temporaryRoot)); %#ok<NASGU>
datasetPath = fullfile(temporaryRoot, 'dataset.mat');
save('-mat7-binary', datasetPath, 'dataset');
[auditPath, audit] = ...
    auditFormationH3MultiscaleSentinelValueModel(struct( ...
        'datasetPath', datasetPath, ...
        'outputDirectory', temporaryRoot, ...
        'lambdaGrid', [0.01, 1]));
assert(exist(auditPath, 'file') == 2);
assert(strcmp(audit.contractVersion, ...
    'formation-h3-multiscale-sentinel-value-audit-v1'));
assert(numel(audit.trainingCvCandidates) == 4);
assert(numel(audit.development.rows) == 6);
assert(all(isfinite(audit.development.targetPearson)));
% This synthetic model has high target correlation and safe selections but
% captures the oracle in its top three only one third of the time.  The new
% scale guard must reject that superficially strong representation.
assert(~audit.scaleCoverageGatePassed);
assert(any(~[audit.developmentScaleDiagnostics(2:end).coverageGatePassed]));
assert(~audit.predictorFreezeReviewAuthorized);
assert(audit.minimumDevelopmentScaleTop3CaptureFraction == 0.50);
assert(audit.minimumDevelopmentScaleSafeSelectionFraction == 1.00);
assert(audit.minimumDevelopmentScaleDynamicSelectionFraction == 0.50);
assert(~audit.finalModelTrainingAuthorized);
assert(~audit.messagePassingModelAuthorized);
assert(~audit.validationClaimAllowed);
assert(isequal(audit.reservedValidationSeeds, ...
    protocol.finalValidationSeedsReserved));
fprintf('test_formation_h3_multiscale_value_audit passed\n');
end

function block = syntheticBlock( ...
        presetName, formationCount, seed, currentTime)
trustGrid = [0.30, 0.50, 0.70];
localCount = 1 + 3 * formationCount;
pairCount = formationCount * (formationCount - 1) / 2;
actionCount = localCount + pairCount;
order = zeros(1, actionCount);
trust = zeros(1, actionCount);
formations = zeros(actionCount, 2);
names = cell(1, actionCount);
names{1} = 'reference';
cursor = 1;
for formationIdx = 1:formationCount
    for trustWeight = trustGrid
        cursor = cursor + 1;
        order(cursor) = 1;
        trust(cursor) = trustWeight;
        formations(cursor, 1) = formationIdx;
        names{cursor} = sprintf( ...
            'formation-%d-trust-%.2f', formationIdx, trustWeight);
    end
end
for first = 1:(formationCount - 1)
    for second = (first + 1):formationCount
        cursor = cursor + 1;
        order(cursor) = 2;
        trust(cursor) = 0.30;
        formations(cursor, :) = [first, second];
        names{cursor} = sprintf('pair-%d-%d', first, second);
    end
end
assert(cursor == actionCount);

features = zeros(actionCount, 6);
targets = zeros(actionCount, 6);
residual = zeros(actionCount, 6);
for actionIdx = 2:actionCount
    active = formations(actionIdx, formations(actionIdx, :) > 0);
    features(actionIdx, :) = [ ...
        formationCount / 6, currentTime / 100, ...
        trust(actionIdx), order(actionIdx), ...
        sum(active) / max(formationCount, 1), ...
        (max(active) - min(active)) / max(formationCount, 1)];
    if order(actionIdx) == 1
        meanGain = 0.4 + 0.2 * formationCount / 6 + ...
            0.5 * trust(actionIdx) + 0.1 * currentTime / 100;
        targets(actionIdx, :) = meanGain * ...
            [1, 0.8, 0.9, 0.6, 0.4, 0.35];
    else
        singletonIndices = zeros(1, 2);
        for k = 1:2
            singletonIndices(k) = find(order == 1 & ...
                formations(:, 1)' == active(k) & ...
                abs(trust - 0.30) <= 1e-12, 1);
        end
        interaction = 0.05 * [1, 0.8, 0.9, 0.6, 0.4, 0.35];
        targets(actionIdx, :) = ...
            sum(targets(singletonIndices, :), 1) + interaction;
        residual(actionIdx, :) = interaction;
    end
end
feasible = all(targets >= 0, 2);
[oracleGain, oracleIdx] = max(targets(:, 1));
split = 'sentinel-training';
if seed == 227
    split = 'sentinel-development';
end
block = struct( ...
    'scaleIndex', formationCount / 2, ...
    'presetName', presetName, 'seed', seed, ...
    'currentTime', currentTime, 'evidenceSplit', split, ...
    'cachePath', '', 'cacheSha256', '', ...
    'actionNames', {names}, ...
    'actionTypes', {repmat({'synthetic'}, 1, actionCount)}, ...
    'interventionOrder', order, 'trustWeights', trust, ...
    'formationIndicesAuditOnly', formations, ...
    'features', features, 'targets', targets, ...
    'additiveBaseTargets', targets - residual, ...
    'interactionResidualTargets', residual, ...
    'deploymentFeasible', feasible, ...
    'oracleDeploymentActionIndex', oracleIdx, ...
    'oracleDeploymentActionName', names{oracleIdx}, ...
    'oracleDeploymentGainPercent', oracleGain, ...
    'featuresUseTruth', false, ...
    'featuresUseFutureMeasurements', false, ...
    'featuresUseFutureOutcome', false, ...
    'targetsUseTruth', true, ...
    'targetsUseFutureMeasurements', true, ...
    'openedSentinelDevelopmentOnly', true, ...
    'validationClaimAllowed', false);
end

function removeTemporaryRoot(path)
if exist(path, 'dir') == 7
    rmdir(path, 's');
end
end
