function test_icassp_moment_exchange_parallel()
% TEST_ICASSP_MOMENT_EXCHANGE_PARALLEL Verify v2 plan/worker/merge gates.

[~, ~, ~, ~, frozen] = ...
    runFusionSufficientMomentExchangeConfirmatory(false);
testAuthorization = fusionSufficientInternalTestAuthorization();
planOne = buildFusionSufficientParallelPlan(frozen);
planTwo = buildFusionSufficientParallelPlan(frozen);
assert(isequaln(planOne, planTwo));
assert(planOne.maxWorkers == 6);
assert(numel(planOne.workerSeeds) == 50);
assert(~isempty(strfind(planOne.launchCommand, ...
    'launchFusionSufficientMomentExchangeWorkers.sh'))); %#ok<STREMP>
[status, output] = system(sprintf('bash -n %s', ...
    shellQuote(planOne.launcherPath)));
assert(status == 0, output);

testOwnershipAndLauncherClaims(frozen, testAuthorization);
testAssemblerRejections(frozen, testAuthorization);
testBurnedBatchRejections(frozen, testAuthorization);
testSignalSpawnCleanup(frozen, testAuthorization);
testPublishClaimRace(frozen, testAuthorization);
testRealLauncherSuccess(frozen, testAuthorization);
testShortWorkerPath(frozen, testAuthorization);
fprintf('test_icassp_moment_exchange_parallel passed\n');
end

function testOwnershipAndLauncherClaims(frozen, testAuthorization)
parent = tempname();
[ok, message] = mkdir(parent);
assert(ok, message);
cleanup = onCleanup(@() removeDirectory(parent)); %#ok<NASGU>

[config, plan] = makeTestConfig( ...
    frozen, parent, 'ownership-existing', 2, 3000, 3);
assertThrows(@() acquireFusionSufficientBatchOwnership(plan, config));
acquireFusionSufficientBatchOwnership(plan, config, testAuthorization);
planHash = fusionSufficientSha256File(plan.batchPlanPath);
assertThrows(@() acquireFusionSufficientBatchOwnership( ...
    plan, config, testAuthorization));
assert(strcmp(planHash, fusionSufficientSha256File(plan.batchPlanPath)));
assert(isempty(dir(fullfile(plan.workerDirectory, 'worker_seed_*.mat'))));

% A second launcher that did not acquire the lifecycle claim cannot poison
% the legitimate owner with a FAILED tombstone.
writeText(plan.lifecycleClaimPath, 'first-launcher\n');
[status, ~] = system(plan.launchCommand);
assert(status ~= 0);
assert(exist(plan.attemptFailedTombstonePath, 'file') ~= 2);
assert(exist(plan.workerFailedTombstonePath, 'file') ~= 2);
removeDirectory(plan.workerDirectory);
assertThrows(@() acquireFusionSufficientBatchOwnership( ...
    plan, config, testAuthorization));
assert(exist(plan.attemptDirectory, 'dir') == 7);

testConcurrentOwnership(frozen, parent, testAuthorization);

[config, plan] = makeTestConfig( ...
    frozen, parent, 'launcher-complete', 2, 3050, 3);
createSyntheticWorkers(config, plan, 'none', testAuthorization);
[status, ~] = system(plan.launchCommand);
assert(status ~= 0);
state = inspectFusionSufficientBatchState(plan);
assert(strcmp(state.name, 'COMPLETE_WORKERS'));
assert(~state.attemptFailedExists && ~state.workerFailedExists);
end

function testSignalSpawnCleanup(frozen, testAuthorization)
parent = tempname();
[ok, message] = mkdir(parent);
assert(ok, message);
cleanup = onCleanup(@() removeDirectory(parent)); %#ok<NASGU>
[config, plan] = makeTestConfig( ...
    frozen, parent, 'signal-spawn-cleanup', 2, 3650, 3);
acquireFusionSufficientBatchOwnership(plan, config, testAuthorization);
command = ['FUSION_SUFFICIENT_TEST_SIGNAL_AFTER_SPAWN=1 ', ...
    plan.launchCommand];
[status, ~] = system(command);
assert(status ~= 0);
state = inspectFusionSufficientBatchState(plan);
assert(strcmp(state.name, 'FAILED_BURNED'));
assert(state.attemptFailedExists && state.workerFailedExists);
[orphanStatus, orphanOutput] = system(sprintf( ...
    'pgrep -f %s', shellQuote(plan.batchPlanPath)));
assert(orphanStatus ~= 0, ['orphan worker process: ', orphanOutput]);
end

function testRealLauncherSuccess(frozen, testAuthorization)
parent = tempname();
[ok, message] = mkdir(parent);
assert(ok, message);
cleanup = onCleanup(@() removeDirectory(parent)); %#ok<NASGU>
[config, plan] = makeTestConfig( ...
    frozen, parent, 'real-launcher-success', 1, 3675, 3);
acquireFusionSufficientBatchOwnership(plan, config, testAuthorization);
helperDirectory = fullfile(parent, 'launcher-test-helper');
[ok, message] = mkdir(helperDirectory);
assert(ok, message);
helperPath = fullfile(helperDirectory, ...
    'test_publish_fusion_sufficient_batch_success.m');
writeText(helperPath, sprintf([ ...
    ['function test_publish_fusion_sufficient_batch_success(', ...
    'planPath, completionToken, launcherCapability)\n'], ...
    ['authorization = ', ...
    'fusionSufficientInternalTestAuthorization();\n'], ...
    ['publishFusionSufficientBatchSuccessReceipt(planPath, ', ...
    'completionToken, launcherCapability, authorization);\n'], ...
    'end\n']));
command = sprintf([ ...
    'FUSION_SUFFICIENT_INTERNAL_TEST_LAUNCH=1 ', ...
    'FUSION_SUFFICIENT_TEST_HELPER_DIR=%s %s'], ...
    shellQuote(helperDirectory), plan.launchCommand);
[status, output] = system(command);
workerLog = '';
if exist(plan.workerLogPaths{1}, 'file') == 2
    workerLog = fileread(plan.workerLogPaths{1});
end
assert(status == 0, [output, workerLog]);
assert(strcmp(inspectFusionSufficientBatchState(plan).name, ...
    'COMPLETE_WORKERS'));
assert(exist(plan.launcherCapabilityPath, 'file') ~= 2);
assert(exist(plan.completionAuthorizationPath, 'file') == 2);
assert(exist(plan.workerSuccessReceiptPath, 'file') == 2);
[summary, execution] = assembleFusionSufficientMomentExchangeWorkers( ...
    plan.workerDirectory, config, struct(), testAuthorization);
assert(summary.numberOfTrials == 1);
assert(execution.receiptValidated);
end

function testPublishClaimRace(frozen, testAuthorization)
parent = tempname();
[ok, message] = mkdir(parent);
assert(ok, message);
cleanup = onCleanup(@() removeDirectory(parent)); %#ok<NASGU>

[config, plan] = makeTestConfig( ...
    frozen, parent, 'publish-race', 3, 3700, 100);
createSyntheticWorkers(config, plan, 'none', testAuthorization);
assertThrows(@() acquireFusionSufficientPublishClaim(plan, config));
requestPath = fullfile(parent, 'publish-request.mat');
save(requestPath, 'config', 'plan', 'testAuthorization', '-v7');
scriptPath = fullfile(parent, 'test_publish_once.m');
scriptText = sprintf([ ...
    'function test_publish_once()\n', ...
    'setPath; addpath(''RUN/GA''); x=load(''%s''); ', ...
    ['claim=acquireFusionSufficientPublishClaim(x.plan,x.config,', ...
    'x.testAuthorization); '], ...
    ['[summary,~]=assembleFusionSufficientMomentExchangeWorkers(', ...
    'x.plan.workerDirectory,x.config,struct(),', ...
    'x.testAuthorization); '], ...
    ['writeFusionSufficientEvidence(summary,x.config,claim,', ...
    'x.testAuthorization); '], ...
    ['publishFusionSufficientPublishedReceipt(x.plan,x.config,', ...
    'claim,x.testAuthorization);\nend\n']], ...
    octaveQuote(requestPath));
writeText(scriptPath, scriptText);
invoke = sprintf('addpath(''%s''); test_publish_once;', ...
    octaveQuote(parent));
statusOne = fullfile(parent, 'publisher-one.status');
statusTwo = fullfile(parent, 'publisher-two.status');
logOne = fullfile(parent, 'publisher-one.log');
logTwo = fullfile(parent, 'publisher-two.log');
command = sprintf([ ...
    '(octave-cli --quiet --eval %s > %s 2>&1; echo $? > %s) & ', ...
    ['(octave-cli --quiet --eval %s > %s 2>&1; ', ...
    'echo $? > %s) & wait']], ...
    shellQuote(invoke), shellQuote(logOne), shellQuote(statusOne), ...
    shellQuote(invoke), shellQuote(logTwo), shellQuote(statusTwo));
[status, output] = system(command);
assert(status == 0, output);
codes = sort([readInteger(statusOne), readInteger(statusTwo)]);
assert(codes(1) == 0 && codes(2) ~= 0, ...
    [fileread(logOne), fileread(logTwo)]);
for pathIdx = 1:numel(plan.finalPaths)
    assert(exist(plan.finalPaths{pathIdx}, 'file') == 2);
end
assert(exist(plan.publishedReceiptPath, 'file') == 2);
publishedHashes = cellfun(@fusionSufficientSha256File, ...
    plan.finalPaths, 'UniformOutput', false);
assertThrows(@() acquireFusionSufficientPublishClaim( ...
    plan, config, testAuthorization));
assert(isequal(publishedHashes, cellfun( ...
    @fusionSufficientSha256File, plan.finalPaths, ...
    'UniformOutput', false)));
validateFusionSufficientEvidence( ...
    plan.finalPaths{1}, plan.finalPaths{2}, plan.finalPaths{3}, ...
    testAuthorization);

[staleConfig, stalePlan] = makeTestConfig( ...
    frozen, parent, 'publish-stale', 2, 3750, 100);
createSyntheticWorkers( ...
    staleConfig, stalePlan, 'none', testAuthorization);
claim = acquireFusionSufficientPublishClaim( ...
    stalePlan, staleConfig, testAuthorization);
assertThrowsContains(@() acquireFusionSufficientPublishClaim( ...
    stalePlan, staleConfig, testAuthorization), 'audit manually');
releaseFusionSufficientPublishClaim(stalePlan, staleConfig, claim);
replacement = acquireFusionSufficientPublishClaim( ...
    stalePlan, staleConfig, testAuthorization);
releaseFusionSufficientPublishClaim( ...
    stalePlan, staleConfig, replacement);
end

function testConcurrentOwnership(frozen, parent, testAuthorization)
[config, plan] = makeTestConfig( ...
    frozen, parent, 'ownership-concurrent', 2, 3100, 3);
requestPath = fullfile(parent, 'ownership-request.mat');
save(requestPath, 'config', 'plan', 'testAuthorization', '-v7');
scriptPath = fullfile(parent, 'test_acquire_once.m');
scriptText = sprintf([ ...
    'function test_acquire_once()\n', ...
    'setPath; addpath(''RUN/GA''); ', ...
    'x=load(''%s''); ', ...
    ['acquireFusionSufficientBatchOwnership(x.plan,x.config,', ...
    'x.testAuthorization);\nend\n']], ...
    octaveQuote(requestPath));
writeText(scriptPath, scriptText);
invoke = sprintf('addpath(''%s''); test_acquire_once;', ...
    octaveQuote(parent));
statusOne = fullfile(parent, 'owner-one.status');
statusTwo = fullfile(parent, 'owner-two.status');
logOne = fullfile(parent, 'owner-one.log');
logTwo = fullfile(parent, 'owner-two.log');
command = sprintf([ ...
    '(octave-cli --quiet --eval %s > %s 2>&1; echo $? > %s) & ', ...
    ['(octave-cli --quiet --eval %s > %s 2>&1; ', ...
    'echo $? > %s) & wait']], ...
    shellQuote(invoke), shellQuote(logOne), shellQuote(statusOne), ...
    shellQuote(invoke), shellQuote(logTwo), shellQuote(statusTwo));
[status, output] = system(command);
assert(status == 0, output);
codes = sort([readInteger(statusOne), readInteger(statusTwo)]);
assert(codes(1) == 0 && codes(2) ~= 0, ...
    [fileread(logOne), fileread(logTwo)]);
assert(exist(plan.batchPlanPath, 'file') == 2);
assert(exist(plan.attemptFailedTombstonePath, 'file') ~= 2);
end

function testBurnedBatchRejections(frozen, testAuthorization)
parent = tempname();
[ok, message] = mkdir(parent);
assert(ok, message);
cleanup = onCleanup(@() removeDirectory(parent)); %#ok<NASGU>

% Real launcher control flow (workers intentionally fail provenance in the
% dirty test checkout) must wait the wave and seal FAILED in both ledgers.
[config, plan] = makeTestConfig( ...
    frozen, parent, 'launcher-worker-failure', 2, 3150, 3);
acquireFusionSufficientBatchOwnership(plan, config, testAuthorization);
[status, ~] = system(plan.launchCommand);
assert(status ~= 0);
state = inspectFusionSufficientBatchState(plan);
assert(strcmp(state.name, 'FAILED_BURNED'));
assert(state.attemptFailedExists && state.workerFailedExists);

% Once FAILED, adding all expected MAT/log files cannot resurrect a batch.
[config, plan] = makeTestConfig( ...
    frozen, parent, 'failed-then-filled', 3, 3200, 3);
acquireFusionSufficientBatchOwnership(plan, config, testAuthorization);
publishFusionSufficientBatchFailure( ...
    plan, config, 'worker-exit-nonzero', testAuthorization);
writeSyntheticWorkerArtifacts(config, plan, 'none');
assertThrows(@() publishFusionSufficientBatchSuccessReceipt( ...
    plan.batchPlanPath, '', '', testAuthorization));
assertThrows(@() assembleFusionSufficientMomentExchangeWorkers( ...
    plan.workerDirectory, config, struct(), testAuthorization));
assert(strcmp(inspectFusionSufficientBatchState(plan).name, ...
    'FAILED_BURNED'));

% A complete artifact set paired with a nonzero launcher outcome is FAILED.
[config, plan] = makeTestConfig( ...
    frozen, parent, 'complete-but-nonzero', 3, 3300, 3);
acquireFusionSufficientBatchOwnership(plan, config, testAuthorization);
writeSyntheticWorkerArtifacts(config, plan, 'none');
[completionToken, launcherCapability] = ...
    writeSyntheticLauncherState( ...
        plan, config, plan.workerSeeds(2), true);
assertThrows(@() publishFusionSufficientBatchSuccessReceipt( ...
    plan.batchPlanPath, completionToken, launcherCapability, ...
    testAuthorization));
publishFusionSufficientBatchFailure( ...
    plan, config, 'worker-exit-nonzero', testAuthorization);
assertThrows(@() assembleFusionSufficientMomentExchangeWorkers( ...
    plan.workerDirectory, config, struct(), testAuthorization));

% Receipt tampering is detected even when both hard-linked copies change.
[config, plan] = makeTestConfig( ...
    frozen, parent, 'tamper-receipt', 3, 3400, 3);
createSyntheticWorkers(config, plan, 'none', testAuthorization);
appendText(plan.workerSuccessReceiptPath, 'unexpected_field=1\n');
assertThrows(@() assembleFusionSufficientMomentExchangeWorkers( ...
    plan.workerDirectory, config, struct(), testAuthorization));
assert(strcmp(inspectFusionSufficientBatchState(plan).name, ...
    'COMPLETE_WORKERS'));

% After capability consumption, disk ledgers plus a completion token are
% insufficient to forge success without the launcher's plaintext secret.
[config, plan] = makeTestConfig( ...
    frozen, parent, 'capability-required', 3, 3450, 3);
acquireFusionSufficientBatchOwnership(plan, config, testAuthorization);
writeSyntheticWorkerArtifacts(config, plan, 'none');
[completionToken, launcherCapability] = ...
    writeSyntheticLauncherState(plan, config, [], true);
assert(exist(plan.launcherCapabilityPath, 'file') ~= 2);
assertThrows(@() publishFusionSufficientBatchSuccessReceipt( ...
    plan.batchPlanPath, completionToken, '', testAuthorization));
wrongCapability = launcherCapability;
if wrongCapability(1) == '0'
    wrongCapability(1) = '1';
else
    wrongCapability(1) = '0';
end
assertThrows(@() publishFusionSufficientBatchSuccessReceipt( ...
    plan.batchPlanPath, completionToken, wrongCapability, ...
    testAuthorization));
assert(exist(plan.workerSuccessReceiptPath, 'file') ~= 2);
assert(exist(plan.attemptSuccessReceiptPath, 'file') ~= 2);
publishFusionSufficientBatchSuccessReceipt( ...
    plan.batchPlanPath, completionToken, launcherCapability, ...
    testAuthorization);
assert(strcmp(inspectFusionSufficientBatchState(plan).name, ...
    'COMPLETE_WORKERS'));

% Artifact tampering after receipt publication is detected by its hash.
[config, plan] = makeTestConfig( ...
    frozen, parent, 'tamper-artifact', 3, 3500, 3);
createSyntheticWorkers(config, plan, 'none', testAuthorization);
appendText(plan.workerLogPaths{2}, 'tampered\n');
assertThrows(@() assembleFusionSufficientMomentExchangeWorkers( ...
    plan.workerDirectory, config, struct(), testAuthorization));
assert(strcmp(inspectFusionSufficientBatchState(plan).name, ...
    'COMPLETE_WORKERS'));

% A publication interruption after COMPLETE_WORKERS remains recoverable;
% it must not be converted into FAILED and must never rerun workers.
[config, plan] = makeTestConfig( ...
    frozen, parent, 'writer-interruption', 3, 3600, 3);
createSyntheticWorkers(config, plan, 'none', testAuthorization);
[summary, ~] = assembleFusionSufficientMomentExchangeWorkers( ...
    plan.workerDirectory, config, struct(), testAuthorization);
publishClaim = acquireFusionSufficientPublishClaim( ...
    plan, config, testAuthorization);
writeText(plan.finalPaths{2}, 'preexisting\n');
assertThrows(@() writeFusionSufficientEvidence( ...
    summary, config, publishClaim, testAuthorization));
state = inspectFusionSufficientBatchState(plan);
assert(strcmp(state.name, 'COMPLETE_WORKERS'));
assert(~state.attemptFailedExists && ~state.workerFailedExists);
end

function testAssemblerRejections(frozen, testAuthorization)
parent = tempname();
[ok, message] = mkdir(parent);
assert(ok, message);
cleanup = onCleanup(@() removeDirectory(parent)); %#ok<NASGU>

[config, plan] = makeTestConfig(frozen, parent, 'ordered', 3, 4000, 3);
createSyntheticWorkers(config, plan, 'none', testAuthorization);
assertThrows(@() assembleFusionSufficientMomentExchangeWorkers( ...
    plan.workerDirectory, config));
[summary, execution] = ...
    assembleFusionSufficientMomentExchangeWorkers( ...
        plan.workerDirectory, config, struct(), testAuthorization);
assert(isequal(summary.trialSeeds, 4001:4003));
assert(isequal(execution.workerSeeds, 4001:4003));
assert(isequal(summary.trials.attemptedPayloadBytes(:, 1)', ...
    100000 + (4001:4003)));

[config, plan] = makeTestConfig(frozen, parent, 'missing', 3, 4100, 3);
createSyntheticWorkers(config, plan, 'missing', testAuthorization);
assertThrows(@() assembleFusionSufficientMomentExchangeWorkers( ...
    plan.workerDirectory, config, struct(), testAuthorization));

[config, plan] = makeTestConfig(frozen, parent, 'extra', 3, 4200, 3);
createSyntheticWorkers(config, plan, 'extra', testAuthorization);
assertThrows(@() assembleFusionSufficientMomentExchangeWorkers( ...
    plan.workerDirectory, config, struct(), testAuthorization));

[config, plan] = makeTestConfig(frozen, parent, 'duplicate', 3, 4300, 3);
createSyntheticWorkers(config, plan, 'duplicate', testAuthorization);
assertThrows(@() assembleFusionSufficientMomentExchangeWorkers( ...
    plan.workerDirectory, config, struct(), testAuthorization));

[config, plan] = makeTestConfig(frozen, parent, 'commit', 3, 4400, 3);
createSyntheticWorkers(config, plan, 'wrong-commit', testAuthorization);
assertThrows(@() assembleFusionSufficientMomentExchangeWorkers( ...
    plan.workerDirectory, config, struct(), testAuthorization));

[config, plan] = makeTestConfig(frozen, parent, 'config', 3, 4500, 3);
createSyntheticWorkers(config, plan, 'wrong-config', testAuthorization);
assertThrows(@() assembleFusionSufficientMomentExchangeWorkers( ...
    plan.workerDirectory, config, struct(), testAuthorization));
end

function testShortWorkerPath(frozen, testAuthorization)
parent = tempname();
[ok, message] = mkdir(parent);
assert(ok, message);
cleanup = onCleanup(@() removeDirectory(parent)); %#ok<NASGU>
[config, plan] = makeTestConfig( ...
    frozen, parent, 'worker-smoke', 1, 5000, 3);
acquireFusionSufficientBatchOwnership(plan, config, testAuthorization);
testOverrides = struct('enabled', true, 'skipGitProvenance', true);
result = runFusionSufficientMomentExchangeWorker( ...
    plan.batchPlanPath, config.firstSeed, ...
    plan.workerMatPaths{1}, testOverrides);
assert(result.seed == config.firstSeed);
assert(result.exactGatePassed);
assert(exist(plan.workerMatPaths{1}, 'file') == 2);
loaded = load(plan.workerMatPaths{1});
assert(strcmp(loaded.workerResult.summarySha256, ...
    hashFusionSufficientValue(loaded.workerResult.summary)));
assertFusionSufficientFrozenRun( ...
    loaded.workerResult.summary, config, config.firstSeed);
assertThrows(@() runFusionSufficientMomentExchangeWorker( ...
    plan.batchPlanPath, config.firstSeed, ...
    plan.workerMatPaths{1}, testOverrides));
end

function [config, plan] = makeTestConfig( ...
    frozen, parent, name, numberOfTrials, baseSeed, simulationLength)
config = frozen;
config.testOnly = true;
config.evidenceSchema = 'fusion-sufficient-moment-exchange-test-v2';
config.workerSchema = 'fusion-sufficient-seed-worker-test-v2';
config.numberOfTrials = numberOfTrials;
config.baseSeed = baseSeed;
config.firstSeed = baseSeed + 1;
config.lastSeed = baseSeed + numberOfTrials;
config.simulationLength = simulationLength;
config.isSmoke = false;
config.artifactStem = ['TEST_FUSION_SUFFICIENT_', upper(name)];
config.batchIdentity = ['test-', regexprep(lower(name), ...
    '[^a-z0-9-]+', '-')];
config.outputDirectory = fullfile(parent, name);
[ok, message] = mkdir(config.outputDirectory);
assert(ok, message);
config.regenerationCommand = 'test-only';
config.configSha256 = hashFusionSufficientConfig(config);
plan = buildFusionSufficientParallelPlan(config);
end

function createSyntheticWorkers( ...
    config, plan, mutation, testAuthorization)
acquireFusionSufficientBatchOwnership(plan, config, testAuthorization);
writeSyntheticWorkerArtifacts(config, plan, mutation);
if ~strcmp(mutation, 'missing') && ~strcmp(mutation, 'extra')
    [completionToken, launcherCapability] = ...
        writeSyntheticLauncherState( ...
        plan, config, [], true);
    publishFusionSufficientBatchSuccessReceipt( ...
        plan.batchPlanPath, completionToken, launcherCapability, ...
        testAuthorization);
end
end

function writeSyntheticWorkerArtifacts(config, plan, mutation)
for reverseIdx = numel(plan.workerSeeds):-1:1
    seed = plan.workerSeeds(reverseIdx);
    summary = makeSingleSummary(config, seed);
    workerResult = makeWorkerResult(config, summary, seed);
    if strcmp(mutation, 'duplicate') && reverseIdx == 2
        workerResult.seed = plan.workerSeeds(1);
        workerResult.summary = makeSingleSummary( ...
            config, plan.workerSeeds(1));
        workerResult.summarySha256 = ...
            hashFusionSufficientValue(workerResult.summary);
    elseif strcmp(mutation, 'wrong-commit') && reverseIdx == 2
        workerResult.gitCommit = repmat('b', 1, 40);
    elseif strcmp(mutation, 'wrong-config') && reverseIdx == 2
        workerResult.config.bootstrapSeed = ...
            workerResult.config.bootstrapSeed + 1;
    end
    save(plan.workerMatPaths{reverseIdx}, 'workerResult', '-v7');
    writeText(plan.workerLogPaths{reverseIdx}, 'synthetic worker log\n');
end
if strcmp(mutation, 'missing')
    delete(plan.workerMatPaths{2});
elseif strcmp(mutation, 'extra')
    workerResult = makeWorkerResult( ...
        config, makeSingleSummary(config, 999999), 999999); %#ok<NASGU>
    save(fullfile(plan.workerDirectory, ...
        'worker_seed_999999.mat'), 'workerResult', '-v7');
    writeText(fullfile(plan.workerDirectory, ...
        'worker_seed_999999.log'), 'extra\n');
end
end

function workerResult = makeWorkerResult(config, summary, seed)
workerResult = struct( ...
    'workerSchema', config.workerSchema, ...
    'evidenceSchema', config.evidenceSchema, ...
    'gitCommit', config.gitCommit, ...
    'requiredSourcesSha256', config.requiredSourcesSha256, ...
    'configSha256', config.configSha256, ...
    'summarySha256', hashFusionSufficientValue(summary), ...
    'octaveVersion', config.octaveVersion, ...
    'seed', seed, ...
    'exactGatePassed', true, ...
    'config', config, ...
    'summary', summary);
end

function summary = makeSingleSummary(config, seed)
trigger = struct( ...
    'capturePosteriorSnapshots', true, ...
    'linkGateEnabled', false, ...
    'forceInitialHeavy', false, ...
    'forceLabelChangeHeavy', false, ...
    'forceStaleHeavy', false, ...
    'useStaleNeighborCache', false, ...
    'labelHeartbeatEnabled', false, ...
    'mixedPayloadEnabled', false, ...
    'mixedPayloadLightForAllActiveLabels', false, ...
    'dynamicTopologyEnabled', false, ...
    'modeAwareFusionWeights', false, ...
    'lightCovarianceInflationEnabled', false, ...
    'lightFusionWeightFactor', 1, ...
    'heavyFusionWeightFactor', 1);
fullTrigger = trigger;
fullTrigger.eventPolicy = 'alwaysHeavy';
momentTrigger = trigger;
momentTrigger.eventPolicy = 'alwaysLight';

summary = struct();
summary.armNames = config.armSelection;
summary.arms = [ ...
    struct('name', config.armSelection{1}, 'purpose', 'synthetic', ...
        'triggerConfig', fullTrigger), ...
    struct('name', config.armSelection{2}, 'purpose', 'synthetic', ...
        'triggerConfig', momentTrigger)];
summary.numberOfTrials = 1;
summary.trialSeeds = seed;
summary.scenarioConfig = struct( ...
    'numberOfSensors', 8, ...
    'simulationLength', config.simulationLength);
summary.trials.localEOspa = zeros(1, 8, 2);
summary.trials.consensusOspa = zeros(1, 2);
summary.trials.consensusPosition = zeros(1, 2);
summary.trials.consensusCardinality = zeros(1, 2);
summary.trials.attemptedPayloadBytes = ...
    [100000 + seed, 40000 + seed];
summary.trials.deliveredPayloadBytes = ...
    [80000 + seed, 32000 + seed];
mask = false(8, 8, config.simulationLength);
mask(1, 2, :) = true;
summary.trials.attemptedMask = {mask, mask};
summary.trials.deliveredMask = {mask, mask};
summary.trials.posteriorMissingSnapshotCount = zeros(1, 2);
summary.trials.posteriorLabelSetMismatchCount = zeros(1, 2);
summary.trials.posteriorMissingLabelCount = zeros(1, 2);
summary.trials.posteriorComparisonCount = 10 * ones(1, 2);
summary.trials.posteriorSnapshotCount = ...
    (8 * config.simulationLength) * ones(1, 2);
summary.trials.posteriorMaxAbsR = zeros(1, 2);
summary.trials.posteriorMaxAbsMu = zeros(1, 2);
summary.trials.posteriorMaxAbsSigma = zeros(1, 2);
summary.trials.posteriorExactMatch = true(1, 2);
summary.equivalence = struct( ...
    'captured', true, ...
    'baselineArm', config.armSelection{1}, ...
    'missingSnapshotCount', [0, 0], ...
    'labelSetMismatchCount', [0, 0], ...
    'missingLabelCount', [0, 0], ...
    'comparisonCount', [10, 10], ...
    'snapshotCount', [8, 8] * config.simulationLength, ...
    'maxAbsR', [0, 0], ...
    'maxAbsMu', [0, 0], ...
    'maxAbsSigma', [0, 0], ...
    'exactMatch', [true, true]);
end

function writeText(path, value)
fid = fopen(path, 'wb');
assert(fid >= 0);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
count = fwrite(fid, uint8(value), 'uint8');
assert(count == numel(uint8(value)));
end

function [completionToken, launcherCapability] = ...
    writeSyntheticLauncherState( ...
    plan, config, nonzeroSeed, includeCompletion)
launcherCapability = strtrim(fileread(plan.launcherCapabilityPath));
assert(~isempty(regexp( ...
    launcherCapability, '^[0-9a-f]{64}$', 'once')));
capabilitySha256 = fusionSufficientSha256Text(launcherCapability);
delete(plan.launcherCapabilityPath);
writeText(plan.lifecycleClaimPath, sprintf([ ...
    'claim_schema=fusion-sufficient-launch-claim-v2\n', ...
    'batch_identity=%s\n', ...
    'launcher_capability_sha256=%s\n'], ...
    config.batchIdentity, capabilitySha256));
parts = {sprintf([ ...
    'exit_schema=fusion-sufficient-worker-exits-v2\n', ...
    'batch_identity=%s\n', ...
    'launcher_capability_sha256=%s\n', ...
    'ordered_seeds=%s\n'], ...
    config.batchIdentity, capabilitySha256, ...
    joinIntegers(plan.workerSeeds))};
for seedIdx = 1:numel(plan.workerSeeds)
    exitCode = 0;
    if ~isempty(nonzeroSeed) && plan.workerSeeds(seedIdx) == nonzeroSeed
        exitCode = 1;
    end
    parts{end + 1} = sprintf('worker_%06d_exit=%d\n', ...
        plan.workerSeeds(seedIdx), exitCode); %#ok<AGROW>
end
writeText(plan.exitLedgerPath, [parts{:}]);
completionToken = '';
if includeCompletion
    completionToken = fusionSufficientSha256Text( ...
        ['completion-', config.batchIdentity]);
    writeText(plan.completionAuthorizationPath, sprintf([ ...
        'completion_schema=fusion-sufficient-completion-v2\n', ...
        'batch_identity=%s\n', ...
        'launcher_capability_sha256=%s\n', ...
        'completion_token_sha256=%s\n'], ...
        config.batchIdentity, capabilitySha256, ...
        fusionSufficientSha256Text(completionToken)));
end
end

function text = joinIntegers(values)
parts = arrayfun(@(value) sprintf('%d', value), values, ...
    'UniformOutput', false);
text = strjoin(parts, ',');
end

function appendText(path, value)
fid = fopen(path, 'ab');
assert(fid >= 0);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
count = fwrite(fid, uint8(value), 'uint8');
assert(count == numel(uint8(value)));
end

function value = readInteger(path)
value = str2double(strtrim(fileread(path)));
assert(isfinite(value) && value == floor(value));
end

function value = octaveQuote(path)
value = strrep(char(path), '''', '''''');
end

function assertThrows(functionHandle)
didThrow = false;
try
    functionHandle();
catch
    didThrow = true;
end
assert(didThrow);
end

function assertThrowsContains(functionHandle, expectedText)
didThrow = false;
try
    functionHandle();
catch exception
    didThrow = true;
    assert(~isempty(strfind(lower(exception.message), ...
        lower(expectedText)))); %#ok<STREMP>
end
assert(didThrow);
end

function removeDirectory(path)
if exist(path, 'dir') == 7
    rmdir(path, 's');
end
end

function quoted = shellQuote(value)
value = char(value);
quoted = ['''', strrep(value, '''', '''"''"'''), ''''];
end
