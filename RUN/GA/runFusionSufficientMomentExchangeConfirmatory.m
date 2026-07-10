function [reportPath, csvPath, summaryPath, summary, config] = ...
    runFusionSufficientMomentExchangeConfirmatory( ...
        runExperiment, numberOfTrials, baseSeed)
% RUNFUSIONSUFFICIENTMOMENTEXCHANGECONFIRMATORY
% Frozen two-arm ICASSP protocol for full versus projected LMB messages.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(scriptDir));
addpath(projectRoot);
addpath(scriptDir);
setPath;

if nargin < 1 || isempty(runExperiment)
    runExperiment = true;
end
if nargin < 2 || isempty(numberOfTrials)
    numberOfTrials = 50;
end
if nargin < 3 || isempty(baseSeed)
    baseSeed = 81;
end
runExperiment = validateLogicalScalar(runExperiment, 'runExperiment');
validateIntegerScalar(numberOfTrials, 'numberOfTrials', 1);
validateIntegerScalar(baseSeed, 'baseSeed', 0);

if ~runExperiment && (numberOfTrials ~= 50 || baseSeed ~= 81)
    error('Dry-run mode exposes only the frozen N50 configuration.');
end
isConfirmatory = numberOfTrials == 50 && baseSeed == 81;
isSmoke = numberOfTrials == 5 && baseSeed == 1000;
if runExperiment && ~(isConfirmatory || isSmoke)
    error(['Only the frozen N50 run (true,50,81) and the disjoint ', ...
        'N5 smoke (true,5,1000) are permitted.']);
end

provenance = struct();
if runExperiment
    provenance = assertFusionSufficientGitProvenance(projectRoot);
end
config = buildFrozenConfig( ...
    scriptDir, projectRoot, numberOfTrials, baseSeed, isSmoke, provenance);
reportPath = '';
csvPath = '';
summaryPath = '';
summary = struct();
printFrozenConfig(config);
if ~runExperiment
    return;
end
assertFusionSufficientExecutionEnvironment(config);

parallelPlan = buildFusionSufficientParallelPlan(config);
batchState = inspectFusionSufficientBatchState(parallelPlan);
mayMarkFailure = false;
if strcmp(batchState.name, 'UNRESERVED')
    try
        acquireFusionSufficientBatchOwnership(parallelPlan, config);
        mayMarkFailure = true;
        launchStatus = system(parallelPlan.launchCommand);
        if launchStatus ~= 0
            error('FusionSufficientConfirmatory:WorkerFailure', ...
                ['At least one seed worker failed. This batch identity ', ...
                'is permanently burned and cannot be retried.']);
        end
        postLaunchState = inspectFusionSufficientBatchState(parallelPlan);
        if ~strcmp(postLaunchState.name, 'COMPLETE_WORKERS')
            error('FusionSufficientConfirmatory:MissingSuccessReceipt', ...
                ['Launcher returned zero without an immutable ', ...
                'COMPLETE_WORKERS receipt.']);
        end
        mayMarkFailure = false;
    catch exception
        if mayMarkFailure
            publishFailureBestEffort( ...
                parallelPlan, config, exception.identifier);
        end
        rethrow(exception);
    end
elseif strcmp(batchState.name, 'COMPLETE_WORKERS')
    % Recovery is assembly-only. Worker recomputation is never resumed.
    mayMarkFailure = false;
    fprintf(['  Resuming assembly from immutable COMPLETE_WORKERS ', ...
        'receipt; no workers will run.\n']);
else
    error('FusionSufficientConfirmatory:BatchIdentityBurned', ...
        ['Batch identity %s is in state %s. Reservation acquisition ', ...
        'burned these seeds; deletion/retry is forbidden.'], ...
        config.batchIdentity, batchState.name);
end
publishClaim = acquireFusionSufficientPublishClaim( ...
    parallelPlan, config);
try
    postRunProvenance = assertFusionSufficientGitProvenance(projectRoot);
    [summary, ~] = assembleFusionSufficientMomentExchangeWorkers( ...
        parallelPlan.workerDirectory, config, postRunProvenance);
    [reportPath, csvPath, summaryPath] = ...
        writeFusionSufficientEvidence(summary, config, publishClaim);
    publishFusionSufficientPublishedReceipt( ...
        parallelPlan, config, publishClaim);
catch exception
    if anyFinalArtifactExists(parallelPlan.finalPaths)
        warning('FusionSufficientConfirmatory:PublishClaimRetained', ...
            ['Final publication started but PUBLISHED sealing did not ', ...
            'complete. The publish claim is retained for manual audit.']);
    else
        releasePublishClaimBestEffort( ...
            parallelPlan, config, publishClaim);
    end
    rethrow(exception);
end
end

function config = buildFrozenConfig( ...
    scriptDir, projectRoot, numberOfTrials, baseSeed, isSmoke, provenance)
schema = getLmbWireSchema();
config = struct();
config.evidenceSchema = 'fusion-sufficient-moment-exchange-v2';
config.workerSchema = 'fusion-sufficient-seed-worker-v2';
config.executionProtocol = 'deterministic-seed-subprocess-v2';
config.testOnly = false;
config.wireSchemaVersion = double(schema.version);
config.gitCommit = readGitCommit(projectRoot);
config.numberOfTrials = numberOfTrials;
config.baseSeed = baseSeed;
config.firstSeed = baseSeed + 1;
config.lastSeed = baseSeed + numberOfTrials;
config.armSelection = { ...
    'Periodic full GM fusion message', ...
    'Periodic moment message on static topology'};
config.internalArmSelectors = { ...
    'Periodic full posterior', ...
    'Periodic light posterior on static topology'};
config.simulationLength = 100;
config.includeFinalPeriodicLightVariants = true;
config.includeDynamicTopologyVariants = false;
config.lightCovarianceInflationEnabled = false;
config.modeAwareFusionWeights = false;
config.capturePosteriorSnapshots = true;
config.skipCalibrationForStaticPair = true;
config.requiredMaxExistenceResidual = 0;
config.requiredMaxMeanResidual = 0;
config.requiredMaxCovarianceResidual = 0;
config.bootstrapSeed = 20270710;
config.bootstrapResamples = 10000;
config.maxWorkers = 6;
config.octaveExecutable = 'octave-cli';
config.octaveVersion = '11.1.0';
config.executionLogicalCores = 10;
config.executionMemoryGiB = 16;
config.estimatedSingleWorkerMemoryMiB = 110;
config.primaryMetric = ...
    'paired attempted application-layer byte reduction per seed';
config.byteSemantics = ['Encoded application-layer bytes only; excludes ', ...
    'MAC/PHY framing, network/transport headers, fragmentation, and ', ...
    'retransmission. The loss model is payload-size independent.'];
config.changedVariable = ...
    'sender-side canonical moment projection before encoding';
config.outputDirectory = scriptDir;
config.projectRoot = projectRoot;
config.isSmoke = isSmoke;
if isSmoke
    config.artifactStem = ...
        'GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_V2_N5_SEEDS1001_1005';
    config.batchIdentity = 'smoke-seeds1001-1005-v2';
else
    config.artifactStem = ...
        'GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131';
    config.batchIdentity = 'confirmatory-primary-seeds82-131-v2';
end
config.regenerationCommand = sprintf([ ...
    'octave-cli --quiet --eval "setPath; addpath(''RUN/GA''); ', ...
    '[r,c,m]=runFusionSufficientMomentExchangeConfirmatory', ...
    '(true,%d,%d); disp(r); disp(c); disp(m);"'], ...
    numberOfTrials, baseSeed);
if isempty(fieldnames(provenance))
    config.requiredSourcesSha256 = ...
        hashFusionSufficientRequiredSources(projectRoot);
else
    if ~strcmp(provenance.head, config.gitCommit)
        error('FusionSufficientConfirmatory:CommitChanged', ...
            'Git HEAD changed while constructing the frozen config.');
    end
    config.requiredSourcesSha256 = ...
        provenance.requiredSourcesSha256;
end
config.configSha256 = hashFusionSufficientConfig(config);
end

function printFrozenConfig(config)
fprintf('\nFrozen ICASSP moment-exchange protocol\n');
fprintf('  Git commit: %s\n', config.gitCommit);
fprintf('  Trials: %d; paired seeds: %d:%d\n', ...
    config.numberOfTrials, config.firstSeed, config.lastSeed);
fprintf('  Immutable batch identity: %s\n', config.batchIdentity);
fprintf('  Arm 1: %s\n', config.armSelection{1});
fprintf('  Arm 2: %s\n', config.armSelection{2});
fprintf('  Simulation length: %d\n', config.simulationLength);
fprintf(['  Execution only (not a paper contribution): Octave %s, ', ...
    'fixed maxWorkers=%d\n'], config.octaveVersion, config.maxWorkers);
fprintf('  Posterior snapshots: %s\n', ...
    logicalText(config.capturePosteriorSnapshots));
fprintf('  Required residuals (r, mu, Sigma): %.17g, %.17g, %.17g\n', ...
    config.requiredMaxExistenceResidual, ...
    config.requiredMaxMeanResidual, ...
    config.requiredMaxCovarianceResidual);
fprintf('  Bootstrap: seed=%d, resamples=%d, percentile=[2.5,97.5]\n', ...
    config.bootstrapSeed, config.bootstrapResamples);
fprintf('  Byte semantics: %s\n\n', config.byteSemantics);
end

function publishFailureBestEffort(plan, config, reason)
try
    publishFusionSufficientBatchFailure(plan, config, reason);
catch failureException
    warning('FusionSufficientConfirmatory:FailureReceipt', ...
        'Unable to publish failure tombstone: %s', ...
        failureException.message);
end
end

function releasePublishClaimBestEffort(plan, config, claim)
try
    releaseFusionSufficientPublishClaim(plan, config, claim);
catch releaseException
    warning('FusionSufficientConfirmatory:PublishClaimRelease', ...
        ['Unable to release failed publish claim; manual audit is ', ...
        'required: %s'], releaseException.message);
end
end

function value = anyFinalArtifactExists(paths)
value = false;
for pathIdx = 1:numel(paths)
    if exist(paths{pathIdx}, 'file') == 2 || ...
            exist(paths{pathIdx}, 'dir') == 7
        value = true;
        return;
    end
end
end

function commit = readGitCommit(projectRoot)
command = sprintf('git -C %s rev-parse HEAD', shellQuote(projectRoot));
[status, output] = system(command);
if status ~= 0
    error('Unable to read Git commit for evidence provenance.');
end
commit = strtrim(output);
if isempty(regexp(commit, '^[0-9a-f]{40}$', 'once'))
    error('Git returned an invalid commit hash: %s', commit);
end
end

function quoted = shellQuote(value)
value = char(value);
quoted = ['''', strrep(value, '''', '''"''"'''), ''''];
end

function value = validateLogicalScalar(value, name)
if ~(islogical(value) || isnumeric(value)) || ~isscalar(value) || ...
        ~isreal(value) || ~isfinite(double(value)) || ...
        ~(double(value) == 0 || double(value) == 1)
    error('%s must be a logical scalar.', name);
end
value = logical(value);
end

function validateIntegerScalar(value, name, lowerBound)
if ~isnumeric(value) || ~isscalar(value) || ~isreal(value) || ...
        ~isfinite(value) || value ~= floor(value) || value < lowerBound
    error('%s must be an integer scalar >= %d.', name, lowerBound);
end
end

function value = logicalText(flag)
if flag
    value = 'true';
else
    value = 'false';
end
end
