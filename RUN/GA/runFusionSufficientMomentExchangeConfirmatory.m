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

if runExperiment
    assertFusionSufficientGitProvenance(projectRoot);
end
config = buildFrozenConfig( ...
    scriptDir, projectRoot, numberOfTrials, baseSeed, isSmoke);
reportPath = '';
csvPath = '';
summaryPath = '';
summary = struct();
printFrozenConfig(config);
if ~runExperiment
    return;
end

overrides = struct( ...
    'includeFinalPeriodicLightVariants', true, ...
    'includeDynamicTopologyVariants', false, ...
    'includeEffectiveKlaGraphVariants', false, ...
    'includeLightFloorVariants', false, ...
    'includePayloadRefinementVariants', false, ...
    'skipCalibrationForStaticPair', true, ...
    'paperStaticPair', true, ...
    'capturePosteriorSnapshots', true, ...
    'simulationLength', config.simulationLength);
[~, summary] = ...
    runMultisensorFilters_formation_4plus4_DualThresholdEventTriggerCompare( ...
        config.numberOfTrials, config.baseSeed, true, false, ...
        'default', config.armSelection, overrides);
assertFrozenRun(summary, config);
[reportPath, csvPath, summaryPath] = ...
    writeFusionSufficientEvidence(summary, config);
end

function config = buildFrozenConfig( ...
    scriptDir, projectRoot, numberOfTrials, baseSeed, isSmoke)
schema = getLmbWireSchema();
config = struct();
config.evidenceSchema = 'fusion-sufficient-moment-exchange-v1';
config.wireSchemaVersion = double(schema.version);
config.gitCommit = readGitCommit(projectRoot);
config.numberOfTrials = numberOfTrials;
config.baseSeed = baseSeed;
config.firstSeed = baseSeed + 1;
config.lastSeed = baseSeed + numberOfTrials;
config.armSelection = { ...
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
        'GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N5_SEEDS1001_1005';
else
    config.artifactStem = ...
        'GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131';
end
config.regenerationCommand = sprintf([ ...
    'octave --quiet --eval "setPath; addpath(''RUN/GA''); ', ...
    '[r,c,m]=runFusionSufficientMomentExchangeConfirmatory', ...
    '(true,%d,%d); disp(r); disp(c); disp(m);"'], ...
    numberOfTrials, baseSeed);
end

function printFrozenConfig(config)
fprintf('\nFrozen ICASSP moment-exchange protocol\n');
fprintf('  Git commit: %s\n', config.gitCommit);
fprintf('  Trials: %d; paired seeds: %d:%d\n', ...
    config.numberOfTrials, config.firstSeed, config.lastSeed);
fprintf('  Arm 1: %s\n', config.armSelection{1});
fprintf('  Arm 2: %s\n', config.armSelection{2});
fprintf('  Simulation length: %d\n', config.simulationLength);
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

function assertFrozenRun(summary, config)
assert(isequal(summary.armNames, config.armSelection));
assert(summary.numberOfTrials == config.numberOfTrials);
assert(isequal(summary.trialSeeds, ...
    config.firstSeed:config.lastSeed));
assert(summary.scenarioConfig.simulationLength == ...
    config.simulationLength);
assert(summary.equivalence.captured);
for armIdx = 1:2
    trigger = summary.arms(armIdx).triggerConfig;
    assert(trigger.capturePosteriorSnapshots);
    assert(~trigger.linkGateEnabled);
    assert(~trigger.forceInitialHeavy);
    assert(~trigger.forceLabelChangeHeavy);
    assert(~trigger.forceStaleHeavy);
    assert(~trigger.useStaleNeighborCache);
    assert(~trigger.labelHeartbeatEnabled);
    assert(~trigger.mixedPayloadEnabled);
    assert(~trigger.mixedPayloadLightForAllActiveLabels);
    assert(~trigger.dynamicTopologyEnabled);
    assert(~trigger.modeAwareFusionWeights);
    assert(~trigger.lightCovarianceInflationEnabled);
    assert(trigger.lightFusionWeightFactor == ...
        trigger.heavyFusionWeightFactor);
end
assert(strcmp(summary.arms(1).triggerConfig.eventPolicy, 'alwaysHeavy'));
assert(strcmp(summary.arms(2).triggerConfig.eventPolicy, 'alwaysLight'));
fullTrigger = rmfield(summary.arms(1).triggerConfig, 'eventPolicy');
momentTrigger = rmfield(summary.arms(2).triggerConfig, 'eventPolicy');
assert(isequaln(orderfields(fullTrigger), orderfields(momentTrigger)));
for trialIdx = 1:config.numberOfTrials
    assert(isequal( ...
        summary.trials.attemptedMask{trialIdx, 1}, ...
        summary.trials.attemptedMask{trialIdx, 2}));
    assert(isequal( ...
        summary.trials.deliveredMask{trialIdx, 1}, ...
        summary.trials.deliveredMask{trialIdx, 2}));
end
assert(all(summary.trials.deliveredPayloadBytes(:) <= ...
    summary.trials.attemptedPayloadBytes(:)));
assert(all(summary.trials.posteriorMissingSnapshotCount(:, 2) == 0));
expectedSnapshotCount = summary.scenarioConfig.numberOfSensors * ...
    summary.scenarioConfig.simulationLength;
assert(all(summary.trials.posteriorSnapshotCount(:) == ...
    expectedSnapshotCount));
assert(all(summary.trials.posteriorLabelSetMismatchCount(:, 2) == 0));
assert(all(summary.trials.posteriorMissingLabelCount(:, 2) == 0));
assert(all(summary.trials.posteriorMaxAbsR(:, 2) == ...
    config.requiredMaxExistenceResidual));
assert(all(summary.trials.posteriorMaxAbsMu(:, 2) == ...
    config.requiredMaxMeanResidual));
assert(all(summary.trials.posteriorMaxAbsSigma(:, 2) == ...
    config.requiredMaxCovarianceResidual));
assert(all(summary.trials.posteriorExactMatch(:, 2)));
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
