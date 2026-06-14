function [reportPath, summaryPath, summary, config] = ...
    runTopologyRoleMatchedRecoveryStressHeldoutN50(runExperiment)
% RUNTOPOLOGYROLEMATCHEDRECOVERYSTRESSHELDOUTN50
% Held-out 50-trial validation for the topology recovery stress line.
%
% This runner keeps all experiment parameters in this script so it can be
% launched from MATLAB without a long command-line --eval string.
%
% Default fixed paired seeds: 107:156. These are intentionally held out from
% the seed=96 attribution/debug run used to identify the role-matched bridge
% repair policy.
%
% MATLAB:
%   cd /path/to/MULTISENSOR-LMB-FILTERS
%   addpath('RUN/GA');
%   [reportPath, summaryPath, summary] = ...
%       runTopologyRoleMatchedRecoveryStressHeldoutN50();
%
% Dry-run configuration check:
%   [~, ~, ~, config] = ...
%       runTopologyRoleMatchedRecoveryStressHeldoutN50(false);

if nargin < 1 || isempty(runExperiment)
    runExperiment = true;
end

scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir)
    scriptDir = pwd;
end
projectRoot = resolveProjectRoot(scriptDir);
addpath(projectRoot);
addpath(scriptDir);
setPath;

config = buildHeldoutConfig();
reportPath = '';
summaryPath = '';
summary = [];

fprintf('\nRole-matched recovery stress held-out validation\n');
fprintf('Trials: %d, baseSeed: %d, fixed paired seeds: %d:%d\n', ...
    config.numberOfTrials, config.baseSeed, ...
    config.baseSeed + 1, config.baseSeed + config.numberOfTrials);
fprintf('Simulation length: %d steps\n', ...
    config.experimentOverrides.simulationLength);
fprintf('Static bridge drop: %.2f, alternate edge drop: %.2f\n', ...
    config.staticBridgeDrop, config.alternateEdgeDrop);
fprintf('Role-matched bridge pairs:\n');
disp(config.roleMatchedBridgePairs);
for armIdx = 1:numel(config.armSelection)
    fprintf('  Arm %d: %s\n', armIdx, config.armSelection{armIdx});
end

if ~runExperiment
    fprintf('Dry run only. No experiment was launched.\n');
    return;
end

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
logPath = fullfile(scriptDir, sprintf( ...
    'topology_role_matched_recovery_stress_n50_seeds107_156_%s.log', ...
    timestamp));
summaryPath = fullfile(scriptDir, sprintf( ...
    'topology_role_matched_recovery_stress_n50_seeds107_156_%s.mat', ...
    timestamp));

fprintf('Log: %s\n', logPath);
diary(logPath);
cleanup = onCleanup(@() diary('off'));

[reportPath, summary] = ...
    runMultisensorFilters_formation_4plus4_DualThresholdEventTriggerCompare( ...
        config.numberOfTrials, config.baseSeed, config.useFixedSeed, ...
        config.writeReport, config.thresholdProfile, ...
        config.armSelection, config.experimentOverrides);

save(summaryPath, 'summary', 'reportPath', 'config');
printSummary(summary, reportPath, summaryPath, ...
    config.staticBaselineArmName);

clear cleanup;
diary('off');
end

function config = buildHeldoutConfig()
config = struct();
config.numberOfTrials = 50;
config.baseSeed = 106;  % Fixed paired held-out seeds 107:156.
config.useFixedSeed = true;
config.writeReport = true;
config.thresholdProfile = 'default';
config.staticBridgeDrop = 0.97;
config.alternateEdgeDrop = 0.08;
config.roleMatchedBridgePairs = [1 6; 2 8; 3 5; 4 7];
config.staticBaselineArmName = ...
    'Recovery stress: Periodic light static topology';

pDropByEdge = buildBridgeFailureDropMatrix( ...
    8, config.staticBridgeDrop, config.alternateEdgeDrop);
config.experimentOverrides = struct( ...
    'simulationLength', 100, ...
    'calibration', buildAlwaysLightCalibrationStub(), ...
    'pDropByEdge', pDropByEdge, ...
    'includeTopologyRecoveryStressVariants', true, ...
    'stableReliabilityBridgeBudget', 4, ...
    'stableReliabilityMaxBridgeDegree', 1, ...
    'matchedReliabilityBridgePairs', config.roleMatchedBridgePairs);

config.armSelection = { ...
    'Recovery stress: Periodic light static topology', ...
    'Recovery stress: Reliability-guarded dynamic topology', ...
    'Recovery stress: Balanced reliability repair topology', ...
    'Recovery stress: Distance-balanced reliability repair topology', ...
    'Recovery stress: Role-matched reliability repair topology'};
end

function calibration = buildAlwaysLightCalibrationStub()
calibration = struct();
calibration.multi.loose = [0.20, 0.40];
calibration.multi.default = [0.25, 0.60];
calibration.multi.strict = [0.35, 0.80];
calibration.information.loose = [0.20, 0.40];
calibration.information.default = [0.25, 0.60];
calibration.information.strict = [0.35, 0.80];
calibration.sampleCount.utility = 0;
calibration.sampleCount.informationGain = 0;
end

function pDropByEdge = buildBridgeFailureDropMatrix( ...
    numberOfSensors, bridgeDrop, defaultDrop)
pDropByEdge = defaultDrop * ones(numberOfSensors);
pDropByEdge(1:numberOfSensors+1:end) = 0;
staticBridgeEdges = [1 5; 2 6; 3 7; 4 8];
for edgeIdx = 1:size(staticBridgeEdges, 1)
    left = staticBridgeEdges(edgeIdx, 1);
    right = staticBridgeEdges(edgeIdx, 2);
    pDropByEdge(left, right) = bridgeDrop;
    pDropByEdge(right, left) = bridgeDrop;
end
end

function printSummary(summary, reportPath, summaryPath, baselineArmName)
fprintf('\nRole-matched recovery stress held-out result\n');
fprintf('Report: %s\n', reportPath);
fprintf('Summary: %s\n', summaryPath);
fprintf('\nArm names:\n');
disp(summary.armNames');
fprintf('Payload bytes:\n');
disp(summary.communication.payloadBytes);
fprintf('Attempt counts:\n');
disp(summary.communication.attemptCount);
fprintf('Delivery counts:\n');
disp(summary.communication.deliveryCount);
fprintf('Local E-OSPA:\n');
disp(summary.local.meanAcrossSensors.eOspa);
fprintf('Consensus OSPA:\n');
disp(summary.consensus.ospa);
fprintf('Position disagreement:\n');
disp(summary.consensus.position);
fprintf('Cardinality dispersion:\n');
disp(summary.consensus.cardinality);
fprintf('Effective-weight lambda2:\n');
disp(summary.effectiveGraph.effectiveWeightConnectivity);
printRecoveryGate(summary, baselineArmName);
end

function printRecoveryGate(summary, baselineArmName)
baselineIdx = find(strcmp(summary.armNames, baselineArmName), 1);
if isempty(baselineIdx)
    fprintf('Recovery gate skipped: static baseline arm not found.\n');
    return;
end
numberOfArms = numel(summary.armNames);
localTrials = squeeze(mean(summary.trials.localEOspa, 2));
if isvector(localTrials)
    localTrials = reshape(localTrials, [], numberOfArms);
end
baselineLocal = localTrials(:, baselineIdx);
baselineConsensus = summary.trials.consensusOspa(:, baselineIdx);
baselinePosition = summary.trials.consensusPosition(:, baselineIdx);
baselineCardinality = summary.trials.consensusCardinality(:, baselineIdx);

fprintf('\nRecovery gate vs `%s`:\n', baselineArmName);
fprintf(['Arm | Attempt delta | Delivery delta | Local %% | ', ...
    'Consensus %% | Position %% | Card. %% | Pass count\n']);
for armIdx = 1:numberOfArms
    localChange = percentChange(baselineLocal, localTrials(:, armIdx));
    consensusChange = percentChange( ...
        baselineConsensus, summary.trials.consensusOspa(:, armIdx));
    positionChange = percentChange( ...
        baselinePosition, summary.trials.consensusPosition(:, armIdx));
    cardinalityChange = percentChange( ...
        baselineCardinality, summary.trials.consensusCardinality(:, armIdx));
    passes = localChange <= 5 & consensusChange <= 10 & ...
        positionChange <= 10 & cardinalityChange <= 10;
    attemptDelta = mean(summary.trials.attemptCount(:, armIdx) - ...
        summary.trials.attemptCount(:, baselineIdx));
    deliveryDelta = mean(summary.trials.deliveryCount(:, armIdx) - ...
        summary.trials.deliveryCount(:, baselineIdx));
    fprintf('%s | %.1f | %.1f | %+0.2f | %+0.2f | %+0.2f | %+0.2f | %d/%d\n', ...
        summary.armNames{armIdx}, attemptDelta, deliveryDelta, ...
        mean(localChange), mean(consensusChange), mean(positionChange), ...
        mean(cardinalityChange), sum(passes), numel(passes));
end
end

function change = percentChange(baseline, value)
change = 100 * (value ./ max(baseline, eps) - 1);
end

function projectRoot = resolveProjectRoot(scriptDir)
projectRoot = scriptDir;
while ~exist(fullfile(projectRoot, 'setPath.m'), 'file')
    parentDir = fileparts(projectRoot);
    if strcmp(parentDir, projectRoot)
        error('Could not locate project root from %s.', scriptDir);
    end
    projectRoot = parentDir;
end
end
