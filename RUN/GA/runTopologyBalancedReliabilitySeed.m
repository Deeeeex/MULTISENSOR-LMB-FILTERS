function [reportPath, summaryPath, summary, config] = ...
    runTopologyBalancedReliabilitySeed( ...
        seed, simulationLength, runExperiment, includeStaticBaseline, ...
        bridgePairs)
% RUNTOPOLOGYBALANCEDRELIABILITYSEED
% Focused seed-level runner for the balanced reliability-repair topology
% candidate under targeted bridge-link failures.

if nargin < 1 || isempty(seed)
    seed = 96;
end
if nargin < 2 || isempty(simulationLength)
    simulationLength = 100;
end
if nargin < 3 || isempty(runExperiment)
    runExperiment = true;
end
if nargin < 4 || isempty(includeStaticBaseline)
    includeStaticBaseline = false;
end
if nargin < 5
    bridgePairs = [];
end

scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir)
    scriptDir = pwd;
end
projectRoot = resolveProjectRoot(scriptDir);
addpath(projectRoot);
addpath(scriptDir);
setPath;

config = struct();
config.seed = seed;
config.baseSeed = seed - 1;
config.simulationLength = simulationLength;
config.useFixedSeed = true;
config.writeReport = true;
config.thresholdProfile = 'default';
config.staticBridgeDrop = 0.97;
config.alternateEdgeDrop = 0.08;
config.includeStaticBaseline = includeStaticBaseline;
config.bridgeBudget = 4;
config.maxBridgeDegree = 1;
config.bridgePairs = bridgePairs;

pDropByEdge = buildBridgeFailureDropMatrix( ...
    8, config.staticBridgeDrop, config.alternateEdgeDrop);
experimentOverrides = struct( ...
    'simulationLength', simulationLength, ...
    'calibration', buildAlwaysLightCalibrationStub(), ...
    'pDropByEdge', pDropByEdge, ...
    'includeTopologyRecoveryStressVariants', true, ...
    'stableReliabilityBridgeBudget', config.bridgeBudget, ...
    'stableReliabilityMaxBridgeDegree', config.maxBridgeDegree);
if ~isempty(bridgePairs)
    experimentOverrides.stableReliabilityBridgePairs = bridgePairs;
end
config.experimentOverrides = experimentOverrides;

if includeStaticBaseline
    config.armSelection = { ...
        'Recovery stress: Periodic light static topology', ...
        'Recovery stress: Balanced reliability repair topology'};
else
    config.armSelection = { ...
        'Recovery stress: Balanced reliability repair topology'};
end

reportPath = '';
summary = [];
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
summaryPath = fullfile(scriptDir, sprintf( ...
    'topology_balanced_reliability_seed%d_len%d_%s.mat', ...
    seed, simulationLength, timestamp));

fprintf('\nBalanced reliability-repair topology seed run\n');
fprintf('Seed: %d, baseSeed: %d, simulationLength: %d\n', ...
    config.seed, config.baseSeed, config.simulationLength);
fprintf('Static bridge drop: %.2f, alternate edge drop: %.2f\n', ...
    config.staticBridgeDrop, config.alternateEdgeDrop);
fprintf('Bridge budget: %d, max bridge degree: %d\n', ...
    config.bridgeBudget, config.maxBridgeDegree);
if ~isempty(bridgePairs)
    fprintf('Forced bridge pairs:\n');
    disp(bridgePairs);
end
disp(config.armSelection');

if ~runExperiment
    fprintf('Dry run only. No experiment was launched.\n');
    return;
end

[reportPath, summary] = ...
    runMultisensorFilters_formation_4plus4_DualThresholdEventTriggerCompare( ...
        1, config.baseSeed, config.useFixedSeed, config.writeReport, ...
        config.thresholdProfile, config.armSelection, ...
        config.experimentOverrides);

save(summaryPath, 'summary', 'reportPath', 'config');
printSummary(summary, reportPath, summaryPath);
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

function printSummary(summary, reportPath, summaryPath)
fprintf('\nBalanced reliability-repair topology result\n');
fprintf('Report: %s\n', reportPath);
fprintf('Summary: %s\n', summaryPath);
fprintf('Arm names:\n');
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
if isfield(summary.trials, 'topologyChurnRate')
    fprintf('Topology churn rate:\n');
    disp(summary.trials.topologyChurnRate);
end
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
