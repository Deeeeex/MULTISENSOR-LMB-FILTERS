function [reportPath, summaryPath, summary, config] = ...
    runHeldoutPeriodicLightCandidateSelectionN20(runExperiment)
% RUNHELDOUTPERIODICLIGHTCANDIDATESELECTIONN20
% Frozen held-out 20-trial candidate-selection run for the effective KLA
% graph validation line.
%
% This entry point intentionally keeps the experiment parameters in one
% visible script so the run can be launched from MATLAB without a long
% command-line --eval string.
%
% MATLAB:
%   cd /path/to/MULTISENSOR-LMB-FILTERS
%   addpath('RUN/GA');
%   [reportPath, summaryPath, summary] = ...
%       runHeldoutPeriodicLightCandidateSelectionN20();
%
% Dry-run configuration check:
%   [~, ~, ~, config] = runHeldoutPeriodicLightCandidateSelectionN20(false);

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

fprintf('\nHeld-out candidate-selection configuration\n');
fprintf('Trials: %d, baseSeed: %d, fixed paired seeds: %d:%d\n', ...
    config.numberOfTrials, config.baseSeed, ...
    config.baseSeed + 1, config.baseSeed + config.numberOfTrials);
fprintf('Simulation length: %d steps\n', config.experimentOverrides.simulationLength);
for armIdx = 1:numel(config.armSelection)
    fprintf('  Arm %d: %s\n', armIdx, config.armSelection{armIdx});
end

if ~runExperiment
    fprintf('Dry run only. No experiment was launched.\n');
    return;
end

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
logPath = fullfile(scriptDir, sprintf( ...
    'heldout_periodic_light_n20_seeds12_31_%s.log', timestamp));
summaryPath = fullfile(scriptDir, sprintf( ...
    'heldout_periodic_light_n20_seeds12_31_%s.mat', timestamp));

fprintf('Log: %s\n', logPath);
diary(logPath);
cleanup = onCleanup(@() diary('off'));

[reportPath, summary] = ...
    runMultisensorFilters_formation_4plus4_DualThresholdEventTriggerCompare( ...
        config.numberOfTrials, config.baseSeed, config.useFixedSeed, ...
        config.writeReport, config.thresholdProfile, ...
        config.armSelection, config.experimentOverrides);

save(summaryPath, 'summary', 'reportPath', 'config');
printSummary(summary, reportPath, summaryPath);

clear cleanup;
diary('off');
end

function config = buildHeldoutConfig()
config = struct();
config.numberOfTrials = 20;
config.baseSeed = 11;  % Fixed paired held-out seeds 12:31.
config.useFixedSeed = true;
config.writeReport = true;
config.thresholdProfile = 'default';

config.experimentOverrides = struct( ...
    'simulationLength', 100, ...
    'includeCandidateSelectionVariants', true, ...
    'lightFloorThresholdLow', 0.20, ...
    'lightFloorThresholdHigh', 0.3708, ...
    'lightFloorStaticEdgeBonus', 0.35);

config.armSelection = { ...
    'Periodic full posterior', ...
    'Periodic light posterior + guarded dynamic topology', ...
    'Old mainline: LightFloor-GuardedTopo', ...
    'C1: LightBackbone-GuardedTopo', ...
    'C2: MixedLabel-LightFloor-GuardedTopo'};
end

function printSummary(summary, reportPath, summaryPath)
fprintf('\nHeld-out candidate-selection result\n');
fprintf('Report: %s\n', reportPath);
fprintf('Summary: %s\n', summaryPath);
fprintf('\nArm names:\n');
disp(summary.armNames');
fprintf('Payload bytes:\n');
disp(summary.communication.payloadBytes);
fprintf('Local E-OSPA:\n');
disp(summary.local.meanAcrossSensors.eOspa);
fprintf('Consensus OSPA:\n');
disp(summary.consensus.ospa);
fprintf('Position disagreement:\n');
disp(summary.consensus.position);
fprintf('Cardinality dispersion:\n');
disp(summary.consensus.cardinality);
fprintf('Byte reduction percent:\n');
disp([summary.acceptance.byteReductionPercent]);
fprintf('Local E-OSPA change percent:\n');
disp([summary.acceptance.localEOspaChangePercent]);
fprintf('Consensus OSPA change percent:\n');
disp([summary.acceptance.consensusOspaChangePercent]);
fprintf('Qualifies for scale-up:\n');
disp([summary.acceptance.qualifiesForScaleUp]);
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
