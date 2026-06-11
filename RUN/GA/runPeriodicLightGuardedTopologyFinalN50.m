function [reportPath, summaryPath, summary, config] = ...
    runPeriodicLightGuardedTopologyFinalN50(runExperiment)
% RUNPERIODICLIGHTGUARDEDTOPOLOGYFINALN50
% Final held-out validation runner for the periodic-light guarded topology
% candidate selected by the effective KLA graph study.
%
% The default run uses deterministic paired seeds 32:81 and compares:
%   1. Full posterior on the static 4+4 topology.
%   2. Full posterior on guarded dynamic topology.
%   3. Light posterior on the static 4+4 topology.
%   4. Light posterior on guarded dynamic topology.
%
% MATLAB:
%   cd /path/to/MULTISENSOR-LMB-FILTERS
%   addpath('RUN/GA');
%   [reportPath, summaryPath, summary] = ...
%       runPeriodicLightGuardedTopologyFinalN50();
%
% Dry-run configuration check:
%   [~, ~, ~, config] = runPeriodicLightGuardedTopologyFinalN50(false);

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

config = buildFinalConfig();
reportPath = '';
summaryPath = '';
summary = [];

fprintf('\nPeriodic-light guarded topology final validation\n');
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
    'periodic_light_guarded_topology_final_n50_seeds32_81_%s.log', ...
    timestamp));
summaryPath = fullfile(scriptDir, sprintf( ...
    'periodic_light_guarded_topology_final_n50_seeds32_81_%s.mat', ...
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
printSummary(summary, reportPath, summaryPath);

clear cleanup;
diary('off');
end

function config = buildFinalConfig()
config = struct();
config.numberOfTrials = 50;
config.baseSeed = 31;  % Fixed paired held-out seeds 32:81.
config.useFixedSeed = true;
config.writeReport = true;
config.thresholdProfile = 'default';

config.experimentOverrides = struct( ...
    'simulationLength', 100, ...
    'includeDynamicTopologyVariants', true, ...
    'includeFinalPeriodicLightVariants', true, ...
    'lightFloorStaticEdgeBonus', 0.35);

config.armSelection = { ...
    'Periodic full posterior', ...
    'Periodic full posterior + dynamic topology', ...
    'Periodic light posterior on static topology', ...
    'Periodic light posterior + guarded dynamic topology'};
end

function printSummary(summary, reportPath, summaryPath)
fprintf('\nPeriodic-light guarded topology final result\n');
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
