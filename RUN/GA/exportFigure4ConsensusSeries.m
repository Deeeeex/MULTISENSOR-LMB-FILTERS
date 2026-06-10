function csvPath = exportFigure4ConsensusSeries(outputPath, sourceMatPath)
% EXPORTFIGURE4CONSENSUSSERIES
% Export fixed, balanced, and cardinality-critical consensus time series for paper Figure 4.
% By default, reuse the saved 50-trial paper run instead of rerunning a
% shorter diagnostic export. The stored series are already trial means.

close all; clc;
scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir)
    scriptDir = pwd;
end
projectRoot = resolveProjectRoot(scriptDir);
addpath(projectRoot);
setPath;
addpath(fullfile(projectRoot, 'RUN', 'GA'));

if nargin < 1 || isempty(outputPath)
    outputPath = fullfile(projectRoot, 'docs', 'paper', 'figures', 'figure4_consensus_series.csv');
end
if nargin < 2 || isempty(sourceMatPath)
    sourceMatPath = fullfile(projectRoot, 'RUN', 'GA', 'mc50_20260527_172137', '01_tiered_main_fidfia_n50_seed1.mat');
end

outDir = fileparts(outputPath);
if ~isempty(outDir) && ~exist(outDir, 'dir')
    mkdir(outDir);
end

if ~exist(sourceMatPath, 'file')
    error('Figure 4 source MAT file not found: %s', sourceMatPath);
end
loaded = load(sourceMatPath, 'summary');
if ~isfield(loaded, 'summary')
    error('Figure 4 source MAT does not contain a summary struct: %s', sourceMatPath);
end
summary = loaded.summary;

if ~isfield(summary, 'consensusSeries')
    error('Consensus time-series were not returned by the ablation runner.');
end

series = summary.consensusSeries;
fixedIdx = findArmIndex(series.armNames, 'fixed weights');
balancedIdx = findArmIndex(series.armNames, '+structure-aware decoupled KLA');
cardinalityIdx = findArmIndex(series.armNames, '+FID-FIA existence refinement');

data = [ ...
    series.time(:), ...
    series.ospa(:, fixedIdx), series.ospa(:, balancedIdx), series.ospa(:, cardinalityIdx), ...
    series.pos(:, fixedIdx), series.pos(:, balancedIdx), series.pos(:, cardinalityIdx), ...
    series.card(:, fixedIdx), series.card(:, balancedIdx), series.card(:, cardinalityIdx)];

fid = fopen(outputPath, 'w');
if fid < 0
    error('Unable to open output CSV: %s', outputPath);
end
fprintf(fid, 'time,ospa_fixed,ospa_balanced,ospa_cardinality,rmse_fixed,rmse_balanced,rmse_cardinality,card_fixed,card_balanced,card_cardinality\n');
fclose(fid);
dlmwrite(outputPath, data, '-append');

csvPath = outputPath;
fprintf('Figure 4 50-trial mean consensus series exported: %s\n', csvPath);
fprintf('Source: %s\n', sourceMatPath);
end

function idx = findArmIndex(armNames, targetName)
idx = [];
for i = 1:numel(armNames)
    if strcmp(char(armNames{i}), targetName)
        idx = i;
        return;
    end
end
error('Required Figure 4 arm not found: %s', targetName);
end

function projectRoot = resolveProjectRoot(scriptDir)
projectRoot = scriptDir;
for k = 1:6
    if exist(fullfile(projectRoot, 'setPath.m'), 'file')
        return;
    end
    parent = fileparts(projectRoot);
    if isempty(parent) || strcmp(parent, projectRoot)
        break;
    end
    projectRoot = parent;
end
end
