function csvPath = exportFigure4ConsensusSeries(outputPath)
% EXPORTFIGURE4CONSENSUSSERIES
% Export fixed, balanced, and cardinality-critical consensus time series for paper Figure 4.

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

outDir = fileparts(outputPath);
if ~isempty(outDir) && ~exist(outDir, 'dir')
    mkdir(outDir);
end

[~, summary] = runMultisensorFilters_formation_4plus4_TieredLinkAblation( ...
    5, 1, true, struct(), false, 'fidFiaExistenceRefinement', struct(), [1 3 4]);

if ~isfield(summary, 'consensusSeries')
    error('Consensus time-series were not returned by the ablation runner.');
end

series = summary.consensusSeries;
if size(series.ospa, 2) ~= 3
    error('Expected three arms for Figure 4 export, got %d.', size(series.ospa, 2));
end

data = [ ...
    series.time(:), ...
    series.ospa(:, 1), series.ospa(:, 2), series.ospa(:, 3), ...
    series.pos(:, 1), series.pos(:, 2), series.pos(:, 3), ...
    series.card(:, 1), series.card(:, 2), series.card(:, 3)];

fid = fopen(outputPath, 'w');
if fid < 0
    error('Unable to open output CSV: %s', outputPath);
end
fprintf(fid, 'time,ospa_fixed,ospa_balanced,ospa_cardinality,rmse_fixed,rmse_balanced,rmse_cardinality,card_fixed,card_balanced,card_cardinality\n');
fclose(fid);
dlmwrite(outputPath, data, '-append');

csvPath = outputPath;
fprintf('Figure 4 consensus series exported: %s\n', csvPath);
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
