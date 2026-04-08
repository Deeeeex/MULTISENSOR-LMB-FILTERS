function test_export_figure4_series()
run('setPath.m');
addpath(fullfile(pwd, 'RUN', 'GA'));

outputDir = fullfile('docs', 'paper', 'figures');
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end
csvPath = fullfile(outputDir, 'figure4_consensus_series.csv');
if exist(csvPath, 'file')
    delete(csvPath);
end

exportedPath = exportFigure4ConsensusSeries(csvPath);
assert(exist(exportedPath, 'file') == 2, 'CSV export file was not created.');

fid = fopen(exportedPath, 'r');
assert(fid >= 0, 'Unable to open exported CSV file.');
headerLine = fgetl(fid);
fclose(fid);

expectedHeader = 'time,ospa_fixed,ospa_adaptive,rmse_fixed,rmse_adaptive,card_fixed,card_adaptive';
assert(strcmp(strtrim(headerLine), expectedHeader), 'Unexpected CSV header.');

data = dlmread(exportedPath, ',', 1, 0);
assert(size(data, 2) == 7, 'Expected 7 CSV columns.');
assert(size(data, 1) >= 50, 'Expected at least 50 time steps in exported series.');
assert(mean(data(:, 3)) < mean(data(:, 2)), 'Adaptive OSPA mean should be below fixed OSPA mean.');
assert(mean(data(:, 5)) < mean(data(:, 4)), 'Adaptive RMSE mean should be below fixed RMSE mean.');
assert(mean(data(:, 7)) < mean(data(:, 6)), 'Adaptive cardinality mean should be below fixed cardinality mean.');

fprintf('Figure 4 consensus series export test passed.\n');
end
