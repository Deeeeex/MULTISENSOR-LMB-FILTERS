% RUNFILTERS - Run the single-sensor LMB or LMBM filters
%
% File guide:
%   Minimal single-sensor smoke test. It builds a model, simulates one
%   measurement stream, runs either LMB or LMBM, and plots the result. Use
%   this script when checking the baseline single-sensor pipeline before
%   moving to the multi-sensor experiments.

%% Admin
close all; clc;
setPath;
%% Generate model
useLmbFilter = true; % Use LMB filter, or use LMBM filter
model = generateModel(10, 0.95, 'LBP', 'Fixed');
%% Generate observations
[groundTruth, measurements, groundTruthRfs] = generateGroundTruth(model);
%% Run a filter
if (useLmbFilter)
    stateEstimates = runLmbFilter(model, measurements);
else
    stateEstimates = runLmbmFilter(model, measurements);
end
%% Plotting
plotResults(model, measurements, groundTruth, stateEstimates, groundTruthRfs);
