function [reportPath, summary] = ...
    runMixtureAwareHeavyFusionAmbiguitySmoke(writeReport)
% RUNMIXTUREAWAREHEAVYFUSIONAMBIGUITYSMOKE Minimal close-crossing proxy.
%
% This smoke test isolates the fusion layer from trajectory generation. Two
% labels share the same two spatial modes after a close crossing. Light
% payloads moment-match each label toward the middle, while mixture-aware
% heavy fusion should keep the correct dominant mode for each label.

if nargin < 1 || isempty(writeReport)
    writeReport = true;
end

model = generateMultisensorModel( ...
    2, [1, 1], [0.9, 0.9], [3, 3], 'GA', 'LBP');
model.existenceThreshold = 0.01;

trueStates = containers.Map();
trueStates(labelKey(1, 1)) = [-4; 0; 0; 0];
trueStates(labelKey(1, 2)) = [4; 0; 0; 0];

leftModes = {[-4.0; 0; 0; 0], [4.0; 0; 0; 0]};
rightModes = {[-4.2; 0; 0; 0], [4.2; 0; 0; 0]};
modeCovariances = {0.5 * eye(4), 0.5 * eye(4)};

sensorOne = [ ...
    makeAmbiguousObject(model, 1, 1, 0.90, [0.65, 0.35], ...
        leftModes, modeCovariances), ...
    makeAmbiguousObject(model, 1, 2, 0.90, [0.35, 0.65], ...
        leftModes, modeCovariances)];
sensorTwo = [ ...
    makeAmbiguousObject(model, 1, 1, 0.90, [0.60, 0.40], ...
        rightModes, modeCovariances), ...
    makeAmbiguousObject(model, 1, 2, 0.90, [0.40, 0.60], ...
        rightModes, modeCovariances)];

weights = [0.5, 0.5];
lightInputs = { ...
    compressLmbPosterior(sensorOne, model, 0.01), ...
    compressLmbPosterior(sensorTwo, model, 0.01)};
lightFused = fuseLmbPosteriorsByLabel( ...
    lightInputs, weights, model, weights);

fusionDetails = struct( ...
    'sourceIndices', [1, 2], ...
    'isStale', [false, false], ...
    'isBaseEdge', [true, true], ...
    'age', [0, 0], ...
    'eventType', [2, 2], ...
    'weights', weights);
mixtureAwareConfig = struct( ...
    'mixtureAwareHeavyFusionEnabled', true, ...
    'mixtureAwareTopComponents', 2, ...
    'mixtureAwareMaxFusedComponents', 4, ...
    'mixtureAwareMaxComponentTuples', 16, ...
    'mixtureAwareMinEntropy', 0.2);
heavyFused = fuseLmbPosteriorsByLabel( ...
    {sensorOne, sensorTwo}, weights, model, weights, ...
    fusionDetails, mixtureAwareConfig);

summary = struct();
summary.lightMeanError = meanLabelError(lightFused, trueStates);
summary.heavyMeanError = meanLabelError(heavyFused, trueStates);
summary.errorReductionPercent = 100 * ...
    (summary.lightMeanError - summary.heavyMeanError) / ...
    max(summary.lightMeanError, eps);
summary.lightComponentCounts = [lightFused.numberOfGmComponents];
summary.heavyComponentCounts = [heavyFused.numberOfGmComponents];
summary.lightModePositions = extractFirstModePositions(lightFused);
summary.heavyModePositions = extractFirstModePositions(heavyFused);

fprintf('Mixture-aware heavy fusion ambiguity smoke\n');
fprintf('  Light mean error: %.4f\n', summary.lightMeanError);
fprintf('  Heavy mean error: %.4f\n', summary.heavyMeanError);
fprintf('  Error reduction: %.2f%%\n', summary.errorReductionPercent);
fprintf('  Light components: %s\n', mat2str(summary.lightComponentCounts));
fprintf('  Heavy components: %s\n', mat2str(summary.heavyComponentCounts));

if summary.heavyMeanError >= summary.lightMeanError
    error('Mixture-aware heavy fusion did not improve the ambiguity proxy.');
end
if any(summary.heavyComponentCounts < 2)
    error('Mixture-aware heavy fusion failed to preserve multiple modes.');
end

reportPath = '';
if writeReport
    reportPath = fullfile('RUN', 'GA', sprintf( ...
        'MIXTURE_AWARE_HEAVY_FUSION_SMOKE_%s.md', ...
        datestr(now, 'yyyymmdd_HHMMSS')));
    writeSmokeReport(reportPath, summary);
end
end

function object = makeAmbiguousObject( ...
    model, birthTime, birthLocation, existence, weights, modes, covariances)
object = model.object;
object(1).birthTime = birthTime;
object(1).birthLocation = birthLocation;
object(1).r = existence;
object(1).numberOfGmComponents = numel(weights);
object(1).w = weights / sum(weights);
object(1).mu = modes;
object(1).Sigma = covariances;
object(1).trajectoryLength = 0;
object(1).trajectory = zeros(model.xDimension, 0);
object(1).timestamps = zeros(1, 0);
end

function value = meanLabelError(objects, trueStates)
errors = zeros(1, numel(objects));
for objectIdx = 1:numel(objects)
    key = labelKey(objects(objectIdx).birthTime, ...
        objects(objectIdx).birthLocation);
    trueState = trueStates(key);
    estimate = objects(objectIdx).mu{1};
    errors(objectIdx) = norm(estimate(1:2) - trueState(1:2));
end
value = mean(errors);
end

function positions = extractFirstModePositions(objects)
positions = zeros(numel(objects), 2);
for objectIdx = 1:numel(objects)
    positions(objectIdx, :) = [ ...
        objects(objectIdx).birthLocation, objects(objectIdx).mu{1}(1)];
end
end

function key = labelKey(birthTime, birthLocation)
key = sprintf('%d:%d', birthTime, birthLocation);
end

function writeSmokeReport(reportPath, summary)
fid = fopen(reportPath, 'w');
if fid < 0
    error('Could not open report for writing: %s', reportPath);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# Mixture-aware heavy fusion ambiguity smoke\n\n');
fprintf(fid, ...
    'This is a deterministic fusion-layer proxy for a close-crossing ambiguity.\n\n');
fprintf(fid, '| Metric | Light | Mixture-aware heavy |\n');
fprintf(fid, '|:--|--:|--:|\n');
fprintf(fid, '| Mean label position error | %.4f | %.4f |\n', ...
    summary.lightMeanError, summary.heavyMeanError);
fprintf(fid, '| Component counts | %s | %s |\n', ...
    mat2str(summary.lightComponentCounts), ...
    mat2str(summary.heavyComponentCounts));
fprintf(fid, '\nError reduction: %.2f%%.\n', ...
    summary.errorReductionPercent);
end
