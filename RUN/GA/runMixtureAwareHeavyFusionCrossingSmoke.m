function [reportPath, summary] = ...
    runMixtureAwareHeavyFusionCrossingSmoke(seed, writeReport, mixtureOverrides)
% RUNMIXTUREAWAREHEAVYFUSIONCROSSINGSMOKE Small real-filter crossing stress.
%
% Two targets cross under elevated clutter. This runner compares periodic
% light, periodic heavy with the legacy moment-matched fusion path, and
% periodic heavy with mixture-aware label-wise GM-KLA enabled.

if nargin < 1 || isempty(seed)
    seed = 7;
end
if nargin < 2 || isempty(writeReport)
    writeReport = true;
end
if nargin < 3 || isempty(mixtureOverrides)
    mixtureOverrides = struct();
end
rng(seed);

simulationLength = 40;
numberOfSensors = 2;
clutterRates = 8 * ones(1, numberOfSensors);
detectionProbabilities = 0.85 * ones(1, numberOfSensors);
measurementNoise = 4 * ones(1, numberOfSensors);

targetFormationConfig = struct();
targetFormationConfig.targetFormationEnabled = true;
targetFormationConfig.targetFormationCount = 2;
targetFormationConfig.targetFormationStaggeredBirths = false;
targetFormationConfig.targetFormationStartTime = 1;
targetFormationConfig.targetFormationLifeSpan = simulationLength;
targetFormationConfig.targetBirthStates = [ ...
    -18,  18; ...
      0,   0; ...
      1,  -1; ...
      0,   0];

model = generateMultisensorModel( ...
    numberOfSensors, clutterRates, detectionProbabilities, ...
    measurementNoise, 'GA', 'LBP', 'Formation', targetFormationConfig);
model.simulationLength = simulationLength;
model.rB = 0.08 * ones(model.numberOfBirthLocations, 1);
model.SigmaB = repmat({diag([10, 10, 4, 4]).^2}, ...
    model.numberOfBirthLocations, 1);
model.gmWeightThreshold = 1e-4;
model.maximumNumberOfGmComponents = 12;

[~, measurements, groundTruthRfs, sensorTrajectories] = ...
    generateMultisensorGroundTruth(model);
neighborMap = {[1, 2], [1, 2]};
commConfig = struct( ...
    'forceDelivery', true, ...
    'pDropBySensor', zeros(1, numberOfSensors));

arms = buildCrossingSmokeArms(mixtureOverrides);
summary = struct();
summary.seed = seed;
summary.armNames = {arms.name};
summary.meanEospa = zeros(1, numel(arms));
summary.crossingEospa = zeros(1, numel(arms));
summary.meanCardinality = zeros(1, numel(arms));
summary.crossingCardinality = zeros(1, numel(arms));
summary.crossingCardinalityError = zeros(1, numel(arms));
summary.payloadBytes = zeros(1, numel(arms));
summary.heavyRate = zeros(1, numel(arms));
summary.lightRate = zeros(1, numel(arms));
summary.componentCounts = zeros(1, numel(arms));

for armIdx = 1:numel(arms)
    rng(seed);
    [stateEstimatesBySensor, diagnostics] = ...
        runEventTriggeredDistributedLmbFilter( ...
            model, measurements, sensorTrajectories, neighborMap, ...
            commConfig, arms(armIdx).triggerConfig);
    [meanEospa, crossingEospa, meanCardinality, ...
        crossingCardinality, crossingCardinalityError] = scoreSensors( ...
        stateEstimatesBySensor, model, groundTruthRfs);
    summary.meanEospa(armIdx) = meanEospa;
    summary.crossingEospa(armIdx) = crossingEospa;
    summary.meanCardinality(armIdx) = meanCardinality;
    summary.crossingCardinality(armIdx) = crossingCardinality;
    summary.crossingCardinalityError(armIdx) = crossingCardinalityError;
    summary.payloadBytes(armIdx) = diagnostics.summary.payloadBytes;
    summary.heavyRate(armIdx) = diagnostics.summary.heavyRate;
    summary.lightRate(armIdx) = diagnostics.summary.lightRate;
    summary.componentCounts(armIdx) = meanSurvivingComponentCount( ...
        stateEstimatesBySensor);
    fprintf('%s: mean E-OSPA %.4f, crossing E-OSPA %.4f, bytes %.0f\n', ...
        arms(armIdx).name, meanEospa, crossingEospa, ...
        summary.payloadBytes(armIdx));
end

lightIdx = 1;
legacyHeavyIdx = 2;
mixtureHeavyIdx = 3;
summary.mixtureVsLightCrossingChangePercent = percentChange( ...
    summary.crossingEospa(lightIdx), summary.crossingEospa(mixtureHeavyIdx));
summary.mixtureVsLegacyHeavyCrossingChangePercent = percentChange( ...
    summary.crossingEospa(legacyHeavyIdx), ...
    summary.crossingEospa(mixtureHeavyIdx));

reportPath = '';
if writeReport
    reportPath = fullfile('RUN', 'GA', sprintf( ...
        'MIXTURE_AWARE_HEAVY_CROSSING_SMOKE_SEED%d_%s.md', ...
        seed, datestr(now, 'yyyymmdd_HHMMSS')));
    writeCrossingSmokeReport(reportPath, summary);
end
end

function arms = buildCrossingSmokeArms(mixtureOverrides)
base = struct( ...
    'linkGateEnabled', false, ...
    'forceInitialHeavy', false, ...
    'forceLabelChangeHeavy', false, ...
    'forceStaleHeavy', false, ...
    'payloadExistenceThreshold', 0.01);

cfg = base;
cfg.eventPolicy = 'alwaysLight';
arms(1) = makeArm('Periodic light posterior', cfg);

cfg = base;
cfg.eventPolicy = 'alwaysHeavy';
arms(2) = makeArm('Periodic heavy legacy fusion', cfg);

cfg = base;
cfg.eventPolicy = 'alwaysHeavy';
cfg.mixtureAwareHeavyFusionEnabled = true;
cfg.mixtureAwareTopComponents = 2;
cfg.mixtureAwareMaxFusedComponents = 6;
cfg.mixtureAwareMaxComponentTuples = 128;
cfg.mixtureAwareMinExistence = 0.90;
cfg.mixtureAwareMinEntropy = 0.2;
cfg.mixtureAwareMinAssociationAmbiguity = 0.0;
cfg.mixtureAwareMinDetectionAssociationMass = 0.0;
cfg.mixtureAwareMaxFusedEntropy = 1.0;
cfg.mixtureAwareMinFusedDominance = 0.55;
cfg.mixtureAwareStateExtractionEnabled = true;
cfg.mixtureAwareExtractionMinSeparation = 4;
cfg.mixtureAwareJointExtractionEnabled = true;
cfg.mixtureAwareJointExtractionTopComponents = 3;
cfg.mixtureAwareJointExtractionMinSeparation = 4;
cfg.mixtureAwareJointExtractionPairwiseScale = 1.0;
cfg.mixtureAwarePredictionConsistencyEnabled = true;
cfg.mixtureAwarePredictionConsistencyStrength = 0.2;
cfg = applyOverrides(cfg, mixtureOverrides);
arms(3) = makeArm('Periodic heavy mixture-aware fusion', cfg);
end

function config = applyOverrides(config, overrides)
if isempty(overrides) || ~isstruct(overrides)
    return;
end
fields = fieldnames(overrides);
for fieldIdx = 1:numel(fields)
    config.(fields{fieldIdx}) = overrides.(fields{fieldIdx});
end
end

function arm = makeArm(name, triggerConfig)
arm = struct('name', name, 'triggerConfig', triggerConfig);
end

function [meanEospa, crossingEospa, meanCardinality, ...
    crossingCardinality, crossingCardinalityError] = scoreSensors( ...
    stateEstimatesBySensor, model, groundTruthRfs)
sensorCount = numel(stateEstimatesBySensor);
simulationLength = numel(groundTruthRfs.x);
eospaBySensor = zeros(sensorCount, simulationLength);
cardinalityBySensor = zeros(sensorCount, simulationLength);
for sensorIdx = 1:sensorCount
    [eospa, ~, cardinality] = computeSimulationOspa( ...
        model, groundTruthRfs, stateEstimatesBySensor{sensorIdx});
    eospaBySensor(sensorIdx, :) = eospa;
    cardinalityBySensor(sensorIdx, :) = cardinality;
end
validWindow = 5:simulationLength;
crossingWindow = 14:26;
validEospa = eospaBySensor(:, validWindow);
crossingEospaValues = eospaBySensor(:, crossingWindow);
validCardinality = cardinalityBySensor(:, validWindow);
crossingCardinalityValues = cardinalityBySensor(:, crossingWindow);
truthCardinality = groundTruthRfs.cardinality(crossingWindow);
truthCardinality = repmat(reshape(truthCardinality, 1, []), ...
    sensorCount, 1);
meanEospa = mean(validEospa(:));
crossingEospa = mean(crossingEospaValues(:));
meanCardinality = mean(validCardinality(:));
crossingCardinality = mean(crossingCardinalityValues(:));
crossingCardinalityError = mean(abs( ...
    crossingCardinalityValues(:) - truthCardinality(:)));
end

function value = meanSurvivingComponentCount(stateEstimatesBySensor)
counts = [];
for sensorIdx = 1:numel(stateEstimatesBySensor)
    objects = stateEstimatesBySensor{sensorIdx}.objects;
    if isempty(objects)
        continue;
    end
    counts = [counts, [objects.numberOfGmComponents]]; %#ok<AGROW>
end
if isempty(counts)
    value = 0;
else
    value = mean(counts);
end
end

function value = percentChange(baseline, candidate)
value = 100 * (candidate - baseline) / max(abs(baseline), eps);
end

function writeCrossingSmokeReport(reportPath, summary)
fid = fopen(reportPath, 'w');
if fid < 0
    error('Could not open report for writing: %s', reportPath);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# Mixture-aware heavy crossing smoke\n\n');
fprintf(fid, '- Seed: %d\n', summary.seed);
fprintf(fid, '- Scenario: two crossing targets, high clutter, periodic delivery.\n\n');
fprintf(fid, '| Arm | Mean E-OSPA | Crossing E-OSPA | Mean card. | Crossing card. err. | Bytes | Light rate | Heavy rate |\n');
fprintf(fid, '|:--|--:|--:|--:|--:|--:|--:|--:|\n');
for armIdx = 1:numel(summary.armNames)
    fprintf(fid, '| %s | %.4f | %.4f | %.2f | %.2f | %.0f | %.3f | %.3f |\n', ...
        summary.armNames{armIdx}, summary.meanEospa(armIdx), ...
        summary.crossingEospa(armIdx), summary.meanCardinality(armIdx), ...
        summary.crossingCardinalityError(armIdx), ...
        summary.payloadBytes(armIdx), summary.lightRate(armIdx), ...
        summary.heavyRate(armIdx));
end
fprintf(fid, '\nMixture-aware heavy vs light crossing E-OSPA change: %.2f%%.\n', ...
    summary.mixtureVsLightCrossingChangePercent);
fprintf(fid, ...
    'Mixture-aware heavy vs legacy-heavy crossing E-OSPA change: %.2f%%.\n', ...
    summary.mixtureVsLegacyHeavyCrossingChangePercent);
end
