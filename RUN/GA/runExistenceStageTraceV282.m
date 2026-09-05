function [reportPath, result] = runExistenceStageTraceV282(options)
% RUNEXISTENCESTAGETRACEV282 Capture an unchanged V242 reference prefix.
% Generate the original full scene before truncating its time axis. No
% sensing, association, topology, payload, fusion or readout rule is changed.
if nargin < 1, options = struct(); end
referencePath = options.referenceResultPath;
loaded = load(referencePath, 'result'); reference = loaded.result;
maximumTime = getField(options, 'maximumTime', 40);
assert(isscalar(maximumTime) && maximumTime >= 1 && ...
    maximumTime == round(maximumTime));
out = options.outputRoot;
if exist(out, 'dir') ~= 7, mkdir(out); end
tracePath = fullfile(out, 'EXISTENCE_STAGE_TRACE_V282.mat');
if exist(tracePath, 'file') == 2
    saved = load(tracePath, 'trace');
    assert(saved.trace.maximumTime == maximumTime && ...
        strcmp(saved.trace.referenceResultPath, referencePath));
    [reportPath, result] = analyzeExistenceStageTraceV282(tracePath);
    return;
end
inputs = generateDynamicTopologyScenarioInputs( ...
    reference.presetName, reference.seed);
originalTimeCount = inputs.config.simulationLength;
assert(maximumTime <= originalTimeCount);
inputs = cropInputs(inputs, maximumTime);
model = removeRealizedTargetTruthFromDynamicTopologyModel(inputs.model);
config = buildCausalMinimumFormationBackboneV242Config(inputs.config);
config.topologyPosteriorCaptureTimes = 1:maximumTime;
config.topologyFusedPosteriorCaptureTimes = 1:maximumTime;
config.captureLabelKlaDiagnosticsEnabled = true;
config.diagnosticsProgressInterval = 5;
context = buildCausalMinimumFormationBackboneV242ExecutionContext( ...
    reference.presetName, reference.seed, maximumTime);
protocol = getCausalMinimumFormationBackboneV242Protocol();
[~, commit] = system('git rev-parse HEAD');
rng(reference.seed + protocol.filterSeedOffset, 'twister');
fprintf('V282 unchanged V242: %s, seed %d, prefix 1:%d of %d.\n', ...
    reference.presetName, reference.seed, maximumTime, originalTimeCount);
started = tic;
[estimates, diagnostics] = runEventTriggeredDistributedLmbFilter( ...
    model, inputs.measurements, inputs.sensorTrajectories, ...
    inputs.neighborMap, inputs.commConfig, config, context);
trace = struct('contractVersion', 'existence-stage-trace-v282-v1', ...
    'generationGitCommit', strtrim(commit), ...
    'referenceResultPath', referencePath, ...
    'referenceGenerationGitCommit', reference.generationGitCommit, ...
    'presetName', reference.presetName, 'seed', reference.seed, ...
    'maximumTime', maximumTime, 'originalTimeCount', originalTimeCount, ...
    'elapsedSeconds', toc(started), 'completedAt', datestr(now, 31), ...
    'model', qualityModelForAnalysis(model), 'config', inputs.config, ...
    'groundTruthRfs', inputs.groundTruthRfs, 'estimates', {estimates}, ...
    'predicted', {diagnostics.topologyPredictedPosteriorSnapshot}, ...
    'local', {diagnostics.topologyLocalPosteriorSnapshot}, ...
    'fused', {diagnostics.topologyFusedPosteriorSnapshot}, ...
    'kla', {diagnostics.topologyLabelKlaSnapshot}, ...
    'attempted', diagnostics.attempted, ...
    'delivered', diagnostics.delivered, ...
    'attemptedPayloadBytes', diagnostics.attemptedPayloadBytes, ...
    'newPolicyEvaluated', false, 'developmentEvidenceOnly', true);
% Save the expensive reference trace before any offline analysis.
partialPath = [tracePath, '.partial'];
save('-mat7-binary', partialPath, 'trace');
movefile(partialPath, tracePath);
fprintf('V282 trace saved after %.1f seconds: %s\n', ...
    trace.elapsedSeconds, tracePath);
[reportPath, result] = analyzeExistenceStageTraceV282(tracePath);
end

function quality = qualityModelForAnalysis(model)
% The live model contains function handles that Octave cannot save as MAT7.
% Preserve exactly the numerical fields used by evaluateSensorQuality and
% the pruning threshold, not an executable or complete filter checkpoint.
fields = {'detectionProbability', 'Q', 'sensorTrajectories', ...
    'sensorInitialStates', 'sensorFovHeadingRad', 'sensorFovHalfAngleDeg', ...
    'sensorFovRange', 'sensorMotionEnabled', 'sensorFovEnabled', ...
    'sensorQuality', 'existenceThreshold'};
quality = struct();
for k = 1:numel(fields)
    if isfield(model, fields{k}), quality.(fields{k}) = model.(fields{k}); end
end
end

function inputs = cropInputs(inputs, maximumTime)
% Same prefix construction as the V250 reference-cache/oracle runners.
inputs.measurements = inputs.measurements(:, 1:maximumTime);
inputs.config.simulationLength = maximumTime;
inputs.model.simulationLength = maximumTime;
inputs.model.dynamicTopologyScenario.config.simulationLength = maximumTime;
for sensorIdx = 1:numel(inputs.sensorTrajectories)
    inputs.sensorTrajectories{sensorIdx} = ...
        inputs.sensorTrajectories{sensorIdx}(:, 1:maximumTime);
end
for name = {'x', 'mu', 'Sigma'}
    inputs.groundTruthRfs.(name{1}) = ...
        inputs.groundTruthRfs.(name{1})(1:maximumTime);
end
inputs.groundTruthRfs.cardinality = ...
    inputs.groundTruthRfs.cardinality(1:maximumTime);
if ndims(inputs.commConfig.pDropByEdge) >= 3
    inputs.commConfig.pDropByEdge = ...
        inputs.commConfig.pDropByEdge(:, :, 1:maximumTime);
end
if isfield(inputs.commConfig, 'linkUniforms') && ...
        ndims(inputs.commConfig.linkUniforms) >= 3
    inputs.commConfig.linkUniforms = ...
        inputs.commConfig.linkUniforms(:, :, 1:maximumTime);
end
inputs.graphData.physicalAdjacency = ...
    inputs.graphData.physicalAdjacency(:, :, 1:maximumTime);
inputs.graphData.positions = inputs.graphData.positions(:, :, 1:maximumTime);
end

function value = getField(s, name, fallback)
if isfield(s, name), value = s.(name); else, value = fallback; end
end
