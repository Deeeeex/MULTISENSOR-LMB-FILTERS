function source = ...
    prepareFormationBackboneBundleM24SourceCaseInputs( ...
        presetName, seed)
% PREPAREFORMATIONBACKBONEBUNDLEM24SOURCECASEINPUTS Deterministic case input.
%
% The public generator is called with no override.  Target truth is then
% removed and every time-indexed filter input is truncated at the registered
% source-window end.  The scenario config itself remains untouched so its
% full registered snapshot hash is preserved.

permit = getFormationBackboneBundleM24SourceCasePermit( ...
    presetName, seed);
inputs = generateDynamicTopologyScenarioInputs(presetName, seed);
if ~inputs.validation.isValid || ...
        ~strcmp(computeCanonicalValueSha256(inputs.config), ...
            permit.configSnapshotSha256)
    error('FormationBundleSourcePermit:GeneratedInputDrift', ...
        'The deterministic source case no longer matches its scene contract.');
end
maximumTime = permit.sourceWindow(2);
measurements = inputs.measurements(:, 1:maximumTime);
sensorTrajectories = cropTrajectoryCells( ...
    inputs.sensorTrajectories, maximumTime, 24, ...
    'passed sensor trajectories');
commConfig = cropCommunicationConfig( ...
    inputs.commConfig, maximumTime);
model = stripDynamicTopologyTargetTruthForSource(inputs.model);
model = cropSourceModel(model, maximumTime, sensorTrajectories);
neighborMap = inputs.neighborMap;

fingerprint = ...
    computeFormationBackboneBundleM24SourceInputFingerprint( ...
        model, measurements, sensorTrajectories, neighborMap, ...
        commConfig);
source = struct();
source.contractVersion = ...
    'formation-backbone-bundle-m24-prepared-source-input-v1';
source.caseId = permit.caseId;
source.caseOrdinal = permit.caseOrdinal;
source.presetName = permit.presetName;
source.seed = permit.seed;
source.sourceWindow = permit.sourceWindow;
source.focusWindow = permit.focusWindow;
source.model = model;
source.measurements = measurements;
source.sensorTrajectories = sensorTrajectories;
source.neighborMap = neighborMap;
source.commConfig = commConfig;
source.inputFingerprint = fingerprint;
end

function trajectories = cropTrajectoryCells( ...
        trajectories, maximumTime, expectedCount, label)
if ~iscell(trajectories) || numel(trajectories) ~= expectedCount
    error('FormationBundleSourcePermit:GeneratedInputDrift', ...
        'The %s have an invalid sensor count.', label);
end
trajectories = reshape(trajectories, 1, []);
for sensorIdx = 1:numel(trajectories)
    trajectory = trajectories{sensorIdx};
    if ~isa(trajectory, 'double') || ~isreal(trajectory) || ...
            ndims(trajectory) ~= 2 || ...
            size(trajectory, 1) ~= 4 || ...
            size(trajectory, 2) < maximumTime
        error('FormationBundleSourcePermit:GeneratedInputDrift', ...
            'A %s entry is malformed.', label);
    end
    prefix = trajectory(:, 1:maximumTime);
    if any(~isfinite(prefix(:)))
        error('FormationBundleSourcePermit:GeneratedInputDrift', ...
            'A %s entry is nonfinite.', label);
    end
    trajectories{sensorIdx} = trajectory(:, 1:maximumTime);
end
end

function config = cropCommunicationConfig(config, maximumTime)
if ~isstruct(config) || ~isscalar(config)
    error('FormationBundleSourcePermit:GeneratedInputDrift', ...
        'The generated communication config is malformed.');
end
timeFields = {'pDropByEdge', 'linkUniforms'};
for fieldIdx = 1:numel(timeFields)
    fieldName = timeFields{fieldIdx};
    if ~isfield(config, fieldName) || ...
            ndims(config.(fieldName)) ~= 3 || ...
            size(config.(fieldName), 3) < maximumTime
        error('FormationBundleSourcePermit:GeneratedInputDrift', ...
            'The generated %s tensor is malformed.', fieldName);
    end
    value = config.(fieldName);
    config.(fieldName) = value(:, :, 1:maximumTime);
end
end

function model = cropSourceModel( ...
        model, maximumTime, sensorTrajectories)
model.simulationLength = maximumTime;
model.explicitSensorTrajectories = sensorTrajectories;
model.sensorTrajectories = sensorTrajectories;
if isfield(model, 'sensorFovHeadingRad') && ...
        isnumeric(model.sensorFovHeadingRad) && ...
        size(model.sensorFovHeadingRad, 2) >= maximumTime
    model.sensorFovHeadingRad = ...
        model.sensorFovHeadingRad(:, 1:maximumTime);
end
if isfield(model, 'birthParameters') && ...
        isstruct(model.birthParameters)
    for birthIdx = 1:numel(model.birthParameters)
        if isfield(model.birthParameters(birthIdx), 'trajectory') && ...
                size(model.birthParameters(birthIdx).trajectory, 2) >= ...
                    maximumTime
            model.birthParameters(birthIdx).trajectory = ...
                model.birthParameters(birthIdx).trajectory( ...
                    :, 1:maximumTime);
        end
        if isfield(model.birthParameters(birthIdx), 'timestamps') && ...
                numel(model.birthParameters(birthIdx).timestamps) >= ...
                    maximumTime
            model.birthParameters(birthIdx).timestamps = ...
                model.birthParameters(birthIdx).timestamps( ...
                    1:maximumTime);
        end
    end
end
scenario = model.dynamicTopologyScenario;
tensorFields = {'physicalAdjacency', 'candidateAdjacency'};
for fieldIdx = 1:numel(tensorFields)
    fieldName = tensorFields{fieldIdx};
    if isfield(scenario, fieldName) && ...
            ndims(scenario.(fieldName)) == 3 && ...
            size(scenario.(fieldName), 3) >= maximumTime
        value = scenario.(fieldName);
        scenario.(fieldName) = value(:, :, 1:maximumTime);
    end
end
if isfield(scenario, 'sensor') && ...
        isstruct(scenario.sensor) && isscalar(scenario.sensor) && ...
        isfield(scenario.sensor, 'centerStates') && ...
        iscell(scenario.sensor.centerStates)
    for formationIdx = 1:numel(scenario.sensor.centerStates)
        states = scenario.sensor.centerStates{formationIdx};
        if size(states, 2) < maximumTime
            error('FormationBundleSourcePermit:GeneratedInputDrift', ...
                'A generated formation-centre trace is too short.');
        end
        scenario.sensor.centerStates{formationIdx} = ...
            states(:, 1:maximumTime);
    end
end
model.dynamicTopologyScenario = scenario;
end
