function test_multistyle_geometry_validation_protocol()
% TEST_MULTISTYLE_GEOMETRY_VALIDATION_PROTOCOL Frozen-manifest gate tests.

protocol = getFormationFovMultistyleGeometryValidationProtocol();
assert(strcmp(protocol.contractVersion, ...
    'formation-fov-multistyle-geometry-validation-v1'));
assert(strcmp(protocol.sceneGeometryVersion, ...
    'formation-fov-multistyle-v5'));
assert(strcmp(protocol.canonicalSha256, ...
    'ec631ed036d4e4693353583bdc5fe02d8c5825f7a9d9e6ae187fcb1d235f0e7a'));
assert(numel(protocol.validationSeeds) == 20);
assert(numel(unique(protocol.validationSeeds)) == 20);
assert(isempty(intersect( ...
    protocol.developmentSeeds, protocol.validationSeeds)));
assert(~protocol.trackingOutcomeAuthorized);
testExplicitGitRootBinding();

for preset = protocol.presets
    config = buildDynamicTopologyScenarioConfig(preset.presetName);
    assert(strcmp(config.sceneGeometryVersion, ...
        protocol.sceneGeometryVersion));
    assert(strcmp(config.sceneContractSha256, ...
        preset.sceneContractSha256));
    assert(config.formalValidationAuthorized);
    assert(config.enforceDifficultyRequirements);
    assert(~config.trackingOutcomeAuthorized);
    assert(config.numberOfSensors == preset.sensorCount);
    assert(config.formationCount == preset.formationCount);
    assert(config.sensorsPerFormation == preset.sensorsPerFormation);
    assert(config.numberOfTargets == preset.targetCount);
    gate = getFormationFovMultistyleAbsoluteGeometryGate( ...
        preset.styleName);
    protocolGate = protocol.absoluteGates(strcmp( ...
        {protocol.absoluteGates.styleName}, preset.styleName));
    assert(isequal(gate, protocolGate));
    assert(config.minimumTargetSeparation == gate.minTargetSeparation);
    assert(config.minimumSensorTargetSeparation == ...
        gate.minSensorTargetSeparation);
end
for presetName = { ...
        'm24-formation-fov-crossing', ...
        'x36-formation-fov-crossing'}
    config = buildDynamicTopologyScenarioConfig(presetName{1});
    assert(strcmp(config.sceneCalibrationStatus, 'stress-only-v5'));
    assert(~config.formalValidationAuthorized);
    assert(~config.enforceDifficultyRequirements);
    assert(~config.trackingOutcomeAuthorized);
end

passingAudit = buildPassingAudit(protocol);
evaluation = evaluateFormationFovMultistyleGeometryValidation( ...
    passingAudit, protocol);
assert(evaluation.allAbsolutePass);
assert(evaluation.allCrossScalePass);
assert(evaluation.gatePassed);

unsafe = passingAudit;
unsafe.records(1, 1).minimumSensorTargetSeparation = 29;
evaluation = evaluateFormationFovMultistyleGeometryValidation( ...
    unsafe, protocol);
assert(~evaluation.allAbsolutePass);
assert(~evaluation.gatePassed);
assert(any(strcmp(evaluation.absoluteRows(1).failureTokens, ...
    'sensor-target-separation')));

nonfinite = passingAudit;
nonfinite.records(1, 1).minimumSensorTargetSeparation = NaN;
evaluation = evaluateFormationFovMultistyleGeometryValidation( ...
    nonfinite, protocol);
assert(~evaluation.allAbsolutePass);
assert(~evaluation.gatePassed);
assert(any(strcmp(evaluation.absoluteRows(1).failureTokens, ...
    'invalid-metric-domain')));

collectedFailure = passingAudit;
collectedFailure.records(1, 1).isValid = false;
collectedFailure.records(1, 1).hardFailures = { ...
    'synthetic-structural-failure'};
collectedFailure.allValid = false;
evaluation = evaluateFormationFovMultistyleGeometryValidation( ...
    collectedFailure, protocol);
assert(~evaluation.allAbsolutePass);
assert(~evaluation.gatePassed);
assert(any(strcmp(evaluation.absoluteRows(1).failureTokens, ...
    'structural-validation')));

invalidPartition = passingAudit;
invalidPartition.records(1, 1).singleFormationFraction = 0.90;
invalidPartition.records(1, 1).multiFormationFraction = 0.90;
evaluation = evaluateFormationFovMultistyleGeometryValidation( ...
    invalidPartition, protocol);
assert(~evaluation.gatePassed);
assert(any(strcmp(evaluation.absoluteRows(1).failureTokens, ...
    'invalid-metric-domain')));

invalidWorstTarget = passingAudit;
invalidWorstTarget.records(1, 1).maximumPerTargetBlackoutFraction = 0.001;
evaluation = evaluateFormationFovMultistyleGeometryValidation( ...
    invalidWorstTarget, protocol);
assert(~evaluation.gatePassed);
assert(any(strcmp(evaluation.absoluteRows(1).failureTokens, ...
    'invalid-metric-domain')));

scaleMismatch = passingAudit;
scaleMismatch.records(2, 1).focusMeanVisibleTargetsPerSensorTime = 5.20;
evaluation = evaluateFormationFovMultistyleGeometryValidation( ...
    scaleMismatch, protocol);
assert(evaluation.allAbsolutePass);
assert(~evaluation.allCrossScalePass);
assert(~evaluation.gatePassed);
convoyPair = find(strcmp({evaluation.pairedRows.styleName}, ...
    'parallel-convoy') & [evaluation.pairedRows.seed] == ...
    protocol.validationSeeds(1));
assert(any(strcmp( ...
    evaluation.pairedRows(convoyPair).failureTokens, ...
    'focus-load-ratio')));

manifestMismatch = passingAudit;
manifestMismatch.seeds(1) = manifestMismatch.seeds(1) - 1;
assertThrowsIdentifier(@() ...
    evaluateFormationFovMultistyleGeometryValidation( ...
        manifestMismatch, protocol), ...
    'FormationFovGeometryProtocol:ManifestMismatch');
inconsistentValidity = passingAudit;
inconsistentValidity.records(1, 1).isValid = false;
assertThrowsIdentifier(@() ...
    evaluateFormationFovMultistyleGeometryValidation( ...
        inconsistentValidity, protocol), ...
    'FormationFovGeometryProtocol:InvalidAudit');
inconsistentFailures = passingAudit;
inconsistentFailures.records(1, 1).hardFailures = {'forged-failure'};
assertThrowsIdentifier(@() ...
    evaluateFormationFovMultistyleGeometryValidation( ...
        inconsistentFailures, protocol), ...
    'FormationFovGeometryProtocol:InvalidAudit');
swappedIdentity = passingAudit;
temporaryName = swappedIdentity.records(1, 1).presetName;
swappedIdentity.records(1, 1).presetName = ...
    swappedIdentity.records(2, 1).presetName;
swappedIdentity.records(2, 1).presetName = temporaryName;
assertThrowsIdentifier(@() ...
    evaluateFormationFovMultistyleGeometryValidation( ...
        swappedIdentity, protocol), ...
    'FormationFovGeometryProtocol:ManifestMismatch');
forgedCount = passingAudit;
forgedCount.records(3, 1).targetCount = 1;
assertThrowsIdentifier(@() ...
    evaluateFormationFovMultistyleGeometryValidation( ...
        forgedCount, protocol), ...
    'FormationFovGeometryProtocol:ManifestMismatch');
forgedProtocol = protocol;
forgedProtocol.absoluteGates(1).maxBlackoutFraction = 1;
assertThrowsIdentifier(@() ...
    evaluateFormationFovMultistyleGeometryValidation( ...
        passingAudit, forgedProtocol), ...
    'FormationFovGeometryProtocol:ProtocolMismatch');
assertThrowsIdentifier(@() ...
    runFormationFovMultistyleGeometryValidation( ...
        struct('writeReport', false)), ...
    'FormationFovGeometryProtocol:UnsafeOption');
assertThrowsIdentifier(@() ...
    runFormationFovMultistyleGeometryValidation( ...
        struct('requireCleanWorktree', false)), ...
    'FormationFovGeometryProtocol:UnsafeOption');

fprintf('test_multistyle_geometry_validation_protocol passed\n');
end

function testExplicitGitRootBinding()
testDirectory = fileparts(mfilename('fullpath'));
sourceRoot = fileparts(testDirectory);
replaceRelativeTestPaths(sourceRoot);
originalDirectory = pwd;
cleanup = onCleanup(@() cd(originalDirectory)); %#ok<NASGU>
cd(tempdir);
gitState = resolveResearchGitState(sourceRoot);
assert(strcmp(gitState.repositoryRoot, sourceRoot));
assert(~strcmp(gitState.commit, 'unavailable'));
end

function replaceRelativeTestPaths(sourceRoot)
relativeEntries = {'common', fullfile('RUN', 'GA'), 'tests'};
pathEntries = regexp(path, pathsep, 'split');
for entryIdx = 1:numel(relativeEntries)
    entry = relativeEntries{entryIdx};
    if any(strcmp(pathEntries, entry))
        rmpath(entry);
    end
end
addpath(fullfile(sourceRoot, 'common'), '-begin');
addpath(fullfile(sourceRoot, 'RUN', 'GA'), '-begin');
addpath(fullfile(sourceRoot, 'tests'), '-begin');
end

function rawAudit = buildPassingAudit(protocol)
presetNames = {protocol.presets.presetName};
seeds = protocol.validationSeeds;
records = repmat(emptyRecord(), numel(presetNames), numel(seeds));
for presetIdx = 1:numel(presetNames)
    preset = protocol.presets(presetIdx);
    for seedIdx = 1:numel(seeds)
        records(presetIdx, seedIdx) = makePassingRecord( ...
            preset, seeds(seedIdx));
    end
end
rawAudit = struct( ...
    'presetNames', {presetNames}, ...
    'seeds', seeds, ...
    'records', records, ...
    'allValid', true);
end

function record = makePassingRecord(preset, seed)
record = emptyRecord();
record.presetName = preset.presetName;
record.seed = seed;
record.sensorCount = preset.sensorCount;
record.targetCount = preset.targetCount;
record.isValid = true;
record.blackoutFraction = 0.005;
record.focusBlackoutFraction = 0.005;
record.maximumPerTargetBlackoutFraction = 0.02;
record.maximumConsecutiveBlackoutSteps = 3;
record.singleFormationFraction = 0.45;
record.multiFormationFraction = 0.545;
record.focusMeanVisibleTargetsPerSensorTime = 4.60;
record.ownershipEntropy = 0.97;
record.blockageFocusOverlapFraction = 0.65;
record.focusCloseEncounterFraction = 1.0;
record.minimumTargetSeparation = 20;
record.minimumSensorTargetSeparation = 40;
if strcmp(preset.styleName, 'parallel-convoy')
    record.focusHandovers = record.targetCount;
else
    record.focusHandovers = ceil( ...
        0.70 * preset.targetCount * (preset.formationCount - 1));
end
end

function record = emptyRecord()
record = struct( ...
    'presetName', '', ...
    'seed', NaN, ...
    'sensorCount', NaN, ...
    'targetCount', NaN, ...
    'isValid', false, ...
    'hardFailures', {{}}, ...
    'blackoutFraction', NaN, ...
    'focusBlackoutFraction', NaN, ...
    'maximumPerTargetBlackoutFraction', NaN, ...
    'maximumConsecutiveBlackoutSteps', NaN, ...
    'singleFormationFraction', NaN, ...
    'multiFormationFraction', NaN, ...
    'focusMeanVisibleTargetsPerSensorTime', NaN, ...
    'focusHandovers', NaN, ...
    'ownershipEntropy', NaN, ...
    'blockageFocusOverlapFraction', NaN, ...
    'focusCloseEncounterFraction', NaN, ...
    'minimumTargetSeparation', NaN, ...
    'minimumSensorTargetSeparation', NaN);
end

function assertThrowsIdentifier(callback, expectedIdentifier)
failed = false;
try
    callback();
catch errorInfo
    failed = strcmp(errorInfo.identifier, expectedIdentifier);
end
assert(failed, 'Expected error identifier: %s', expectedIdentifier);
end
