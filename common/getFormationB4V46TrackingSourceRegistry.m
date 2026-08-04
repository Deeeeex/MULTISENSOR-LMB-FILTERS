function registry = getFormationB4V46TrackingSourceRegistry()
% GETFORMATIONB4V46TRACKINGSOURCEREGISTRY Outcome-sealed case registry.
%
% This registry fixes the V46 tracking inputs before any tracking outcome
% is opened.  It authorizes deterministic source discovery and the scenario
% generator's geometry/truth validation only: no filter arm, state estimate,
% estimate-versus-truth tracking metric, or advance decision is allowed.

structural = getFormationCausalMinimalEditV46Protocol();
payload = struct();
payload.id = 'formation-b4-v46-tracking-source-registry-v1';
payload.contractVersion = ...
    'formation-b4-v46-tracking-source-registry-core-v1';
payload.structuralProtocolId = structural.id;
payload.structuralProtocolCanonicalSha256 = structural.canonicalSha256;
payload.orderedArmIds = structural.primaryArms;
payload.developmentSeed = 1009;
payload.confirmationSeeds = [1013, 1019, 1021, 1033];
payload.orderedSeeds = ...
    [payload.developmentSeed, payload.confirmationSeeds];
payload.caseOrdering = 'scene-major-then-seed-registry-order';
payload.deliverySeedContract = ...
    '46000000-plus-100-times-source-seed-plus-scene-ordinal';
payload.physicalUidDeliveryUniformsRequired = true;
payload.sourceDiscoveryAuthorized = true;
payload.sourceArtifactPublicationAuthorized = false;
payload.sourceInputFingerprintsFrozen = false;
payload.rawSourceArtifactReturned = false;
payload.confirmationTruthSecrecyGuaranteed = false;
payload.confirmationTrackingOutcomeUnopenedByDiscovery = true;
payload.filterExecutionAuthorized = false;
payload.stateEstimateOutputAuthorized = false;
payload.trackingOutcomeScoringAuthorized = false;
payload.sourceGeometryTruthValidationAuthorized = true;
payload.estimateVsTruthTrackingMetricAuthorized = false;
payload.armSelectionAuthorized = false;
payload.developmentAdvanceDecisionAuthorized = false;
payload.confirmationTrackingAuthorized = false;
payload.validationClaimAllowed = false;
payload.developmentEvidenceOnly = true;
payload.scenes = buildScenes();
payload.cases = buildCases(payload.scenes, payload.orderedSeeds);
payload.sceneCount = numel(payload.scenes);
payload.seedCount = numel(payload.orderedSeeds);
payload.caseCount = numel(payload.cases);
payload.developmentCaseOrdinals = ...
    find([payload.cases.seed] == payload.developmentSeed);
payload.confirmationCaseOrdinals = ...
    find(ismember([payload.cases.seed], payload.confirmationSeeds));
payload.primaryCaseOrdinals = find([payload.cases.primaryMatrix]);
payload.stressCaseOrdinals = find([payload.cases.stressMatrix]);
payload.evidenceBoundary = [ ...
    'This registry fixes deterministic V46 source cases and physical-', ...
    'UID-keyed delivery draws. It may generate and cryptographically ', ...
    'hash truth-bearing input objects and run the generator built-in ', ...
    'source-geometry validation. It cannot expose raw source objects, ', ...
    'execute a filter, compute an estimate-versus-truth tracking metric, ', ...
    'select an arm, open a tracking outcome, advance confirmation, or ', ...
    'support a validation claim. This is an artifact/API boundary, not a ', ...
    'secrecy guarantee: the public deterministic scenario generator can ', ...
    'reconstruct registered truth when seeds are known.'];

actualSha256 = computeCanonicalValueSha256(payload);
expectedSha256 = ...
    '2d86496732a97b8b597f046d384990363b5be66576d2b4031548eb72e2b75cec';
if ~strcmp(actualSha256, expectedSha256)
    error('FormationB4V46TrackingSource:RegistryDrift', ...
        ['The V46 tracking source registry is not frozen: ', ...
         'actual=%s expected=%s.'], actualSha256, expectedSha256);
end
registry = payload;
registry.canonicalSha256 = actualSha256;
end

function scenes = buildScenes()
presetNames = { ...
    'm24-formation-fov', ...
    'm24-formation-fov-convoy', ...
    'm24-formation-fov-relay', ...
    'm24-formation-fov-crossing', ...
    'x36-formation-fov', ...
    'x36-formation-fov-convoy', ...
    'x36-formation-fov-relay', ...
    'x36-formation-fov-crossing'};
sceneIds = { ...
    'm24-radial', 'm24-convoy', 'm24-relay', 'm24-crossing', ...
    'x36-radial', 'x36-convoy', 'x36-relay', 'x36-crossing'};
styles = { ...
    'radial', 'convoy', 'relay', 'crossing', ...
    'radial', 'convoy', 'relay', 'crossing'};
expectedNodeCounts = [24, 24, 24, 24, 36, 36, 36, 36];
expectedFormationCounts = [4, 4, 4, 4, 6, 6, 6, 6];
expectedConfigSha256 = { ...
    '7dd98b5753c8f06d2a5499a1504d6a9231843cef74204e5ba5c61f9e077ba9e5', ...
    '5db817feb7295010b96aa771a93072eca0f049c924359e64ade1abaa454b5406', ...
    'a86eebd7ba806bff316824a471c85597164bc71cdbe0c22676e01c38c7685c11', ...
    '5378ed0fea4f1132b48ad7f390f8bcd3df33273142942810d9132441ea6bae84', ...
    'e53a109ff9fbed76b166b75fe03022108b239bf04427c245fd4551179863b867', ...
    'd3710894bd387a4fa8aceff7f1cd3775e290d31ac9d9b34e9002afa7b942dd37', ...
    '6f29d7af0f62c70f89430d2e67c2c52870fa6644fe2f0c680ee6d9aa9da4a178', ...
    '9b2115231cc0b5aae387857a745f42baafe78a920c255c8088e9c1149b8b533c'};
scenes = repmat(struct(), 1, numel(presetNames));
for sceneIdx = 1:numel(presetNames)
    config = buildDynamicTopologyScenarioConfig(presetNames{sceneIdx});
    configSha256 = computeCanonicalValueSha256(config);
    if ~strcmp(configSha256, expectedConfigSha256{sceneIdx}) || ...
            config.numberOfSensors ~= expectedNodeCounts(sceneIdx) || ...
            config.formationCount ~= expectedFormationCounts(sceneIdx) || ...
            config.simulationLength ~= 160 || ...
            config.fovHalfAngleDeg ~= 60 || config.fovRange ~= 300
        error('FormationB4V46TrackingSource:SceneDrift', ...
            'The registered scene changed: %s.', presetNames{sceneIdx});
    end
    scene = struct();
    scene.ordinal = sceneIdx;
    scene.id = sceneIds{sceneIdx};
    scene.presetName = presetNames{sceneIdx};
    scene.scale = expectedNodeCounts(sceneIdx);
    scene.style = styles{sceneIdx};
    scene.numberOfSensors = config.numberOfSensors;
    scene.formationCount = config.formationCount;
    scene.simulationLength = config.simulationLength;
    scene.focusWindow = reshape(config.focusWindow, 1, []);
    scene.configSnapshotSha256 = configSha256;
    scene.primaryMatrix = ~strcmp(styles{sceneIdx}, 'crossing');
    scene.stressMatrix = strcmp(styles{sceneIdx}, 'crossing');
    scene.formalValidationScene = scene.primaryMatrix;
    scene.trackingOutcomeAuthorizedBySceneConfig = ...
        isfield(config, 'trackingOutcomeAuthorized') && ...
        isequal(config.trackingOutcomeAuthorized, true);
    scene.sceneContractSha256 = '';
    if isfield(config, 'sceneContractSha256')
        scene.sceneContractSha256 = config.sceneContractSha256;
    end
    scenes(sceneIdx) = scene;
end
end

function cases = buildCases(scenes, seeds)
cases = repmat(struct(), 1, numel(scenes) * numel(seeds));
cursor = 0;
for sceneIdx = 1:numel(scenes)
    for seedIdx = 1:numel(seeds)
        cursor = cursor + 1;
        seed = seeds(seedIdx);
        payload = struct();
        payload.ordinal = cursor;
        payload.id = sprintf('%s__seed%d__v46-source', ...
            scenes(sceneIdx).presetName, seed);
        payload.sceneOrdinal = scenes(sceneIdx).ordinal;
        payload.sceneId = scenes(sceneIdx).id;
        payload.presetName = scenes(sceneIdx).presetName;
        payload.seed = seed;
        payload.seedRole = 'confirmation';
        if seedIdx == 1
            payload.seedRole = 'development-sentinel';
        end
        payload.deliverySeed = 46000000 + 100 * seed + sceneIdx;
        payload.numberOfSensors = scenes(sceneIdx).numberOfSensors;
        payload.formationCount = scenes(sceneIdx).formationCount;
        payload.simulationLength = scenes(sceneIdx).simulationLength;
        payload.focusWindow = scenes(sceneIdx).focusWindow;
        payload.configSnapshotSha256 = ...
            scenes(sceneIdx).configSnapshotSha256;
        payload.primaryMatrix = scenes(sceneIdx).primaryMatrix;
        payload.stressMatrix = scenes(sceneIdx).stressMatrix;
        payload.inputFingerprintFrozen = false;
        payload.trackingOutcomeAuthorized = false;
        payload.caseCanonicalSha256 = ...
            computeCanonicalValueSha256(payload);
        cases(cursor) = payload;
    end
end
end
