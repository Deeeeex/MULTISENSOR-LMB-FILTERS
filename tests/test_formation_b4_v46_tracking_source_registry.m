function test_formation_b4_v46_tracking_source_registry()
% Source registry stays outcome-sealed and deterministic.

registry = getFormationB4V46TrackingSourceRegistry();
assert(registry.sceneCount == 8);
assert(registry.seedCount == 5);
assert(registry.caseCount == 40);
assert(registry.developmentSeed == 1009);
assert(isequal(registry.confirmationSeeds, [1013, 1019, 1021, 1033]));
assert(~ismember(1031, registry.orderedSeeds));
assert(numel(registry.developmentCaseOrdinals) == 8);
assert(numel(registry.confirmationCaseOrdinals) == 32);
assert(numel(registry.primaryCaseOrdinals) == 30);
assert(numel(registry.stressCaseOrdinals) == 10);
assert(registry.sourceDiscoveryAuthorized);
assert(~registry.sourceArtifactPublicationAuthorized);
assert(~registry.sourceInputFingerprintsFrozen);
assert(~registry.rawSourceArtifactReturned);
assert(~registry.confirmationTruthSecrecyGuaranteed);
assert(registry.confirmationTrackingOutcomeUnopenedByDiscovery);
assert(~registry.filterExecutionAuthorized);
assert(~registry.stateEstimateOutputAuthorized);
assert(~registry.trackingOutcomeScoringAuthorized);
assert(registry.sourceGeometryTruthValidationAuthorized);
assert(~registry.estimateVsTruthTrackingMetricAuthorized);
assert(~registry.armSelectionAuthorized);
assert(~registry.developmentAdvanceDecisionAuthorized);
assert(~registry.confirmationTrackingAuthorized);
assert(~registry.validationClaimAllowed);
assert(all(~[registry.scenes.trackingOutcomeAuthorizedBySceneConfig]));
assert(exist('materializeFormationB4V46TrackingSourceCase', ...
    'file') == 0);

rng(81427, 'twister');
initialRngState = rng();
discovery = discoverFormationB4V46TrackingSourceFingerprints(struct( ...
    'presets', {{'m24-formation-fov'}}, 'seeds', 1009));
finalRngState = rng();
assert(isequaln(finalRngState, initialRngState));
assert(discovery.caseCount == 1);
assert(~discovery.fullRegistryRequested);
assert(~discovery.sourceInputFingerprintsFrozen);
assert(~discovery.sourceArtifactPublicationAuthorized);
assert(~discovery.rawSourceObjectsReturned);
assert(~discovery.rawTruthReturned);
assert(~discovery.rawMeasurementsReturned);
assert(~discovery.runtimeFilterModelReturned);
assert(discovery.runtimeTargetTrajectoryRealizationAbsent);
assert(discovery.registeredBirthPriorRetained);
assert(~isfield(discovery, 'inputs'));
assert(~isfield(discovery, 'groundTruth'));
assert(~isfield(discovery, 'measurements'));
assert(~discovery.filterExecuted);
assert(~discovery.stateEstimateGenerated);
assert(discovery.sourceGeometryTruthValidationExecuted);
assert(discovery.truthHashedForSealing);
assert(~discovery.estimateVsTruthTrackingMetricComputed);
assert(~discovery.armComparedOrSelected);
assert(~discovery.trackingOutcomeScored);
assert(~discovery.developmentAdvanceDecisionMade);
assert(~discovery.confirmationOpened);
assert(~discovery.validationClaimAllowed);
record = discovery.records(1);
assert(~isfield(record, 'inputs'));
assert(~isfield(record, 'groundTruth'));
assert(~isfield(record, 'measurements'));
assert(strcmp(record.presetName, 'm24-formation-fov'));
assert(record.seed == 1009);
assert(record.deliverySeed == 46100901);
assert(numel(record.inputFingerprint.canonicalSha256) == 64);
assert(numel(record.runtimeFilterInputFingerprint.canonicalSha256) == 64);
assert(record.runtimeTargetTrajectoryRealizationAbsent);
assert(record.registeredBirthPriorRetained);
assert(~record.runtimeFilterInputFingerprint. ...
    targetTrajectoryRealizationPassedToFilter);
assert(~record.runtimeFilterInputFingerprint.groundTruthPassedToFilter);
assert(record.runtimeFilterInputFingerprint.registeredBirthPriorRetained);
assert(numel(record.sourceEnvelopeCanonicalSha256) == 64);
assert(strcmp(discovery.runtimeEngine, 'GNU Octave') || ...
    strcmp(discovery.runtimeEngine, 'MATLAB'));
assert(~isempty(discovery.runtimeVersion));

caseContract = registry.cases(1);
directInitialRngState = rng();
restoreDirectRng = onCleanup(@() rng(directInitialRngState));
inputs = generateDynamicTopologyScenarioInputs( ...
    caseContract.presetName, caseContract.seed);
identity = buildDynamicTopologyPhysicalIdentityRegistry(inputs.config);
[uniforms, metadata] = ...
    materializePhysicalUidDirectedDeliveryUniforms( ...
        caseContract.deliverySeed, identity.sensorPhysicalUids, ...
        caseContract.simulationLength);
inputs.commConfig.linkUniforms = uniforms;
fingerprint = computeFormationB4V46TrackingInputFingerprint( ...
    inputs, identity, metadata, caseContract, registry);
assert(isequaln(fingerprint, record.inputFingerprint));

extraResult = inputs;
extraResult.stateEstimatesBySensor = cell(1, 1);
assertErrorId(@() computeFormationB4V46TrackingInputFingerprint( ...
    extraResult, identity, metadata, caseContract, registry), ...
    'FormationB4V46TrackingSource:MalformedInput');

badMetadata = metadata;
badMetadata.canonicalTensorSha256 = repmat('0', 1, 64);
assertErrorId(@() computeFormationB4V46TrackingInputFingerprint( ...
    inputs, identity, badMetadata, caseContract, registry), ...
    'FormationB4V46TrackingSource:DeliveryMetadataDrift');

wrongSeedMetadata = metadata;
wrongSeedMetadata.seed = wrongSeedMetadata.seed + 1;
wrongSeedMetadata.canonicalSha256 = computeCanonicalValueSha256( ...
    rmfield(wrongSeedMetadata, 'canonicalSha256'));
assertErrorId(@() computeFormationB4V46TrackingInputFingerprint( ...
    inputs, identity, wrongSeedMetadata, caseContract, registry), ...
    'FormationB4V46TrackingSource:DeliveryMetadataDrift');

badUniforms = inputs;
badUniforms.commConfig.linkUniforms(1, 2, 1) = ...
    1 - badUniforms.commConfig.linkUniforms(1, 2, 1);
assertErrorId(@() computeFormationB4V46TrackingInputFingerprint( ...
    badUniforms, identity, metadata, caseContract, registry), ...
    'FormationB4V46TrackingSource:DeliveryMetadataDrift');

otherCase = registry.cases(2);
assertErrorId(@() computeFormationB4V46TrackingInputFingerprint( ...
    inputs, identity, metadata, otherCase, registry), ...
    'FormationB4V46TrackingSource:OwnershipDrift');

forgedRegistry = registry;
forgedRegistry.id = 'forged-v46-source-registry';
forgedRegistry.canonicalSha256 = computeCanonicalValueSha256( ...
    rmfield(forgedRegistry, 'canonicalSha256'));
assertErrorId(@() computeFormationB4V46TrackingInputFingerprint( ...
    inputs, identity, metadata, caseContract, forgedRegistry), ...
    'FormationB4V46TrackingSource:RegistryAuthorityDrift');
clear restoreDirectRng;
directFinalRngState = rng();
assert(isequaln(directFinalRngState, directInitialRngState));

assertErrorId(@() ...
    discoverFormationB4V46TrackingSourceFingerprints(struct( ...
        'presets', {{'m24-formation-fov'}}, 'seeds', 41)), ...
    'FormationB4V46TrackingSource:InvalidOptions');
fprintf('PASS: V46 tracking source registry tests\n');
end

function assertErrorId(callback, expectedIdentifier)
failed = false;
actualIdentifier = '';
try
    callback();
catch errorInfo
    actualIdentifier = errorInfo.identifier;
    failed = strcmp(actualIdentifier, expectedIdentifier);
end
assert(failed, 'Expected %s, received %s.', ...
    expectedIdentifier, actualIdentifier);
end
