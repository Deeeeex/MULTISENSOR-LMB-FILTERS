function test_formation_b4_v46_tracking_source_freeze_candidate()
% A complete discovery can propose hashes but cannot authorize execution.

registry = getFormationB4V46TrackingSourceRegistry();
[discovery, metadata] = buildSyntheticDiscovery(registry);
candidate = buildFormationB4V46TrackingSourceFreezeCandidate( ...
    discovery, metadata);
runtimeCandidate = ...
    buildFormationB4V46DevelopmentRuntimeAcceptanceCandidate( ...
        discovery, metadata);

assert(candidate.caseCount == 40);
assert(candidate.fullRegistryCovered);
assert(candidate.candidateOnly);
assert(candidate.independentAcceptancePending);
assert(candidate.sourceInputFingerprintsProposed);
assert(~candidate.sourceInputFingerprintsFrozen);
assert(candidate.truthBearingSourceCommitmentsPresent);
assert(candidate.offlineProvenanceAuditOnly);
assert(~candidate.eligibleAsPermitDependency);
assert(~candidate.runtimeExecutionAuthorityProduced);
assert(candidate.runtimeOnlyAcceptanceRequiredForPermit);
assert(~candidate.sourceArtifactPublicationAuthorized);
assert(~candidate.filterExecutionAuthorized);
assert(~candidate.stateEstimateOutputAuthorized);
assert(~candidate.trackingOutcomeScoringAuthorized);
assert(~candidate.armSelectionAuthorized);
assert(~candidate.developmentAdvanceDecisionAuthorized);
assert(~candidate.confirmationTrackingAuthorized);
assert(~candidate.validationClaimAllowed);
assert(~candidate.runnerSelfAuthorizationAllowed);
assert(~candidate.confirmationTruthSecrecyGuaranteed);
assert(candidate.confirmationTrackingOutcomeUnopened);
assert(numel(candidate.entries) == registry.caseCount);
assert(isequal({candidate.entries.caseId}, {registry.cases.id}));
assert(~isfield(candidate.entries, 'inputFingerprint'));
assert(~isfield(candidate.entries, 'runtimeFilterInputFingerprint'));
assert(strcmp(computeCanonicalValueSha256( ...
    rmfield(candidate, 'canonicalSha256')), ...
    candidate.canonicalSha256));

developmentOrdinals = registry.developmentCaseOrdinals;
assert(runtimeCandidate.caseCount == 8);
assert(strcmp(runtimeCandidate.phase, 'development-sentinel'));
assert(strcmp(runtimeCandidate.runtimeFilterProjectionContractVersion, ...
    'formation-b4-v46-runtime-filter-input-fingerprint-v3'));
assert(runtimeCandidate.candidateOnly);
assert(runtimeCandidate.independentAcceptancePending);
assert(runtimeCandidate.runtimeInputsProposed);
assert(~runtimeCandidate.runtimeInputsFrozen);
assert(~runtimeCandidate.eligibleAsPermitDependency);
assert(~runtimeCandidate.runtimeExecutionAuthorityProduced);
assert(runtimeCandidate.offlineProvenanceAuditCompleted);
assert(~runtimeCandidate.truthBearingSourceCommitmentsPresent);
assert(runtimeCandidate.fullSourceFingerprintHashesAbsent);
assert(runtimeCandidate.sourceEnvelopeHashesAbsent);
assert(runtimeCandidate.discoveryRecordHashesAbsent);
assert(runtimeCandidate.discoveryHashAbsent);
assert(runtimeCandidate.offlineProvenanceRegistryHashAbsent);
assert(runtimeCandidate.confirmationRuntimeEntriesAbsent);
assert(~runtimeCandidate.filterExecutionAuthorized);
assert(~runtimeCandidate.stateEstimateOutputAuthorized);
assert(~runtimeCandidate.trackingOutcomeScoringAuthorized);
assert(~runtimeCandidate.armSelectionAuthorized);
assert(~runtimeCandidate.developmentAdvanceDecisionAuthorized);
assert(~runtimeCandidate.confirmationTrackingAuthorized);
assert(~runtimeCandidate.validationClaimAllowed);
assert(~runtimeCandidate.runnerSelfAuthorizationAllowed);
assert(isequal({runtimeCandidate.entries.caseId}, ...
    {registry.cases(developmentOrdinals).id}));
assert(isempty(intersect([runtimeCandidate.entries.caseOrdinal], ...
    registry.confirmationCaseOrdinals)));
assert(isequal(sort(fieldnames(runtimeCandidate.entries)), ...
    sort({ ...
        'caseOrdinal'; 'caseId'; 'coreCaseCanonicalSha256'; ...
        'runtimeFilterProjectionCanonicalSha256'})));

truthOnlyChanged = discovery;
truthOnlyChanged.records(1).inputFingerprint.groundTruthSha256 = ...
    taggedSha('changed-ground-truth-only');
truthOnlyChanged.records(1).inputFingerprint. ...
    targetTrajectoriesSha256 = taggedSha('changed-target-truth-only');
truthOnlyChanged.records(1).inputFingerprint.canonicalSha256 = ...
    computeCanonicalValueSha256(rmfield( ...
        truthOnlyChanged.records(1).inputFingerprint, ...
        'canonicalSha256'));
truthOnlyChanged.records(1).runtimeFilterInputFingerprint = ...
    projectFormationB4V46RuntimeFilterInputFingerprint( ...
        truthOnlyChanged.records(1).inputFingerprint);
truthOnlyChanged.records(1).canonicalSha256 = ...
    computeCanonicalValueSha256(rmfield( ...
        truthOnlyChanged.records(1), 'canonicalSha256'));
truthOnlyChanged.canonicalSha256 = computeCanonicalValueSha256( ...
    rmfield(truthOnlyChanged, 'canonicalSha256'));
truthOnlyMetadata = bindMetadata(metadata, truthOnlyChanged);
changedOfflineCandidate = ...
    buildFormationB4V46TrackingSourceFreezeCandidate( ...
        truthOnlyChanged, truthOnlyMetadata);
changedRuntimeCandidate = ...
    buildFormationB4V46DevelopmentRuntimeAcceptanceCandidate( ...
        truthOnlyChanged, truthOnlyMetadata);
assert(~strcmp(changedOfflineCandidate.canonicalSha256, ...
    candidate.canonicalSha256));
assert(isequaln(changedRuntimeCandidate, runtimeCandidate));

opened = discovery;
opened.confirmationOpened = true;
opened.canonicalSha256 = computeCanonicalValueSha256( ...
    rmfield(opened, 'canonicalSha256'));
openedMetadata = bindMetadata(metadata, opened);
assertErrorId(@() ...
    buildFormationB4V46TrackingSourceFreezeCandidate( ...
        opened, openedMetadata), ...
    'FormationB4V46TrackingFreeze:DiscoveryDrift');

truthLeaked = discovery;
truthLeaked.records(1).runtimeFilterInputFingerprint. ...
    groundTruthPassedToFilter = true;
truthLeaked.records(1).runtimeFilterInputFingerprint. ...
    canonicalSha256 = computeCanonicalValueSha256(rmfield( ...
        truthLeaked.records(1).runtimeFilterInputFingerprint, ...
        'canonicalSha256'));
truthLeaked.records(1).canonicalSha256 = ...
    computeCanonicalValueSha256(rmfield( ...
        truthLeaked.records(1), 'canonicalSha256'));
truthLeaked.canonicalSha256 = computeCanonicalValueSha256( ...
    rmfield(truthLeaked, 'canonicalSha256'));
truthMetadata = bindMetadata(metadata, truthLeaked);
assertErrorId(@() ...
    buildFormationB4V46TrackingSourceFreezeCandidate( ...
        truthLeaked, truthMetadata), ...
    'FormationB4V46TrackingFreeze:RecordDrift');

configDrift = discovery;
configDrift.records(1).inputFingerprint.configSha256 = ...
    taggedSha('unregistered-config');
configDrift.records(1).inputFingerprint.canonicalSha256 = ...
    computeCanonicalValueSha256(rmfield( ...
        configDrift.records(1).inputFingerprint, ...
        'canonicalSha256'));
configDrift.records(1).runtimeFilterInputFingerprint = ...
    projectFormationB4V46RuntimeFilterInputFingerprint( ...
        configDrift.records(1).inputFingerprint);
configDrift.records(1).canonicalSha256 = ...
    computeCanonicalValueSha256(rmfield( ...
        configDrift.records(1), 'canonicalSha256'));
configDrift.canonicalSha256 = computeCanonicalValueSha256( ...
    rmfield(configDrift, 'canonicalSha256'));
configMetadata = bindMetadata(metadata, configDrift);
assertErrorId(@() ...
    buildFormationB4V46TrackingSourceFreezeCandidate( ...
        configDrift, configMetadata), ...
    'FormationB4V46TrackingFreeze:RecordDrift');

extra = discovery;
extra.selfAuthorized = true;
extra.canonicalSha256 = computeCanonicalValueSha256( ...
    rmfield(extra, 'canonicalSha256'));
extraMetadata = bindMetadata(metadata, extra);
assertErrorId(@() ...
    buildFormationB4V46TrackingSourceFreezeCandidate( ...
        extra, extraMetadata), ...
    'FormationB4V46TrackingFreeze:DiscoveryDrift');

fprintf('PASS: V46 non-authorizing source-freeze candidate tests\n');
end

function [discovery, metadata] = buildSyntheticDiscovery(registry)
records = repmat(emptyRecord(), 1, registry.caseCount);
for caseIdx = 1:registry.caseCount
    contract = registry.cases(caseIdx);
    source = buildSourceFingerprint(contract, registry);
    runtime = ...
        projectFormationB4V46RuntimeFilterInputFingerprint(source);
    record = emptyRecord();
    record.caseId = contract.id;
    record.caseOrdinal = contract.ordinal;
    record.presetName = contract.presetName;
    record.seed = contract.seed;
    record.seedRole = contract.seedRole;
    record.primaryMatrix = contract.primaryMatrix;
    record.stressMatrix = contract.stressMatrix;
    record.deliverySeed = contract.deliverySeed;
    record.inputFingerprint = source;
    record.runtimeFilterInputFingerprint = runtime;
    record.runtimeTargetTrajectoryRealizationAbsent = true;
    record.registeredBirthPriorRetained = true;
    record.sourceGeometryTruthValidationExecuted = true;
    record.truthHashedForSealing = true;
    record.filterExecuted = false;
    record.estimateVsTruthTrackingMetricComputed = false;
    record.trackingOutcomeScored = false;
    record.sourceEnvelopeCanonicalSha256 = ...
        taggedSha(sprintf('envelope-%d', caseIdx));
    record.canonicalSha256 = computeCanonicalValueSha256( ...
        rmfield(record, 'canonicalSha256'));
    records(caseIdx) = record;
end

payload = struct();
payload.contractVersion = ...
    'formation-b4-v46-tracking-source-fingerprint-discovery-v2';
payload.registryId = registry.id;
payload.registryCanonicalSha256 = registry.canonicalSha256;
payload.presets = {registry.scenes.presetName};
payload.seeds = registry.orderedSeeds;
payload.records = records;
payload.caseCount = registry.caseCount;
payload.runtimeEngine = 'Synthetic test runtime';
payload.runtimeVersion = '1';
payload.fullRegistryRequested = true;
payload.sourceInputFingerprintsFrozen = false;
payload.sourceArtifactPublicationAuthorized = false;
payload.rawSourceObjectsReturned = false;
payload.rawTruthReturned = false;
payload.rawMeasurementsReturned = false;
payload.runtimeFilterModelReturned = false;
payload.runtimeTargetTrajectoryRealizationAbsent = true;
payload.registeredBirthPriorRetained = true;
payload.filterExecuted = false;
payload.stateEstimateGenerated = false;
payload.sourceGeometryTruthValidationExecuted = true;
payload.truthHashedForSealing = true;
payload.estimateVsTruthTrackingMetricComputed = false;
payload.armComparedOrSelected = false;
payload.trackingOutcomeScored = false;
payload.developmentAdvanceDecisionMade = false;
payload.confirmationOpened = false;
payload.validationClaimAllowed = false;
payload.developmentEvidenceOnly = true;
discovery = payload;
discovery.canonicalSha256 = computeCanonicalValueSha256(payload);

metadataPayload = struct();
metadataPayload.contractVersion = ...
    'formation-b4-v46-tracking-source-discovery-run-metadata-v1';
metadataPayload.generationGitCommit = repmat('a', 1, 40);
metadataPayload.generationGitClean = true;
metadataPayload.endingGitCommit = repmat('a', 1, 40);
metadataPayload.endingGitClean = true;
metadataPayload.registryId = registry.id;
metadataPayload.registryCanonicalSha256 = registry.canonicalSha256;
metadataPayload.discoveryContractVersion = discovery.contractVersion;
metadataPayload.discoveryCanonicalSha256 = discovery.canonicalSha256;
metadataPayload.caseCount = registry.caseCount;
metadataPayload.fullRegistryRequested = true;
metadataPayload.sourceInputFingerprintsFrozen = false;
metadataPayload.sourceArtifactPublicationAuthorized = false;
metadataPayload.filterExecuted = false;
metadataPayload.stateEstimateGenerated = false;
metadataPayload.trackingOutcomeScored = false;
metadataPayload.developmentAdvanceDecisionMade = false;
metadataPayload.confirmationOpened = false;
metadataPayload.validationClaimAllowed = false;
metadataPayload.startedAt = '2026-08-03 00:00:00';
metadataPayload.completedAt = '2026-08-03 00:00:01';
metadataPayload.elapsedSeconds = 1;
metadataPayload.evidenceBoundary = 'synthetic non-authorizing test';
metadata = metadataPayload;
metadata.canonicalSha256 = ...
    computeCanonicalValueSha256(metadataPayload);
end

function source = buildSourceFingerprint(contract, registry)
payload = struct();
payload.contractVersion = ...
    'formation-b4-v46-tracking-source-input-fingerprint-v2';
payload.seed = contract.seed;
payload.registryId = registry.id;
payload.registryCanonicalSha256 = registry.canonicalSha256;
payload.caseId = contract.id;
payload.caseOrdinal = contract.ordinal;
payload.caseCanonicalSha256 = contract.caseCanonicalSha256;
payload.seedRole = contract.seedRole;
payload.primaryMatrix = contract.primaryMatrix;
payload.stressMatrix = contract.stressMatrix;
payload.configSha256 = contract.configSnapshotSha256;
payload.sourceModelSha256 = taggedSha([contract.id, '-source-model']);
payload.runtimeFilterModelSha256 = ...
    taggedSha([contract.id, '-runtime-model']);
payload.runtimeFilterModelContractVersion = ...
    'formation-b4-v46-runtime-filter-model-envelope-v1';
payload.runtimeFilterBirthPriorSha256 = ...
    taggedSha([contract.id, '-birth-prior']);
payload.measurementsSha256 = taggedSha([contract.id, '-measurements']);
payload.groundTruthSha256 = taggedSha([contract.id, '-ground-truth']);
payload.groundTruthRfsSha256 = ...
    taggedSha([contract.id, '-ground-truth-rfs']);
payload.sensorTrajectoriesSha256 = ...
    taggedSha([contract.id, '-sensor-trajectories']);
payload.targetTrajectoriesSha256 = ...
    taggedSha([contract.id, '-target-trajectories']);
payload.neighborMapSha256 = taggedSha([contract.id, '-neighbor-map']);
payload.communicationConfigSha256 = ...
    taggedSha([contract.id, '-communication']);
payload.graphDataSha256 = taggedSha([contract.id, '-graph-data']);
payload.linkMetadataSha256 = taggedSha([contract.id, '-link-metadata']);
payload.validationSha256 = taggedSha([contract.id, '-validation']);
payload.physicalIdentityRegistrySha256 = ...
    taggedSha([contract.id, '-physical-identity']);
payload.deliveryUniformMetadataSha256 = ...
    taggedSha([contract.id, '-delivery-metadata']);
payload.deliveryUniformCanonicalTensorSha256 = ...
    taggedSha([contract.id, '-delivery-canonical']);
payload.deliveryUniformMaterializedTensorSha256 = ...
    taggedSha([contract.id, '-delivery-materialized']);
source = payload;
source.canonicalSha256 = computeCanonicalValueSha256(payload);
end

function record = emptyRecord()
record = struct( ...
    'caseId', '', 'caseOrdinal', NaN, 'presetName', '', ...
    'seed', NaN, 'seedRole', '', 'primaryMatrix', false, ...
    'stressMatrix', false, 'deliverySeed', NaN, ...
    'inputFingerprint', struct(), ...
    'runtimeFilterInputFingerprint', struct(), ...
    'runtimeTargetTrajectoryRealizationAbsent', false, ...
    'registeredBirthPriorRetained', false, ...
    'sourceGeometryTruthValidationExecuted', false, ...
    'truthHashedForSealing', false, 'filterExecuted', false, ...
    'estimateVsTruthTrackingMetricComputed', false, ...
    'trackingOutcomeScored', false, ...
    'sourceEnvelopeCanonicalSha256', '', 'canonicalSha256', '');
end

function metadata = bindMetadata(metadata, discovery)
metadata.discoveryContractVersion = discovery.contractVersion;
metadata.discoveryCanonicalSha256 = discovery.canonicalSha256;
metadata.canonicalSha256 = computeCanonicalValueSha256( ...
    rmfield(metadata, 'canonicalSha256'));
end

function value = taggedSha(tag)
value = computeCanonicalValueSha256(struct('tag', tag));
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
