function candidate = ...
    buildFormationB4V46DevelopmentRuntimeAcceptanceCandidate( ...
        discovery, runMetadata)
% BUILDFORMATIONB4V46DEVELOPMENTRUNTIMEACCEPTANCECANDIDATE Eight inputs only.
%
% The full source discovery is audited offline, then discarded.  The output
% contains only the eight development core-case identities and their
% truth-free runtime-v3 projection hashes.  It is still a candidate: a later
% independent commit must hard-code accepted entries before a case-and-arm
% permit can exist.

offline = buildFormationB4V46TrackingSourceFreezeCandidate( ...
    discovery, runMetadata);
if ~offline.candidateOnly || ...
        ~offline.offlineProvenanceAuditOnly || ...
        offline.eligibleAsPermitDependency || ...
        offline.runtimeExecutionAuthorityProduced || ...
        ~offline.runtimeOnlyAcceptanceRequiredForPermit
    error('FormationB4V46RuntimeAcceptance:OfflineAuditDrift', ...
        'The source-provenance audit expanded beyond offline scope.');
end
clear offline;

registry = getFormationB4V46TrackingSourceRegistry();
ordinals = reshape(registry.developmentCaseOrdinals, 1, []);
if numel(ordinals) ~= 8 || ...
        any([registry.cases(ordinals).seed] ~= ...
            registry.developmentSeed) || ...
        any(~strcmp({registry.cases(ordinals).seedRole}, ...
            'development-sentinel'))
    error('FormationB4V46RuntimeAcceptance:DevelopmentScopeDrift', ...
        'The core registry no longer defines exactly eight development cases.');
end

entries = repmat(emptyEntry(), 1, numel(ordinals));
for entryIdx = 1:numel(ordinals)
    ordinal = ordinals(entryIdx);
    contract = registry.cases(ordinal);
    runtime = discovery.records(ordinal).runtimeFilterInputFingerprint;
    entry = emptyEntry();
    entry.caseOrdinal = contract.ordinal;
    entry.caseId = contract.id;
    entry.coreCaseCanonicalSha256 = contract.caseCanonicalSha256;
    entry.runtimeFilterProjectionCanonicalSha256 = ...
        runtime.canonicalSha256;
    entries(entryIdx) = entry;
end

payload = struct();
payload.id = ...
    'formation-b4-v46-development-runtime-acceptance-candidate-v1';
payload.contractVersion = ...
    'formation-b4-v46-development-runtime-acceptance-candidate-v1';
payload.coreRegistryId = registry.id;
payload.coreRegistryCanonicalSha256 = registry.canonicalSha256;
payload.phase = 'development-sentinel';
payload.developmentSeed = registry.developmentSeed;
payload.runtimeFilterProjectionContractVersion = ...
    discovery.records(ordinals(1)).runtimeFilterInputFingerprint. ...
        contractVersion;
payload.canonicalSerializationFunction = ...
    'computeCanonicalValueSha256';
payload.caseOrdering = ...
    'core-registry-development-case-ordinal-order';
payload.caseCount = numel(entries);
payload.entries = entries;
payload.candidateOnly = true;
payload.independentAcceptancePending = true;
payload.runtimeInputsProposed = true;
payload.runtimeInputsFrozen = false;
payload.eligibleAsPermitDependency = false;
payload.runtimeExecutionAuthorityProduced = false;
payload.offlineProvenanceAuditCompleted = true;
payload.truthBearingSourceCommitmentsPresent = false;
payload.fullSourceFingerprintHashesAbsent = true;
payload.sourceEnvelopeHashesAbsent = true;
payload.discoveryRecordHashesAbsent = true;
payload.discoveryHashAbsent = true;
payload.offlineProvenanceRegistryHashAbsent = true;
payload.confirmationRuntimeEntriesAbsent = true;
payload.filterExecutionAuthorized = false;
payload.stateEstimateOutputAuthorized = false;
payload.trackingOutcomeScoringAuthorized = false;
payload.armSelectionAuthorized = false;
payload.developmentAdvanceDecisionAuthorized = false;
payload.confirmationTrackingAuthorized = false;
payload.validationClaimAllowed = false;
payload.runnerSelfAuthorizationAllowed = false;
payload.evidenceBoundary = [ ...
    'This proposal contains only eight development core-case identities ', ...
    'and runtime-v3 projection hashes. Full-source, source-envelope, ', ...
    'discovery-record, discovery, offline-registry, and confirmation ', ...
    'runtime commitments are absent. It does not freeze inputs or ', ...
    'authorize filter execution, outputs, scoring, selection, advance, ', ...
    'confirmation, or validation.'];
candidate = payload;
candidate.canonicalSha256 = ...
    computeCanonicalValueSha256(payload);
end

function entry = emptyEntry()
entry = struct( ...
    'caseOrdinal', NaN, 'caseId', '', ...
    'coreCaseCanonicalSha256', '', ...
    'runtimeFilterProjectionCanonicalSha256', '');
end
