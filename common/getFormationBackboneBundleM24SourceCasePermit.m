function permit = ...
    getFormationBackboneBundleM24SourceCasePermit(presetName, seed)
% GETFORMATIONBACKBONEBUNDLEM24SOURCECASEPERMIT Registered source case.

protocol = getFormationBackboneBundleM24DevelopmentProtocol();
if ~ischar(presetName) || ~isscalar(seed) || ...
        ~isfinite(seed) || seed ~= round(seed)
    error('FormationBundleSourcePermit:UnknownCase', ...
        'The requested M24 source case is not registered.');
end
matching = strcmp({protocol.cases.presetName}, presetName) & ...
    [protocol.cases.seed] == seed;
if nnz(matching) ~= 1
    error('FormationBundleSourcePermit:UnknownCase', ...
        'The requested M24 source case is not registered.');
end
caseContract = protocol.cases(find(matching, 1));
arm = protocol.sourceArm;
payload = struct();
payload.id = sprintf('%s:%s:%s', ...
    protocol.id, caseContract.id, arm.id);
payload.contractVersion = ...
    'formation-backbone-bundle-m24-source-case-permit-v1';
payload.capability = protocol.sourceCapability;
payload.filterAction = protocol.sourceFilterAction;
payload.protocolId = protocol.id;
payload.protocolContractVersion = protocol.contractVersion;
payload.protocolCanonicalSha256 = protocol.canonicalSha256;
payload.methodPolicyConfigSha256 = ...
    protocol.methodPolicyConfigSha256;
payload.methodSourceCommit = protocol.methodSourceCommit;
payload.executableSourceFrozen = protocol.executableSourceFrozen;
payload.executableSourceCommit = protocol.executableSourceCommit;
payload.executableSourceManifestSha256 = ...
    protocol.executableSourceManifestSha256;
payload.sourceInputFingerprintsFrozen = ...
    protocol.sourceInputFingerprintsFrozen;
payload.sourcePosteriorAuthorized = ...
    protocol.sourcePosteriorAuthorized;
payload.caseId = caseContract.id;
payload.caseOrdinal = caseContract.ordinal;
payload.caseCanonicalSha256 = ...
    caseContract.caseCanonicalSha256;
payload.presetName = caseContract.presetName;
payload.seed = caseContract.seed;
payload.sourceWindow = caseContract.sourceWindow;
payload.focusWindow = caseContract.focusWindow;
payload.continuationSnapshotTime = ...
    caseContract.continuationSnapshotTime;
payload.configSnapshotSha256 = ...
    caseContract.configSnapshotSha256;
payload.sceneContractKind = caseContract.sceneContractKind;
payload.sceneContractSha256 = ...
    caseContract.sceneContractSha256;
payload.armId = arm.id;
payload.armOrdinal = arm.ordinal;
payload.topologyPolicyFunction = arm.topologyPolicyFunction;
payload.sourceTriggerFingerprintSha256 = ...
    caseContract.sourceTriggerFingerprintSha256;
payload.sourceInputFingerprint = ...
    caseContract.sourceInputFingerprint;
payload.explicitTargetTruthMustBeAbsent = true;
payload.stateEstimateOutputAuthorized = ...
    protocol.sourceStateEstimateOutputAuthorized;
payload.trackingOutcomeScoringAuthorized = false;
payload.groundTruthAccessAuthorized = false;
payload.futureOutcomeAccessAuthorized = false;
payload.candidateContinuationAuthorized = false;
payload.pairedTrackingAuthorized = false;
payload.validationClaimAllowed = false;
payload.developmentEvidenceOnly = true;
permit = payload;
permit.permitCanonicalSha256 = ...
    computeCanonicalValueSha256(payload);
end
