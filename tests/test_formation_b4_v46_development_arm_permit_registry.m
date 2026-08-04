function test_formation_b4_v46_development_arm_permit_registry()
% Data-only registry is either exactly sealed or the complete 8-by-2 matrix.

registry = getFormationB4V46DevelopmentArmPermitRegistry();
accepted = getFormationB4V46DevelopmentRuntimeAcceptanceRegistry();
structural = getFormationCausalMinimalEditV46Protocol();
assert(strcmp(registry.runtimeAcceptanceRegistryCanonicalSha256, ...
    accepted.canonicalSha256));
assert(strcmp(registry.structuralProtocolCanonicalSha256, ...
    structural.canonicalSha256));
assert(isequal(registry.orderedArmIds, structural.primaryArms));
assert(~registry.trackingOutcomeScoringAuthorized);
assert(~registry.groundTruthAccessAuthorized);
assert(~registry.futureOutcomeAccessAuthorized);
assert(~registry.armSelectionAuthorized);
assert(~registry.developmentAdvanceDecisionAuthorized);
assert(~registry.confirmationTrackingAuthorized);
assert(~registry.validationClaimAllowed);
assert(~registry.runnerSelfAuthorizationAllowed);
assert(registry.developmentEvidenceOnly);
assert(registry.activationDataRegularFileVerified);
assert(registry.activationDataSymlinkRejected);
assert(registry.activationDataWorktreeMatchesIndexBlob);
assert(~isempty(regexp(registry.activationDataGitBlobId, ...
    '^[0-9a-f]+$', 'once')));
assert(strcmp(computeCanonicalValueSha256( ...
    rmfield(registry, 'canonicalSha256')), registry.canonicalSha256));

if ~registry.executionSurfaceFrozen
    assert(~registry.filterExecutionAuthorized);
    assert(~registry.stateEstimateOutputAuthorized);
    assert(registry.permitCount == 0);
    assert(isempty(registry.permits));
    assert(isempty(registry.executionSurfaceCommit));
    assert(isempty(registry.executionSurfaceManifestSha256));
    assert(~registry.completeCaseArmCartesianProduct);
    assertErrorId(@() loadFormationB4V46DevelopmentArmPermit( ...
        accepted.entries(1).caseOrdinal, structural.primaryArms{1}), ...
        'FormationB4V46DevelopmentPermit:ExecutionSealed');
    assertErrorId(@() buildFormationB4V46DevelopmentArmExecutionContext( ...
        accepted.entries(1).caseOrdinal, structural.primaryArms{1}), ...
        'FormationB4V46DevelopmentPermit:ExecutionSealed');
    context = struct( ...
        'contractVersion', ...
            'formation-b4-v46-development-arm-execution-context-v1', ...
        'permitId', 'sealed', 'permitCanonicalSha256', '', ...
        'capability', 'development-arm-state-estimate', ...
        'action', 'filter-development-arm', ...
        'caseOrdinal', accepted.entries(1).caseOrdinal, ...
        'armId', structural.primaryArms{1});
    assertErrorId(@() assertDynamicTopologyTrackingOutcomeAuthorized( ...
        struct(), context, struct()), ...
        'FormationB4V46DevelopmentPermit:ExecutionSealed');
else
    assert(registry.filterExecutionAuthorized);
    assert(registry.stateEstimateOutputAuthorized);
    assert(registry.permitCount == 16);
    assert(registry.completeCaseArmCartesianProduct);
    assert(~isempty(regexp(registry.executionSurfaceCommit, ...
        '^[0-9a-f]{40}$', 'once')));
    assert(~isempty(regexp(registry.executionSurfaceManifestSha256, ...
        '^[0-9a-f]{64}$', 'once')));
    cursor = 0;
    for caseIdx = 1:numel(accepted.entries)
        for armIdx = 1:numel(structural.primaryArms)
            cursor = cursor + 1;
            permit = registry.permits(cursor);
            assert(permit.caseOrdinal == ...
                accepted.entries(caseIdx).caseOrdinal);
            assert(strcmp(permit.armId, structural.primaryArms{armIdx}));
            assert(strcmp(computeCanonicalValueSha256( ...
                rmfield(permit, 'permitCanonicalSha256')), ...
                permit.permitCanonicalSha256));
            assert(permit.stateEstimateOutputAuthorized);
            assert(~permit.trackingOutcomeScoringAuthorized);
            assert(~permit.groundTruthAccessAuthorized);
            assert(~permit.validationClaimAllowed);
        end
    end
end

getterText = fileread(which( ...
    'getFormationB4V46DevelopmentArmPermitRegistry'));
forbidden = { ...
    'OfflineSourceProvenance', 'source_discovery_v2.mat', ...
    'sourceEnvelopeCanonicalSha256', ...
    'fullSourceFingerprintCanonicalSha256'};
for idx = 1:numel(forbidden)
    assert(isempty(strfind(getterText, forbidden{idx}))); %#ok<STREMP>
end
fprintf('PASS: V46 development arm permit registry tests\n');
end

function assertErrorId(callback, expectedIdentifier)
actualIdentifier = '';
try
    callback();
catch errorInfo
    actualIdentifier = errorInfo.identifier;
end
assert(strcmp(actualIdentifier, expectedIdentifier), ...
    'Expected %s, received %s.', ...
    expectedIdentifier, actualIdentifier);
end
