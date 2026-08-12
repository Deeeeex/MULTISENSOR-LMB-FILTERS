function test_set_trust_sequence_v134_generalization_protocol()
% TEST_SET_TRUST_SEQUENCE_V134_GENERALIZATION_PROTOCOL Frozen split checks.

protocol = getSetTrustSequenceV134GeneralizationProtocol();
assert(strcmp(protocol.contractVersion, ...
    'v134-policy-generalization-protocol-v1'));
assert(strcmp(protocol.canonicalSha256, ...
    '4d6a177fd5eae7ed58bab585367195fcc54923fb045b9b3ee0c5c2472ec979a7'));
assert(strcmp(protocol.methodProtocolId, ...
    getSetTrustSequenceV134Protocol().id));
assert(protocol.trackingOutcomePermit.required);
assert(~protocol.trackingOutcomePermit.issueBeforeMethodJointGate);
assert(~protocol.trackingOutcomePermit.issueBeforePolicyFreeze);
assert(~protocol.trackingOutcomePermit.issueBeforeGeometryGate);
assert(protocol.trackingOutcomePermit.bindExactPresetSeedManifest);
assert(~protocol.trackingOutcomePermit.authorizeTruthForPolicy);
assert(~protocol.trackingOutcomePermit. ...
    authorizeValidationClaimBeforeAggregateGate);
assert(protocol.policyFreeze.oneSharedScaleNormalizedCheckpoint);
assert(~protocol.policyFreeze.perScalePolicyCheckpointAllowed);
assert(~protocol.policyFreeze.perPresetTuningAllowed);
assert(~protocol.policyFreeze.sceneIdentityInputAllowed);
assert(~protocol.policyFreeze.truthInputAllowed);
assert(protocol.policyFreeze.radialValidationPassRequiredBeforeTransfer);
assert(~protocol.reference.perSceneDirectionOracleAllowed);
assert(~protocol.reference.perSeedDirectionOracleAllowed);
assert(protocol.transferGate.requireEachScaleIndependently);
assert(protocol.transferGate.requireEachStyleIndependently);
assert(~protocol.transferGate.crossScaleCompensationAllowed);
assert(~protocol.transferGate.crossStyleCompensationAllowed);
assert(~protocol.crossingStress.formalSuccessGate);
assert(~protocol.crossingStress.includedInPrimaryAverage);
assert(protocol.reportingPolicy.belowGateResultsRepositoryOnly);
assert(~protocol.validationClaimAllowedBeforeExecution);

splitCells = { ...
    protocol.developmentSeeds, ...
    protocol.radialValidationSeeds, ...
    protocol.convoyTransferSeeds, ...
    protocol.relayTransferSeeds, ...
    protocol.crossingStressSeeds};
allSeeds = [];
for splitIdx = 1:numel(splitCells)
    seeds = splitCells{splitIdx};
    assert(numel(seeds) == numel(unique(seeds)));
    assert(isempty(intersect(allSeeds, seeds)));
    allSeeds = [allSeeds, seeds]; %#ok<AGROW>
end

assertPresetSet(protocol.radialPresets, { ...
    'm24-formation-fov', 'x36-formation-fov'});
assertPresetSet(protocol.transferPresets, { ...
    'm24-formation-fov-convoy', ...
    'x36-formation-fov-convoy', ...
    'm24-formation-fov-relay', ...
    'x36-formation-fov-relay'});
assertPresetSet(protocol.stressPresets, { ...
    'm24-formation-fov-crossing', ...
    'x36-formation-fov-crossing'});

geometry = getFormationFovMultistyleGeometryValidationProtocol();
assert(strcmp(protocol.requiredGeometryProtocolSha256, ...
    geometry.canonicalSha256));
for entry = protocol.transferPresets
    config = buildDynamicTopologyScenarioConfig(entry.presetName);
    assert(strcmp(config.sceneStyle, entry.styleName));
    assert(config.formalValidationAuthorized);
    assert(~config.trackingOutcomeAuthorized);
end
for entry = protocol.stressPresets
    config = buildDynamicTopologyScenarioConfig(entry.presetName);
    assert(strcmp(config.sceneStyle, entry.styleName));
    assert(~config.formalValidationAuthorized);
    assert(~config.trackingOutcomeAuthorized);
end

fprintf('test_set_trust_sequence_v134_generalization_protocol passed\n');
end

function assertPresetSet(entries, expected)
assert(isequal({entries.presetName}, expected));
for entry = entries
    config = buildDynamicTopologyScenarioConfig(entry.presetName);
    assert(config.numberOfSensors == str2double(entry.scaleName(2:end)));
end
end
