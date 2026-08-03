function config = getFormationBackboneConditionalBundlePolicyConfig()
% GETFORMATIONBACKBONECONDITIONALBUNDLEPOLICYCONFIG V40 mechanism probe.
%
% V40 keeps the frozen v38 topology and safety parameters but changes the
% counterfactual represented by a suspended residual input.  After the
% input is removed, every remaining reference weight in that receiver row
% is renormalized proportionally.  The candidate therefore reproduces the
% reference fusion weights conditional on that residual message not being
% delivered, instead of also increasing the receiver's relative self trust.

base = getFormationBackboneBundleStaggeredRecoveryPolicyConfig();
payload = rmfield(base, 'canonicalSha256');
payload.id = ...
    'formation-backbone-conditional-bundle-v40-v1';
payload.contractVersion = ...
    'formation-backbone-conditional-bundle-policy-config-v1';
payload.basePolicyId = base.id;
payload.basePolicyConfigSha256 = base.canonicalSha256;
payload.suspensionReweightingMode = ...
    'renormalize-remaining-reference-row';
payload.referenceMissingInputEquivalent = true;
payload.singleCandidateCoverage = 'all-safe-singles';
payload.entryDisagreementUse = 'diagnostic-only';
payload.recoveryDisagreementUse = ...
    'incumbent-relative-ordering-only';
payload.windowContractionCertificateClaimed = false;
payload.trackingBenefitClaimed = false;
payload.developmentMechanismProbeOnly = true;
payload.truthUsed = false;
payload.futureOutcomeUsed = false;

actualSha256 = computeCanonicalValueSha256(payload);
expectedSha256 = ...
    '91e18276b3a3af1825cc0bc21b831bfe2c0f3d6177624b86de57e4558eaf99b9';
if ~strcmp(actualSha256, expectedSha256)
    error('FormationConditionalBundlePolicy:UnregisteredDrift', [ ...
        'The shared v40 policy changed without a policy-version ', ...
        'update: actual=%s expected=%s.'], ...
        actualSha256, expectedSha256);
end
config = payload;
config.canonicalSha256 = actualSha256;
end
