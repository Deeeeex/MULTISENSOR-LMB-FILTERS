function config = getFormationBackboneBundleParetoRiskPolicyConfig()
% GETFORMATIONBACKBONEBUNDLEPARETORISKPOLICYCONFIG Shared v39 gate.
%
% V39 keeps the frozen v38 causal proposal family and selects over its
% distinct safe routes.  The primary objective is maximum message saving,
% subject to exact current one-round network disagreement no larger than
% the registered reference.  Risk, selected-history switching and stable
% proposal order are deterministic tie breaks.

base = getFormationBackboneBundleStaggeredRecoveryPolicyConfig();
payload = struct();
payload.id = 'formation-backbone-bundle-pareto-risk-v39-v2';
payload.contractVersion = ...
    'formation-backbone-bundle-pareto-risk-policy-config-v1';
payload.basePolicyId = base.id;
payload.basePolicyConfigSha256 = base.canonicalSha256;
payload.minimumMessageSavingCount = base.minimumMessageSavingCount;
payload.minimumReferenceNetworkRiskImprovementFraction = 0;
payload.fallbackAction = 'registered-reference';
payload.candidateGenerator = ...
    'formation-backbone-bundle-staggered-recovery-v38-v1';
payload.selectionRule = ...
    'lexicographic-max-saving-min-risk-min-switch-stable-order';
payload.referenceRiskConstraint = ...
    'candidate-network-risk-less-than-or-equal-reference-exactly';
payload.candidateBank = [ ...
    'reference-v38-incumbent-requested-safe-singles-', ...
    'safe-staggered-release-exactly-deduplicated'];
payload.truthUsed = false;
payload.futureOutcomeUsed = false;

actualSha256 = computeCanonicalValueSha256(payload);
expectedSha256 = ...
    '7b7ee6eee318af79156efd1de0bb77eb1ea2a67bc1abc559eac35b423f188e6a';
if ~strcmp(actualSha256, expectedSha256)
    error('FormationBundleParetoRisk:UnregisteredDrift', [ ...
        'The shared v39 policy changed without a policy-version ', ...
        'update: actual=%s expected=%s.'], ...
        actualSha256, expectedSha256);
end
config = payload;
config.canonicalSha256 = actualSha256;
end
