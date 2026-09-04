function config = buildExistenceMarginBeneficiaryV269Config(scenario)
% BUILDEXISTENCEMARGINBENEFICIARYV269CONFIG V268 with robust beneficiary.

protocol = getExistenceMarginBeneficiaryV269Protocol();
config = buildSourcePreservingLabelPacketV268Config(scenario);
config.topologyPolicyName = protocol.policyName;
config.topologyPolicyFcn = @selectExistenceMarginBeneficiaryV269Policy;
config.topologyPolicySourcePreservingLabelPacketBeneficiaryMode = ...
    protocol.beneficiaryMode;
end
