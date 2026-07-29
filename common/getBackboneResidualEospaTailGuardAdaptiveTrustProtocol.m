function protocol = ...
    getBackboneResidualEospaTailGuardAdaptiveTrustProtocol()
% GETBACKBONERESIDUALEOSPATAILGUARDADAPTIVETRUSTPROTOCOL Frozen M24 gate.

surrogate = ...
    getBackboneResidualTailGuardAdaptiveTrustProtocol();
protocol = surrogate;
protocol.id = ...
    'backbone-residual-eospa-tail-guard-adaptive-trust-m24-h3-v1';
protocol.predecessorProtocolId = surrogate.id;
protocol.armName = [ ...
    'oracle-backbone-residual-spliced-cycle-', ...
    'eospa-tail-guard-adaptive-current-a70'];
protocol.protectedReceiverObjective = ...
    'maximum-matched-static-current-eospa';
protocol.protectionRiskMetric = ...
    'current-eospa';
protocol.evidencePending = false;
protocol.headroomGatePassed = false;
protocol.singleArgmaxGuardRejected = true;
protocol.smokeFeasibilityPassed = true;
protocol.smokeFallbackUsed = false;
protocol.smokeEospa = ...
    17.6608942413502;
protocol.smokeWorstSensorEospa = ...
    42.6103813158315;
protocol.smokeWorstSensorIndex = 16;
protocol.smokeCrossFormationEdgeCount = 4;
protocol.smokeSourceSha256 = ...
    '9d9d26e997f4761e7f1592c8832a5e49f1e7ee8a9e47be50fbcf6dc33bde967b';
protocol = removeFieldIfPresent(protocol, ...
    'surrogateTailGuardRejected');
end

function structure = removeFieldIfPresent( ...
        structure, fieldName)
if isfield(structure, fieldName)
    structure = rmfield(structure, fieldName);
end
end
