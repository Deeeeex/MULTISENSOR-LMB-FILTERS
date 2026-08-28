function protocol = getCounterfactualEdgeRoleV146Protocol()
% GETCOUNTERFACTUALEDGEROLEV146PROTOCOL Pre-learning headroom screen.

protocol = getReferenceAnchoredBurstRoleScheduleV144Protocol();
protocol.id = 'counterfactual-edge-role-headroom-v146-v1';
protocol.contractVersion = ...
    'v146-counterfactual-edge-role-headroom-protocol-v1';
protocol.methodName = ...
    'counterfactual single-payload edge-role headroom bank';
protocol.outcomePolicyName = ...
    'counterfactual-edge-role-headroom-v146-screen-v1';
protocol.roleActionBank = buildCounterfactualEdgeRoleBankV146();
protocol.screen.requireBothScales = true;
protocol.mechanismUpperBoundOnly = true;
protocol.communicationMethodClaimAllowedAfterJointGate = false;
protocol.preLearningHeadroomOnly = true;
protocol.validationClaimAllowed = false;
protocol.preOutcomeFreezeTime = '2026-08-29 01:43 CST';
protocol.outputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v146', 'counterfactual_edge_role_headroom_v1');
end
