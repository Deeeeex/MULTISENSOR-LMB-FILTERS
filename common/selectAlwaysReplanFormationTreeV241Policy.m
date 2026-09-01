function [adjacency, details] = ...
        selectAlwaysReplanFormationTreeV241Policy(context)
% SELECTALWAYSREPLANFORMATIONTREEV241POLICY Memory-free current tree.

protocol = getFormationBraidRoutingComparisonV241Protocol();
restricted = context;
nodeCount = numel(context.localPosteriorBySensor);
restricted.previousAdjacencyHistory = false(nodeCount, nodeCount, 0);
restricted.previousAdjacencyHistoryCount = 0;
restricted.previousAdjacencyHistoryTimes = zeros(1, 0);
[adjacency, details] = ...
    selectCausalMinimalEditFormationTreeV240Policy(restricted);
details.contractVersion = ...
    'formation-braid-v241-always-replan-policy-v1';
details.protocolId = protocol.id;
details.armId = protocol.alwaysReplanArmId;
details.mode = 'current-physical-formation-tree-always-replan';
details.backboneMode = details.mode;
details.previousPolicyHistoryUsed = false;
details.treeReselectionUsed = true;
details.initialTreeSelectionUsed = context.currentTime == 1;
details.routeReselectionUsed = true;
details.scheduleCertificate.contractVersion = ...
    'formation-braid-v241-always-replan-schedule-v1';
details.scheduleCertificate.phase = 'current-tree-always-replan';
details.scheduleCertificate.treeReselectionUsed = true;
end
