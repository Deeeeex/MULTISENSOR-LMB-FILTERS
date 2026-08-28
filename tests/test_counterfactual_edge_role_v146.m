function test_counterfactual_edge_role_v146()
% TEST_COUNTERFACTUALEDGEROLEV146 Frozen bank and selector contract.

protocol = getCounterfactualEdgeRoleV146Protocol();
assert(numel(protocol.roleActionBank.actions) == 7);
upperM24 = resolveCounterfactualEdgeRoleV146Action( ...
    protocol.roleActionBank.actions(5), 4);
lowerX36 = resolveCounterfactualEdgeRoleV146Action( ...
    protocol.roleActionBank.actions(6), 6);
assert(isequal(upperM24.selectedFormationRanks, [1, 2]));
assert(isequal(lowerX36.selectedFormationRanks, [4, 5, 6]));
assert(isequal(upperM24.workingPageOffsets, [1, 2, 3, 5]));

config = struct( ...
    'lineageRelayRoleMultiplexedSinglePayloadEnabled', true, ...
    'lineageRelayRoleMultiplexedCounterfactualBankEnabled', true, ...
    'lineageRelayRoleMultiplexedAnchorTime', 95, ...
    'lineageRelayRoleMultiplexedFormationOrder', [2, 3, 4, 1], ...
    'lineageRelayRoleMultiplexedSelectedFormationRanks', [1, 3], ...
    'lineageRelayRoleMultiplexedWorkingPageOffsets', [1, 2, 3, 5]);
groups = repelem(1:4, 6);

[role, details] = selectCounterfactualEdgeRoleV146( ...
    config, groups, 8, 7, 96, true);
assert(strcmp(role, 'working'));
assert(details.receiverFormationRank == 1);
[role, details] = selectCounterfactualEdgeRoleV146( ...
    config, groups, 2, 7, 96, true);
assert(strcmp(role, 'reference'));
assert(~details.intraFormation);
[role, details] = selectCounterfactualEdgeRoleV146( ...
    config, groups, 8, 7, 99, true);
assert(strcmp(role, 'reference'));
assert(details.referencePhase);
[role, details] = selectCounterfactualEdgeRoleV146( ...
    config, groups, 8, 7, 100, true);
assert(strcmp(role, 'working'));
assert(details.workingPhase);
[role, details] = selectCounterfactualEdgeRoleV146( ...
    config, groups, 8, 7, 100, false);
assert(strcmp(role, 'reference'));
assert(~details.receiverProtected);
fprintf('PASS: V146 counterfactual edge-role bank contract\n');
end
