function test_formation_common_label_repair_proposals_v188()
% TEST_FORMATIONCOMMONLABELREPAIRPROPOSALSV188 Executable payload contract.

inputs = generateDynamicTopologyScenarioInputs( ...
    'm24-formation-fov', 1601);
model = inputs.model;
source = makeObject([0; 0; 0; 0], 4, 2);
peer = makeObject([1; 1; 0; 0], 4, 2);
receiver = makeObject([40; 40; 0; 0], 25000, 1);
local = {receiver, receiver, source, peer};
fused = {receiver, receiver, source, peer};
groupIds = [1, 1, 2, 2];
physical = false(4);
physical(1:2, 3:4) = true;

cache = buildFormationRepairLightSynopsisCacheV188( ...
    fused, local, model, 1, struct());
[proposals, details] = ...
    buildFormationCommonLabelRepairProposalsV188( ...
        cache, local, physical, groupIds, model, struct());
proposalIdx = find([proposals.formationId] == 1, 1);
assert(~isempty(proposalIdx));
proposal = proposals(proposalIdx);
protocol = getBudgetRecycledFormationRepairV188Protocol();
assert(isequal(proposal.receiverIds, [1, 2]));
assert(ismember(proposal.sourceId, [3, 4]));
assert(proposal.sourceObject.numberOfGmComponents == 2);
assert(proposal.minimumRiskReduction > 0);
assert(proposal.safetyPassed);
assert(proposal.commonSourcePassed);
assert(proposal.attemptedBytes == ...
    protocol.richSynopsisBytesPerShortlistedLabel + ...
    protocol.completeLabelRequestBytes + ...
    2 * proposal.completeResponseBytesPerReceiver);
assert(details.lightSynopsisBytesExcluded);
assert(~details.truthUsed);

[credit, preflight] = preflightBudgetRecycledRepairSynopsisV188( ...
    [], 1e6, cache.totalAttemptedBytes, 2e6);
assert(preflight.lightSynopsisAuthorized);
projection = projectBudgetRecycledRepairActionsV188( ...
    proposals(proposalIdx), credit);
assert(isequal(projection.selectedProposalIndices, 1));
[repaired, applied] = applyFormationCommonLabelRepairV188( ...
    fused, proposals(proposalIdx), ...
    projection.selectedProposalIndices);
assert(applied.appliedActionCount == 1);
assert(applied.appliedReceiverCount == 2);
assert(repaired{1}.numberOfGmComponents == 2);
assert(repaired{2}.numberOfGmComponents == 2);
assert(isequal(repaired{1}.mu, proposal.sourceObject.mu));
assert(isequal(repaired{2}.Sigma, proposal.sourceObject.Sigma));
assert(repaired{3}.numberOfGmComponents == 2);

[forcedRepair, ~, forcedDetails] = ...
    selectBudgetRecycledFormationRepairV188( ...
        fused, local, physical, false(4), groupIds, model, 1, [], ...
        2e6, 1e6, struct( ...
            'idealDeliveryTeacherMode', true, ...
            'maximumActions', 1, ...
            'forcedFormationId', 1));
assert(forcedDetails.fixedFormationIdentifierUsed);
assert(forcedDetails.forcedFormationId == 1);
assert(forcedDetails.applyDetails.appliedActionCount == 1);
assert(isequal(forcedRepair{1}.mu, proposal.sourceObject.mu));

[~, ~, noActionDetails] = ...
    selectBudgetRecycledFormationRepairV188( ...
        fused, local, physical, false(4), groupIds, model, 1, [], ...
        2e6, 1e6, struct( ...
            'idealDeliveryTeacherMode', true, ...
            'maximumActions', 1, ...
            'forcedFormationId', 2));
assert(noActionDetails.applyDetails.appliedActionCount == 0);

fprintf('test_formation_common_label_repair_proposals_v188 passed\n');
end

function object = makeObject(meanVector, positionVariance, components)
object = struct();
object.numberOfGmComponents = components;
object.w = ones(1, components) / components;
object.mu = cell(1, components);
object.Sigma = cell(1, components);
for componentIdx = 1:components
    offset = [componentIdx - 1; componentIdx - 1; 0; 0];
    object.mu{componentIdx} = meanVector + offset;
    object.Sigma{componentIdx} = diag( ...
        [positionVariance, positionVariance, 4, 4]);
end
object.r = 0.9;
object.birthTime = 1;
object.birthLocation = 1;
object.associationConfidence = 0.9;
object.detectionAssociationMass = 0.9;
object.advertisedObservationOpportunity = 0.9;
end
