function test_budget_recycled_formation_repair_v188()
% TEST_BUDGETRECYCLEDFORMATIONREPAIRV188 Core budget/safety invariants.

protocol = getBudgetRecycledFormationRepairV188Protocol();
assert(protocol.communicationCreditReserveFraction == 0.20);
assert(protocol.lightSynopsisPreauthorizationRequired);

% Earn 1000 B: lock 200 B, expose 800 B, then charge a 100 B synopsis.
[state, preflight] = preflightBudgetRecycledRepairSynopsisV188( ...
    [], 1000, 100, 2000);
assert(preflight.lightSynopsisAuthorized);
assert(abs(state.protectedSavingBytes - 200) < 1e-12);
assert(abs(state.spendableCreditBytes - 700) < 1e-12);
assert(abs(state.certifiedCumulativeSavingBytes - 900) < 1e-12);

proposals = repmat(struct( ...
    'formationId', 0, 'attemptedBytes', 0, ...
    'estimatedUtilityLowerBound', 0, ...
    'safetyPassed', false, 'rollingConnectivityPassed', false), 1, 4);
proposals(1) = makeProposal(1, 100, 2.0, true, true);
proposals(2) = makeProposal(2, 200, 3.0, true, true);
proposals(3) = makeProposal(3, 50, 100.0, false, true);
proposals(4) = makeProposal(4, 50, 100.0, true, false);
projection = projectBudgetRecycledRepairActionsV188(proposals, state);
assert(isequal(projection.selectedProposalIndices, 1));
assert(projection.selectedActionBytes == 100);
assert(projection.cumulativeByteSafetyPassed);
assert(projection.protectedSavingPreserved);
twoActionProjection = projectBudgetRecycledRepairActionsV188( ...
    proposals, state, struct('maximumActions', 2));
assert(isequal(twoActionProjection.selectedProposalIndices, [1, 2]));
assert(twoActionProjection.selectedActionBytes == 300);
state = projection.nextCreditState;
assert(abs(state.protectedSavingBytes - 200) < 1e-12);
assert(abs(state.spendableCreditBytes - 600) < 1e-12);
assert(abs(state.certifiedCumulativeSavingBytes - 800) < 1e-12);

% No new saving: another synopsis may spend old disposable credit, but the
% original 200 B protected saving is invariant rather than recomputed.
[state, preflight] = preflightBudgetRecycledRepairSynopsisV188( ...
    state, 0, 500, 2000);
assert(preflight.lightSynopsisAuthorized);
assert(abs(state.protectedSavingBytes - 200) < 1e-12);
assert(abs(state.spendableCreditBytes - 100) < 1e-12);
assert(abs(state.certifiedCumulativeSavingBytes - 300) < 1e-12);

% An unaffordable synopsis is not sent or charged.
[unchanged, denied] = preflightBudgetRecycledRepairSynopsisV188( ...
    state, 0, 101, 2000);
assert(~denied.lightSynopsisAuthorized);
assert(denied.chargedSynopsisBytes == 0);
assert(unchanged.pagesProcessed == state.pagesProcessed + 1);
assert(unchanged.spendableCreditBytes == state.spendableCreditBytes);
assert(unchanged.protectedSavingBytes == state.protectedSavingBytes);
assert(unchanged.cumulativeControlAndRepairSpendBytes == ...
    state.cumulativeControlAndRepairSpendBytes);

% Cap overflow becomes protected saving and can never be spent later.
[capped, capDecision] = preflightBudgetRecycledRepairSynopsisV188( ...
    unchanged, 4000, 0, 1000);
assert(capDecision.capOverflowLockedBytes > 0);
assert(capped.spendableCreditBytes == 1000);
protectedBefore = capped.protectedSavingBytes;
expensive = makeProposal(1, 1000, 1e9, true, true);
projection = projectBudgetRecycledRepairActionsV188(expensive, capped);
assert(projection.selectedActionBytes == 1000);
assert(projection.nextCreditState.protectedSavingBytes == protectedBefore);
assert(projection.nextCreditState.certifiedCumulativeSavingBytes == ...
    protectedBefore);

% No model score can override either deterministic safety certificate.
unsafe = [ ...
    makeProposal(1, 1, 1e12, false, true), ...
    makeProposal(2, 1, 1e12, true, false)];
projection = projectBudgetRecycledRepairActionsV188( ...
    unsafe, projection.nextCreditState);
assert(~any(projection.selectedMask));
assert(strcmp(projection.fallbackReason, ...
    'base-admission-without-repair'));

fprintf('test_budget_recycled_formation_repair_v188 passed\n');
end

function proposal = makeProposal( ...
        formationId, attemptedBytes, utility, safety, connectivity)
proposal = struct( ...
    'formationId', formationId, ...
    'attemptedBytes', attemptedBytes, ...
    'estimatedUtilityLowerBound', utility, ...
    'safetyPassed', safety, ...
    'rollingConnectivityPassed', connectivity);
end
