function test_rolling_safe_sparse_action_bank()
% TEST_ROLLINGSAFESPARSEACTIONBANK Freeze the equal-message bank contract.

protocol = getLabelSetSimulatorPolicyProtocol();
expectedCodes = [24, 22, 61, 63, 67, 77, 80, 83];
assert(strcmp( ...
    protocol.rollingSafeSparseActionBankContractVersion, ...
    'm24-rolling-safe-truth-free-eight-action-bank-v1'));
assert(isequal( ...
    protocol.rollingSafeSparseActionBankCodes, expectedCodes));
assert(isequal( ...
    protocol.rollingSafeSparseProbeActionCodes, expectedCodes));
assert(numel(protocol.rollingSafeSparseActionBankNames) == ...
    numel(expectedCodes));
assert(numel(unique(expectedCodes)) == numel(expectedCodes));
assert(nnz(expectedCodes == ...
    protocol.rollingSafeSparseReferenceActionCode) == 1);
assert(all(ismember(expectedCodes, ...
    protocol.rollingSafeSparseTruthFreeActionCodes)));
assert(protocol.rollingSafeSparseMessageBudget == ...
    protocol.expectedNodeCount);
assert(isequal( ...
    protocol.rollingSafeSparseSentinelSeedTimes, ...
    [11, 75; 17, 75; 19, 75; 27, 76]));
assert(isequal( ...
    protocol.rollingSafeSparseExcludedOutlierSeedTime, ...
    [19, 75]));
assert(protocol.rollingSafeSparseMinimumSentinelImprovedStateCount == 2);
assert(protocol.rollingSafeSparseMinimumNonOutlierGainPercent == 5);
assert(protocol.rollingSafeSparseMaximumByteDeviationPercent == 2);
assert(protocol.rollingSafeSparseRequireWorstAndConsensusNonregression);

[registry, metadata] = ...
    getRollingSafeObservableScoreBasisRegistry();
assert(~metadata.truthUsed);
assert(~metadata.futureOutcomeUsed);
assertBasis(registry, 61, 'link-advantage', 2);
assertBasis(registry, 63, 'posterior-gain', 1);
assertBasis(registry, 67, ...
    'compatibility-conservative', 2);
assertBasis(registry, 77, 'history-continuity', 3);
assert(exist('runRollingSafeSparsePhasePreflightM24', 'file') == 2);
assert(exist( ...
    'summarizeRollingSafeSparseActionBankSentinelM24', 'file') == 2);

fprintf('test_rolling_safe_sparse_action_bank passed\n');
end

function assertBasis(registry, actionCode, basisId, exactCount)
entry = registry([registry.actionCode] == actionCode);
assert(numel(entry) == 1);
assert(strcmp(entry.basisId, basisId));
assert(entry.exactCurrentCrossEdgeCount == exactCount);
assert(~entry.truthUsed);
assert(~entry.futureOutcomeUsed);
end
