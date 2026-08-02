function test_formation_staggered_recovery_x36_v37_gate()
% TEST_FORMATIONSTAGGEREDRECOVERYX36V37GATE Aggregate X36 gate tests.

protocol = getFormationStaggeredRecoveryX36SourceProtocol();
states = repmat(struct( ...
    'strictStrong', false, ...
    'meanGainPercent', 0, ...
    'terminalConsensusGainPercent', 0), 1, 3);
states(1).strictStrong = true;
states(1).meanGainPercent = 3.0;
states(1).terminalConsensusGainPercent = 1.0;
states(2).strictStrong = true;
states(2).meanGainPercent = 2.5;
states(2).terminalConsensusGainPercent = 0.2;
states(3).meanGainPercent = -0.5;
states(3).terminalConsensusGainPercent = -0.1;

gate = evaluateFormationStaggeredRecoveryX36V37Gate( ...
    states, protocol);
assert(gate.passed);
assert(strcmp(gate.contractVersion, ...
    'formation-staggered-recovery-x36-v37-gate-v1'));
assert(gate.strongStateCount == protocol.minimumStrongStateCount);
assert(abs(gate.medianMeanGainPercent - 2.5) <= 1e-12);
assert(abs(gate.minimumStateMeanGainPercent + 0.5) <= 1e-12);
assert(gate.positiveTerminalConsensusStateCount == ...
    protocol.minimumPositiveTerminalStateCount);

states(3).meanGainPercent = -1.01;
gate = evaluateFormationStaggeredRecoveryX36V37Gate( ...
    states, protocol);
assert(~gate.passed);
assert(~gate.minimumStateMeanGainPassed);

states(3).meanGainPercent = -0.5;
states(2).terminalConsensusGainPercent = 0;
gate = evaluateFormationStaggeredRecoveryX36V37Gate( ...
    states, protocol);
assert(~gate.passed);
assert(~gate.positiveTerminalCountPassed);

states(2).terminalConsensusGainPercent = 0.2;
states(2).strictStrong = false;
gate = evaluateFormationStaggeredRecoveryX36V37Gate( ...
    states, protocol);
assert(~gate.passed);
assert(~gate.strongCountPassed);

fprintf('PASS: v37 X36 outcome gate tests\n');
end
