function test_formation_staggered_recovery_m24_v36_gate()
% TEST_FORMATIONSTAGGEREDRECOVERYM24V36GATE Aggregate gate tests.

protocol = ...
    getFormationStaggeredRecoveryM24ReplicationProtocol();
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

gate = evaluateFormationStaggeredRecoveryM24V36Gate( ...
    states, protocol);
assert(gate.passed);
assert(gate.strongStateCount == 2);
assert(abs(gate.medianMeanGainPercent - 2.5) <= 1e-12);
assert(abs(gate.minimumStateMeanGainPercent + 0.5) <= 1e-12);
assert(gate.positiveTerminalConsensusStateCount == 2);

states(3).meanGainPercent = -1.01;
gate = evaluateFormationStaggeredRecoveryM24V36Gate( ...
    states, protocol);
assert(~gate.passed);
assert(~gate.minimumStateMeanGainPassed);

states(3).meanGainPercent = -0.5;
states(2).terminalConsensusGainPercent = 0;
gate = evaluateFormationStaggeredRecoveryM24V36Gate( ...
    states, protocol);
assert(~gate.passed);
assert(~gate.positiveTerminalCountPassed);

fprintf('PASS: v36 M24 replication gate tests\n');
end
