function protocol = getProtectionLoadGatedRoleV145Protocol()
% GETPROTECTIONLOADGATEDROLEV145PROTOCOL Observable one-burst W/R gate.

protocol = getReferenceAnchoredBurstRoleScheduleV144Protocol();
protocol.id = 'protection-load-gated-role-v145-v1';
protocol.contractVersion = ...
    'v145-protection-load-gated-role-protocol-v1';
protocol.methodName = ...
    'protection-load-gated single-payload W/R routing';
protocol.outcomePolicyName = ...
    'protection-load-gated-role-v145-screen-v1';
protocol.roleSchedule.initialReferencePageRequired = true;
protocol.roleSchedule.minimumWorkingProtectedFormationFractionExclusive = ...
    0.5;
protocol.roleSchedule.referenceLatchEnabled = true;
protocol.roleSchedule.referenceLatchCondition = ...
    'protected-formation-fraction-at-or-below-one-half';
protocol.roleSchedule.onlineTruthUsed = false;
protocol.roleSchedule.numericSensorIdUsed = false;
protocol.roleSchedule.futureOutcomeUsed = false;
protocol.finiteTimeMixingPreflight.inheritsV144WorstPrefix = true;
protocol.finiteTimeMixingPreflight.allReferenceAfterLatch = true;
protocol.finiteTimeMixingPreflight.preOutcomeFreezeTime = ...
    '2026-08-29 01:12 CST';
protocol.outputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v145', 'protection_load_gated_role_v1');
end

