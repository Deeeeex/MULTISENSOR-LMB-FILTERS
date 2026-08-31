function [reportPath, result] = ...
        runPersistentSupportGapFallbackV207X36T72(options)
% RUNPERSISTENTSUPPORTGAPFALLBACKV207X36T72 Add one causal late fallback.
%
% V206 restores F5 on the first page and then applies the five supported
% label-KLA actions.  Its only remaining formation tail is recreated after
% F5 is withheld again.  In the resulting observable state, label [19,16]
% is absent at all six F5 receivers on both t=78 and t=79 while 24 external
% local posteriors retain strong support.  V207 keeps the V206 sequence and
% releases F5 only on the second consecutive support-gap page.  Formation,
% label, and page identifiers remain opened teacher inputs; this run tests
% the fallback mechanism, not an online detector.

if nargin < 1 || isempty(options)
    options = struct();
end
releasePlan = struct();
releasePlan.contractVersion = ...
    'online-positive-net-formation-release-v191-plan-v1';
releasePlan.times = [72, 79];
releasePlan.formationIdsByTime = {5, 5};
releasePlan.teacherMode = true;

defaults = struct();
defaults.onlinePositiveNetFormationReleasePlan = releasePlan;
defaults.outputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v207', 'persistent_support_gap_teacher', ...
    'x36_seed211_t72_h8');
names = fieldnames(defaults);
for fieldIdx = 1:numel(names)
    name = names{fieldIdx};
    if ~isfield(options, name)
        options.(name) = defaults.(name);
    end
end

[reportPath, result] = ...
    runSupportAwareFormationControllerV206X36T72(options);
result.sequenceContractVersion = ...
    'persistent-support-gap-fallback-v207-teacher-v1';
result.persistentSupportGapDwellPages = 2;
result.persistentSupportGapFallbackTimes = 79;
result.sequenceNumericIdentifiersUsedForTeacherRouting = true;
result.sequenceValidationClaimAllowed = false;
end
