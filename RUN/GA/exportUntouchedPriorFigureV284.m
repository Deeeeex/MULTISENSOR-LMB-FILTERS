function exportUntouchedPriorFigureV284(resultPath)
% Numerical export only. No filtering, graphics device or selective samples.
loaded = load(resultPath, 'result'); r = loaded.result;
assert(r.maximumTime == 40 && r.screenEvaluated);
out = fileparts(resultPath);
fid = fopen(fullfile(out, 'V284_TIME_SERIES.csv'), 'w'); assert(fid >= 0);
fprintf(fid, 'step,reference_eospa,candidate_eospa,reference_count_error,candidate_count_error,reference_disagreement,candidate_disagreement,excluded_sources\n');
for t = 1:r.maximumTime
    fprintf(fid, '%d,%.15g,%.15g,%.15g,%.15g,%.15g,%.15g,%d\n', t, ...
        mean(r.reference.eospaBySensorTime(:, t)), mean(r.candidate.eospaBySensorTime(:, t)), ...
        mean(r.reference.countErrorBySensorTime(:, t)), mean(r.candidate.countErrorBySensorTime(:, t)), ...
        r.reference.disagreementByTime(t), r.candidate.disagreementByTime(t), ...
        r.lineageExclusionsByTime(t));
end
fclose(fid);
fid = fopen(fullfile(out, 'V284_JOINT_METRICS.csv'), 'w'); assert(fid >= 0);
fprintf(fid, 'metric,reference,candidate,change_percent\n');
names = {'E-OSPA', 'Count error', 'Matched RMSE', 'Disagreement', 'Payload bytes'};
reference = [r.reference.eospa, r.reference.countError, r.commonReferenceRmse, ...
    r.reference.representativeDisagreement, r.reference.attemptedBytes];
candidate = [r.candidate.eospa, r.candidate.countError, r.commonCandidateRmse, ...
    r.candidate.representativeDisagreement, r.candidate.attemptedBytes];
for k = 1:numel(names)
    fprintf(fid, '%s,%.15g,%.15g,%.15g\n', names{k}, reference(k), candidate(k), ...
        100 * (candidate(k) / reference(k) - 1));
end
fclose(fid);
fprintf('V284 figure data exported: 40 steps, 5 joint metrics; no filter rerun.\n');
end
