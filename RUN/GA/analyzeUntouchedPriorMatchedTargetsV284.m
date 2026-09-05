function [reportPath, analysis] = analyzeUntouchedPriorMatchedTargetsV284(resultPath)
% Post-hoc interpretation only. Does not redefine or override the V284 gate.
loaded = load(resultPath, 'result'); r = loaded.result;
loaded = load(r.rawPath, 'run'); candidate = loaded.run;
loaded = load(candidate.baselineTracePath, 'trace'); reference = loaded.trace;
assert(r.maximumTime == 40 && ~r.screenPassed);
out = fileparts(resultPath); nSensors = numel(candidate.estimates);
groupIds = candidate.config.sensorGroupIds;
% Rows: n, t, ref count, candidate count, common count, added count, lost count,
% sums of squared position errors for common/ref, common/candidate, added, lost.
rows = zeros(nSensors * 40, 11); cursor = 0; fallbackCount = 0; maxResidual = 0;
for n = 1:nSensors
    for t = 1:40
        truth = candidate.groundTruthRfs.x{t};
        [left, f1, d1] = matchedDistances(truth, reference.estimates{n}.mu{t}, ...
            r.reference.rmseBySensorTime(n, t));
        [right, f2, d2] = matchedDistances(truth, candidate.estimates{n}.mu{t}, ...
            r.candidate.rmseBySensorTime(n, t));
        common = isfinite(left) & isfinite(right);
        added = ~isfinite(left) & isfinite(right);
        lost = isfinite(left) & ~isfinite(right);
        cursor = cursor + 1;
        rows(cursor, :) = [n, t, nnz(isfinite(left)), nnz(isfinite(right)), ...
            nnz(common), nnz(added), nnz(lost), sum(left(common).^2), ...
            sum(right(common).^2), sum(right(added).^2), sum(left(lost).^2)];
        fallbackCount = fallbackCount + f1 + f2;
        maxResidual = max([maxResidual, d1, d2]);
    end
    if mod(n, 6) == 0, fprintf('V284 matched-target analysis: %d/%d sensors.\n', n, nSensors); end
end
full = load(reference.referenceResultPath, 'result'); fixed = full.result.referenceFixedTree;
fixedEospa = fixed.positionEospaBySensorTime(:, 1:40);
fixedRmse = fixed.positionRmseBySensorTime(:, 1:40);
fixedCount = fixed.absoluteCardinalityErrorBySensorTime(:, 1:40);
fixedSummary = struct('eospa', mean(fixedEospa(:)), ...
    'conditionalRmse', mean(fixedRmse(isfinite(fixedRmse))), ...
    'countError', mean(fixedCount(:)), ...
    'disagreement', mean(fixed.interFormationPositionOspaByTime(1:40)), ...
    'attemptedPrefixBytesAvailable', false);
groups = unique(groupIds, 'stable'); perFormation = repmat(pool(rows([],:)), 1, numel(groups));
for g = 1:numel(groups)
    mask = ismember(rows(:, 1), find(groupIds == groups(g)));
    perFormation(g) = pool(rows(mask, :));
end
analysis = struct('contractVersion', 'v284-posthoc-matched-target-analysis-v1', ...
    'candidateGenerationGitCommit', candidate.generationGitCommit, ...
    'originalJointScreenPassed', r.screenPassed, 'rows', rows, ...
    'pooled', pool(rows), 'perFormation', perFormation, 'fixedPrefix', fixedSummary, ...
    'assignmentFallbackCount', fallbackCount, 'maximumOfficialRmseResidual', maxResidual, ...
    'developmentEvidenceOnly', true);
save('-mat7-binary', fullfile(out, 'V284_MATCHED_TARGET_ANALYSIS.mat'), 'analysis');
fid = fopen(fullfile(out, 'V284_MATCHED_TARGET_CELLS.csv'), 'w'); assert(fid >= 0);
fprintf(fid, 'sensor,step,reference_count,candidate_count,common_count,added_count,lost_count,common_reference_squared_error,common_candidate_squared_error,added_squared_error,lost_squared_error\n');
for k = 1:size(rows, 1), fprintf(fid, '%d,%d,%d,%d,%d,%d,%d,%.15g,%.15g,%.15g,%.15g\n', rows(k, :)); end
fclose(fid);
reportPath = fullfile(out, 'V284_MATCHED_TARGET_ANALYSIS.md');
fid = fopen(reportPath, 'w'); assert(fid >= 0); cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# V284: interpretation of the matched-RMSE increase\n\n');
fprintf(fid, 'Post-hoc, self-check only. The original joint screen remains failed; these additional quantities do not replace its RMSE definition.\n\n');
fprintf(fid, 'Each arm uses minimum-sum-distance assignment to the same truth positions at the same sensor-time. Common targets are the intersection of assigned truth indices, not confirmed track-label identities or an independent evaluation set.\n\n');
fprintf(fid, 'The existing vectorized `munkres` solver is used for speed; any per-cell RMSE mismatch invokes the original `Hungarian` solver. Fallbacks: %d. Maximum difference from the saved official per-cell RMSE: %.3g m.\n\n', fallbackCount, maxResidual);
fprintf(fid, '| Scope | Common pairs | Added pairs | Lost pairs | Common reference pooled RMSE | Common candidate pooled RMSE | Added-target pooled RMSE |\n| --- | ---: | ---: | ---: | ---: | ---: | ---: |\n');
writePoolRow(fid, 'All', analysis.pooled);
for g = 1:numel(groups), writePoolRow(fid, sprintf('Formation %d', groups(g)), perFormation(g)); end
fprintf(fid, '\nPooled RMSE is sqrt(total squared error / matched-pair count), not the official average of per-cell RMSE. Pair counts repeat targets across sensors/time and are not independent samples. Added/lost mean assignment-set membership relative to the reference, not a causal identity ground truth.\n\n');
fprintf(fid, '## Cached fixed-routing prefix (original fusion rule)\n\n');
fprintf(fid, '| Metric | Fixed routing | V284 on sparse routing |\n| --- | ---: | ---: |\n');
fprintf(fid, '| E-OSPA | %.6f | %.6f |\n', fixedSummary.eospa, r.candidate.eospa);
fprintf(fid, '| Official conditional RMSE | %.6f | %.6f |\n', fixedSummary.conditionalRmse, r.candidate.conditionalRmse);
fprintf(fid, '| Count error | %.6f | %.6f |\n', fixedSummary.countError, r.candidate.countError);
fprintf(fid, '| Entire-prefix disagreement | %.6f | %.6f |\n', fixedSummary.disagreement, r.candidate.representativeDisagreement);
fprintf(fid, '\nThe cached fixed summary has no per-step payload-byte array; its 160-step bytes must not be compared with V284 40-step bytes. A fixed-routing arm with V284 semantics has not run. No joint improvement or routing attribution is established by this extra readout.\n');
fprintf('V284 common-target pooled RMSE %.6f -> %.6f; added %.6f; original screen remains failed.\n', ...
    analysis.pooled.commonReferenceRmse, analysis.pooled.commonCandidateRmse, analysis.pooled.addedRmse);
end

function [byTruth, fallback, residual] = matchedDistances(truth, estimates, expectedRmse)
byTruth = nan(numel(truth), 1); fallback = 0; residual = 0;
if isempty(truth) || isempty(estimates), return; end
left = cell2mat(truth); right = cell2mat(estimates);
cost = hypot(bsxfun(@minus, left(1, :)', right(1, :)), ...
    bsxfun(@minus, left(2, :)', right(2, :)));
[matching, ~] = munkres(cost);
actual = sqrt(mean(cost(matching).^2));
if ~isfinite(actual) || abs(actual - expectedRmse) > 1e-8 * max(1, expectedRmse)
    [matching, ~] = Hungarian(cost); matching = logical(matching); fallback = 1;
    actual = sqrt(mean(cost(matching).^2));
end
residual = abs(actual - expectedRmse);
assert(isfinite(residual) && residual <= 1e-8 * max(1, expectedRmse));
assert(nnz(matching) == min(size(cost)) && all(sum(matching, 1) <= 1) && all(sum(matching, 2) <= 1));
[i, j] = find(matching);
byTruth(i) = cost(sub2ind(size(cost), i, j));
end

function s = pool(rows)
totals = sum(rows, 1);
s = struct('commonCount', totals(5), 'addedCount', totals(6), 'lostCount', totals(7), ...
    'commonReferenceRmse', safeRmse(totals(8), totals(5)), ...
    'commonCandidateRmse', safeRmse(totals(9), totals(5)), ...
    'addedRmse', safeRmse(totals(10), totals(6)), 'lostRmse', safeRmse(totals(11), totals(7)));
end

function value = safeRmse(squared, count)
if count == 0, value = NaN; else, value = sqrt(squared / count); end
end

function writePoolRow(fid, label, p)
fprintf(fid, '| %s | %d | %d | %d | %.6f | %.6f | %.6f |\n', label, ...
    p.commonCount, p.addedCount, p.lostCount, p.commonReferenceRmse, p.commonCandidateRmse, p.addedRmse);
end
