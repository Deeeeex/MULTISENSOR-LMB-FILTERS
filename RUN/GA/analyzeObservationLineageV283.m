function [reportPath, result] = analyzeObservationLineageV283(tracePath)
% Trace whether any observation opportunity can reach each actual input.
% A negative observation is informative too. This is not detection provenance,
% true-target recall, or a counterfactual tracking policy.
loaded = load(tracePath, 'trace'); trace = loaded.trace;
out = fileparts(tracePath);
loaded = load(fullfile(out, 'EXISTENCE_STAGE_ANALYSIS_V282.mat'), 'result');
stages = loaded.result;
assert(stages.referencePrefixMatches);
labelRows = stages.labels;
labels = unique([[labelRows.birthTime]', [labelRows.birthLocation]'], 'rows');
K = size(labels, 1); n = numel(trace.estimates); T = trace.maximumTime;
[~, labelIndex] = ismember( ...
    [[labelRows.birthTime]', [labelRows.birthLocation]'], labels, 'rows');
cellIndex = sub2ind([n, K, T], [labelRows.receiver]', ...
    labelIndex, [labelRows.time]');
pd = nan(n, K, T); localR = nan(n, K, T); predictedR = nan(n, K, T);
pd(cellIndex) = [labelRows.predictedMeanPd];
localR(cellIndex) = [labelRows.localExistence];
predictedR(cellIndex) = [labelRows.predictedExistence];
previous = false(n, K); perTime = repmat(struct(), 1, 0);
for t = 1:T
    % Pruned/nonexistent prior states do not retain a lineage flag.
    localInformed = (previous & isfinite(predictedR(:, :, t))) | pd(:, :, t) > 0;
    localInformed = localInformed & isfinite(localR(:, :, t));
    fusedInformed = false(n, K);
    nonemptyPayload = false(1, n);
    payloadHasLabel = false(n, K);
    for j = 1:n
        o = trace.local{t}{j};
        activeObjects = o([o.r] > 0.01 & [o.numberOfGmComponents] > 0);
        nonemptyPayload(j) = ~isempty(activeObjects);
        if ~isempty(activeObjects)
            [~, indices] = ismember([[activeObjects.birthTime]', ...
                [activeObjects.birthLocation]'], labels, 'rows');
            payloadHasLabel(j, indices) = true;
        end
    end
    a = struct('time', t, 'labelFusionCount', 0, ...
        'uninformedWeightSum', 0, 'noCurrentOpportunityWeightSum', 0, ...
        'negativeLogOddsMagnitude', 0, 'uninformedNegativeMagnitude', 0, ...
        'weakBeforeSpatialCount', 0, 'strongInputWeakPoolCount', 0, ...
        'strongWeakWithUninformedCount', 0, ...
        'allInputsUninformedCount', 0, 'localLabelCount', 0, ...
        'localUninformedLabelCount', 0, 'retainedLabelCount', 0, ...
        'retainedInformedLabelCount', 0);
    for i = 1:n
        senders = find(reshape(trace.delivered(:, i, t), 1, []) & nonemptyPayload);
        senders = senders(senders ~= i); sources = [i, senders];
        records = trace.kla{t}{i}.labelKlaRecords;
        for k = 1:numel(records)
            r = records(k);
            assert(r.sourceCount == numel(sources));
            ell = find(all(bsxfun(@eq, labels, reshape(r.label, 1, [])), 2), 1);
            w = r.activeExistenceWeights;
            active = r.existenceParticipating & w > 1e-12;
            sourceLocalR = reshape(localR(sources, ell, t), 1, []);
            sourcePresent = [isfinite(sourceLocalR(1)), ...
                reshape(payloadHasLabel(senders, ell), 1, [])];
            informed = reshape(localInformed(sources, ell), 1, []);
            % Active absent inputs are the existing observable-absence
            % censored evidence, not a never-observed birth prior.
            informed(active & ~sourcePresent) = true;
            noCurrent = reshape(pd(sources, ell, t), 1, []) == 0;
            pure = active & ~informed;
            probabilities = min(max(r.inputExistence(active), 1e-12), 1-1e-12);
            negative = max(0, -(log(probabilities)-log1p(-probabilities)));
            a.labelFusionCount = a.labelFusionCount + 1;
            a.uninformedWeightSum = a.uninformedWeightSum + sum(w(pure));
            a.noCurrentOpportunityWeightSum = a.noCurrentOpportunityWeightSum + sum(w(active & noCurrent));
            a.negativeLogOddsMagnitude = a.negativeLogOddsMagnitude + sum(w(active).*negative);
            a.uninformedNegativeMagnitude = a.uninformedNegativeMagnitude + ...
                sum(w(active).*negative.*(~informed(active)));
            weak = r.weightedInputLogOdds < 0;
            strongWeak = weak && any(r.inputExistence(active) >= 0.9);
            a.weakBeforeSpatialCount = a.weakBeforeSpatialCount + weak;
            a.strongInputWeakPoolCount = a.strongInputWeakPoolCount + strongWeak;
            a.strongWeakWithUninformedCount = a.strongWeakWithUninformedCount + ...
                (strongWeak && any(pure));
            a.allInputsUninformedCount = a.allInputsUninformedCount + ~any(active & informed);
            if r.fusedExistence > 0.01
                fusedInformed(i, ell) = any(active & informed);
                a.retainedLabelCount = a.retainedLabelCount + 1;
                a.retainedInformedLabelCount = a.retainedInformedLabelCount + fusedInformed(i, ell);
            end
        end
    end
    a.localLabelCount = nnz(isfinite(localR(:, :, t)));
    a.localUninformedLabelCount = nnz(isfinite(localR(:, :, t)) & ~localInformed);
    perTime(end+1) = a; %#ok<AGROW>
    previous = fusedInformed;
end
[~, commit] = system('git rev-parse HEAD');
result = struct('analysis', 'observation-opportunity-lineage-v283-v1', ...
    'sourceCommit', strtrim(commit), 'tracePath', tracePath, ...
    'traceGenerationGitCommit', trace.generationGitCommit, ...
    'presetName', trace.presetName, 'seed', trace.seed, ...
    'maximumTime', T, 'perTime', perTime, 'newPolicyEvaluated', false, ...
    'actualDetectionLineage', false, 'independentValidation', false);
save('-mat7-binary', fullfile(out, 'OBSERVATION_LINEAGE_V283.mat'), 'result');
writeCsv(fullfile(out, 'observation_lineage_by_time.csv'), perTime);
reportPath = fullfile(out, 'OBSERVATION_LINEAGE_V283.md');
fid = fopen(reportPath, 'w'); assert(fid >= 0);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V283 observation-opportunity lineage\n\n');
fprintf(fid, '%s, seed %d, unchanged reference steps 1--%d. No filter rerun or candidate evaluated.\n\n', ...
    trace.presetName, trace.seed, T);
fprintf(fid, ['An input is marked informed if a component-mean observation opportunity ', ...
    '(pD>0, including missed detections) occurred locally or reached it through ', ...
    'an actually used positive-weight input. Observable-absence censoring counts ', ...
    'as evidence. Pruned states lose their flag. This traces an opportunity ', ...
    'path, not a confirmed detection or evidence magnitude.\n\n']);
fprintf(fid, '| Steps | Mean never-informed input weight | Mean no-current-opportunity weight | Never-informed share of negative log odds | Weak pools with a strong input | Of those: any never-informed input |\n');
fprintf(fid, '|:--|--:|--:|--:|--:|--:|\n');
for first = 1:5:T
    last = min(first+4, T); a = perTime(first:last);
    count = sum([a.labelFusionCount]);
    fprintf(fid, '| %d--%d | %.4f | %.4f | %.3f%% | %d | %d |\n', ...
        first, last, sum([a.uninformedWeightSum])/count, ...
        sum([a.noCurrentOpportunityWeightSum])/count, ...
        100*sum([a.uninformedNegativeMagnitude])/max(sum([a.negativeLogOddsMagnitude]), eps), ...
        sum([a.strongInputWeakPoolCount]), sum([a.strongWeakWithUninformedCount]));
end
fprintf(fid, ['\nA weak pool means weighted input log odds below zero, before spatial ', ...
    'overlap; a strong input means existence at least 0.9. These are descriptive ', ...
    'levels, not the MAP-cardinality extraction rule or a tuning grid. ', ...
    'A historical opportunity flag does not quantify retained information and ', ...
    'cannot distinguish detection from missed detection. Thus low never-informed ', ...
    'mass rules out a pure birth-prior-only explanation in this trace, but ', ...
    'does not rule out attenuation of weak historical evidence or delayed ', ...
    'fresh measurements. Neither geometry-only censoring nor a new fusion ', ...
    'rule is validated here.\n']);
fprintf('V283 complete: %s\n', reportPath);
end

function writeCsv(path, rows)
fid = fopen(path, 'w'); assert(fid >= 0);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
names = fieldnames(rows); fprintf(fid, '%s\n', strjoin(names, ','));
for k = 1:numel(rows)
    values = cell(1, numel(names));
    for j = 1:numel(names), values{j} = sprintf('%.15g', rows(k).(names{j})); end
    fprintf(fid, '%s\n', strjoin(values, ','));
end
end
