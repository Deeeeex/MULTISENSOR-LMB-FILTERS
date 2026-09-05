function [reportPath, result] = analyzeFullMixtureSourceRiskV286(v285ResultPath)
% Final saved-snapshot source-ranking diagnostic. No filter/candidate execution.
loaded = load(v285ResultPath, 'result'); previous = loaded.result;
loaded = load(previous.v284ResultPath, 'result'); v284 = loaded.result;
loaded = load(v284.rawPath, 'run'); run = loaded.run;
loaded = load(run.baselineTracePath, 'trace'); reference = loaded.trace;
t = run.maximumTime; assert(t == 40);
queries = previous.queries(previous.queries(:, 2) == t, :);
nSensors = numel(run.estimates); groups = reshape(run.config.sensorGroupIds, 1, []);
out = fullfile('RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v286', 'x36_final_snapshot_source_risk_seed1301');
if exist(out, 'dir') ~= 7, mkdir(out); end
cache = cell(1, nSensors); maximumResidual = 0; archivedExcluded = 0;
% Cached columns: selected-component trace, full GM output risk, within,
% between, output/mixture-mean offset, GM count, existence probability.
for n = 1:nSensors
    state = run.estimates{n}; cached = nan(numel(state.mu{t}), 7);
    objects = state.objects;
    current = false(1, numel(objects));
    for k = 1:numel(objects)
        length = objects(k).trajectoryLength;
        current(k) = length > 0 && objects(k).timestamps(length) == t;
    end
    archivedExcluded = archivedExcluded + sum(~current);
    for j = 1:numel(state.mu{t})
        label = state.labels{t}(:, j);
        match = find(current & [objects.birthTime] == label(1) & ...
            [objects.birthLocation] == label(2));
        assert(numel(match) == 1);
        object = objects(match); mu = state.mu{t}{j}; covariance = state.Sigma{t}{j};
        matchingComponent = false(1, object.numberOfGmComponents);
        for c = 1:object.numberOfGmComponents
            matchingComponent(c) = norm(object.mu{c} - mu) < 1e-9 && ...
                norm(object.Sigma{c} - covariance, 'fro') < 1e-9;
        end
        assert(any(matchingComponent));
        [risk, parts] = computeGmOutputPositionRisk(object, mu(1:2));
        maximumResidual = max(maximumResidual, parts.decompositionResidual);
        cached(j, :) = [trace(covariance(1:2, 1:2)), risk, parts.withinComponent, ...
            parts.betweenComponents, parts.outputMeanOffset, object.numberOfGmComponents, object.r];
    end
    cache{n} = cached;
end
physical = false(nSensors);
for left = 1:nSensors-1
    for right = left+1:nSensors
        connected = norm(reference.model.sensorTrajectories{left}(1:2, t) - ...
            reference.model.sensorTrajectories{right}(1:2, t)) <= run.config.commRange;
        physical(left, right) = connected; physical(right, left) = connected;
    end
end
% Detail: query, pool, choices, receiver error, oracle error, min-trace error,
% min-full-risk error, oracle sender, min-trace sender, min-full-risk sender.
details = nan(size(queries, 1) * 5, 10); cursor = 0;
% Source records repeat senders across receiver queries; they are not IID.
% Columns: query, receiver, source, source formation, actual squared error,
% component trace, full output risk, within, between, mean offset, GM count, r.
sourceRecords = zeros(0, 12);
for q = 1:size(queries, 1)
    receiver = queries(q, 1); label = queries(q, 4:5)';
    truth = run.groundTruthRfs.x{t}{queries(q, 3)}(1:2);
    errors = nan(1, nSensors); traces = errors; risks = errors;
    for sender = 1:nSensors
        state = run.estimates{sender};
        j = find(all(bsxfun(@eq, state.labels{t}, label), 1));
        if isempty(j), continue; end
        assert(numel(j) == 1);
        errors(sender) = norm(state.mu{t}{j}(1:2) - truth);
        traces(sender) = cache{sender}(j, 1); risks(sender) = cache{sender}(j, 2);
        sourceRecords(end+1, :) = [q, receiver, sender, groups(sender), ...
            errors(sender)^2, cache{sender}(j, :)]; %#ok<AGROW>
    end
    assert(abs(errors(receiver) - queries(q, 7)) < 1e-9);
    masks = [true(1, nSensors); groups == groups(receiver); ...
        reshape(logical(run.attempted(:, receiver, t)), 1, []); ...
        reshape(logical(run.delivered(:, receiver, t)), 1, []); ...
        reshape(physical(:, receiver), 1, [])];
    for pool = 1:5
        available = find(isfinite(errors) & masks(pool, :) & (1:nSensors) ~= receiver);
        choices = [receiver, available]; % Retain receiver on exact score ties.
        [~, oracle] = min(errors(choices));
        [~, component] = min(traces(choices));
        [~, full] = min(risks(choices));
        cursor = cursor + 1;
        details(cursor, :) = [q, pool, numel(choices), errors(receiver), ...
            errors(choices(oracle)), errors(choices(component)), errors(choices(full)), ...
            choices(oracle), choices(component), choices(full)];
    end
end
% Summary: formation, pool, queries, receiver/oracle/component/full pooled RMSE,
% component/full worsening fraction, full switching fraction.
summary = zeros(0, 10);
for group = [0, unique(groups, 'stable')]
    for pool = 1:5
        mask = details(:, 2) == pool;
        if group > 0
            mask = mask & reshape(groups(queries(details(:, 1), 1)), [], 1) == group;
        end
        d = details(mask, :); receivers = queries(d(:, 1), 1);
        summary(end+1, :) = [group, pool, size(d, 1), sqrt(mean(d(:, 4:7).^2, 1)), ...
            mean(d(:, 6) > d(:, 4) + 1e-10), mean(d(:, 7) > d(:, 4) + 1e-10), ...
            mean(d(:, 10) ~= receivers)]; %#ok<AGROW>
    end
end
% First retain one unique receiver/sender/truth record. Also emit global
% source calibration summary, explicitly descriptive and not independent data.
calibration = zeros(0, 8);
for group = [0, unique(groups, 'stable')]
    mask = true(size(sourceRecords, 1), 1);
    if group > 0, mask = sourceRecords(:, 4) == group; end
    s = sourceRecords(mask, :);
    % Same source and same truth recur across receiver queries: collapse them.
    truthIds = queries(s(:, 1), 3);
    [~, keep] = unique([s(:, 3), truthIds], 'rows'); s = s(keep, :);
    componentCorr = corr(log1p(s(:, 6)), log1p(s(:, 5)));
    fullCorr = corr(log1p(s(:, 7)), log1p(s(:, 5)));
    calibration(end+1, :) = [group, size(s, 1), mean(s(:, 5)), mean(s(:, 6)), ...
        mean(s(:, 7)), componentCorr, fullCorr, mean(s(:, 11) > 1)]; %#ok<AGROW>
end
result = struct('contractVersion', 'full-mixture-output-source-risk-v286-v1', ...
    'v285ResultPath', v285ResultPath, 'v284RawPath', v284.rawPath, 'time', t, ...
    'queries', queries, 'details', details, 'sourceRecords', sourceRecords, ...
    'summary', summary, 'calibration', calibration, ...
    'maximumRiskDecompositionResidual', maximumResidual, ...
    'archivedObjectsExcluded', archivedExcluded, 'cachedEmittedCount', sum(cellfun(@(x) size(x, 1), cache)), ...
    'newPolicyEvaluated', false, 'developmentEvidenceOnly', true);
save('-mat7-binary', fullfile(out, 'FULL_MIXTURE_SOURCE_RISK_V286.mat'), 'result');
writeCsv(fullfile(out, 'V286_SOURCE_RISK_SUMMARY.csv'), ...
    'formation,pool,query_count,receiver_pooled_rmse,oracle_pooled_rmse,min_component_trace_pooled_rmse,min_full_risk_pooled_rmse,component_worsening_fraction,full_risk_worsening_fraction,full_risk_switching_fraction', summary);
writeCsv(fullfile(out, 'V286_SOURCE_RISK_CALIBRATION.csv'), ...
    'source_formation,source_truth_pairs,mean_actual_squared_error,mean_component_trace,mean_full_output_risk,log_component_risk_error_correlation,log_full_risk_error_correlation,multicomponent_fraction', calibration);
reportPath = fullfile(out, 'FULL_MIXTURE_SOURCE_RISK_V286.md');
writeReport(reportPath, result);
s = summary(summary(:, 1) == 0 & summary(:, 2) == 1, :);
fprintf('V286 complete: %d final-snapshot queries, %d emitted full GM objects; no filter or candidate run.\n', ...
    size(queries, 1), result.cachedEmittedCount);
fprintf('global pooled RMSE: receiver %.6f, oracle %.6f, min-component %.6f, min-full-risk %.6f.\n', s(4:7));
fprintf('risk decomposition residual %.3g; full-risk worsening fraction %.4f.\n', maximumResidual, s(9));
end

function writeCsv(path, header, data)
fid = fopen(path, 'w'); assert(fid >= 0); cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s\n', header);
for k = 1:size(data, 1)
    fprintf(fid, '%.12g', data(k, 1)); fprintf(fid, ',%.12g', data(k, 2:end)); fprintf(fid, '\n');
end
end

function writeReport(path, r)
fid = fopen(path, 'w'); assert(fid >= 0); cleanup = onCleanup(@() fclose(fid));
names = {'Global', 'Same formation', 'Planned incoming', 'Delivered incoming', 'Physical one-hop'};
fprintf(fid, '# V286: full-mixture output-risk source ranking\n\n');
fprintf(fid, 'Final time %d, X36 seed 1301, %d added-target snapshot queries. Diagnostic only / self-check only. No filter, routing action, packet cost or official metric changed.\n\n', r.time, size(r.queries, 1));
fprintf(fid, 'For output a, risk = sum_c w_c [tr(P_c,position) + ||mu_c,position-a||^2]. This exact posterior expectation includes all GM modes; it is not a theorem about truth error or a new method. The retained self output and peer outputs are all from the same final snapshot; this is not an online decision.\n\n');
fprintf(fid, 'Full objects match emitted label, timestamp at trajectoryLength (timestamps are preallocated), and a complete emitted mean/covariance pair. All %d emitted objects are represented; %d archived objects were excluded.\n\n', r.cachedEmittedCount, r.archivedObjectsExcluded);
fprintf(fid, '| Pool | Cases | Receiver | Truth oracle | Min-component trace | Min-full risk | Full risk worsens (%%) |\n| --- | ---: | ---: | ---: | ---: | ---: | ---: |\n');
s = r.summary(r.summary(:, 1) == 0, :);
for k = 1:size(s, 1)
    fprintf(fid, '| %s | %d | %.6f | %.6f | %.6f | %.6f | %.2f |\n', names{s(k, 2)}, s(k, 3:7), 100*s(k, 9));
end
fprintf(fid, '\nAll displayed errors are pooled matched-position RMSE (m), not official average per-cell RMSE. Truth is used only for query construction, the explicitly privileged oracle, and evaluation; the two risk selectors never read truth. All selectors may retain self.\n\n');
fprintf(fid, '| Formation (global pool) | Cases | Receiver | Truth oracle | Min-component trace | Min-full risk | Full risk worsens (%%) |\n| --- | ---: | ---: | ---: | ---: | ---: | ---: |\n');
s = r.summary(r.summary(:, 1) > 0 & r.summary(:, 2) == 1, :);
for k = 1:size(s, 1)
    fprintf(fid, '| %d | %d | %.6f | %.6f | %.6f | %.6f | %.2f |\n', s(k, 1), s(k, 3:7), 100*s(k, 9));
end
fprintf(fid, '\n## Descriptive risk/error check\n\n');
fprintf(fid, 'Source-truth pairs are collapsed within the snapshot to avoid counting the same sender state repeatedly for multiple receiver queries. They still share labels, measurements and previous fusion, so correlations are descriptive, not independent calibration evidence. The entire GM spatial distribution is conditional on existence; this score does not evaluate missed/false targets or recursive set effects.\n\n');
fprintf(fid, '| Source formation | Pairs | Actual MSE | Component trace | Full risk | Log-risk/error correlation: component / full | Multi-GM (%%) |\n| --- | ---: | ---: | ---: | ---: | ---: | ---: |\n');
for k = 1:size(r.calibration, 1)
    s = r.calibration(k, :);
    fprintf(fid, '| %d | %d | %.3f | %.3f | %.3f | %.3f / %.3f | %.1f |\n', s(1:7), 100*s(8));
end
fprintf(fid, '\nMaximum algebraic risk-decomposition residual %.3g. No past full-mixture snapshots were inferred from trajectory means; no aged or full-episode claim is supported. A favorable snapshot score would still require actual causal transport, charged metadata, losses, and static-routing comparison under identical fusion.\n', r.maximumRiskDecompositionResidual);
end
