function [reportPath, result] = analyzeSameLabelSpatialAvailabilityV285(v284ResultPath)
% Output-only source headroom. No filter, packet action, gate or truth-fed policy.
loaded = load(v284ResultPath, 'result'); prior = loaded.result;
loaded = load(prior.rawPath, 'run'); run = loaded.run;
loaded = load(run.baselineTracePath, 'trace'); reference = loaded.trace;
assert(run.maximumTime == 40 && ~prior.screenPassed);
out = fullfile('RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v285', 'x36_same_label_spatial_availability_seed1301');
if exist(out, 'dir') ~= 7, mkdir(out); end
nSensors = numel(run.estimates); groups = reshape(run.config.sensorGroupIds, 1, []);
physical = rebuildPhysicalPrefix(reference.model.sensorTrajectories, run.config.commRange, 40);
assert(~any(run.attempted(:) & ~physical(:)));
% Registered CV model in generateMultisensorModel; the scenario factory does
% not override A, u or R. This is filter prediction, not a new truth generator.
A = [eye(2), eye(2); zeros(2), eye(2)];
R = [eye(2)/3, eye(2)/2; eye(2)/2, eye(2)];
% Query: receiver, time, truth index, birth time/location, emitted index,
% current error, current selected-component position covariance trace.
queries = zeros(0, 8); residual = 0; fallbackCount = 0;
for n = 1:nSensors
    for t = 1:40
        truth = run.groundTruthRfs.x{t};
        [referenceMap, d1, f1] = matchMap(truth, reference.estimates{n}.mu{t}, ...
            prior.reference.rmseBySensorTime(n, t));
        [candidateMap, d2, f2] = matchMap(truth, run.estimates{n}.mu{t}, ...
            prior.candidate.rmseBySensorTime(n, t));
        residual = max([residual, d1, d2]); fallbackCount = fallbackCount + f1 + f2;
        added = find(referenceMap == 0 & candidateMap > 0);
        for target = reshape(added, 1, [])
            j = candidateMap(target); s = run.estimates{n};
            label = s.labels{t}(:, j);
            delta = s.mu{t}{j}(1:2) - truth{target}(1:2);
            covariance = s.Sigma{t}{j};
            queries(end+1, :) = [n, t, target, label', j, norm(delta), ...
                trace(covariance(1:2, 1:2))]; %#ok<AGROW>
        end
    end
end
assert(size(queries, 1) == 7850);
% Detail columns: query index, lag, pool, peer count, current error,
% oracle-with-retention error, minimum-trace error, oracle source,
% minimum-trace source, minimum-trace score, receiver trace, raw best-peer error.
details = nan(size(queries, 1) * 10, 12); cursor = 0;
for q = 1:size(queries, 1)
    receiver = queries(q, 1); t = queries(q, 2); target = queries(q, 3);
    label = queries(q, 4:5)'; selfError = queries(q, 7); selfScore = queries(q, 8);
    truthPosition = run.groundTruthRfs.x{t}{target}(1:2);
    for lag = 0:1
        sourceTime = t - lag;
        if sourceTime < 1, continue; end
        errors = nan(1, nSensors); scores = errors;
        for sender = 1:nSensors
            if sender == receiver, continue; end
            state = run.estimates{sender};
            j = find(all(bsxfun(@eq, state.labels{sourceTime}, label), 1), 1);
            if isempty(j), continue; end
            mu = state.mu{sourceTime}{j}; covariance = state.Sigma{sourceTime}{j};
            if lag == 1, mu = A * mu; covariance = A * covariance * A' + R; end
            errors(sender) = norm(mu(1:2) - truthPosition);
            scores(sender) = trace(covariance(1:2, 1:2));
        end
        available = isfinite(errors) & isfinite(scores);
        masks = [true(1, nSensors); groups == groups(receiver); ...
            reshape(logical(run.attempted(:, receiver, t)), 1, []); ...
            reshape(logical(run.delivered(:, receiver, t)), 1, []); ...
            reshape(physical(:, receiver, t), 1, [])];
        for poolId = 1:5
            peers = find(available & masks(poolId, :));
            % Retention is included in both summaries; ties retain the receiver.
            choices = [receiver, peers]; choiceErrors = [selfError, errors(peers)];
            choiceScores = [selfScore, scores(peers)];
            [best, oracleIndex] = min(choiceErrors);
            [score, scoreIndex] = min(choiceScores);
            rawBest = NaN;
            if ~isempty(peers), rawBest = min(errors(peers)); end
            cursor = cursor + 1;
            details(cursor, :) = [q, lag, poolId, numel(peers), selfError, best, ...
                choiceErrors(scoreIndex), choices(oracleIndex), choices(scoreIndex), ...
                score, selfScore, rawBest];
        end
    end
    if mod(q, 1500) == 0, fprintf('V285 output source queries: %d/%d.\n', q, size(queries, 1)); end
end
details = details(1:cursor, :);
% Summary: scope group (0 = all), lag, pool, queries, peer coverage,
% pooled receiver/oracle/min-trace RMSE, oracle-improving fraction,
% min-trace switching/improving/worsening fractions.
summary = zeros(0, 12);
for group = [0, unique(groups, 'stable')]
    for lag = 0:1
        for poolId = 1:5
            mask = details(:, 2) == lag & details(:, 3) == poolId;
            if group > 0
                receivers = queries(details(:, 1), 1);
                mask = mask & reshape(groups(receivers), [], 1) == group;
            end
            d = details(mask, :); receivers = queries(d(:, 1), 1);
            summary(end+1, :) = [group, lag, poolId, size(d, 1), mean(d(:, 4) > 0), ...
                sqrt(mean(d(:, 5).^2)), sqrt(mean(d(:, 6).^2)), sqrt(mean(d(:, 7).^2)), ...
                mean(d(:, 6) < d(:, 5) - 1e-10), mean(d(:, 9) ~= receivers), ...
                mean(d(:, 7) < d(:, 5) - 1e-10), mean(d(:, 7) > d(:, 5) + 1e-10)]; %#ok<AGROW>
        end
    end
end
result = struct('contractVersion', 'same-label-output-spatial-availability-v285-v2', ...
    'v284ResultPath', v284ResultPath, 'candidateGenerationGitCommit', run.generationGitCommit, ...
    'queries', queries, 'details', details, 'summary', summary, ...
    'maximumOfficialRmseResidual', residual, 'assignmentFallbackCount', fallbackCount, ...
    'A', A, 'R', R, 'physicalRange', run.config.commRange, ...
    'attemptedEdgesPhysical', true, 'developmentEvidenceOnly', true, 'newPolicyEvaluated', false);
save('-mat7-binary', fullfile(out, 'SAME_LABEL_SPATIAL_AVAILABILITY_V285.mat'), 'result');
writeCsv(fullfile(out, 'V285_SOURCE_AVAILABILITY_SUMMARY.csv'), summary);
reportPath = fullfile(out, 'SAME_LABEL_SPATIAL_AVAILABILITY_V285.md');
writeReport(reportPath, result);
fprintf('V285 complete: %d added-target queries, %d source-pool records; no filter or candidate run.\n', ...
    size(queries, 1), size(details, 1));
for lag = 0:1
    s = summary(summary(:, 1) == 0 & summary(:, 2) == lag & summary(:, 3) == 1, :);
    fprintf('lag %d global same-label pooled RMSE: receiver %.6f, oracle %.6f, min-trace %.6f.\n', lag, s(6:8));
end
end

function physical = rebuildPhysicalPrefix(trajectories, range, timeCount)
% Exactly the distance/range rule in buildDynamicTopologyGraphs/buildPhysicalGraph.
n = numel(trajectories); physical = false(n, n, timeCount);
for t = 1:timeCount
    for left = 1:n-1
        for right = left+1:n
            distance = norm(trajectories{left}(1:2, t) - trajectories{right}(1:2, t));
            connected = ~isfinite(range) || distance <= range;
            physical(left, right, t) = connected; physical(right, left, t) = connected;
        end
    end
end
end

function [map, residual, fallback] = matchMap(truth, estimates, expected)
map = zeros(numel(truth), 1); residual = 0; fallback = 0;
if isempty(truth) || isempty(estimates), return; end
x = cell2mat(truth); y = cell2mat(estimates);
cost = hypot(bsxfun(@minus, x(1, :)', y(1, :)), bsxfun(@minus, x(2, :)', y(2, :)));
[matching, ~] = munkres(cost);
rmse = sqrt(mean(cost(matching).^2));
if abs(rmse - expected) > 1e-8 * max(1, expected)
    [matching, ~] = Hungarian(cost); matching = logical(matching); fallback = 1;
    rmse = sqrt(mean(cost(matching).^2));
end
residual = abs(rmse - expected);
assert(isfinite(residual) && residual <= 1e-8 * max(1, expected));
[i, j] = find(matching); map(i) = j;
end

function writeCsv(path, data)
fid = fopen(path, 'w'); assert(fid >= 0); cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'formation,lag,pool,query_count,peer_coverage,receiver_pooled_rmse,oracle_pooled_rmse,min_trace_pooled_rmse,oracle_improving_fraction,min_trace_switch_fraction,min_trace_improving_fraction,min_trace_worsening_fraction\n');
for i = 1:size(data, 1)
    fprintf(fid, '%d,%d,%d,%d,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n', data(i, :));
end
end

function writeReport(path, r)
fid = fopen(path, 'w'); assert(fid >= 0); cleanup = onCleanup(@() fclose(fid));
names = {'All network peers', 'Same formation', 'Planned incoming peers', ...
    'Delivered incoming peers', 'Physical one-hop peers'};
fprintf(fid, '# V285: same-label spatial information in saved X36 outputs\n\n');
fprintf(fid, 'Output-level diagnostic / optimistic bound, self-check only. No filter run, new message, policy evaluation or joint-gate revision. All %d added geometrically matched target cases are retained.\n\n', size(r.queries, 1));
fprintf(fid, 'Lag 0 uses current end-of-round outputs and cannot be inserted into the same round before they exist. Lag 1 predicts the previous round with the registered 4D one-step CV model; it does not include the next measurement or simulate packet fusion. Pools use sender-to-receiver attempted/delivered masks.\n\n');
fprintf(fid, 'The physical one-hop pool is reconstructed from the saved sensor trajectories with the registered %.0f m range rule, not from selected edges. Every attempted edge lies inside it. Unselected physical edges have not been charged or subjected to a new packet-delivery trial.\n\n', r.physicalRange);
fprintf(fid, 'Oracle selects by truth error and may retain the receiver. Min-trace selects only by the emitted component covariance trace (ties retain receiver); source selection excludes truth, but the query set, receiver snapshot timing and uncharged source synopsis make this a diagnostic, not a deployable controller.\n\n');
fprintf(fid, '| Lag | Source pool | Cases | Peer available (%%) | Receiver pooled RMSE | Oracle pooled RMSE | Min-trace pooled RMSE | Min-trace worsens (%%) |\n| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |\n');
s = r.summary(r.summary(:, 1) == 0, :);
for k = 1:size(s, 1)
    fprintf(fid, '| %d | %s | %d | %.2f | %.6f | %.6f | %.6f | %.2f |\n', ...
        s(k, 2), names{s(k, 3)}, s(k, 4), 100*s(k, 5), s(k, 6:8), 100*s(k, 12));
end
fprintf(fid, '\n## Formation-level global-pool comparison\n\n');
fprintf(fid, '| Formation | Lag | Cases | Receiver pooled RMSE | Oracle pooled RMSE | Min-trace pooled RMSE |\n| --- | --- | ---: | ---: | ---: | ---: |\n');
s = r.summary(r.summary(:, 1) > 0 & r.summary(:, 3) == 1, :);
for k = 1:size(s, 1)
    fprintf(fid, '| %d | %d | %d | %.6f | %.6f | %.6f |\n', s(k, 1:2), s(k, 4), s(k, 6:8));
end
fprintf(fid, '\nThe quantities are pooled sqrt(mean squared matched-position error), not official mean per-cell RMSE, OSPA or a counterfactual tracking trajectory. Cases repeat targets across nodes/time and are not independent samples. Same labels are required; no truth-based cross-label association is used.\n\n');
fprintf(fid, 'Only emitted component means/covariances are inspected. Hidden components and non-emitted labels may contain other information. A favorable oracle does not show calibrated, available source selection; an unfavorable output comparison does not prove the full network posterior has no useful information.\n\n');
fprintf(fid, 'Assignment readback: maximum discrepancy from saved official per-cell RMSE %.3g m; original-solver fallbacks %d. Full records remain in the local MAT; summary CSV preserves all formation/time-layer/source-pool aggregates.\n', ...
    r.maximumOfficialRmseResidual, r.assignmentFallbackCount);
end
