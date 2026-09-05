function [reportPath, result] = analyzeExistenceStageTraceV282(tracePath)
% Offline stage accounting only: label mass is not true-target recall.
loaded = load(tracePath, 'trace'); trace = loaded.trace;
loaded = load(trace.referenceResultPath, 'result'); reference = loaded.result.candidate;
out = fileparts(tracePath); T = trace.maximumTime;
n = numel(trace.estimates); groups = trace.config.sensorGroupIds;
receiverRows = cell(1, n*T); labelRows = cell(1, n*T);
eospa = zeros(n, T); rmse = nan(n, T);
for i = 1:n
    rmse(i, :) = computeSetRmseOverTime( ...
        trace.estimates{i}, trace.groundTruthRfs);
end
for t = 1:T
    for i = 1:n
        p = trace.predicted{t}{i}; l = trace.local{t}{i};
        f = trace.fused{t}{i}; diagnostic = trace.kla{t}{i};
        records = diagnostic.labelKlaRecords;
        preEta = logistic([records.weightedInputLogOdds]);
        retained = f([f.r] > trace.model.existenceThreshold);
        localRetained = l([l.r] > trace.model.existenceThreshold);
        [localMap, ~] = mapReadout(localRetained);
        outputCount = numel(trace.estimates{i}.mu{t});
        [~, selected] = mapReadout(retained);
        selectedLabels = labelsOf(retained(selected));
        a = struct('time', t, 'receiver', i, 'formation', groups(i), ...
            'predictedLabelCount', numel(p), 'localLabelCount', numel(l), ...
            'fusedLabelCount', numel(f), ...
            'predictedMass', sum([p.r]), 'localMass', sum([l.r]), ...
            'localUpdateMassChange', sum([l.r])-sum([p.r]), ...
            'preEtaMass', sum(preEta), 'fusedMass', sum([f.r]), ...
            'poolingMassChangeFromLocal', sum(preEta)-sum([l.r]), ...
            'spatialMassLoss', sum(preEta)-sum([f.r]), ...
            'prunedMass', sum([f.r])-sum([retained.r]), ...
            'retainedNotOutputMass', sum([retained.r])-sum([retained(selected).r]), ...
            'localMapCount', localMap, 'outputMapCount', outputCount, ...
            'zeroPdLabelCount', 0, 'zeroPdMaxAbsLocalChange', 0, ...
            'weakBeforeSpatialCount', nnz(preEta < 0.5), ...
            'normalizerFallbackCount', 0);
        labels = unique([labelsOf(p); labelsOf(l); labelsOf(f)], 'rows');
        rows = repmat(struct(), 1, 0);
        for k = 1:size(labels, 1)
            label = labels(k, :);
            ip = findLabel(p, label); il = findLabel(l, label);
            jf = findRecord(records, label);
            rp = existence(p, ip); rl = existence(l, il);
            pd = NaN;
            if ~isempty(ip), pd = expectedPd(trace.model, i, p(ip), t); end
            row = struct('time', t, 'receiver', i, 'formation', groups(i), ...
                'birthTime', label(1), 'birthLocation', label(2), ...
                'predictedExistence', rp, 'localExistence', rl, ...
                'predictedMeanPd', pd, 'localExistenceChange', rl-rp, ...
                'localLogOddsChange', logOdds(rl)-logOdds(rp), ...
                'weightedInputLogOdds', NaN, 'spatialLogNormalizer', NaN, ...
                'preEtaExistence', NaN, 'fusedExistence', NaN, ...
                'activeExistenceSourceCount', 0, 'strongInputPresent', false, ...
                'selectedForOutput', ismember(label, selectedLabels, 'rows'), ...
                'mixtureUsed', false, 'normalizerFallback', false);
            if ~isempty(jf)
                r = records(jf);
                row.weightedInputLogOdds = r.weightedInputLogOdds;
                row.spatialLogNormalizer = r.spatialLogNormalizer;
                row.preEtaExistence = logistic(r.weightedInputLogOdds);
                row.fusedExistence = r.fusedExistence;
                active = logical(r.existenceParticipating);
                row.activeExistenceSourceCount = nnz(active);
                row.strongInputPresent = any(r.inputExistence(active) >= 0.9);
                row.mixtureUsed = r.mixtureAwareSpatialUsed;
                row.normalizerFallback = strcmp( ...
                    r.mixtureAwareFallbackReason, 'powered-gm-normalizer-exceeds-one');
                a.normalizerFallbackCount = a.normalizerFallbackCount + row.normalizerFallback;
            end
            if pd == 0 && isfinite(rl-rp)
                a.zeroPdLabelCount = a.zeroPdLabelCount + 1;
                a.zeroPdMaxAbsLocalChange = max(a.zeroPdMaxAbsLocalChange, abs(rl-rp));
            end
            rows(end+1) = row; %#ok<AGROW>
        end
        receiverRows{(t-1)*n+i} = a; labelRows{(t-1)*n+i} = rows;
        metric = computePositionEuclideanOspa( ...
            trace.groundTruthRfs.x{t}, trace.estimates{i}.mu{t}, ...
            trace.config.ospaPositionCutoff, 2, [1, 2]);
        eospa(i, t) = metric(1);
    end
end
oldE = reference.positionEospaBySensorTime(:, 1:T);
oldR = reference.positionRmseBySensorTime(:, 1:T);
finite = isfinite(rmse) & isfinite(oldR);
rmseDifference = abs(rmse(finite)-oldR(finite));
if isempty(rmseDifference), maxRmseDifference = 0; else, maxRmseDifference = max(rmseDifference); end
result = struct('analysis', 'existence-stage-trace-v282-v1', ...
    'tracePath', tracePath, 'sourceCommit', trace.generationGitCommit, ...
    'presetName', trace.presetName, 'seed', trace.seed, ...
    'maximumTime', T, 'originalTimeCount', trace.originalTimeCount, ...
    'elapsedSeconds', trace.elapsedSeconds, ...
    'maxAbsoluteEospaDifference', max(abs(eospa(:)-oldE(:))), ...
    'rmseFiniteMaskMatches', isequal(isfinite(rmse), isfinite(oldR)), ...
    'maxAbsoluteFiniteRmseDifference', maxRmseDifference, ...
    'receivers', [receiverRows{:}], 'labels', [labelRows{:}], ...
    'newPolicyEvaluated', false, 'independentValidation', false);
result.referencePrefixMatches = result.maxAbsoluteEospaDifference <= 1e-9 && ...
    result.rmseFiniteMaskMatches && result.maxAbsoluteFiniteRmseDifference <= 1e-9;
save('-mat7-binary', fullfile(out, 'EXISTENCE_STAGE_ANALYSIS_V282.mat'), 'result');
writeCsv(fullfile(out, 'receiver_stages.csv'), result.receivers);
writeCsv(fullfile(out, 'label_stages.csv'), result.labels);
reportPath = fullfile(out, 'EXISTENCE_STAGE_TRACE_V282.md');
writeReport(reportPath, result);
fprintf('V282 analysis: %d receiver stages, %d label stages; saved-prefix match=%d, max E/RMSE differences %.3g/%.3g.\n', ...
    numel(result.receivers), numel(result.labels), result.referencePrefixMatches, ...
    result.maxAbsoluteEospaDifference, result.maxAbsoluteFiniteRmseDifference);
end

function pd = expectedPd(model, sensor, object, t)
w = reshape(object.w, 1, []); w = w/sum(w);
pd = 0;
for k = 1:object.numberOfGmComponents
    pd = pd + w(k)*evaluateSensorQuality(model, sensor, object.mu{k}, t);
end
end

function labels = labelsOf(o)
labels = zeros(0, 2);
if ~isempty(o), labels = [reshape([o.birthTime], [], 1), reshape([o.birthLocation], [], 1)]; end
end

function k = findLabel(o, label)
labels = labelsOf(o); k = find(all(bsxfun(@eq, labels, label), 2), 1);
end

function k = findRecord(records, label)
k = [];
for j = 1:numel(records)
    if isequal(reshape(records(j).label, 1, []), label), k = j; return; end
end
end

function r = existence(objects, k)
if isempty(k), r = NaN; else, r = objects(k).r; end
end

function [count, selected] = mapReadout(objects)
count = 0; selected = [];
if ~isempty(objects), [count, selected] = lmbMapCardinalityEstimate([objects.r]); end
end

function y = logOdds(r)
if isnan(r), y = NaN; return; end
r = min(max(r, 1e-12), 1-1e-12); y = log(r)-log1p(-r);
end

function y = logistic(x)
y = zeros(size(x)); positive = x >= 0;
y(positive) = 1./(1+exp(-x(positive)));
z = exp(x(~positive)); y(~positive) = z./(1+z);
end

function writeCsv(path, rows)
fid = fopen(path, 'w'); assert(fid >= 0); cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
names = fieldnames(rows);
fprintf(fid, '%s\n', strjoin(names, ','));
for k = 1:numel(rows)
    cells = cell(1, numel(names));
    for j = 1:numel(names), cells{j} = sprintf('%.15g', rows(k).(names{j})); end
    fprintf(fid, '%s\n', strjoin(cells, ','));
end
end

function writeReport(path, result)
fid = fopen(path, 'w'); assert(fid >= 0); cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V282 early existence-stage trace\n\n');
fprintf(fid, '%s, seed %d, steps 1--%d of the original %d-step scene. Runtime %.1f s. Source `%s`.\n\n', ...
    result.presetName, result.seed, result.maximumTime, result.originalTimeCount, ...
    result.elapsedSeconds, result.sourceCommit);
fprintf(fid, 'Unchanged-reference prefix correspondence: `%d`; max E-OSPA difference %.3g, finite RMSE difference %.3g, finite-mask match `%d`. This is not full-state equivalence or a new policy gain.\n\n', ...
    result.referencePrefixMatches, result.maxAbsoluteEospaDifference, ...
    result.maxAbsoluteFiniteRmseDifference, result.rmseFiniteMaskMatches);
fprintf(fid, '| Time | Predicted mass | Local mass | Pre-spatial pooled mass | Fused mass | Local MAP count | Output MAP count |\n');
fprintf(fid, '|--:|--:|--:|--:|--:|--:|--:|\n');
for t = 1:result.maximumTime
    a = result.receivers([result.receivers.time] == t);
    fprintf(fid, '| %d | %.6f | %.6f | %.6f | %.6f | %.3f | %.3f |\n', ...
        t, mean([a.predictedMass]), mean([a.localMass]), mean([a.preEtaMass]), ...
        mean([a.fusedMass]), mean([a.localMapCount]), mean([a.outputMapCount]));
end
a = result.receivers;
fprintf(fid, '\nZero expected-pD label stages: %d; max absolute local existence change there: %.6g. pD is evaluated at predicted mixture-component means, matching the existing local-update approximation.\n', ...
    sum([a.zeroPdLabelCount]), max([a.zeroPdMaxAbsLocalChange]));
fprintf(fid, ['\nThe columns are averages over receivers, not true-target recall. ', ...
    'The predicted state precedes the current measurement update; local precedes ', ...
    'topology and fusion. The pre-spatial column applies the logistic function ', ...
    'to each actual weighted input log odds; the fused column includes the ', ...
    'spatial overlap term. Pooling can add neighbor-only labels, so its difference ', ...
    'from local mass is not a per-target loss. Output uses MAP cardinality, ', ...
    'not an r>0.5 threshold; retained unselected labels remain in the filter. ', ...
    'Only offline metrics use truth. This opened-seed prefix cannot establish ', ...
    'full-episode causality, multiseed performance or a deployable improvement.\n']);
end
