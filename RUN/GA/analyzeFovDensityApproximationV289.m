function [reportPath, result] = analyzeFovDensityApproximationV289(tracePath, outputRoot)
% Read-only quadrature of cached predicted densities, not a tracking arm.
% Use the V282 sensing geometry/quality unchanged; never read target truth.
% Nested deterministic Halton rules expose numerical sensitivity, not CIs.
if exist(outputRoot, 'dir') ~= 7, mkdir(outputRoot); end
saved = load(tracePath, 'trace'); trace = saved.trace;
model = trace.model; T = trace.maximumTime; n = numel(trace.estimates);
assert(model.sensorMotionEnabled && model.sensorFovEnabled);
z = normalHalton(8192); coarseCount = 2048;
names = {'time','sensor','formation','birth_time','birth_location', ...
    'predicted_r','component_count','mean_pd','density_pd', ...
    'coarse_density_pd','pd_difference','quadrature_difference', ...
    'mean_miss_r','density_miss_r','miss_r_difference', ...
    'miss_position_difference_m','weighted_pd_abs_difference'};
rows = zeros(0, numel(names)); maximumMeanCheckError = 0;
started = tic;
for t = 1:T
    for s = 1:n
        geometry = sensingGeometry(model, s, t);
        objects = trace.predicted{t}{s};
        for k = 1:numel(objects)
            object = objects(k); w = object.w(:)'/sum(object.w);
            J = object.numberOfGmComponents;
            meanPd = zeros(1,J); pd = meanPd; coarsePd = meanPd;
            mu = zeros(2,J); centeredPdMoment = zeros(2,J);
            for j = 1:J
                mu(:,j) = object.mu{j}(1:2);
                covariance = object.Sigma{j}(1:2,1:2);
                L = chol((covariance+covariance')/2, 'lower');
                d = L*z; x = bsxfun(@plus, mu(:,j), d);
                q = pointPd(x, geometry, model.sensorQuality);
                pd(j) = mean(q); coarsePd(j) = mean(q(1:coarseCount));
                centeredPdMoment(:,j) = mean(bsxfun(@times, d, q), 2);
                meanPd(j) = evaluateSensorQuality(model,s,object.mu{j},t);
                maximumMeanCheckError = max(maximumMeanCheckError, ...
                    abs(meanPd(j)-pointPd(mu(:,j),geometry,model.sensorQuality)));
            end
            a = sum(w.*meanPd); b = sum(w.*pd); c = sum(w.*coarsePd);
            meanMissPosition = mu*(w.*(1-meanPd))'/(1-a);
            densityMissPosition = (mu*(w.*(1-pd))' - ...
                centeredPdMoment*w')/(1-b);
            r = object.r; ra = missedExistence(r,a); rb = missedExistence(r,b);
            rows(end+1,:) = [t,s,trace.config.sensorGroupIds(s), ...
                object.birthTime,object.birthLocation,r,J,a,b,c,b-a,b-c, ...
                ra,rb,rb-ra,norm(densityMissPosition-meanMissPosition),r*abs(b-a)];
        end
    end
    if mod(t,5)==0
        fprintf('V289 quadrature: %d/%d times, %d label stages, %.1f s.\n', ...
            t,T,size(rows,1),toc(started));
    end
end
assert(maximumMeanCheckError < 1e-12, 'Point-quality formula differs from runtime.');
% A transparent refinement on the twelve largest discrepancies, selected
% without truth or downstream tracking errors, checks the headline cases.
[~,order] = sort(abs(rows(:,11)), 'descend');
topRows = rows(order(1:min(12,numel(order))),:);
fineZ = normalHalton(65536); finePd = zeros(size(topRows,1),1);
for a = 1:size(topRows,1)
    row = topRows(a,:); t = row(1); s = row(2);
    objects = trace.predicted{t}{s};
    idx = find([objects.birthTime]==row(4) & [objects.birthLocation]==row(5),1);
    object = objects(idx); w = object.w(:)'/sum(object.w);
    geometry = sensingGeometry(model,s,t);
    for j = 1:object.numberOfGmComponents
        covariance = object.Sigma{j}(1:2,1:2);
        x = bsxfun(@plus,object.mu{j}(1:2), ...
            chol((covariance+covariance')/2,'lower')*fineZ);
        finePd(a) = finePd(a) + w(j)*mean(pointPd(x,geometry,model.sensorQuality));
    end
end
topRows = [topRows,finePd,finePd-topRows(:,9)];
summaryNames = {'group','label_stages','mean_abs_pd_difference', ...
    'p95_abs_pd_difference','maximum_abs_pd_difference', ...
    'abs_pd_difference_ge_005_count','mean_zero_pd_count', ...
    'mean_zero_density_pd_ge_005_count','high_r_count', ...
    'high_r_abs_pd_difference_ge_005_count','mean_abs_miss_r_difference', ...
    'maximum_abs_miss_r_difference','mean_miss_position_difference_m', ...
    'p95_miss_position_difference_m','mean_abs_quadrature_difference', ...
    'maximum_abs_quadrature_difference','existence_weighted_abs_pd_difference'};
summary = summarizeRows(0,rows);
for f = unique(trace.config.sensorGroupIds)
    summary(end+1,:) = summarizeRows(f,rows(rows(:,3)==f,:));
end
timeSummary = zeros(T,17);
for t = 1:T, timeSummary(t,:) = summarizeRows(t,rows(rows(:,1)==t,:)); end
result = struct('analysis','cached-fov-density-approximation-v289-v1', ...
    'tracePath',tracePath,'traceCommit',trace.generationGitCommit, ...
    'presetName',trace.presetName,'seed',trace.seed,'maximumTime',T, ...
    'coarsePointCount',coarseCount,'pointCount',size(z,2), ...
    'topCasePointCount',size(fineZ,2),'rows',rows,'names',{names}, ...
    'summary',summary,'summaryNames',{summaryNames},'topRows',topRows, ...
    'maximumMeanCheckError',maximumMeanCheckError, ...
    'elapsedSeconds',toc(started),'usesTargetTruth',false, ...
    'newPolicyEvaluated',false,'independentValidation',false);
save('-mat7-binary',fullfile(outputRoot,'FOV_DENSITY_APPROXIMATION_V289.mat'),'result');
writeCsv(fullfile(outputRoot,'V289_FORMATION_SUMMARY.csv'),summaryNames,summary);
timeNames = summaryNames; timeNames{1} = 'time';
writeCsv(fullfile(outputRoot,'V289_TIME_SUMMARY.csv'),timeNames,timeSummary);
writeCsv(fullfile(outputRoot,'V289_TOP_CASES.csv'), ...
    [names,{'fine_density_pd','fine_minus_density_pd'}],topRows);
reportPath = fullfile(outputRoot,'FOV_DENSITY_APPROXIMATION_V289.md');
writeReport(reportPath,result);
fprintf('V289 completed: MAE pD %.6f, >=.05 %d/%d, zero-to>=.05 %d/%d, high-r material %d/%d; max point check %.3g.\n', ...
    summary(1,3),summary(1,6),summary(1,2),summary(1,8),summary(1,7), ...
    summary(1,10),summary(1,9),maximumMeanCheckError);
end

function g = sensingGeometry(model,s,t)
traj = model.sensorTrajectories{s};
g.position = traj(1:2,min(t,size(traj,2)));
heading = model.sensorFovHeadingRad(s,t);
assert(isfinite(heading), 'This diagnostic requires the recorded explicit heading.');
g.heading = [cos(heading);sin(heading)];
g.halfAngle = sensorValue(model.sensorFovHalfAngleDeg,s);
g.range = sensorValue(model.sensorFovRange,s);
g.basePd = sensorValue(model.detectionProbability,s);
end

function q = pointPd(x,g,cfg)
% Vectorized numerical equivalent of evaluateSensorQuality's pD branch.
d = bsxfun(@minus,x,g.position); r = sqrt(sum(d.^2,1));
cosine = (g.heading'*d)./max(r,realmin);
angle = acosd(min(max(cosine,-1),1)); angle(r<=1e-9)=0;
inFov = r<=g.range & angle<=g.halfAngle;
q = g.basePd*ones(1,size(x,2));
if cfg.enabled
    rangeRatio = max(r/max(cfg.referenceRange,eps),0);
    angleRatio = min(max(angle/g.halfAngle,0),1);
    q = g.basePd*exp(-max(cfg.detectionRangeDecay,0)* ...
        rangeRatio.^max(cfg.detectionRangePower,eps)).* ...
        (1-min(max(cfg.edgeDetectionPenalty,0),1)*angleRatio.^max(cfg.anglePower,eps));
    q = min(max(q,min(max(cfg.minDetectionProbability,0),1)),1);
end
q(~inFov)=0;
end

function z = normalHalton(N)
% Fixed, unrandomized two-dimensional Halton rule, no filter RNG changes.
u = zeros(2,N);
for d = 1:2
    base = d+1; indices = 1:N; factor = 1/base;
    while any(indices)
        u(d,:) = u(d,:)+factor*mod(indices,base);
        indices=floor(indices/base); factor=factor/base;
    end
end
z = sqrt(2)*erfinv(2*u-1);
end

function r = missedExistence(r,pd)
r = r*(1-pd)/(1-r*pd);
end

function row = summarizeRows(group,a)
e = abs(a(:,11)); zero = a(:,8)==0; high = a(:,6)>=.5;
row = [group,size(a,1),mean(e),empiricalQuantile(e,.95),max(e), ...
    nnz(e>=.05),nnz(zero),nnz(zero & a(:,9)>=.05),nnz(high), ...
    nnz(high & e>=.05),mean(abs(a(:,15))),max(abs(a(:,15))), ...
    mean(a(:,16)),empiricalQuantile(a(:,16),.95), ...
    mean(abs(a(:,12))),max(abs(a(:,12))),sum(a(:,17))/sum(a(:,6))];
end

function q = empiricalQuantile(x,p)
x = sort(x); q = x(max(1,ceil(p*numel(x))));
end

function v = sensorValue(a,s)
if isscalar(a),v=a;else,v=a(s);end
end

function writeCsv(path,names,a)
fid=fopen(path,'w'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',strjoin(names,','));
format=[repmat('%.12g,',1,size(a,2)-1),'%.12g\n']; fprintf(fid,format,a');
end

function writeReport(path,r)
a=r.summary(1,:); fid=fopen(path,'w'); assert(fid>=0);
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'# V289: cached FoV density-integration diagnostic\n\n');
fprintf(fid,'Scope: %s, seed %d, steps 1--%d; %d predicted label stages. No filter rerun, target truth, new routing or tracking-performance result.\n\n', ...
    r.presetName,r.seed,r.maximumTime,a(2));
fprintf(fid,'The runtime uses sum_j w_j pD(mu_j). This diagnostic estimates sum_j w_j E[pD(X_j)] with the same 120-degree, 300-m sensing model and state-dependent quality.\n\n');
fprintf(fid,'| Quantity | Result |\n| --- | ---: |\n');
fprintf(fid,'| Mean absolute pD difference | %.8f |\n',a(3));
fprintf(fid,'| 95th percentile absolute pD difference | %.8f |\n',a(4));
fprintf(fid,'| Maximum absolute pD difference | %.8f |\n',a(5));
fprintf(fid,'| Label stages with absolute pD difference >= 0.05 | %d / %d |\n',a(6),a(2));
fprintf(fid,'| Mean-pD-zero stages with density pD >= 0.05 | %d / %d |\n',a(8),a(7));
fprintf(fid,'| r >= 0.5 stages with absolute pD difference >= 0.05 | %d / %d |\n',a(10),a(9));
fprintf(fid,'| Existence-weighted mean absolute pD difference | %.8f |\n',a(17));
fprintf(fid,'| Hypothetical no-detection mean absolute existence difference | %.8f |\n',a(11));
fprintf(fid,'| Hypothetical no-detection maximum absolute existence difference | %.8f |\n',a(12));
fprintf(fid,'| Hypothetical no-detection mean position difference, m | %.8f |\n',a(13));
fprintf(fid,'| Hypothetical no-detection 95th percentile position difference, m | %.8f |\n',a(14));
fprintf(fid,'| 2048 vs 8192 point mean / maximum absolute pD difference | %.8f / %.8f |\n',a(15),a(16));
fprintf(fid,'| Top twelve cases: 8192 vs 65536 point maximum pD difference | %.8f |\n',max(abs(r.topRows(:,end))));
fprintf(fid,'| Runtime vs vectorized point-pD maximum difference | %.3g |\n',r.maximumMeanCheckError);
fprintf(fid,'\nThe no-detection values are isolated likelihood counterfactuals, not the actual posterior at every stage: the cached update may include detections and association competition. They evaluate r*(1-pDbar)/(1-r*pDbar) and the normalized density (1-pD(x))*p(x). Replacing only scalar pD does not reproduce this spatial update.\n\n');
fprintf(fid,'Numerical method: fixed unrandomized 2D Halton normal quadrature (%d and %d points), refined to %d on the twelve largest absolute differences. Sensitivity values are numerical diagnostics, not confidence intervals or rigorous integration-error bounds. Component covariance is marginalized to position; all original mixture weights are retained.\n\n', ...
    r.coarsePointCount,r.pointCount,r.topCasePointCount);
fprintf(fid,'Source trace: `%s`, source commit `%s`. Runtime %.1f seconds. Self-check only; single opened development episode, not independent validation.\n',r.tracePath,r.traceCommit,r.elapsedSeconds);
end
