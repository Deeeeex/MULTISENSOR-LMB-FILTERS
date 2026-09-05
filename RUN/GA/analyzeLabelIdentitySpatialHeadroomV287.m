function [reportPath, result] = analyzeLabelIdentitySpatialHeadroomV287(v285ResultPath)
% Geometric identity diagnostics only. No filter, relabeling or controller.
loaded = load(v285ResultPath, 'result'); v285 = loaded.result;
loaded = load(v285.v284ResultPath, 'result'); v284 = loaded.result;
loaded = load(v284.rawPath, 'run'); run = loaded.run;
loaded = load(run.baselineTracePath, 'trace'); reference = loaded.trace;
assert(run.maximumTime == 40 && run.config.numberOfTargets == 24);
assert(all(run.config.targetBirthTimesByGroup == 1));
assert(all(run.groundTruthRfs.cardinality == 24));
groups = reshape(run.config.sensorGroupIds, 1, []); nSensors = numel(groups);
arms = {reference.estimates, run.estimates}; metrics = {v284.reference, v284.candidate};
maps = cell(2, nSensors, 40); nearest = maps; records = zeros(0, 9);
residual = 0; fallbackCount = 0;
% Records: arm, receiver, time, output index, birth location, assigned truth,
% nearest truth, assigned position error, birth-anchored position error.
for arm = 1:2
    for t = 1:40
        truth = cell2mat(run.groundTruthRfs.x{t});
        for n = 1:nSensors
            state = arms{arm}{n}; labels = state.labels{t};
            assert(all(labels(1, :) == 1) && all(ismember(labels(2, :), 1:24)));
            assert(numel(unique(labels(2, :))) == size(labels, 2));
            means = cell2mat(state.mu{t});
            cost = hypot(bsxfun(@minus, truth(1, :)', means(1, :)), ...
                bsxfun(@minus, truth(2, :)', means(2, :)));
            [matching, ~] = munkres(cost); expected = metrics{arm}.rmseBySensorTime(n, t);
            value = sqrt(mean(cost(matching).^2));
            if abs(value - expected) > 1e-8 * max(1, expected)
                [matching, ~] = Hungarian(cost); matching = logical(matching);
                value = sqrt(mean(cost(matching).^2)); fallbackCount = fallbackCount + 1;
            end
            assert(isfinite(value) && abs(value - expected) <= 1e-8 * max(1, expected));
            residual = max(residual, abs(value - expected));
            [i, j] = find(matching); map = zeros(1, size(means, 2)); map(j) = i;
            assert(all(map > 0)); maps{arm, n, t} = map;
            [~, closest] = min(cost, [], 1); nearest{arm, n, t} = closest;
            for j = 1:size(means, 2)
                records(end+1, :) = [arm, n, t, j, labels(2, j), map(j), closest(j), ...
                    cost(map(j), j), cost(labels(2, j), j)]; %#ok<AGROW>
            end
        end
    end
end
% Pair records: arm, time, pool (1 global undirected / 2 delivered directed),
% pair count, assigned-truth disagreements, nearest-truth disagreements,
% sum of truth separations for assignment-disagreeing pairs.
pairRows = zeros(0, 7);
for arm = 1:2
    for t = 1:40
        counters = zeros(2, 4); truth = cell2mat(run.groundTruthRfs.x{t});
        for label = 1:24
            nodes = []; assigned = []; closest = [];
            for n = 1:nSensors
                j = find(arms{arm}{n}.labels{t}(2, :) == label);
                if isempty(j), continue; end
                nodes(end+1) = n; assigned(end+1) = maps{arm, n, t}(j); %#ok<AGROW>
                closest(end+1) = nearest{arm, n, t}(j); %#ok<AGROW>
            end
            for a = 1:numel(nodes)-1
                for b = a+1:numel(nodes)
                    mismatch = assigned(a) ~= assigned(b);
                    separation = norm(truth(1:2, assigned(a)) - truth(1:2, assigned(b)));
                    row = [1, mismatch, closest(a) ~= closest(b), separation * mismatch];
                    counters(1, :) = counters(1, :) + row;
                    weight = run.delivered(nodes(a), nodes(b), t) + run.delivered(nodes(b), nodes(a), t);
                    counters(2, :) = counters(2, :) + weight * row;
                end
            end
        end
        pairRows(end+1:end+2, :) = [[arm,t,1,counters(1,:)]; [arm,t,2,counters(2,:)]];
    end
end
recordSummary = zeros(0, 8); pairSummary = zeros(0, 8);
windows = [1,40;1,5;6,20;21,40];
for arm = 1:2
    for w = 1:size(windows, 1)
        inWindow = records(:,3) >= windows(w,1) & records(:,3) <= windows(w,2);
        d = records(records(:,1) == arm & inWindow, :);
        recordSummary(end+1,:) = [arm,windows(w,:),size(d,1),mean(d(:,5)==d(:,6)), ...
            mean(d(:,5)==d(:,7)),sqrt(mean(d(:,8).^2)),sqrt(mean(d(:,9).^2))]; %#ok<AGROW>
        for pool = 1:2
            keep = pairRows(:,1)==arm & pairRows(:,2)>=windows(w,1) & ...
                pairRows(:,2)<=windows(w,2) & pairRows(:,3)==pool;
            sums = sum(pairRows(keep,4:7),1);
            pairSummary(end+1,:) = [arm,windows(w,:),pool,sums(1), ...
                sums(2)/max(1,sums(1)),sums(3)/max(1,sums(1)),sums(4)/max(1,sums(2))]; %#ok<AGROW>
        end
    end
end
q = v285.queries; details = zeros(0, 11); maxOracleResidual = 0;
poolIds = [1,4,5]; oldOracle = nan(size(q,1),2,5);
oldIndices = sub2ind(size(oldOracle),v285.details(:,1), ...
    v285.details(:,2)+1,v285.details(:,3));
oldOracle(oldIndices) = v285.details(:,6);
% Detail: query, lag, pool, peers, assignment-coherent peers, nearest-coherent
% peers, receiver error, original oracle, assignment-coherent oracle,
% nearest-coherent oracle, birth-aligned query flag.
for k = 1:size(q, 1)
    receiver = q(k,1); t = q(k,2); target = q(k,3); label = q(k,5);
    truthPosition = run.groundTruthRfs.x{t}{target}(1:2);
    for lag = 0:1
        st = t-lag; if st < 1, continue; end
        errors = nan(1,nSensors); agree = false(1,nSensors); nearAgree = agree;
        physical = false(1,nSensors);
        for n = 1:nSensors
            if n == receiver, continue; end
            physical(n) = norm(reference.model.sensorTrajectories{n}(1:2,t) - ...
                reference.model.sensorTrajectories{receiver}(1:2,t)) <= run.config.commRange;
            j = find(run.estimates{n}.labels{st}(2,:) == label);
            if isempty(j), continue; end
            mu = run.estimates{n}.mu{st}{j};
            if lag == 1, mu = v285.A * mu; end
            errors(n) = norm(mu(1:2) - truthPosition);
            agree(n) = maps{2,n,st}(j) == target;
            nearAgree(n) = nearest{2,n,st}(j) == target;
        end
        pools = [true(1,nSensors);reshape(logical(run.delivered(:,receiver,t)),1,[]);physical];
        for p = 1:3
            poolId = poolIds(p);
            available = isfinite(errors) & pools(p,:);
            original = min([q(k,7),errors(available)]);
            coherent = min([q(k,7),errors(available & agree)]);
            nearCoherent = min([q(k,7),errors(available & nearAgree)]);
            old = oldOracle(k,lag+1,poolId);
            assert(numel(old)==1 && abs(original-old) < 1e-8);
            maxOracleResidual = max(maxOracleResidual,abs(original-old));
            details(end+1,:) = [k,lag,poolId,sum(available),sum(available & agree), ...
                sum(available & nearAgree),q(k,7),original,coherent,nearCoherent,target==label]; %#ok<AGROW>
        end
    end
    if mod(k,1500)==0, fprintf('V287 identity-restricted queries %d/%d.\n',k,size(q,1)); end
end
headroom = zeros(0, 11);
for stratum = [-1,0,1] % All / mismatched birth identity / matched birth identity.
    for lag = 0:1
        for pool = [1,4,5]
            keep = details(:,2)==lag & details(:,3)==pool;
            if stratum >= 0, keep = keep & details(:,11)==stratum; end
            d = details(keep,:);
            headroom(end+1,:) = [stratum,lag,pool,size(d,1),sqrt(mean(d(:,7:10).^2,1)), ...
                mean(d(:,4)>0),mean(d(:,5)>0),mean(d(:,6)>0)]; %#ok<AGROW>
        end
    end
end
out = fullfile('RUN','GA','dynamic_topology','evidence','tracking_aligned_v287', ...
    'x36_label_identity_spatial_headroom_seed1301');
if exist(out,'dir')~=7, mkdir(out); end
result = struct('contractVersion','label-identity-spatial-headroom-v287-v1', ...
    'v285ResultPath',v285ResultPath,'records',records,'recordSummary',recordSummary, ...
    'pairRows',pairRows,'pairSummary',pairSummary,'headroomDetails',details, ...
    'headroomSummary',headroom,'maximumOfficialRmseResidual',residual, ...
    'assignmentFallbackCount',fallbackCount,'maximumV285OracleResidual',maxOracleResidual, ...
    'newPolicyEvaluated',false,'developmentEvidenceOnly',true);
save('-mat7-binary',fullfile(out,'LABEL_IDENTITY_SPATIAL_HEADROOM_V287.mat'),'result');
writeCsv(fullfile(out,'V287_BIRTH_ALIGNMENT.csv'), ...
    'arm,start,end,outputs,birth_assignment_agreement,birth_nearest_agreement,geometric_pooled_rmse,birth_anchored_pooled_rmse',recordSummary);
writeCsv(fullfile(out,'V287_CROSS_NODE_ASSIGNMENT.csv'), ...
    'arm,start,end,pool,label_pairs,assignment_disagreement,nearest_disagreement,mean_disagreeing_truth_separation',pairSummary);
writeCsv(fullfile(out,'V287_IDENTITY_RESTRICTED_HEADROOM.csv'), ...
    'birth_stratum,lag,pool,queries,receiver_pooled_rmse,original_oracle_pooled_rmse,assignment_coherent_oracle_pooled_rmse,nearest_coherent_oracle_pooled_rmse,original_peer_coverage,assignment_coherent_peer_coverage,nearest_coherent_peer_coverage',headroom);
reportPath = fullfile(out,'LABEL_IDENTITY_SPATIAL_HEADROOM_V287.md'); writeReport(reportPath,result);
fprintf('V287 complete: %d paired emitted states; %.2f%% of added queries agree with birth identity.\n', ...
    size(records,1),100*mean(q(:,3)==q(:,5)));
s = headroom(headroom(:,1)==-1 & headroom(:,2)==1 & headroom(:,3)==1,:);
fprintf('Lag-1 global pooled RMSE: receiver %.6f, original oracle %.6f, assignment-coherent %.6f, nearest-coherent %.6f.\n',s(5:8));
end

function writeCsv(path,header,data)
fid=fopen(path,'w'); assert(fid>=0); cleanup=onCleanup(@() fclose(fid));
fprintf(fid,'%s\n',header);
for k=1:size(data,1)
    fprintf(fid,'%.12g',data(k,1)); fprintf(fid,',%.12g',data(k,2:end)); fprintf(fid,'\n');
end
end

function writeReport(path,r)
fid=fopen(path,'w'); assert(fid>=0); cleanup=onCleanup(@() fclose(fid));
fprintf(fid,'# V287: geometric identity and same-label spatial headroom\n\n');
fprintf(fid,'X36 seed 1301, unchanged stored prefix 1--40. Diagnostic / self-check only; no filter, relabeling, observation resimulation or communication action. The scene has 24 distinct initial birth components, one per explicit truth trajectory, active throughout this prefix. Birth identity is a diagnostic anchor, not a change to unlabeled OSPA/RMSE.\n\n');
fprintf(fid,'## Emitted states\n\n| Arm | Steps | Outputs | Birth / assigned truth agrees (%%) | Birth / nearest truth agrees (%%) | Geometric pooled RMSE | Birth-anchored pooled RMSE |\n| --- | --- | ---: | ---: | ---: | ---: | ---: |\n');
for k=1:size(r.recordSummary,1)
    s=r.recordSummary(k,:); fprintf(fid,'| %d | %d--%d | %d | %.2f | %.2f | %.6f | %.6f |\n',s(1:4),100*s(5:6),s(7:8));
end
fprintf(fid,'\nArm 1 is V242; arm 2 is V284. Pooled RMSE is not the official mean per-cell RMSE. A coherent common label permutation would not itself invalidate label-wise fusion; birth disagreement alone is not a causal diagnosis.\n\n');
fprintf(fid,'## Cross-node same-label output correspondence\n\n| Arm | Steps | Pool | Label pairs | Assignment disagreement (%%) | Nearest disagreement (%%) | Mean truth separation when assigned identities disagree (m) |\n| --- | --- | --- | ---: | ---: | ---: | ---: |\n');
for k=1:size(r.pairSummary,1)
    s=r.pairSummary(k,:); fprintf(fid,'| %d | %d--%d | %d | %d | %.2f | %.2f | %.3f |\n',s(1:5),100*s(6:7),s(8));
end
fprintf(fid,'\nPool 1 counts undirected global pairs; pool 2 counts directed delivered edges. Outputs are from round end, not the actual incoming packets earlier in the same round. These are geometric correspondences, not confirmed identity switches. Close targets and one-to-one assignment competition can affect correspondence; no distance gate is introduced.\n\n');
fprintf(fid,'## Restricted source oracle\n\n| Birth stratum | Lag | Pool | Queries | Receiver | Original oracle | Assignment-coherent oracle | Nearest-coherent oracle |\n| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |\n');
for k=1:size(r.headroomSummary,1)
    s=r.headroomSummary(k,:); fprintf(fid,'| %d | %d | %d | %d | %.6f | %.6f | %.6f | %.6f |\n',s(1:8));
end
fprintf(fid,'\nBirth strata: -1 all, 0 mismatch, 1 agreement. Source pools: 1 global, 4 delivered incoming, 5 physical one-hop. Both restrictions require source-time geometric correspondence to the receiver query target; nearest and assignment restrictions are separate. All oracles may retain the receiver and use truth, so none is an online selector. Lag 1 predicts prior output one step; lag 0 uses current end-of-round output. Neither accounts for actual multihop transit or a changed tracking trajectory.\n\n');
fprintf(fid,'Maximum official per-cell RMSE discrepancy %.3g; assignment solver fallbacks %d. Original V285 oracle readback discrepancy %.3g. No source-local positive-measurement timestamp is established here.\n',r.maximumOfficialRmseResidual,r.assignmentFallbackCount,r.maximumV285OracleResidual);
end
