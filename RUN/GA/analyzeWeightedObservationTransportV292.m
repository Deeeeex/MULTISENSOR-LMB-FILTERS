function [reportPath,result] = analyzeWeightedObservationTransportV292()
% Packet-level weighted ancestry diagnostic; no tracker or new policy.
% Visibility is an offline geometric opportunity, not a realized detection.
sourcePath=fullfile('RUN','GA','dynamic_topology','evidence', ...
    'tracking_aligned_v280','observation_reachability_seed1301', ...
    'OBSERVATION_REACHABILITY_V280.mat');
s=load(sourcePath,'result'); cache=s.result;
[~,commit]=system('git rev-parse HEAD');
result=struct('contractVersion','weighted-observation-transport-v292-v1', ...
    'sourcePath',sourcePath,'generationGitCommit',strtrim(commit), ...
    'horizons',cache.horizons,'filterRerun',false,'newPolicy',false, ...
    'packetLevelOnly',true,'postHoc',true,'developmentEvidenceOnly',true);
for sceneIdx=1:numel(cache.scenes)
    original=cache.scenes{sceneIdx};
    inputs=generateDynamicTopologyScenarioInputs(original.preset,original.seed);
    cfg=inputs.config; n=cfg.numberOfSensors; T=cfg.simulationLength;
    identity=buildDynamicTopologyPhysicalIdentityRegistry(cfg);
    registration=buildInitialFormationBraidRouteV241Registration(inputs);
    q=original.targetCount;
    assert(isequal(size(original.visible),[n,q,T]));
    active=repmat(reshape(original.alive,1,q,T),n,1,1);
    scene=struct('preset',original.preset,'seed',original.seed, ...
        'sensorCount',n,'targetCount',q,'timeCount',T,'arms',{{}});
    for armIdx=1:3
        history=false(n,n,0); times=[]; weights=zeros(n,n,T);
        messageCounts=zeros(1,T);
        for t=1:T
            context=struct('localPosteriorBySensor',{repmat({struct([])},1,n)}, ...
                'model',struct('dynamicTopologyScenario',struct('config',cfg, ...
                    'staticAdjacency',inputs.graphData.staticAdjacency)), ...
                'baseAdjacency',inputs.graphData.staticAdjacency, ...
                'physicalAdjacency',inputs.graphData.physicalAdjacency(:,:,t), ...
                'positions',inputs.graphData.positions(:,:,t),'currentTime',t, ...
                'commConfig',struct('pDropByEdge',inputs.commConfig.pDropByEdge(:,:,t)), ...
                'directedMessageBudget',2*n, ...
                'sensorPhysicalUids',identity.sensorPhysicalUids, ...
                'formationPhysicalUidsBySensor',identity.formationPhysicalUidsBySensor, ...
                'previousAdjacencyHistory',history,'previousAdjacencyHistoryTimes',times, ...
                'previousAdjacencyHistoryCount',size(history,3));
            switch armIdx
                case 1
                    [adj,detail]=selectFrozenFormationBraidDropoutV241Policy(context,registration);
                case 2
                    [adj,detail]=selectCausalMinimalEditFormationTreeV240Policy(context);
                case 3
                    [adj,detail]=selectCausalMinimumFormationBackboneV242Policy(context);
            end
            messageCounts(t)=nnz(adj);
            delivered=logical(adj);
            if ~inputs.commConfig.forceDelivery
                delivered=delivered & (inputs.commConfig.linkUniforms(:,:,t)' >= ...
                    inputs.commConfig.pDropByEdge(:,:,t)');
            end
            kept=detail.fusionWeightMatrix.*(double(delivered)+eye(n));
            weights(:,:,t)=kept./sum(kept,2);
            assert(max(abs(sum(weights(:,:,t),2)-1))<1e-12);
            history=cat(3,history,logical(adj)); times(end+1)=t;
            if size(history,3)>2,history=history(:,:,end-1:end);times=times(end-1:end);end
        end
        old=original.arms{armIdx};
        assert(isequal(messageCounts,old.messageCounts));
        rec=struct('name',old.name,'messageCounts',messageCounts, ...
            'deliveredPacketWeights',weights,'records',struct([]));
        for h=cache.horizons
            score=zeros(n,q,T);
            for t=1:T
                z=zeros(n,q);
                for k=max(1,t-h):t
                    opportunity=double(original.visible(:,:,k));
                    z=weights(:,:,k)*(opportunity+(1-opportunity).*z);
                    z(:,~original.alive(:,k))=0;
                end
                score(:,:,t)=z;
            end
            reachable=active & old.ageBySensorTargetTime<=h;
            % Checks orientation and synchronous-round timing against V280.
            assert(isequal(active & score>0,reachable));
            assert(min(score(:))>=-1e-12 && max(score(:))<=1+1e-12);
            values=score(reachable);
            r=struct('horizon',h,'pathCoverage',nnz(reachable)/nnz(active), ...
                'meanScore',mean(score(active)), ...
                'meanScoreGivenReachable',mean(values), ...
                'medianScoreGivenReachable',median(values), ...
                'p10ScoreGivenReachable',quantile(values,0.10), ...
                'activeTripleCount',nnz(active),'reachableTripleCount',nnz(reachable));
            rec.records(end+1)=r;
            if h==8,rec.eightStepSourceHitScore=score;end
            fprintf('V292 N%d %s h%d: path %.3f%%, mean weighted hit %.3f%%, reachable median %.6f.\n', ...
                n,rec.name,h,100*r.pathCoverage,100*r.meanScore,r.medianScoreGivenReachable);
        end
        scene.arms{armIdx}=rec;
    end
    result.scenes{sceneIdx}=scene;
end
result.completedAt=datestr(now,31);
out=fullfile('RUN','GA','dynamic_topology','evidence','tracking_aligned_v292', ...
    'weighted_observation_transport_seed1301');
if exist(out,'dir')~=7,mkdir(out);end
save('-mat7-binary',fullfile(out,'WEIGHTED_OBSERVATION_TRANSPORT_V292.mat'),'result');
reportPath=fullfile(out,'WEIGHTED_OBSERVATION_TRANSPORT_V292.md');
fid=fopen(reportPath,'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'# V292: path existence versus weighted observation transport\n\n');
fprintf(fid,'Opened M24/X36 seed 1301, original 160-step scenes. Source `%s`. Self-check only. No filter, policy or parameter changes.\n\n',result.generationGitCommit);
fprintf(fid,'For each active sensor-target-time triple and window t-h:t, initialize z=0 and iterate z_k = W_k [o_k + (1-o_k).*z_(k-1)]. W is the delivered packet-level row-stochastic matrix; o is the cached geometric visibility indicator. A current opportunity is inserted before one synchronous fusion round.\n\n');
fprintf(fid,'z is the probability that a backwards stochastic source path, sampled using these weights, encounters an observation opportunity in that window. It is not posterior mass, label recall, detection probability, or a tracking-error bound. Delivered empty packets, label-specific censoring, spatial overlap, local likelihoods, pruning and association are not represented.\n\n');
fprintf(fid,'| Scale | Arm | Maximum age | Path coverage | Mean weighted source-hit score | Mean / median / p10 score, given a path |\n| --- | --- | ---: | ---: | ---: | --- |\n');
for s=1:numel(result.scenes)
    scene=result.scenes{s};
    for a=1:numel(scene.arms)
        arm=scene.arms{a};
        for r=arm.records
            fprintf(fid,'| N=%d | %s | %d | %.3f%% | %.3f%% | %.6f / %.6f / %.6f |\n', ...
                scene.sensorCount,arm.name,r.horizon,100*r.pathCoverage,100*r.meanScore, ...
                r.meanScoreGivenReachable,r.medianScoreGivenReachable,r.p10ScoreGivenReachable);
        end
    end
end
fprintf(fid,'\nAll four pre-existing horizons 0, 3, 8 and 16 are reported, with shorter windows at episode start. Percentages average over active sensor-target-time triples, not independent trials; no confidence intervals or significance claim. Positive weighted score exactly matches the cached V280 binary reachability at each horizon; route message counts also match.\n\n');
fprintf(fid,'This diagnostic tests whether binary temporal paths overstate weighted access. It does not establish that stronger mixing improves tracking; earlier stronger-mixing controls were unfavorable, and changing weights would also alter density compatibility. No automatic weight sweep or paper-method promotion follows.\n');
clear cleanup;
fid=fopen(fullfile(out,'V292_TRANSPORT_METRICS.csv'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'sensors,arm,horizon,path_coverage,mean_source_hit,reachable_mean,reachable_median,reachable_p10,active_triples,reachable_triples\n');
for s=1:numel(result.scenes)
    scene=result.scenes{s};
    for a=1:numel(scene.arms)
        arm=scene.arms{a};
        for r=arm.records
            fprintf(fid,'%d,%s,%d,%.12g,%.12g,%.12g,%.12g,%.12g,%d,%d\n', ...
                scene.sensorCount,arm.name,r.horizon,r.pathCoverage,r.meanScore, ...
                r.meanScoreGivenReachable,r.medianScoreGivenReachable, ...
                r.p10ScoreGivenReachable,r.activeTripleCount,r.reachableTripleCount);
        end
    end
end
end
