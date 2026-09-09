function recordAssociationReplay(configPath,unitIndex)
% Isolated recursive controls on immutable, externally registered inputs.
out=fileparts(mfilename('fullpath'));root=fileparts(fileparts(out));
config=jsondecode(fileread(configPath));unit=config.units(unitIndex);
original=fullfile(root,'trials','icra_external_fusion');
addpath(fullfile(root,'trials','icra_reviewer_revision'));
priorIteration=fullfile(root,'trials','icra_method_iteration');
folders={'icra_fusion_holdout','icra_asymmetric_evidence','icra_selective_innovation', ...
    'icra_evidence_iteration','icra_ceiling_iteration','icra_marked_iteration', ...
    'icra_gaussian_evidence','icra_reunion_fusion'};
for k=1:numel(folders),addpath(fullfile(root,'trials',folders{k}));end
addpath(original,priorIteration,fullfile(root,'common'),fullfile(root,'lmb'),fullfile(root,'multisensorLmb'));
checkStableMarkedEvidence();checkGaussianEvidence();checkReviewerEvidence();checkDirectObservationSummary();
baseQuality=@evaluateSensorQuality;
assert(strcmp(functions(baseQuality).file,fullfile(root,'common','evaluateSensorQuality.m')));
qualityPath=fullfile(original,'replay_quality');addpath(qualityPath);
cleanup=onCleanup(@()rmpath(qualityPath)); %#ok<NASGU>
runtimePath=fullfile(priorIteration,'runtime');addpath(runtimePath);
runtimeCleanup=onCleanup(@()rmpath(runtimePath)); %#ok<NASGU>
assert(strcmp(which('Hungarian'),fullfile(runtimePath,'Hungarian.m')));
data=load(fullfile(root,unit.data_path));
if isfield(data,'likelihoodRatios')
    ratios=struct('likelihoodRatios',{data.likelihoodRatios});
    marks=struct('rawScores',{data.rawScores},'calibratedScores',{data.calibratedScores});
else
    marks=load(fullfile(root,unit.marks_path));ratios=load(fullfile(root,unit.ratios_path));
end
T=double(data.T);measurements=data.measurements;positions=data.positions;
model=makeModel(positions,T,baseQuality,config.pd);
resultDir=fullfile(out,'results',config.stage);if ~isfolder(resultDir),mkdir(resultDir);end
for condition={'reliable','intermittent'}
    radio=condition{1};delivered=repmat(logical([0,1;1,0]),1,1,T);
    if strcmp(radio,'intermittent')
        rng(unit.radio_seed,'twister');draws=rand(2,2,T);delivered=delivered & draws>=.1;
        delivered(:,:,floor(.4*T)+1:floor(.6*T))=false;
    end
    for a=1:numel(config.arms)
        arm=config.arms{a};
        path=fullfile(resultDir,sprintf('%s_%s_%s.json',unit.sequence,radio,arm));
        assert(~isfile(path) && ~isfile([path,'.gz']),'Never overwrite a registered run.');
        fprintf('START REVIEW %s %s %s %s %d frames\n',config.stage,unit.sequence,radio,arm,T);
        run=runDensity(model,measurements,positions,delivered,arm,marks,ratios);
        % Truth first enters after this arm has completed its full trajectory.
        run=scoreOutputs(run,data.truth);
        result=struct('protocol','icra-reviewer-revision-v1','stage',config.stage, ...
            'cohort',config.cohort,'sourceSha256',config.source_sha256, ...
            'inputSha256',unit.input_sha256,'sequence',unit.sequence,'condition',radio, ...
            'pd',config.pd,'time',double(data.time),'positions',positions, ...
            'truth',{data.truth},'truthIds',{data.truthIds},'delivered',delivered,'runs',run);
        fid=fopen(path,'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(result));fclose(fid);gzip(path);delete(path);
        fprintf('DONE REVIEW %s %s %s %s OSPA %.6f runtime %.2fs\n', ...
            config.stage,unit.sequence,radio,arm,mean(run.ospa,'all'),run.runtimeSeconds);
    end
end
fprintf('COMPLETED REVIEW %s %s\n',config.stage,unit.sequence);
end

function model=makeModel(positions,T,baseQuality,pd)
model=generateMultisensorModel(2,[3,3],[pd,pd],[1,1],'GA','LBP');dt=.1;
model.T=dt;model.A=[eye(2),dt*eye(2);zeros(2),eye(2)];model.u=zeros(4,1);
model.R=25*[dt^3/3*eye(2),dt^2/2*eye(2);dt^2/2*eye(2),dt*eye(2)];
model.survivalProbability=.99;model.existenceThreshold=.001;
model.maximumNumberOfGmComponents=1000;model.gmWeightThreshold=1e-6;
model.sensorMotionEnabled=true;model.sensorFovEnabled=true;
model.sensorFovRange=40;model.sensorFovHalfAngleDeg=180;
model.sensorQuality=struct('enabled',false);model.sensorTrajectories=cell(1,2);
for n=1:2,model.sensorTrajectories{n}=[reshape(positions(:,n,:),2,T);zeros(2,T)];end
model.observationSpaceLimits=[-70.4,70.4;-40,40];model.observationSpaceVolume=pi*40^2;
model.clutterPerUnitVolume=[3,3]/model.observationSpaceVolume;
template=model.birthParameters(1);template.hasObservationLineage=true;
template.lastDirectOpportunity=0;template.directEvidenceCeiling=0;template.positiveConfirmation=false;template.localLogOddsIncrement=0;template.positiveInnovationSupport=0;template.negativeInnovationSupport=0;template.localSpatialLogRatio=zeros(1,15);
template.numberOfGmComponents=1;template.w=1;
model.birthParameters=template;model.numberOfBirthLocations=0;model.object=template([]);
model.replayBaseSensorQuality=baseQuality;
end

function run=emptyRun(arm,T)
run=struct('arm',arm,'estimates',{cell(2,T)},'rawEstimates',{cell(2,T)},'labels',{cell(2,T)}, ...
    'ospa',zeros(2,T),'countError',zeros(2,T),'matchedSquaredError',zeros(2,T),'matchedCount',zeros(2,T), ...
    'rawPayloadBytes',zeros(1,T),'deliveredRawBytes',zeros(1,T),'wireBytes',zeros(1,T), ...
    'controlBytes',zeros(1,T),'attemptedMessages',zeros(1,T),'deliveredMessages',zeros(1,T), ...
    'weightChangeL1',zeros(2,T),'matchedLabels',zeros(2,T),'observableAbsences',zeros(2,T), ...
    'packetBytes',zeros(2,T),'localIncrementRecords',zeros(0,12),'localGaussianRecords',zeros(0,32), ...
    'packetGaussianRecords',zeros(0,19),'localAssociationWeights',{cell(2,T)},'localDirectRecords',zeros(0,12),'iterationRecords',zeros(0,60),'maximumBernoulliCount',0,'runtimeSeconds',0,'totalWireBytes',0);
end

function run=runDensity(model,measurements,positions,delivered,arm,marks,ratios)
% No truth, IDs from annotations, future detections or other-source births.
marked=startsWith(arm,'marked_');rule=erase(arm,'marked_');
T=size(measurements,2); posterior=repmat({model.object},1,2);
direct={containers.Map('KeyType','char','ValueType','double'),containers.Map('KeyType','char','ValueType','double')};
run=emptyRun(arm,T);cfg=buildMixtureAwareKlaReferenceConfig();
cfg.missingLabelFusionMode='fov-aware-censored';cfg.untouchedPriorExclusionEnabled=true;
cfg.activeExistenceThreshold=model.existenceThreshold;
cfg.payloadExistenceThreshold=model.existenceThreshold;
cfg.captureIterationRecords=true;
timer=tic;
for t=1:T
    local=cell(1,2);
    for n=1:2
        predicted=lmbPredictionStep(posterior{n},model,t);
        if t>1
            previous=measurements{n,t-1};
            for j=1:size(previous,2)
                o=model.birthParameters(1);o.birthTime=t;o.birthLocation=n*100000+j;o.r=.01;
                o.mu={[previous(:,j);0;0]};o.Sigma={diag([16,16,225,225])};
                o.lastDirectOpportunity=t-1;direct{n}(key(o))=t-1;
                predicted(end+1)=o; %#ok<AGROW>
            end
        end
        predictedPd=zeros(1,numel(predicted));
        for j=1:numel(predicted)
            o=predicted(j);pd=evaluateSensorQuality(model,n,o.mu{1},t);predictedPd(j)=pd;
            if pd>0, direct{n}(key(o))=t;predicted(j).hasObservationLineage=true;end
            predicted(j).lastDirectOpportunity=stamp(direct{n},key(o));
        end
        W=[];
        if isempty(predicted),local{n}=predicted;
        elseif marked,[local{n},~,W]=updateMarkedLmbStable(predicted,num2cell(measurements{n,t},1),model,n,t,ratios.likelihoodRatios{n,t});
        else,[local{n},~,W]=updateLmbWithAssociationWeights(predicted,num2cell(measurements{n,t},1),model,n,t);end
        opportunity=[local{n}.lastDirectOpportunity]==t;
        [support,associationMass]=positiveInnovationSupport(W,ratios.likelihoodRatios{n,t}, ...
            opportunity,strcmp(rule,'gaussian_evidence_no_mark'));
        run.localAssociationWeights{n,t}=W;
        directValues=directObservationSummary(W,measurements{n,t},model.Q{n},opportunity);
        predictedPd=reshape(predictedPd,size(opportunity));
        assert(isequal(opportunity,predictedPd>0));
        negativeSupport=negativeInnovationSupport(associationMass,predictedPd);
        if startsWith(rule,'selective'),negativeSupport(:)=0;end
        for j=1:numel(local{n})
            assert(strcmp(key(local{n}(j)),key(predicted(j))));
            before=min(max(predicted(j).r,1e-9),1-1e-9);
            after=min(max(local{n}(j).r,1e-9),1-1e-9);
            delta=log(after)-log1p(-after)-log(before)+log1p(-before);
            local{n}(j).localLogOddsIncrement=delta;
            local{n}(j).directEvidenceCeiling=0;local{n}(j).positiveInnovationSupport=support(j);local{n}(j).negativeInnovationSupport=negativeSupport(j);
            current=local{n}(j).lastDirectOpportunity==t;
            if ~current,assert(abs(delta)<1e-10);end
            run.localIncrementRecords(end+1,:)=[t,n,local{n}(j).birthTime,local{n}(j).birthLocation, ...
                predicted(j).r,local{n}(j).r,delta,current,support(j),associationMass(j),predictedPd(j),negativeSupport(j)]; %#ok<AGROW>
        end
        local{n}=reduce(local{n},model);
        if ~isempty(local{n})
            localKeys=[[local{n}.birthTime];[local{n}.birthLocation]]';
            priorKeys=[[predicted.birthTime];[predicted.birthLocation]]';
            [found,indices]=ismember(localKeys,priorKeys,'rows');assert(all(found));
            for j=1:numel(local{n})
                before=predicted(indices(j));after=local{n}(j);
                run.localDirectRecords(end+1,:)=[t,n,after.birthTime,after.birthLocation,directValues(indices(j),:)]; %#ok<AGROW>
                assert(before.numberOfGmComponents==1 && after.numberOfGmComponents==1);
                [encoded,packed]=gaussianLogRatio(before.mu{1},before.Sigma{1},after.mu{1},after.Sigma{1});
                local{n}(j).localSpatialLogRatio=encoded;
                run.localGaussianRecords(end+1,:)=[t,n,after.birthTime,after.birthLocation,packed]; %#ok<AGROW>
            end
        end
        run.maximumBernoulliCount=max(run.maximumBernoulliCount,numel(local{n}));
    end
    if strcmp(rule,'local')
        posterior=local;
    else
        decoded=cell(1,2);sizes=zeros(1,2);
        for n=1:2
            if startsWith(rule,'gaussian_evidence')
                [decoded{n},packet]=gaussianEvidencePacket(local{n},model,n,t);
                for j=1:numel(decoded{n})
                    o=decoded{n}(j);
                    run.packetGaussianRecords(end+1,:)=[t,n,o.birthTime,o.birthLocation,o.localSpatialLogRatio]; %#ok<AGROW>
                end
            elseif strcmp(rule,'lineage') || strcmp(rule,'er')
                [decoded{n},packet]=ceilingLmbPacket(local{n},model,n,t);
            else
                [decoded{n},packet]=asymmetricInnovationPacket(local{n},model,n,t);
            end
            sizes(n)=numel(packet);
        end
        for n=1:2
            other=3-n;
            if ~delivered(n,other,t),posterior{n}=local{n};continue;end
            [remote,matching]=alignGaussianLmbPair(local{n},decoded{other},50);
            inputs={local{n},remote};details=struct('eventType',[0,2],'sourceIndices',[n,other], ...
                'isStale',[false,false],'isSelf',[true,false],'currentTime',t);
            details.originalLabels={[[local{n}.birthTime];[local{n}.birthLocation]], ...
                [[decoded{other}.birthTime];[decoded{other}.birthLocation]]};
            if strcmp(rule,'asymmetric')
                [posterior{n},stats]=fuseAsymmetricEvidence(inputs,[.5,.5],model,details,cfg,rule,t);
                stats.records=[stats.records,zeros(size(stats.records,1),23)];
            elseif strcmp(rule,'lineage') || strcmp(rule,'er')
                [posterior{n},stats]=fuseMarkedInputsStable(inputs,[.5,.5],model,details,cfg,rule,t);
                stats.records=[stats.records,zeros(size(stats.records,1),34)];
            elseif strcmp(rule,'gaussian_evidence_guarded_scalar') || startsWith(rule,'gaussian_evidence_fixed_')
                [posterior{n},stats]=fuseReviewerEvidence(inputs,[.5,.5],model,details,cfg,rule,t);
            else
                [posterior{n},stats]=fuseGaussianEvidence(inputs,[.5,.5],model,details,cfg,rule,t);
            end
            run.iterationRecords=[run.iterationRecords;stats.records]; %#ok<AGROW>
            run.weightChangeL1(n,t)=stats.weightChange;
            run.observableAbsences(n,t)=stats.observableAbsences;
            run.matchedLabels(n,t)=matching.knownPairs+matching.assignedPairs;
        end
        run.attemptedMessages(t)=2;run.deliveredMessages(t)=nnz(delivered(:,:,t));
        run.packetBytes(:,t)=sizes(:);run.rawPayloadBytes(t)=sum(sizes);run.deliveredRawBytes(t)=sum(sum(delivered(:,:,t),1).*sizes);
        run.controlBytes(t)=256;run.wireBytes(t)=sum(ceil(sizes/16384))*16384+256;
    end
    for n=1:2
        posterior{n}=reduce(posterior{n},model);
        for j=1:numel(posterior{n})
            posterior{n}(j).lastDirectOpportunity=stamp(direct{n},key(posterior{n}(j)));
        end
        [states,labels]=extract(posterior{n});
        run.rawEstimates{n,t}=states;run.labels{n,t}=labels;
        run.estimates{n,t}=crop(states,positions(:,:,t));
    end
end
run.runtimeSeconds=toc(timer);run.totalWireBytes=sum(run.wireBytes);
end

function objects=reduce(objects,model)
if isempty(objects),return;end
objects=objects([objects.r]>model.existenceThreshold);
assert(numel(objects)<=2000,'V2V:TrackBound','More than 2000 Bernoullis; no silent truncation.');
for j=1:numel(objects)
    o=objects(j);w=o.w(:);w=w/sum(w);mu=cell2mat(o.mu)*w;P=zeros(4);
    for g=1:numel(w),delta=o.mu{g}-mu;P=P+w(g)*(o.Sigma{g}+delta*delta');end
    P=(P+P')/2;[~,flag]=chol(P);assert(flag==0 && all(isfinite(P),'all'));
    assert(all(isfinite(mu)) && o.r>=0 && o.r<=1);
    objects(j).mu={mu};objects(j).Sigma={P};objects(j).w=1;objects(j).numberOfGmComponents=1;
end
keys=[[objects.birthTime];[objects.birthLocation]];assert(size(unique(keys','rows'),1)==numel(objects));
end

function [states,labels]=extract(objects)
if isempty(objects),states={};labels=zeros(2,0);return;end
[count,index]=lmbMapCardinalityEstimate([objects.r]);states=cell(1,count);labels=zeros(2,count);
for k=1:count,o=objects(index(k));states{k}=o.mu{1};labels(:,k)=[o.birthTime;o.birthLocation];end
end

function value=key(o),value=sprintf('%d_%d',o.birthTime,o.birthLocation);end
function value=stamp(map,k),value=0;if isKey(map,k),value=map(k);end;end

function states=crop(states,positions)
if isempty(states),return;end
x=cell2mat(states);delta=reshape(x(1:2,:),2,[],1)-reshape(positions,2,1,2);
nearest=min(reshape(sum(delta.^2,1),size(x,2),2),[],2)';
keep=abs(x(1,:))<=70.4 & abs(x(2,:))<=40 & nearest<=1600 & nearest>9;
states=states(keep);
end

function run=scoreOutputs(run,truth)
for t=1:numel(truth)
    for n=1:2
        states=run.estimates{n,t};x=cell2mat(states);reference=truth{t};
        parts=computePositionEuclideanOspa(num2cell(reference,1),states,12,2,[1,2]);
        run.ospa(n,t)=parts(1);run.countError(n,t)=abs(size(reference,2)-numel(states));
        if isempty(x) || isempty(reference),continue;end
        cost=reshape(sum((reshape(reference(1:2,:),2,[],1)-reshape(x(1:2,:),2,1,[])).^2,1),size(reference,2),size(x,2));
        assignment=Hungarian(min(cost,144));valid=assignment>0 & cost<144;
        run.matchedSquaredError(n,t)=sum(cost(valid));run.matchedCount(n,t)=nnz(valid);
    end
end
end

function run=runTc(local,positions,delivered,window)
T=size(delivered,3);run=emptyRun(sprintf('tc_ospa2_w%d',window),T);
histories=cell(1,2);memories=cell(1,2);
for n=1:2
    histories{n}=struct('source_id',n,'X',{cell(T,1)},'L',{cell(T,1)},'N',zeros(T,1));
    memories{n}=struct('space',{cell(2,1)},'association',{cell(2,2)});
end
ospa=struct('winlen_lm',window,'min_track_len',floor((window-1)/2), ...
    'use_consecutive_len',false,'c_lm',12,'p',1,'metric_type','ospa_union');
timer=tic;
for t=1:T
    decoded=cell(1,2);sizes=zeros(1,2);
    for n=1:2
        x=cell2mat(local.rawEstimates{n,t});if isempty(x),x=zeros(4,0);end
        histories{n}.X{t}=x([1,3,2,4],:);histories{n}.L{t}=local.labels{n,t};
        [decoded{n},packet]=tcHistoryPacket(histories{n},t,window);sizes(n)=numel(packet);
    end
    for n=1:2
        sources=[n,find(delivered(n,:,t))];
        [x,labels,memories{n}]=tcFuseReceiver(decoded(sources),sources,memories{n},ospa,t);
        if isempty(x),x=zeros(4,0);end
        states=num2cell(x([1,3,2,4],:),1);run.rawEstimates{n,t}=states;run.labels{n,t}=labels;
        run.estimates{n,t}=crop(states,positions(:,:,t));
    end
    run.attemptedMessages(t)=2;run.deliveredMessages(t)=nnz(delivered(:,:,t));
    run.packetBytes(:,t)=sizes(:);run.rawPayloadBytes(t)=sum(sizes);run.deliveredRawBytes(t)=sum(sum(delivered(:,:,t),1).*sizes);
    run.controlBytes(t)=256;run.wireBytes(t)=sum(ceil(sizes/16384))*16384+256;
end
run.runtimeSeconds=toc(timer);run.totalWireBytes=sum(run.wireBytes);
end
