function runConservativeReplay(firstSequence,lastSequence,smoke,arms)
% Generated from the frozen replay by make_replay_runner.py.
if nargin<1,firstSequence=0;end
if nargin<2,lastSequence=8;end
if nargin<3,smoke=false;end
if nargin<4,arms={'conservative_recency'};end
out=fileparts(mfilename('fullpath'));root=fileparts(fileparts(out));
original=fullfile(root,'trials','icra_external_fusion');
addpath(fullfile(root,'common'),fullfile(root,'lmb'),fullfile(root,'multisensorLmb'), ...
    fullfile(root,'trials','icra_reunion_fusion'),original);
baseQuality=@evaluateSensorQuality;
assert(strcmp(functions(baseQuality).file,fullfile(root,'common','evaluateSensorQuality.m')));
qualityPath=fullfile(original,'replay_quality');addpath(qualityPath);
cleanup=onCleanup(@()rmpath(qualityPath)); %#ok<NASGU>
resultDir=fullfile(out,'results_v2v_conservative');if ~isfolder(resultDir),mkdir(resultDir);end
manifest=jsondecode(fileread(fullfile(original,'v2v4real_input_manifest.json')));
sourceManifest=jsondecode(fileread(fullfile(out,'source_sha256_cr.json')));
for seq=firstSequence:lastSequence
    data=load(fullfile(original,'data',sprintf('v2v4real_%04d.mat',seq)));
    T=double(data.T);if smoke,T=min(T,12);end
    measurements=data.measurements(:,1:T);positions=data.positions(:,:,1:T);
    model=makeModel(positions,T,baseQuality);
    for condition={'reliable','intermittent'}
        radio=condition{1};delivered=repmat(logical([0,1;1,0]),1,1,T);
        if strcmp(radio,'intermittent')
            rng(8301+seq,'twister');draws=rand(2,2,T);delivered=delivered & draws>=.1;
            delivered(:,:,floor(.4*T)+1:floor(.6*T))=false;
        end
        runs=cell(1,numel(arms));
        for a=1:numel(arms)
            fprintf('START iteration V2V %04d %s %s %d frames\n',seq,radio,arms{a},T);
            runs{a}=runDensity(model,measurements,positions,delivered,arms{a});
            % Truth first enters here, after the complete tracking run.
            runs{a}=scoreOutputs(runs{a},data.truth(1:T));
            fprintf('DONE iteration V2V %04d %s %s OSPA %.6f count %.6f runtime %.2fs\n', ...
                seq,radio,arms{a},mean(runs{a}.ospa,'all'),mean(runs{a}.countError,'all'),runs{a}.runtimeSeconds);
        end
        result=struct('protocol','conservative-existence-recency-v1','implementation','cr-v1', ...
            'sourceSha256',sourceManifest,'inputSha256',manifest.sequences(seq+1).input_sha256, ...
            'sequence',sprintf('%04d',seq),'condition',radio,'smoke',smoke, ...
            'time',double(data.time(1:T)),'positions',positions,'truth',{data.truth(1:T)}, ...
            'truthIds',{data.truthIds(1:T)},'delivered',delivered,'runs',[runs{:}]);
        suffix='';if smoke,suffix='_smoke';end
        path=fullfile(resultDir,sprintf('%04d_%s%s.json',seq,radio,suffix));
        fid=fopen(path,'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(result));fclose(fid);gzip(path);delete(path);
    end
end
fprintf('COMPLETED iteration sequences %04d--%04d smoke=%d\n',firstSequence,lastSequence,smoke);
end

function model=makeModel(positions,T,baseQuality)
model=generateMultisensorModel(2,[3,3],[.9,.9],[1,1],'GA','LBP');dt=.1;
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
template.lastDirectOpportunity=0;template.localLogOddsIncrement=0;template.positiveConfirmation=false;
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
    'iterationRecords',zeros(0,26),'maximumBernoulliCount',0,'runtimeSeconds',0,'totalWireBytes',0);
end

function run=runDensity(model,measurements,positions,delivered,arm)
% No truth, IDs from annotations, future detections or other-source births.
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
        for j=1:numel(predicted)
            o=predicted(j);pd=evaluateSensorQuality(model,n,o.mu{1},t);
            if pd>0, direct{n}(key(o))=t;predicted(j).hasObservationLineage=true;end
            predicted(j).lastDirectOpportunity=stamp(direct{n},key(o));
        end
        if isempty(predicted),local{n}=predicted;
        else,local{n}=updateLmbWithSensorMeasurement(predicted,num2cell(measurements{n,t},1),model,n,t);end
        local{n}=reduce(local{n},model);
        run.maximumBernoulliCount=max(run.maximumBernoulliCount,numel(local{n}));
    end
    if strcmp(arm,'local')
        posterior=local;
    else
        decoded=cell(1,2);sizes=zeros(1,2);
        for n=1:2,[decoded{n},packet]=gaussianLmbPacket(local{n},model,n,t);sizes(n)=numel(packet);end
        for n=1:2
            other=3-n;
            if ~delivered(n,other,t),posterior{n}=local{n};continue;end
            [remote,matching]=alignGaussianLmbPair(local{n},decoded{other},50);
            inputs={local{n},remote};details=struct('eventType',[0,2],'sourceIndices',[n,other], ...
                'isStale',[false,false],'isSelf',[true,false],'currentTime',t);
            [posterior{n},stats]=fuseConservativeRecency(inputs,[.5,.5],model,details,cfg,arm,t);
            run.iterationRecords=[run.iterationRecords;stats.records]; %#ok<AGROW>
            run.weightChangeL1(n,t)=stats.weightChange;
            run.observableAbsences(n,t)=stats.observableAbsences;
            run.matchedLabels(n,t)=matching.knownPairs+matching.assignedPairs;
        end
        run.attemptedMessages(t)=2;run.deliveredMessages(t)=nnz(delivered(:,:,t));
        run.rawPayloadBytes(t)=sum(sizes);run.deliveredRawBytes(t)=sum(sum(delivered(:,:,t),1).*sizes);
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
