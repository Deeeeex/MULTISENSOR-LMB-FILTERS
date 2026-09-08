function runConfirmedCases(firstSeed,lastSeed)
if nargin<1,firstSeed=2901;end
if nargin<2,lastSeed=2920;end
assert(firstSeed>=2901 && lastSeed<=2920);
out=fileparts(mfilename('fullpath'));root=fileparts(fileparts(out));
original=fullfile(root,'trials','icra_reunion_fusion');
addpath(fullfile(root,'common'),fullfile(root,'lmb'),fullfile(root,'multisensorLmb'),original, ...
    fullfile(root,'trials','icra_group_tracking'));
dest=fullfile(out,'results_cases_confirmed');if ~isfolder(dest),mkdir(dest);end
sources=jsondecode(fileread(fullfile(out,'source_sha256_cgr_cases.json')));
for seed=firstSeed:lastSeed
    for name={'split_latebirth','churn_departure','split_no_new'}
        stem=sprintf('%s_seed%d_validation',name{1},seed);
        data=load(fullfile(original,'results',[stem,'.mat']));
        fprintf('START CR case %s seed %d\n',name{1},seed);
        run=runArm(data.model,data.truth,data.measurements,data.uniforms,data.scene, ...
            data.adjacency,'confirmation_recency',data.scene.T);
        reference=data.result.runs(strcmp({data.result.runs.arm},'qualified_exist'));
        assert(run.totalWireBytes==reference.totalWireBytes);
        assert(isequal(run.deliveredMessages,reference.deliveredMessages));
        result=data.result;result.runs=run;result.protocol='confirmation-recency-case-studies-v1';
        result.sourceSha256=sources;
        path=fullfile(dest,[stem,'_cgr.json']);writeJson(path,result);gzip(path);delete(path);
        fprintf('DONE CR case %s seed %d OSPA %.6f count %.6f runtime %.2fs\n', ...
            name{1},seed,mean(run.ospa,'all'),mean(run.countError,'all'),run.runtimeSeconds);
    end
end
fprintf('COMPLETED CR cases seeds %d--%d\n',firstSeed,lastSeed);
end

function run=runArm(model,truth,measurements,uniforms,scene,adjacency,arm,T)
N=scene.N; K=model.numberOfBirthLocations;
posterior=repmat({model.object},1,N); directTimes=zeros(N,K); localHits=zeros(N,K);
cfg=buildMixtureAwareKlaReferenceConfig();
if ismember(arm,{'fov','lineage','recent'}), cfg.missingLabelFusionMode='fov-aware-censored'; end
if strcmp(arm,'lineage'), cfg.untouchedPriorExclusionEnabled=true; end
if strcmp(arm,'mil'), cfg.lmbFusionRule='mil-common-label'; end
run=struct('arm',arm,'ospa',zeros(N,T),'preOspa',zeros(N,T), ...
    'countError',zeros(N,T),'preCountError',zeros(N,T), ...
    'matchedSquaredError',zeros(N,T),'matchedCount',zeros(N,T), ...
    'wireBytes',zeros(1,T),'rawPayloadBytes',zeros(1,T), ...
    'deliveredRawBytes',zeros(1,T),'controlBytes',zeros(1,T), ...
    'attemptedMessages',zeros(1,T),'deliveredMessages',zeros(1,T), ...
    'estimates',{cell(N,T)},'preEstimates',{cell(N,T)},'labels',{cell(N,T)}, ...
    'labelExistence',zeros(N,K,T),'directOpportunity',zeros(N,K,T), ...
    'weightChangeL1',zeros(N,T),'lineageExclusions',zeros(N,T), ...
    'observableAbsences',zeros(N,T), ...
    'localAssociationMass',zeros(N,K,T),'confirmation',false(N,K,T), ...
    'runtimeSeconds',0,'totalWireBytes',0);
timer=tic;
for t=1:T
    local=cell(1,N);
    for n=1:N
        predicted=lmbPredictionStep(posterior{n},model,t);
        for k=1:numel(predicted)
            o=predicted(k); pd=0;
            for j=1:o.numberOfGmComponents
                pd=pd+o.w(j)*evaluateSensorQuality(model,n,o.mu{j},t);
            end
            if pd>0
                directTimes(n,o.birthLocation)=t;
                predicted(k).hasObservationLineage=true;
            end
            predicted(k).lastDirectOpportunity=directTimes(n,o.birthLocation);
        end
        if isempty(predicted), local{n}=predicted;
        else, local{n}=updateLmbWithSensorMeasurement(predicted,measurements{n,t},model,n,t); end
        previousHits=localHits(n,:); localHits(n,:)=0;
        for j=1:numel(local{n})
            region=local{n}(j).birthLocation; mass=0;
            if ~isempty(measurements{n,t}) && directTimes(n,region)==t
                mass=local{n}(j).detectionAssociationMass;
            end
            if mass>=.5, localHits(n,region)=min(2,previousHits(region)+1); end
            local{n}(j).positiveConfirmation=localHits(n,region)>=2;
            run.localAssociationMass(n,region,t)=mass;
        end
        local{n}=pruneValidate(local{n},model);
        [states,~]=extract(local{n}); run.preEstimates{n,t}=states;
        [run.preOspa(n,t),run.preCountError(n,t)]=score(truth{t},states);
    end
    graphNow=adjacency(:,:,t);
    if strcmp(arm,'local')
        posterior=local;
    else
        degree=sum(graphNow,2); W=zeros(N);
        for n=1:N
            for s=find(graphNow(n,:)), W(n,s)=1/(1+max(degree(n),degree(s))); end
            W(n,n)=1-sum(W(n,:));
        end
        decoded=cell(1,N); sizes=zeros(1,N);
        for s=1:N
            [packet,sizes(s)]=encode(local{s},s,t);
            decoded{s}=decode(packet,model,s,t);
            assertSameDensity(local{s},decoded{s});
        end
        delivered=graphNow & uniforms(:,:,t)>=.1;
        for n=1:N
            dropped=find(graphNow(n,:) & ~delivered(n,:));
            W(n,n)=W(n,n)+sum(W(n,dropped)); W(n,dropped)=0;
        end
        assert(all(abs(sum(W,2)-1)<1e-12));
        posterior=cell(1,N);
        for n=1:N
            senders=find(delivered(n,:)); sources=[n,senders];
            if isempty(senders), posterior{n}=local{n}; continue; end
            inputs=[local(n),decoded(senders)]; weights=W(n,sources);
            details=struct('eventType',[0,2*ones(1,numel(senders))], ...
                'sourceIndices',sources,'isStale',false(size(sources)), ...
                'isSelf',[true,false(1,numel(senders))],'currentTime',t);
            [posterior{n},stats]=fuseConfirmedRecency(inputs,weights,model,details,cfg,arm,t);
            run.weightChangeL1(n,t)=stats.weightChange;
            run.lineageExclusions(n,t)=stats.lineageExcluded;
            run.observableAbsences(n,t)=stats.observableAbsences;
        end
        run.attemptedMessages(t)=nnz(graphNow); run.deliveredMessages(t)=nnz(delivered);
        run.rawPayloadBytes(t)=sum(sum(graphNow,1).*sizes);
        run.deliveredRawBytes(t)=sum(sum(delivered,1).*sizes);
        run.controlBytes(t)=128*N;
        run.wireBytes(t)=16384*nnz(graphNow)+run.controlBytes(t);
    end
    for n=1:N
        posterior{n}=pruneValidate(posterior{n},model);
        for k=1:numel(posterior{n})
            j=posterior{n}(k).birthLocation;
            posterior{n}(k).lastDirectOpportunity=directTimes(n,j);
            posterior{n}(k).positiveConfirmation=localHits(n,j)>=2;
            run.labelExistence(n,j,t)=posterior{n}(k).r;
            run.confirmation(n,j,t)=posterior{n}(k).positiveConfirmation;
        end
        [states,labels]=extract(posterior{n});
        run.estimates{n,t}=states; run.labels{n,t}=labels;
        [run.ospa(n,t),run.countError(n,t),run.matchedSquaredError(n,t),run.matchedCount(n,t)]=score(truth{t},states);
    end
    run.directOpportunity(:,:,t)=directTimes;
end
run.runtimeSeconds=toc(timer); run.totalWireBytes=sum(run.wireBytes);
end

function objects=pruneValidate(objects,model)
if isempty(objects), return; end
objects=objects([objects.r]>model.existenceThreshold);
labels=zeros(numel(objects),2);
for k=1:numel(objects)
    o=objects(k); labels(k,:)=[o.birthTime,o.birthLocation];
    assert(isfinite(o.r) && o.r>=0 && o.r<=1 && o.numberOfGmComponents<=8);
    assert(o.numberOfGmComponents>0 && abs(sum(o.w)-1)<1e-8);
    for j=1:o.numberOfGmComponents
        assert(all(isfinite(o.mu{j})) && all(isfinite(o.Sigma{j}),'all'));
        [~,flag]=chol((o.Sigma{j}+o.Sigma{j}')/2); assert(flag==0);
    end
end
assert(size(unique(labels,'rows'),1)==numel(objects));
end

function [states,labels]=extract(objects)
if isempty(objects), states={}; labels=zeros(2,0); return; end
[count,indices]=lmbMapCardinalityEstimate([objects.r]);
states=cell(1,count); labels=zeros(2,count);
for k=1:count
    o=objects(indices(k)); [~,j]=max(o.w); states{k}=o.mu{j};
    labels(:,k)=[o.birthTime;o.birthLocation];
end
end

function [ospaValue,countError,squaredError,support]=score(truth,states)
parts=computePositionEuclideanOspa(num2cell(truth,1),states,12,2,[1,2]);
ospaValue=parts(1); countError=abs(size(truth,2)-numel(states));
squaredError=0; support=0;
if isempty(states) || isempty(truth), return; end
estimated=cell2mat(states);
cost=squeeze(sum((reshape(truth(1:2,:),2,[],1)-reshape(estimated(1:2,:),2,1,[])).^2,1));
cost=reshape(cost,size(truth,2),numel(states));
[matching,~]=Hungarian(min(cost,12^2));
valid=matching>0 & cost<12^2;
squaredError=sum(cost(valid)); support=nnz(valid);
end

function [packet,used]=encode(objects,sender,t)
values=[1,sender,t,numel(objects)];
for k=1:numel(objects)
    o=objects(k);
    values=[values,o.birthTime,o.birthLocation,o.r,o.numberOfGmComponents, ...
        double(o.hasObservationLineage),o.lastDirectOpportunity,double(o.positiveConfirmation)]; %#ok<AGROW>
    for j=1:o.numberOfGmComponents
        values=[values,o.w(j),o.mu{j}(:)',o.Sigma{j}(:)']; %#ok<AGROW>
    end
end
raw=typecast(double(values),'uint8'); used=numel(raw);
assert(used<=16384); packet=zeros(1,16384,'uint8'); packet(1:used)=raw;
end

function objects=decode(packet,model,sender,t)
v=typecast(packet,'double'); assert(isequal(v(1:3),[1,sender,t]));
count=v(4); cursor=5; objects=model.object;
for k=1:count
    o=model.birthParameters(1);
    o.birthTime=v(cursor); o.birthLocation=v(cursor+1); o.r=v(cursor+2);
    o.numberOfGmComponents=v(cursor+3); o.hasObservationLineage=logical(v(cursor+4));
    o.lastDirectOpportunity=v(cursor+5); o.positiveConfirmation=logical(v(cursor+6)); cursor=cursor+7;
    assert(o.lastDirectOpportunity<=t);
    o.w=zeros(1,o.numberOfGmComponents); o.mu=cell(1,o.numberOfGmComponents); o.Sigma=o.mu;
    for j=1:o.numberOfGmComponents
        o.w(j)=v(cursor); o.mu{j}=v(cursor+1:cursor+4)';
        o.Sigma{j}=reshape(v(cursor+5:cursor+20),4,4); cursor=cursor+21;
    end
    objects(k)=o;
end
end

function assertSameDensity(a,b)
assert(numel(a)==numel(b));
for k=1:numel(a)
    assert(a(k).birthTime==b(k).birthTime && a(k).birthLocation==b(k).birthLocation);
    assert(a(k).r==b(k).r && isequal(a(k).w,b(k).w));
    assert(isequal(a(k).mu,b(k).mu) && isequal(a(k).Sigma,b(k).Sigma));
    assert(a(k).hasObservationLineage==b(k).hasObservationLineage);
    assert(a(k).lastDirectOpportunity==b(k).lastDirectOpportunity);
    assert(a(k).positiveConfirmation==b(k).positiveConfirmation);
end
end

function writeJson(path,value)
fid=fopen(path,'w'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s',jsonencode(value));
end
