function runRobotReunionPilot(mode)
% Bounded scene/fusion change; shared estimator primitives remain untouched.
if nargin<1, mode='full'; end
out=fileparts(mfilename('fullpath')); root=fileparts(fileparts(out));
addpath(fullfile(root,'common'),fullfile(root,'lmb'),fullfile(root,'multisensorLmb'), ...
    fullfile(root,'trials','icra_group_tracking'));
if ~isfolder(fullfile(out,'results')), mkdir(fullfile(out,'results')); end
scenes={'split_latebirth','churn_departure','split_no_new'};
arms={'local','kla','fov','lineage','mil','recent'}; seeds=[2801,2802,2803];
for sc=1:numel(scenes)
    scene=buildRobotReunionScene(scenes{sc}); topo=buildGroupPilotTopologies(scene);
    scene.componentCount=topo.componentCount;
    scene.reconnectionFrames=topo.reconnectionFrames;
    adjacency=topo.adjacency(:,:,:,2);
    assert(all(squeeze(sum(sum(adjacency,1),2))'==2*(scene.N-scene.componentCount)));
    maxSpeed=0; maxAcceleration=0;
    for n=1:scene.N
        v=scene.trajectories{n}(3:4,:);
        maxSpeed=max(maxSpeed,max(vecnorm(v)));
        maxAcceleration=max(maxAcceleration,max(vecnorm(diff(v,1,2)/scene.dt)));
    end
    assert(maxSpeed<3 && maxAcceleration<2.1);
    fprintf('GEOMETRY %s speed %.3f acceleration %.3f reunions %d\n', ...
        scenes{sc},maxSpeed,maxAcceleration,numel(scene.reconnectionFrames));
    if strcmp(mode,'geometry'), continue; end
    for seed=seeds
        [model,truth,truthRegions,measurements,uniforms,visibility]=makeInputs(scene,seed);
        T=scene.T; if strcmp(mode,'smoke'), T=4; end
        runCells=cell(1,numel(arms));
        for a=1:numel(arms)
            fprintf('START %s seed %d arm %s\n',scenes{sc},seed,arms{a});
            runCells{a}=runArm(model,truth,measurements,uniforms,scene,adjacency,arms{a},T);
            r=runCells{a};
            fprintf('DONE OSPA %.6f count %.6f wire %.3f MiB runtime %.2fs\n', ...
                mean(r.ospa,'all'),mean(r.countError,'all'),r.totalWireBytes/2^20,r.runtimeSeconds);
        end
        runs=[runCells{:}];
        assert(numel(unique([runs(2:end).totalWireBytes]))==1);
        for a=3:numel(runs)
            assert(isequal(runs(a).deliveredMessages,runs(2).deliveredMessages));
        end
        result=struct('protocol','robot-reunion-fusion-v1','scene',scene.name, ...
            'seed',seed,'mode',mode,'time',scene.time(1:T), ...
            'componentCount',scene.componentCount(1:T), ...
            'reconnectionFrames',scene.reconnectionFrames, ...
            'truth',{truth(1:T)},'truthRegions',{truthRegions(1:T)}, ...
            'visibility',visibility(:,:,1:T),'runs',runs);
        prefix=fullfile(out,'results',sprintf('%s_seed%d_%s',scene.name,seed,mode));
        writeJson([prefix,'.json'],result); gzip([prefix,'.json']);
        save([prefix,'.mat'],'result','model','scene','truth','truthRegions', ...
            'measurements','uniforms','adjacency','-v7');
        if strcmp(mode,'smoke'), break; end
    end
end
fprintf('COMPLETED %s\n',mode);
end

function [model,truth,truthRegions,measurements,uniforms,visibility]=makeInputs(scene,seed)
N=scene.N; T=scene.T; dt=scene.dt; K=size(scene.priors,2);
model=generateMultisensorModel(N,ones(1,N),.9*ones(1,N),.4*ones(1,N),'GA','LBP');
model.T=dt; model.A=[eye(2),dt*eye(2);zeros(2),eye(2)];
model.R=.0128*[dt^3/3*eye(2),dt^2/2*eye(2);dt^2/2*eye(2),dt*eye(2)];
model.survivalProbability=.995; model.maximumNumberOfGmComponents=8;
model.sensorMotionEnabled=true; model.sensorTrajectories=scene.trajectories;
model.sensorFovEnabled=true; model.sensorFovHalfAngleDeg=180; model.sensorFovRange=14;
model.sensorQuality=struct('enabled',false);
model.observationSpaceLimits=[-52,52;-32,48]; model.observationSpaceVolume=104*80;
model.clutterPerUnitVolume=ones(1,N)/model.observationSpaceVolume;
template=model.birthParameters(1); template.hasObservationLineage=false;
template.lastDirectOpportunity=0;
model.numberOfBirthLocations=K; model.birthLocationLabels=1:K;
model.birthParameters=repmat(template,1,K); model.object=template([]);
model.rB=.1*ones(1,K); model.muB=cell(1,K); model.SigmaB=cell(1,K);
model.birthActiveMask=false(K,T);
for k=1:K
    model.muB{k}=scene.priors(:,k); model.SigmaB{k}=diag([3.2,3.2,.2,.2].^2);
    o=template; o.birthLocation=k; o.r=.1;
    o.mu={model.muB{k}}; o.Sigma={model.SigmaB{k}};
    o.trajectory=[]; o.timestamps=[];
    model.birthParameters(k)=o;
    model.birthActiveMask(k,scene.birthFrames(k))=true;
end
% Truth streams are separate from birth priors and shared across every arm.
rng(seed,'twister'); allTruth=nan(4,K,T);
for k=1:K
    first=scene.birthFrames(k);
    allTruth(:,k,first)=scene.priors(:,k)+[1.2*randn(2,1);.048*randn(2,1)];
    for t=first+1:T
        acceleration=.016*randn(2,1);
        allTruth(:,k,t)=model.A*allTruth(:,k,t-1)+[.5*dt^2*acceleration;dt*acceleration];
    end
end
truth=cell(1,T); truthRegions=cell(1,T);
for t=1:T
    active=scene.activeRegions(scene.birthFrames(scene.activeRegions)<=t);
    if t>=scene.departureFrame, active(active==3)=[]; end
    truthRegions{t}=active; truth{t}=allTruth(:,active,t);
end
% All regions receive draws even when inactive. No arm-dependent RNG calls.
rng(seed+10000,'twister'); detectionDraws=rand(N,K,T); noise=randn(2,N,K,T);
measurements=cell(N,T); visibility=false(N,K,T);
for t=1:T
    for n=1:N
        z={};
        for k=truthRegions{t}
            [pd,Q,info]=evaluateSensorQuality(model,n,allTruth(:,k,t),t);
            visibility(n,k,t)=info.inFov;
            if detectionDraws(n,k,t)<pd
                z{end+1}=allTruth(1:2,k,t)+chol(Q,'lower')*noise(:,n,k,t); %#ok<AGROW>
            end
        end
        product=1; count=-1;
        while product>exp(-1), product=product*rand; count=count+1; end
        for k=1:count, z{end+1}=[-52;-32]+[104;80].*rand(2,1); end %#ok<AGROW>
        if ~isempty(z), z=z(randperm(numel(z))); end
        measurements{n,t}=z;
    end
end
rng(seed+100000,'twister'); uniforms=rand(N,N,T);
end

function run=runArm(model,truth,measurements,uniforms,scene,adjacency,arm,T)
N=scene.N; K=model.numberOfBirthLocations;
posterior=repmat({model.object},1,N); directTimes=zeros(N,K);
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
    'observableAbsences',zeros(N,T),'runtimeSeconds',0,'totalWireBytes',0);
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
            [posterior{n},stats]=fuseRobotReunionInputs(inputs,weights,model,details,cfg,arm,t);
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
            run.labelExistence(n,j,t)=posterior{n}(k).r;
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
        double(o.hasObservationLineage),o.lastDirectOpportunity]; %#ok<AGROW>
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
    o.lastDirectOpportunity=v(cursor+5); cursor=cursor+6;
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
end
end

function writeJson(path,value)
fid=fopen(path,'w'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s',jsonencode(value));
end
