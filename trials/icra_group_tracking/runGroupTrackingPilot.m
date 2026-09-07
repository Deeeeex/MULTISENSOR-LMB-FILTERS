function runGroupTrackingPilot(mode)
% Independent harness sharing repository LMB primitives; no legacy scene edits.
if nargin<1, mode='full'; end
out=fileparts(mfilename('fullpath')); root=fileparts(fileparts(out));
addpath(fullfile(root,'common'),fullfile(root,'lmb'),fullfile(root,'multisensorLmb'));
names={'split_rejoin','boundary_churn'}; seeds=[2701,2702,2703];
if ~isfolder(fullfile(out,'results')), mkdir(fullfile(out,'results')); end
geometry=cell(1,2);
for sc=1:2
    scene=buildGroupPilotScene(names{sc}); topo=buildGroupPilotTopologies(scene);
    speed=[]; acceleration=[];
    for n=1:scene.N
        v=scene.trajectories{n}(3:4,:);
        speed=[speed,sqrt(sum(v.^2,1))]; %#ok<AGROW>
        acceleration=[acceleration,sqrt(sum((diff(v,1,2)/scene.dt).^2,1))]; %#ok<AGROW>
    end
    assert(max(speed)<10 && max(acceleration)<10);
    geometry{sc}=struct('scene',names{sc},'time',scene.time, ...
        'componentCount',topo.componentCount,'reconnectionFrames',topo.reconnectionFrames, ...
        'disconnectionFrames',topo.disconnectionFrames,'pairEdits',topo.pairEdits, ...
        'edgeEdits',topo.edgeEdits,'maxSpeedMps',max(speed), ...
        'maxAccelerationMps2',max(acceleration)); %#ok<AGROW>
    fprintf('%s geometry: disconnected %d frames, %d reunions, dynamic/stable pair edits %d/%d.\n', ...
        names{sc},nnz(topo.componentCount>1),numel(topo.reconnectionFrames), ...
        sum(topo.pairEdits(3,:)),sum(topo.pairEdits(4,:)));
    if strcmp(mode,'geometry'), continue; end
    for seed=seeds
        [model,truth,measurements,uniforms,visibility]=makeInputs(scene,seed);
        if strcmp(mode,'smoke'), lastFrame=4; else, lastFrame=scene.T; end
        runCells=cell(1,4);
        for arm=1:4
            fprintf('START %s seed %d arm %s\n',names{sc},seed,topo.names{arm});
            runCells{arm}=runArm(model,truth,measurements,uniforms,scene,topo,arm,lastFrame);
            current=runCells{arm};
            fprintf('DONE mean OSPA %.4f; count MAE %.4f; %.2f MiB; %.1f s\n', ...
                current.meanOspa,current.meanCountError, ...
                current.totalWireBytes/2^20,current.runtimeSeconds);
        end
        runs=[runCells{:}];
        assert(runs(2).totalWireBytes==runs(3).totalWireBytes && ...
            runs(3).totalWireBytes==runs(4).totalWireBytes);
        result=struct('protocol','icra-group-tracking-v1','scene',names{sc}, ...
            'seed',seed,'mode',mode,'time',scene.time(1:lastFrame), ...
            'reconnectionFrames',topo.reconnectionFrames, ...
            'componentCount',topo.componentCount(1:lastFrame), ...
            'visibility',visibility(:,:,1:lastFrame),'runs',runs);
        path=fullfile(out,'results',sprintf('%s_seed%d_%s',names{sc},seed,mode));
        writeJson([path,'.json'],result);
        gzip([path,'.json']);
        save([path,'.mat'],'result','scene','topo','model','truth','measurements','uniforms','-v7');
        if strcmp(mode,'smoke'), break; end
    end
end
writeJson(fullfile(out,'geometry.json'),[geometry{:}]);
fprintf('Completed mode %s.\n',mode);
end

function [model,truth,measurements,uniforms,visibility]=makeInputs(scene,seed)
N=scene.N; T=scene.T; dt=scene.dt;
model=generateMultisensorModel(N,ones(1,N),0.9*ones(1,N),ones(1,N),'GA','LBP');
model.T=dt; model.A=[eye(2),dt*eye(2);zeros(2),eye(2)];
model.R=0.08*[dt^3/3*eye(2),dt^2/2*eye(2);dt^2/2*eye(2),dt*eye(2)];
model.survivalProbability=0.995; model.maximumNumberOfGmComponents=8;
model.sensorMotionEnabled=true; model.sensorTrajectories=scene.trajectories;
model.sensorFovEnabled=true; model.sensorFovHalfAngleDeg=180; model.sensorFovRange=90;
model.sensorQuality=struct('enabled',false);
model.observationSpaceLimits=[-130,130;-80,120]; model.observationSpaceVolume=260*200;
model.clutterPerUnitVolume=ones(1,N)/model.observationSpaceVolume;
prior=[-42,-14,14,42;-8,24,-24,8;zeros(1,4);0.6*ones(1,4)];
for j=1:4
    model.rB(j)=0.7; model.muB{j}=prior(:,j); model.SigmaB{j}=diag([12,12,.5,.5].^2);
    model.birthParameters(j).r=0.7;
    model.birthParameters(j).mu={prior(:,j)};
    model.birthParameters(j).Sigma={diag([12,12,.5,.5].^2)};
    model.birthParameters(j).trajectory=[]; model.birthParameters(j).timestamps=[];
end
model.birthActiveMask=false(4,T); model.birthActiveMask(:,1)=true;
% Truth generation is deliberately separated from the estimator's priors.
rng(seed,'twister'); active=[1,2,4]; truth=zeros(4,3,T);
truth(:,:,1)=prior(:,active)+[7*randn(2,3);.12*randn(2,3)];
for t=2:T
    acceleration=.04*randn(2,3);
    truth(:,:,t)=model.A*truth(:,:,t-1)+[.5*dt^2*acceleration;dt*acceleration];
end
measurements=cell(N,T); visibility=false(N,3,T);
for t=1:T
    for n=1:N
        z={};
        for j=1:3
            [pd,Q,info]=evaluateSensorQuality(model,n,truth(:,j,t),t);
            visibility(n,j,t)=info.inFov;
            if rand<pd, z{end+1}=truth(1:2,j,t)+chol(Q,'lower')*randn(2,1); end %#ok<AGROW>
        end
        % Knuth Poisson sampler (lambda=1), avoids extra toolbox dependency.
        product=1; count=-1;
        while product>exp(-1), product=product*rand; count=count+1; end
        for k=1:count
            z{end+1}=[-130;-80]+[260;200].*rand(2,1); %#ok<AGROW>
        end
        if ~isempty(z), z=z(randperm(numel(z))); end
        measurements{n,t}=z;
    end
end
% Link draws have a separate stream, identical across scenarios and arms.
rng(seed+100000,'twister'); uniforms=rand(N,N,T);
end

function run=runArm(model,truth,measurements,uniforms,scene,topo,arm,T)
N=scene.N; posterior=repmat({model.object},1,N);
cfg=buildMixtureAwareKlaReferenceConfig();
run=struct('arm',topo.names{arm},'ospa',zeros(N,T),'preOspa',zeros(N,T), ...
    'countError',zeros(N,T),'preCountError',zeros(N,T),'count',zeros(N,T), ...
    'matchedSquaredError',zeros(N,T),'matchedCount',zeros(N,T), ...
    'wireBytes',zeros(1,T),'payloadBytes',zeros(1,T),'controlBytes',zeros(1,T), ...
    'attemptedMessages',zeros(1,T),'deliveredMessages',zeros(1,T), ...
    'estimates',{cell(N,T)},'labels',{cell(N,T)}, ...
    'pairEdits',topo.pairEdits(arm,1:T),'edgeEdits',topo.edgeEdits(arm,1:T), ...
    'topologySeconds',sum(topo.runtimeSeconds(arm,1:T)), ...
    'runtimeSeconds',0,'meanOspa',0,'meanCountError',0,'totalWireBytes',0);
timer=tic;
for t=1:T
    local=cell(1,N);
    for n=1:N
        predicted=lmbPredictionStep(posterior{n},model,t);
        if isempty(predicted), local{n}=predicted;
        else, local{n}=updateLmbWithSensorMeasurement(predicted,measurements{n,t},model,n,t); end
        local{n}=pruneValidate(local{n},model);
        [states,~]=extract(local{n});
        [run.preOspa(n,t),run.preCountError(n,t)]=score(truth(:,:,t),states);
    end
    adjacency=topo.adjacency(:,:,t,arm);
    if arm==1
        posterior=local;
    else
        degree=sum(adjacency,2); W=zeros(N);
        for n=1:N
            for sender=find(adjacency(n,:))
                W(n,sender)=1/(1+max(degree(n),degree(sender)));
            end
            W(n,n)=1-sum(W(n,:));
        end
        decoded=cell(1,N); sizes=zeros(1,N);
        for sender=1:N
            [bytes,sizes(sender)]=encode(local{sender},sender,t);
            assert(numel(bytes)==8192 && sizes(sender)<=8192);
            decoded{sender}=decode(bytes,model,sender,t);
            assertSameDensity(local{sender},decoded{sender});
        end
        delivered=adjacency & uniforms(:,:,t)>=0.1;
        for receiver=1:N
            dropped=find(adjacency(receiver,:) & ~delivered(receiver,:));
            W(receiver,receiver)=W(receiver,receiver)+sum(W(receiver,dropped));
            W(receiver,dropped)=0;
        end
        assert(all(abs(sum(W,2)-1)<1e-12) && all(diag(W)>0));
        % Sources are all frozen pre-fusion posteriors; exactly one hop.
        posterior=cell(1,N);
        for receiver=1:N
            senders=find(delivered(receiver,:)); sources=[receiver,senders];
            if isempty(senders), posterior{receiver}=local{receiver}; continue; end
            inputs=[local(receiver),decoded(senders)];
            details=struct('eventType',[0,2*ones(1,numel(senders))]);
            posterior{receiver}=fuseLmbPosteriorsByLabel(inputs,W(receiver,sources), ...
                model,W(receiver,sources),details,cfg);
        end
        run.attemptedMessages(t)=nnz(adjacency); run.deliveredMessages(t)=nnz(delivered);
        run.payloadBytes(t)=sum(sum(adjacency,1).*sizes);
        run.controlBytes(t)=128*N;
        run.wireBytes(t)=8192*nnz(adjacency)+run.controlBytes(t);
    end
    for n=1:N
        posterior{n}=pruneValidate(posterior{n},model);
        [states,labels]=extract(posterior{n});
        run.estimates{n,t}=states; run.labels{n,t}=labels;
        [run.ospa(n,t),run.countError(n,t),run.matchedSquaredError(n,t),run.matchedCount(n,t)]=score(truth(:,:,t),states);
        run.count(n,t)=numel(states);
    end
    if mod(t,30)==0, fprintf('  frame %d/%d, OSPA %.3f\n',t,T,mean(run.ospa(:,t))); end
end
run.runtimeSeconds=toc(timer); run.meanOspa=mean(run.ospa,'all');
run.meanCountError=mean(run.countError,'all'); run.totalWireBytes=sum(run.wireBytes);
end

function objects=pruneValidate(objects,model)
if isempty(objects), return; end
objects=objects([objects.r]>model.existenceThreshold);
labels=zeros(numel(objects),2);
for k=1:numel(objects)
    o=objects(k); labels(k,:)=[o.birthTime,o.birthLocation];
    assert(isfinite(o.r) && o.r>=0 && o.r<=1);
    assert(o.numberOfGmComponents<=8 && o.numberOfGmComponents>0);
    assert(all(isfinite(o.w)) && all(o.w>=0) && abs(sum(o.w)-1)<1e-8);
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
parts=computePositionEuclideanOspa(num2cell(truth,1),states,30,2,[1,2]);
ospaValue=parts(1); countError=abs(size(truth,2)-numel(states));
squaredError=0; support=0;
if isempty(states), return; end
estimated=cell2mat(states);
cost=squeeze(sum((reshape(truth(1:2,:),2,[],1)-reshape(estimated(1:2,:),2,1,[])).^2,1));
cost=reshape(cost,size(truth,2),numel(states));
[matching,~]=Hungarian(min(cost,30^2));
valid=matching>0 & cost<30^2;
squaredError=sum(cost(valid)); support=nnz(valid);
end

function [bytes,used]=encode(objects,sender,t)
values=[1,sender,t,numel(objects)];
for k=1:numel(objects)
    o=objects(k); values=[values,o.birthTime,o.birthLocation,o.r,o.numberOfGmComponents]; %#ok<AGROW>
    for j=1:o.numberOfGmComponents
        values=[values,o.w(j),o.mu{j}(:)',o.Sigma{j}(:)']; %#ok<AGROW>
    end
end
raw=typecast(double(values),'uint8'); used=numel(raw);
assert(used<=8192); bytes=zeros(1,8192,'uint8'); bytes(1:used)=raw;
end

function objects=decode(bytes,model,sender,t)
values=typecast(bytes,'double');
assert(isequal(values(1:3),[1,sender,t])); count=values(4); cursor=5;
objects=model.object;
for k=1:count
    o=model.birthParameters(1);
    o.birthTime=values(cursor); o.birthLocation=values(cursor+1); o.r=values(cursor+2);
    o.numberOfGmComponents=values(cursor+3); cursor=cursor+4;
    o.w=zeros(1,o.numberOfGmComponents); o.mu=cell(1,o.numberOfGmComponents); o.Sigma=o.mu;
    for j=1:o.numberOfGmComponents
        o.w(j)=values(cursor); o.mu{j}=values(cursor+1:cursor+4)';
        o.Sigma{j}=reshape(values(cursor+5:cursor+20),4,4); cursor=cursor+21;
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
end
end

function writeJson(path,value)
fid=fopen(path,'w'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s',jsonencode(value));
end
