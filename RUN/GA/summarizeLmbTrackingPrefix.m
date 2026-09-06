function s=summarizeLmbTrackingPrefix(run)
% Shared prefix metric contract: same definitions as completed V288/V290.
N=numel(run.estimates); T=run.maximumTime; truth=run.groundTruthRfs; cfg=run.config;
E=zeros(N,T); C=E; R=nan(N,T);
groups=unique(cfg.sensorGroupIds,'stable'); reps=zeros(size(groups));
for f=1:numel(groups), reps(f)=find(cfg.sensorGroupIds==groups(f),1); end
for i=1:N
    R(i,:)=computeSetRmseOverTime(run.estimates{i},truth);
    for t=1:T
        metric=computePositionEuclideanOspa(truth.x{t},run.estimates{i}.mu{t}, ...
            cfg.ospaPositionCutoff,2,[1,2]);
        E(i,t)=metric(1); C(i,t)=abs(numel(run.estimates{i}.mu{t})-numel(truth.x{t}));
    end
end
disagreement=zeros(1,T);
for t=1:T
    values=[];
    for a=1:numel(reps)-1
        for b=a+1:numel(reps)
            metric=computePositionEuclideanOspa(run.estimates{reps(a)}.mu{t}, ...
                run.estimates{reps(b)}.mu{t},cfg.ospaPositionCutoff,2,[1,2]);
            values(end+1)=metric(1); %#ok<AGROW>
        end
    end
    disagreement(t)=mean(values);
end
nodeR=zeros(N,1); for i=1:N,nodeR(i)=finiteMean(R(i,:));end
formationE=zeros(size(groups)); formationR=formationE;
for f=1:numel(groups)
    selected=cfg.sensorGroupIds==groups(f);
    values=E(selected,:); formationE(f)=mean(values(:));
    formationR(f)=finiteMean(R(selected,:));
end
bytes=run.attemptedPayloadBytes;
s=struct('eospa',mean(E(:)),'conditionalRmse',finiteMean(R(:)), ...
    'countError',mean(C(:)),'representativeDisagreement',mean(disagreement), ...
    'finiteRmseFraction',nnz(isfinite(R))/numel(R),'attemptedBytes',sum(bytes(:)), ...
    'deliveredBytes',sum(bytes(logical(run.delivered))), ...
    'attemptedMessages',nnz(run.attempted),'deliveredMessages',nnz(run.delivered), ...
    'eospaBySensorTime',E,'rmseBySensorTime',R,'countErrorBySensorTime',C, ...
    'disagreementByTime',disagreement,'perSensorEospa',mean(E,2), ...
    'perSensorRmse',nodeR,'perFormationEospa',formationE, ...
    'perFormationRmse',formationR,'worstSensorEospa',max(mean(E,2)), ...
    'representatives',reps);
end

function v=finiteMean(x)
x=x(isfinite(x)); if isempty(x),v=NaN;else,v=mean(x(:));end
end
