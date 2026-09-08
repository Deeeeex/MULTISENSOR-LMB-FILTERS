function runExternalCaseStudies(firstSeed,lastSeed)
if nargin<1, firstSeed=2901; end
if nargin<2, lastSeed=2920; end
out=fileparts(mfilename('fullpath')); root=fileparts(fileparts(out));
addpath(fullfile(root,'common')); setupTcDependency();
if ~isfolder(fullfile(out,'results')), mkdir(fullfile(out,'results')); end
scenes={'split_latebirth','churn_departure','split_no_new'};
for seed=firstSeed:lastSeed
    for sc=1:numel(scenes)
        stem=sprintf('%s_seed%d_validation',scenes{sc},seed);
        input=fullfile(root,'trials','icra_reunion_fusion','results',[stem,'.mat']);
        saved=load(input,'result','uniforms','adjacency'); base=saved.result;
        assert(strcmp(base.runs(1).arm,'local') && numel(base.time)==120);
        extended=base; extended.protocol='external-tc-case-studies-v1';
        newRuns=cell(1,2);
        for wi=1:2
            windows=[5,10]; window=windows(wi); T=numel(base.time); N=8;
            run=base.runs(1); run.arm=sprintf('tc_ospa2_w%d',window);
            run.rawPayloadBytes(:)=0; run.deliveredRawBytes(:)=0;
            run.wireBytes(:)=0; run.controlBytes(:)=0;
            run.attemptedMessages(:)=0; run.deliveredMessages(:)=0;
            run.preEstimates=base.runs(1).estimates;
            run.preOspa=base.runs(1).ospa; run.preCountError=base.runs(1).countError;
            memories=cell(1,N); histories=cell(1,N);
            for n=1:N
                memories{n}=struct('space',{cell(N,1)},'association',{cell(N,N)});
                histories{n}=struct('source_id',n,'X',{cell(T,1)},'L',{cell(T,1)},'N',zeros(T,1));
            end
            ospa=struct('winlen_lm',window,'min_track_len',floor((window-1)/2), ...
                'use_consecutive_len',false,'c_lm',12,'p',1,'metric_type','ospa_union');
            timer=tic;
            for t=1:T
                packets=cell(1,N); decoded=cell(1,N); sizes=zeros(1,N);
                for n=1:N
                    x=cell2mat(base.runs(1).estimates{n,t});
                    if isempty(x), x=zeros(4,0); end
                    l=base.runs(1).labels{n,t}; if isempty(l), l=zeros(2,0); end
                    l(2,:)=n*1e5+l(2,:); % No shared-label shortcut for TC.
                    histories{n}.X{t}=x([1,3,2,4],:); % Author [x,vx,y,vy].
                    histories{n}.L{t}=l; histories{n}.N(t)=size(l,2);
                    [decoded{n},packets{n}]=tcHistoryPacket(histories{n},t,window);
                    sizes(n)=numel(packets{n});
                end
                graphNow=logical(saved.adjacency(:,:,t));
                delivered=graphNow & saved.uniforms(:,:,t)>=.1;
                for n=1:N
                    sources=[n,find(delivered(n,:))];
                    [x,l,memories{n}]=tcFuseReceiver(decoded(sources),sources,memories{n},ospa,t);
                    if isempty(x), x=zeros(4,0); end
                    states=num2cell(x([1,3,2,4],:),1);
                    run.estimates{n,t}=states; run.labels{n,t}=l;
                    parts=computePositionEuclideanOspa(num2cell(base.truth{t},1),states,12,2,[1,2]);
                    run.ospa(n,t)=parts(1); run.countError(n,t)=abs(size(base.truth{t},2)-numel(states));
                    [run.matchedSquaredError(n,t),run.matchedCount(n,t)]=matchedError(base.truth{t},x([1,3,2,4],:));
                end
                run.attemptedMessages(t)=nnz(graphNow); run.deliveredMessages(t)=nnz(delivered);
                run.rawPayloadBytes(t)=sum(sum(graphNow,1).*sizes);
                run.deliveredRawBytes(t)=sum(sum(delivered,1).*sizes);
                run.controlBytes(t)=128*N;
                run.wireBytes(t)=sum(sum(graphNow,1).*ceil(sizes/16384)*16384)+128*N;
            end
            run.runtimeSeconds=toc(timer); run.totalWireBytes=sum(run.wireBytes);
            assert(isequal(run.deliveredMessages,base.runs(2).deliveredMessages));
            fprintf('DONE %s seed %d %s OSPA %.6f fusion %.2fs raw %.0f B\n', ...
                scenes{sc},seed,run.arm,mean(run.ospa,'all'),run.runtimeSeconds,sum(run.rawPayloadBytes));
            newRuns{wi}=run;
        end
        extended.runs=[newRuns{:}]; extended.originalLocalRuntimeSeconds=base.runs(1).runtimeSeconds;
        path=fullfile(out,'results',[stem,'_tc.json']);
        fid=fopen(path,'w'); assert(fid>=0); fprintf(fid,'%s',jsonencode(extended)); fclose(fid); gzip(path);
        delete(path);
    end
end
fprintf('COMPLETED external TC cases seeds %d--%d\n',firstSeed,lastSeed);
end

function [sse,count]=matchedError(truth,x)
sse=0; count=0; if isempty(truth) || isempty(x), return; end
cost=reshape(sum((reshape(truth(1:2,:),2,[],1)-reshape(x(1:2,:),2,1,[])).^2,1),size(truth,2),size(x,2));
assignment=Hungarian(min(cost,144)); valid=assignment>0 & cost<144;
sse=sum(cost(valid)); count=nnz(valid);
end
