function result = runWhalesRoutingPreflight()
% Geometry-only execution of unchanged routing policies on external data.
out = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(out));
addpath(fullfile(root,'common'),fullfile(root,'multisensorLmb'));
s = load(fullfile(out,'converted_scene.mat'));
N = size(s.positions,2); T = size(s.positions,3);
assert(N==9 && T==99 && all(abs(diff(s.timestamps)-0.5)<1e-12));
groups = repelem(1:3,3);
context = struct('localPosteriorBySensor',{repmat({struct([])},1,N)}, ...
    'model',struct('dynamicTopologyScenario',struct('config', ...
    struct('sensorGroupIds',groups))), ...
    'sensorPhysicalUids',double(s.sensorSlots(:)')+1, ...
    'formationPhysicalUidsBySensor',groups, ...
    'baseAdjacency',false(N),'currentTime',1, ...
    'previousAdjacencyHistory',false(N,N,0));
functions = {@selectCausalMinimalEditFormationTreeV240Policy, ...
             @selectCausalMinimumFormationBackboneV242Policy};
names = {'full_causal','sparse_v242'};
result = struct('contractVersion','whales-routing-preflight-v1', ...
    'scene','2024-02-26-07-10-14','radioRangeM',100, ...
    'frameCount',T,'timestamps',s.timestamps, ...
    'trackingExecuted',false,'observationsGenerated',false, ...
    'physicalConnected',false(1,T),'groupCyclesFeasible',false(1,T));
for t=1:T
    p=s.positions(:,:,t);
    distances=sqrt(sum((permute(p,[2,3,1])-permute(p,[3,2,1])).^2,3));
    physical=distances<=100 & ~eye(N);
    result.physicalConnected(t)=max(conncomp(graph(physical)))==1;
    result.groupCyclesFeasible(t)=all(arrayfun(@(g) ...
        all(physical(3*g-2:3*g,3*g-2:3*g)|eye(3),'all'),1:3));
end
for arm=1:2
    passed=false(1,T); errors=cell(1,T); messages=nan(1,T);
    context.previousAdjacencyHistory=false(N,N,0);
    for t=1:T
        p=s.positions(:,:,t);
        distances=sqrt(sum((permute(p,[2,3,1])-permute(p,[3,2,1])).^2,3));
        context.currentTime=t;
        context.positions=p;
        context.physicalAdjacency=distances<=100 & ~eye(N);
        context.commConfig=struct('pDropByEdge',0.1*ones(N));
        context.directedMessageBudget=2*N;
        if arm==2, context.directedMessageBudget=N+2*(3-1); end
        try
            [adjacency,details]=functions{arm}(context);
            passed(t)=true; messages(t)=nnz(adjacency);
            assert(~details.truthUsed && ~details.futureOutcomeUsed);
            context.previousAdjacencyHistory=adjacency;
        catch err
            errors{t}=struct('identifier',err.identifier,'message',err.message);
        end
    end
    result.(names{arm})=struct('passed',passed,'errors',{errors}, ...
        'messageCounts',messages,'passedFrames',nnz(passed), ...
        'firstFailureFrame',find(~passed,1));
    fprintf('%s: %d/%d feasible frames; first failure %d.\n', ...
        names{arm},nnz(passed),T,find(~passed,1));
    first=find(~passed,1);
    if ~isempty(first), fprintf('%s: %s\n',errors{first}.identifier,errors{first}.message); end
end
result.filterGatePassed=all(result.full_causal.passed) && all(result.sparse_v242.passed);
assert(isequal(result.full_causal.passed,result.sparse_v242.passed));
assert(all(~result.full_causal.passed | result.groupCyclesFeasible));
fid=fopen(fullfile(out,'routing_preflight.json'),'w');
assert(fid>=0); cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));
fprintf('Physical graph connected: %d/%d frames. Filter gate: %d.\n', ...
    nnz(result.physicalConnected),T,result.filterGatePassed);
end
