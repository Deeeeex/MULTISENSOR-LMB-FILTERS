function result = verifyDynamicGroupCandidates()
% Validate selected per-frame candidates with unchanged V240/V242 policies.
% Cold routing history is intentional: this certifies geometry construction
% only, not correct posterior/cache transfer during an actual regrouping.
out = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(out));
addpath(fullfile(root,'common'),fullfile(root,'multisensorLmb'));
s = load(fullfile(root,'trials','whales_pilot','converted_scene.mat'));
audit = jsondecode(fileread(fullfile(out,'feasibility.json')));
N = size(s.positions,2);
result = struct('contract','whales-dynamic-candidate-policy-check-v1', ...
    'trackingExecuted',false,'routingHistory','empty for each frame', ...
    'sensorUidSource','original WHALES vehicle slot plus one', ...
    'frames',[],'fullMessages',[],'sparseMessages',[]);
policies = {@selectCausalMinimalEditFormationTreeV240Policy, ...
            @selectCausalMinimumFormationBackboneV242Policy};
for t=1:numel(audit.frames)
    frame = audit.frames(t);
    if frame.feasible_triple_partitions==0, continue; end
    selected = frame.selected_groups_by_input_index;
    assert(isequal(size(selected),[3,3]));
    groups = zeros(1,N);
    for g=1:3, groups(selected(g,:)+1)=g; end
    p=s.positions(:,:,t);
    distances=sqrt(sum((permute(p,[2,3,1])-permute(p,[3,2,1])).^2,3));
    context=struct('localPosteriorBySensor',{repmat({struct([])},1,N)}, ...
        'model',struct('dynamicTopologyScenario',struct('config', ...
        struct('sensorGroupIds',groups))), ...
        'sensorPhysicalUids',double(s.sensorSlots(:)')+1, ...
        'formationPhysicalUidsBySensor',groups, ...
        'baseAdjacency',false(N),'currentTime',t, ...
        'previousAdjacencyHistory',false(N,N,0), ...
        'positions',p,'physicalAdjacency',distances<=100 & ~eye(N), ...
        'commConfig',struct('pDropByEdge',0.1*ones(N)), ...
        'directedMessageBudget',18);
    messages=zeros(1,2);
    for arm=1:2
        if arm==2, context.directedMessageBudget=13; end
        [adjacency,details]=policies{arm}(context);
        assert(~details.truthUsed && ~details.futureOutcomeUsed);
        assert(~any(adjacency & ~context.physicalAdjacency,'all'));
        assert(max(conncomp(digraph(adjacency),'Type','strong'))==1);
        messages(arm)=nnz(adjacency);
    end
    assert(isequal(messages,[18,13]));
    result.frames(end+1)=t;
    result.fullMessages(end+1)=messages(1);
    result.sparseMessages(end+1)=messages(2);
end
assert(numel(result.frames)==audit.summary.dynamic_triples_feasible_frames);
fid=fopen(fullfile(out,'policy_verification.json'),'w');
assert(fid>=0); cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));
fprintf('Verified %d selected frames with both unchanged policies.\n',numel(result.frames));
end
