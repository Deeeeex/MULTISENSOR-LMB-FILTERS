function topo = buildGroupPilotTopologies(scene)
% Geometry-only centralized policies. Group labels are not sensor identities.
N=scene.N; T=scene.T; names={'local','tree','dynamic','stable'};
topo=struct('names',{names},'adjacency',false(N,N,T,4), ...
    'groupIds',zeros(N,T,4),'componentCount',zeros(1,T), ...
    'pairEdits',zeros(4,T),'edgeEdits',zeros(4,T), ...
    'runtimeSeconds',zeros(4,T));
previous=cell(1,4); age=zeros(N);
for t=1:T
    p=scene.positions(:,:,t);
    distance=sqrt(sum((permute(p,[2,3,1])-permute(p,[3,2,1])).^2,3));
    physical=distance<=scene.radioRange & ~eye(N);
    age=(age+1).*physical;
    cc=conncomp(graph(physical)); topo.componentCount(t)=max(cc);
    for arm=1:4
        timer=tic;
        if arm==1
            groups=num2cell(1:N); adjacency=false(N);
        elseif arm==2
            groups=num2cell(1:N); adjacency=minimumForest(physical,distance,false(N));
        elseif arm==3 || t==1
            groups=seedGroups(1:N,physical,distance);
            adjacency=groupTree(groups,physical,distance);
        else
            groups={};
            for g=1:numel(previous{arm})
                members=previous{arm}{g};
                if all(physical(members,members)|eye(numel(members)),'all')
                    groups{end+1}=members; %#ok<AGROW>
                else
                    groups=[groups,seedGroups(members,physical,distance)]; %#ok<AGROW>
                end
            end
            groups=mergeStable(groups,physical,distance,age);
            adjacency=groupTree(groups,physical,distance);
        end
        topo.runtimeSeconds(arm,t)=toc(timer);
        assert(~any(adjacency & ~physical,'all'));
        assert(isequal(adjacency,adjacency'));
        if arm>1
            assert(isequal(conncomp(graph(adjacency)),cc));
            assert(nnz(adjacency)==2*(N-max(cc)));
        end
        ids=zeros(1,N);
        for g=1:numel(groups), ids(groups{g})=min(groups{g}); end
        assert(all(ids>0));
        topo.groupIds(:,t,arm)=ids;
        topo.adjacency(:,:,t,arm)=adjacency;
        if t>1
            old=topo.groupIds(:,t-1,arm);
            topo.pairEdits(arm,t)=nnz(triu(xor(ids'==ids,old==old'),1));
            topo.edgeEdits(arm,t)=nnz(triu(xor(adjacency,topo.adjacency(:,:,t-1,arm)),1));
        end
        previous{arm}=groups;
    end
end
assert(isequal(topo.groupIds(:,1,3),topo.groupIds(:,1,4)));
topo.reconnectionFrames=find(diff(topo.componentCount)==-1 & topo.componentCount(2:end)==1)+1;
topo.disconnectionFrames=find(diff(topo.componentCount)>0)+1;
assert(~isempty(topo.reconnectionFrames) && all(ismember(topo.componentCount,[1,2])));
end

function groups=seedGroups(nodes,physical,distance)
remaining=sort(nodes); groups={};
while ~isempty(remaining)
    seed=remaining(1); group=seed; remaining(1)=[];
    while numel(group)<3 && ~isempty(remaining)
        eligible=remaining(all(physical(group,remaining),1));
        if isempty(eligible), break; end
        [~,order]=sortrows([distance(seed,eligible)',eligible'],[1,2]);
        chosen=eligible(order(1)); group(end+1)=chosen; %#ok<AGROW>
        remaining(remaining==chosen)=[];
    end
    groups{end+1}=sort(group); %#ok<AGROW>
end
end

function groups=mergeStable(groups,physical,distance,age)
while true
    candidates=zeros(0,3);
    for a=1:numel(groups)-1
        for b=a+1:numel(groups)
            left=groups{a}; right=groups{b};
            if numel(left)+numel(right)<=3 && all(physical(left,right),'all') ...
                    && all(age(left,right)>=3,'all')
                candidates(end+1,:)=[max(distance(left,right),[],'all'),a,b]; %#ok<AGROW>
            end
        end
    end
    if isempty(candidates), break; end
    candidates=sortrows(candidates,[1,2,3]); a=candidates(1,2); b=candidates(1,3);
    groups{a}=sort([groups{a},groups{b}]); groups(b)=[];
end
end

function adjacency=groupTree(groups,physical,distance)
N=size(physical,1); adjacency=false(N);
for g=1:numel(groups)
    members=groups{g}; allowed=false(N); allowed(members,members)=physical(members,members);
    adjacency=minimumForest(allowed,distance,adjacency);
end
adjacency=minimumForest(physical,distance,adjacency);
end

function result=minimumForest(physical,distance,result)
[a,b]=find(triu(physical,1));
edges=sortrows([distance(sub2ind(size(distance),a,b)),a,b],[1,2,3]);
labels=conncomp(graph(result));
for e=1:size(edges,1)
    a=edges(e,2); b=edges(e,3);
    if labels(a)~=labels(b)
        result(a,b)=true; result(b,a)=true;
        labels(labels==labels(b))=labels(a);
    end
end
end
