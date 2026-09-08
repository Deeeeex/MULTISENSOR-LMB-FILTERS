function [states,labels,memory]=tcFuseReceiver(histories,sources,memory,ospa,t)
% Author Algorithms 1/2 kinematic fusion, preserving source and neighbor order.
% Only decoded delivered histories enter. Isolated nodes keep local output.
N=size(memory.association,1); neighbors=numel(sources)-1;
if neighbors==0
    states=histories{1}.X{t}; labels=histories{1}.L{t}; return;
end
intermediate=cell(neighbors,1);
for i=neighbors:-1:1
    one=struct('source_id',sources(i+1),'X',{{}},'N',[],'L',{{}});
    [one.X{1},one.N(1),one.L{1},memory.space,memory.association]= ...
        fuse_two_estimated_tracks(histories{1},histories{i+1}, ...
        memory.space,memory.association,ospa,t,i/(neighbors+1));
    intermediate{i}=one;
end
combined=intermediate{neighbors}; finalOspa=ospa;
finalOspa.min_track_len=1; finalOspa.win_len=1;
% Preserve the author code: win_len (not winlen_lm) is changed here. k=1
% already means the intermediate stage has only the current estimate.
for i=neighbors-1:-1:1
    [combined.X{1},combined.N(1),combined.L{1}]= ...
        fuse_two_estimated_tracks(combined,intermediate{i}, ...
        cell(N,1),cell(N,N),finalOspa,1,.5);
end
states=combined.X{1}; labels=combined.L{1};
assert(size(states,2)==size(labels,2) && all(isfinite(states),'all'));
% Network-wide author B.3 needs exchanged association histories. It only
% renames reported labels, not these kinematic states or the local filter.
% Omit that reporting operation instead of reading non-neighbor memory.
end
