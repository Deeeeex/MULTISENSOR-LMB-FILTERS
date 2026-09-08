function values=directEvidenceCeilings(W,marks,opportunity)
% Current missed-detection branch contributes zero; empty/no-opportunity reset.
values=zeros(size(opportunity));marks=reshape(marks,1,[]);
assert(all(isfinite(marks)) && all(marks>=0 & marks<=1));
if isempty(marks) || isempty(W),return;end
assert(size(W,1)==numel(opportunity) && size(W,2)==numel(marks)+1);
for j=1:numel(opportunity)
    if ~opportunity(j),continue;end
    w=W(j,:);w(~isfinite(w))=0;w=max(w,0);
    if sum(w)<=0,continue;end
    w=w/sum(w);values(j)=min(max(sum(w(2:end).*marks),0),1);
end
end
