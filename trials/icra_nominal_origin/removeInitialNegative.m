function [objects,stats,events]=removeInitialNegative(original,originalStats,t,n)
% A single declared scalar event; the original GCE spatial state is retained.
objects=original;stats=originalStats;events=zeros(0,18);
assert(numel(original)==size(originalStats.records,1));
for k=1:numel(original)
    row=originalStats.records(k,:);o=original(k);
    assert(isequal(row(1:4),[t,n,o.birthTime,o.birthLocation]));
    active=row(14:15)>0;logits=zeros(1,2);r=min(max(row(18:19),1e-9),1-1e-9);
    logits(active)=log(r(active))-log1p(-r(active));
    beta=row(29:30);kept=row(53:54);increment=row(20:21);
    inherited=sum(beta.*logits);allShift=sum(kept.*increment);integral=row(57);
    assert(logistic(inherited+allShift+integral)==o.r && row(7)==o.r && row(10)==o.r);
    negative=sum(kept.*min(increment,0));positive=sum(kept.*max(increment,0));
    assert(negative<=0 && positive>=0);
    if negative<0
        objects(k).r=logistic(inherited+positive+integral);
        stats.records(k,[7,10])=objects(k).r;
    end
    for field=setdiff(fieldnames(o),{'r'})'
        assert(isequaln(o.(field{1}),objects(k).(field{1})));
    end
    events(k,:)=[t,n,o.birthTime,o.birthLocation,o.r,objects(k).r,negative,positive, ...
        inherited,integral,kept,increment,beta,row(14:15)]; %#ok<AGROW>
end
keep=setdiff(1:60,[7,10]);assert(isequaln(stats.records(:,keep),originalStats.records(:,keep)));
assert(all(isfinite(events),'all'));
end

function r=logistic(z)
if z>=0,r=1/(1+exp(-z));else,e=exp(z);r=e/(1+e);end
end
