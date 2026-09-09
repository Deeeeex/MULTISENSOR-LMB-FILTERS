function [objects,stats,records]=spliceRecursionIntervention(scalar,scalarStats,joint,jointStats,mode,t,n)
% One declared event changes only r and/or the complete spatial Gaussian.
assert(ismember(mode,{'none','joint','existence','spatial'}));
objects=scalar;stats=scalarStats;records=zeros(0,54);
if strcmp(mode,'none'),return;end
assert(numel(scalar)==numel(joint));
assert(isequaln(scalarStats.records(:,[1:4,8:9,11:40,53:56,58:60]), ...
    jointStats.records(:,[1:4,8:9,11:40,53:56,58:60])));
for field={'lineageExcluded','observableAbsences','weightChange'}
    assert(isequaln(scalarStats.(field{1}),jointStats.(field{1})));
end
code=find(strcmp(mode,{'joint','existence','spatial'}));
lower=find(tril(true(4)));
for k=1:numel(scalar)
    a=scalar(k);b=joint(k);
    assert(a.birthTime==b.birthTime && a.birthLocation==b.birthLocation);
    for field=setdiff(fieldnames(a),{'r','mu','Sigma'})'
        assert(isequaln(a.(field{1}),b.(field{1})));
    end
    if ismember(mode,{'joint','existence'})
        objects(k).r=b.r;stats.records(k,[7,10,57])=jointStats.records(k,[7,10,57]);
    end
    if ismember(mode,{'joint','spatial'})
        objects(k).mu=b.mu;objects(k).Sigma=b.Sigma;
        stats.records(k,[5:6,41:52])=jointStats.records(k,[5:6,41:52]);
    end
    applied=objects(k);
    records(k,:)=[t,n,a.birthTime,a.birthLocation,code,a.r,b.r,applied.r, ...
        a.mu{1}',b.mu{1}',applied.mu{1}', ...
        a.Sigma{1}(lower)',b.Sigma{1}(lower)',applied.Sigma{1}(lower)', ...
        scalarStats.records(k,53:54),scalarStats.records(k,57),jointStats.records(k,57)]; %#ok<AGROW>
end
if strcmp(mode,'joint')
    assert(isequaln(objects,joint) && isequaln(stats,jointStats));
end
assert(size(records,2)==54 && all(isfinite(records),'all'));
end
