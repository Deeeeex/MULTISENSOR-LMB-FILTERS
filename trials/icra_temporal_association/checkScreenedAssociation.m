function checkScreenedAssociation()
model=generateMultisensorModel(2,[3,3],[.9,.9],[1,1],'GA','LBP');
o=model.birthParameters(1);o.birthTime=1;o.birthLocation=100001;o.r=1;
o.numberOfGmComponents=1;o.w=1;o.mu={zeros(4,1)};o.Sigma={eye(4)};
o.localDirectObservation=[1,0,0,1,0,1,1,0];
a=o;b=o;b.localDirectObservation(2)=4;
history=struct('frame',{},'left',{},'right',{},'qualified',{},'distance',{});
empty=struct('snapshots',history,'blocked',zeros(2,0));
[~,oldCutoffs]=associationScreenSettings('quality');
% The reference settings reproduce the fixed-column S implementation exactly.
extra=b;extra.birthLocation=200002;extra.mu={[1000;0;0;0]};extra.localDirectObservation(2)=8;
cases={{a,b},{[a,extra],b},{a,[b,extra]},{[a,extra],[b,extra]}, ...
       {a,a([])},{a([]),[b,extra]},{a([]),a([])}};
for c=1:numel(cases)
    pair=cases{c};s0=empty;s1=empty;
    for t=1:3
        [old,d0,s0]=alignPersistentColumn(pair{1},pair{2},50,s0,'split',t,1);
        [new,d1,s1]=alignScreenedLmbPair(pair{1},pair{2},50,s1,'split',t,1,.5,oldCutoffs);
        assert(isequaln(old,new) && isequaln(d0,d1) && isequaln(s0,s1));
    end
end
% Stronger majority confidence rejects an otherwise conflicting weak pair.
weak=b;weak.localDirectObservation(1)=.6;
s0=empty;s1=empty;
for t=1:2
    [~,d0,s0]=alignPersistentColumn(a,weak,50,s0,'split',t,1);
    [~,d1,s1]=alignScreenedLmbPair(a,weak,50,s1,'split',t,1,.9,oldCutoffs);
end
assert(size(d0.splitRecords,1)==1 && isempty(d1.splitRecords) && isempty(s1.blocked));
% The stronger discrepancy threshold delays this high-quality case by a frame.
for name={'nis','quality_nis'}
    [q,cuts]=associationScreenSettings(name{1});state=empty;
    for t=1:3
        [~,d,state]=alignScreenedLmbPair(a,b,50,state,'split',t,1,q,cuts);
        assert(size(d.splitRecords,1)==double(t==3));
    end
end
% Exact qualification boundary and one-to-many shape.
edge=b;edge.localDirectObservation(1)=.9;
[e,~]=observationScreenEvidence(a,[edge,extra],history,1,.9,oldCutoffs);
assert(isequal(e.qualified,[true,true]) && isequal(size(e.currentCost),[1,2]));
edge.localDirectObservation(1)=.9-eps(.9);
[e,~]=observationScreenEvidence(a,[edge,extra],history,1,.9,oldCutoffs);
assert(isequal(e.qualified,[false,true]));
fprintf('SCREENED ASSOCIATION CHECK PASSED\n');
end
