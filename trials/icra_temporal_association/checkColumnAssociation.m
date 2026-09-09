function checkColumnAssociation()
% The previously untested 1-by-many qualified-evidence shape must be legal.
model=generateMultisensorModel(2,[3,3],[.9,.9],[1,1],'GA','LBP');
o=model.birthParameters(1);o.birthTime=1;o.birthLocation=100001;o.r=.99;
o.numberOfGmComponents=1;o.w=1;o.mu={zeros(4,1)};o.Sigma={eye(4)};
o.localDirectObservation=[.99,0,0,1,0,1,1,0];
left=o;one=o;one.localDirectObservation(2)=4;
two=o;two.birthLocation=200002;two.localDirectObservation(2)=8;two.mu={[1000;0;0;0]};
history=struct('frame',{},'left',{},'right',{},'qualified',{},'distance',{});
failed=false;
try,observationPairEvidence(left,[one,two],history,1);catch,failed=true;end
assert(failed,'The fixture must reproduce the original row/column failure.');
[e,h]=observationPairEvidenceColumn(left,[one,two],history,1);
assert(isequal(size(e.currentCost),[1,2]));
assert(max(abs(e.currentCost-[8,32]/9.21034037197618))<1e-12);
[e,~]=observationPairEvidenceColumn(left,[one,two],h,2);
assert(isequal(e.count,[2,2]) && max(abs(e.currentCost-[16,64]/13.2767041359876))<1e-12);
state=struct('snapshots',history,'blocked',zeros(2,0));
[~,~,state]=alignPersistentColumn(left,[one,two],50,state,'split',1,1);
[remote,d]=alignPersistentColumn(left,[one,two],50,state,'split',2,1);
assert(numel(remote)==2 && size(d.splitRecords,1)==1 && d.reopenedKnown==1);
% Existing scalar, column, matrix and empty cases remain exactly identical.
alt=left;alt.birthLocation=100002;alt.localDirectObservation(2)=1;
cases={{left,one},{[left,alt],one},{[left,alt],[one,two]}, ...
       {left,left([])},{left([]),[one,two]},{left([]),left([])}};
for k=1:numel(cases)
    pair=cases{k};[old,oldHistory]=observationPairEvidence(pair{1},pair{2},history,1);
    [new,newHistory]=observationPairEvidenceColumn(pair{1},pair{2},history,1);
    assert(isequaln(old,new) && isequaln(oldHistory,newHistory));
end
fprintf('COLUMN ASSOCIATION CHECK PASSED\n');
end
