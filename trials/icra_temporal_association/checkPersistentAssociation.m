function checkPersistentAssociation()
checkObservationAssociation();
model=generateMultisensorModel(2,[3,3],[.9,.9],[1,1],'GA','LBP');model.T=.1;
o=model.birthParameters(1);o.birthTime=1;o.birthLocation=100001;o.r=.99;
o.numberOfGmComponents=1;o.w=1;o.mu={zeros(4,1)};o.Sigma={eye(4)};
o.hasObservationLineage=true;o.lastDirectOpportunity=2;o.positiveConfirmation=false;
o.directEvidenceCeiling=0;o.localLogOddsIncrement=0;o.positiveInnovationSupport=0;o.negativeInnovationSupport=0;
o.localSpatialLogRatio=zeros(1,15);o.localDirectObservation=[.99,0,0,1,0,1,1,0];
model.birthParameters=o;model.object=o([]);
history=struct('frame',{},'left',{},'right',{},'qualified',{},'distance',{});
empty=struct('snapshots',history,'blocked',zeros(2,0));
a=o;b=o;b.localDirectObservation(2)=4;
[one,d,state]=alignPersistentLmbPair(a,b,50,empty,'reopen',1,1);
[old,~]=alignGaussianLmbPair(a,b,50);assert(isequaln(one,old) && d.reopenedKnown==0);
[one,d,state]=alignPersistentLmbPair(a,b,50,state,'reopen',2,1);
assert(isempty(one) && d.reopenedKnown==1 && isequal(state.blocked,[1;100001]));
% Unqualified measurements do not silently reinstate a conflicting identity.
weakA=a;weakB=b;weakA.localDirectObservation(:)=0;weakB.localDirectObservation(:)=0;
[one,d,state]=alignPersistentLmbPair(weakA,weakB,50,state,'reopen',3,1);
assert(isempty(one) && d.reopenedKnown==1 && d.qualifiedPairs==0);
% After a gap, one agreeing sample is insufficient, two clear the state.
agreeB=a;
[one,~,state]=alignPersistentLmbPair(a,agreeB,50,state,'reopen',8,1);assert(isempty(one));
[one,d,state]=alignPersistentLmbPair(a,agreeB,50,state,'reopen',9,1);
assert(numel(one)==1 && isempty(state.blocked) && d.reopenedKnown==0);
% Splitting retains a distinct remote branch using a deterministic label.
[~,~,splitState]=alignPersistentLmbPair(a,b,50,empty,'split',1,1);
[remote,d,splitState]=alignPersistentLmbPair(a,b,50,splitState,'split',2,1);
alias=[1;1e9+1e6+100001];
assert(numel(remote)==1 && isequal([remote.birthTime;remote.birthLocation],alias));
assert(isequal(d.remoteOnlyLabels,alias) && isequal(d.splitRecords,[1,100001,alias']));
assert(isequal(d.abstainLabels,[1;100001]) && d.droppedRemote==0);
% Occupied aliases are not duplicated, even if their Gaussian gate rejects.
branch=remote;branch.mu={[1000;0;0;0]};branch.localDirectObservation(:)=0;
[one,occupied]=alignPersistentLmbPair([a,branch],b,50,splitState,'split',3,1);
assert(isempty(one) && isempty(occupied.splitRecords) && occupied.droppedRemote==1);
% A generated label cannot be split recursively.
aliasA=a;aliasB=b;aliasA.birthLocation=alias(2);aliasB.birthLocation=alias(2);
[~,~,aliasState]=alignPersistentLmbPair(aliasA,aliasB,50,empty,'split',1,1);
[one,d]=alignPersistentLmbPair(aliasA,aliasB,50,aliasState,'split',2,1);
assert(isempty(one) && isempty(d.splitRecords));
% Actual packet identifier round trip and two source-only GCE components.
[received,bytes]=observationEvidencePacket(remote,model,2,2);
assert(received.birthLocation==alias(2) && numel(bytes)==32+416);
cfg=buildMixtureAwareKlaReferenceConfig();cfg.captureIterationRecords=true;
details=struct('eventType',[0,2],'sourceIndices',[1,2],'isStale',[false,false], ...
    'isSelf',[true,false],'currentTime',2);
details.originalLabels={[1;100001],[1;100001]};
details.labelSpecificWeightOverrides(1)=struct('label',[1;100001], ...
    'sourceInputIndex',1,'replacedInputIndex',2,'mode','dominant-nonself-transfer');
details.labelSpecificWeightOverrides(2)=struct('label',alias, ...
    'sourceInputIndex',2,'replacedInputIndex',1,'mode','dominant-nonself-transfer');
[objects,stats]=fuseGaussianEvidence({a,remote},[.5,.5],model,details,cfg,'gaussian_evidence',2);
assert(numel(objects)==2 && max(abs([objects.r]-.99))<1e-12 && stats.observableAbsences==0);
assert(size(unique([[objects.birthTime];[objects.birthLocation]]','rows'),1)==2);
[objects,~,state]=alignPersistentLmbPair(model.object,model.object,50,empty,'split',1,1);
assert(isempty(objects) && isempty(state.blocked));
fprintf('PERSISTENT ASSOCIATION CHECK PASSED\n');
end
