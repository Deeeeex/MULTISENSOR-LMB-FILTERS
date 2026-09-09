function checkObservationAssociation()
out=fileparts(mfilename('fullpath'));root=fileparts(fileparts(out));
for folder={'icra_asymmetric_evidence','icra_external_fusion','icra_gaussian_evidence','icra_ceiling_iteration','icra_selective_innovation','icra_reunion_fusion','icra_evidence_iteration','icra_method_iteration'}
    addpath(fullfile(root,'trials',folder{1}));
end
addpath(fullfile(root,'common'),fullfile(root,'lmb'),fullfile(root,'multisensorLmb'), ...
    fullfile(root,'trials','icra_method_iteration','runtime'));
model=generateMultisensorModel(2,[3,3],[.9,.9],[1,1],'GA','LBP');model.T=.1;
o=model.birthParameters(1);o.birthTime=1;o.birthLocation=100001;o.r=.99;
o.numberOfGmComponents=1;o.w=1;o.mu={zeros(4,1)};o.Sigma={eye(4)};
o.hasObservationLineage=true;o.lastDirectOpportunity=2;o.positiveConfirmation=false;
o.directEvidenceCeiling=0;o.localLogOddsIncrement=0;o.positiveInnovationSupport=0;o.negativeInnovationSupport=0;
o.localSpatialLogRatio=zeros(1,15);o.localDirectObservation=[.99,0,0,1,0,1,1,0];
model.birthParameters=o;model.object=o([]);
memory=struct('frame',{},'left',{},'right',{},'qualified',{},'distance',{});
a=o;b=o;b.localDirectObservation(2)=4;
[one,d]=alignObservationLmbPair(a,b,50,memory,'direct',1);
assert(numel(one)==1 && d.knownPairs==1 && d.reopenedKnown==0);
[~,~,history]=alignObservationLmbPair(a,b,50,memory,'temporal',1);
[one,d,history]=alignObservationLmbPair(a,b,50,history,'temporal',2);
assert(isempty(one) && d.reopenedKnown==1 && d.droppedRemote==1 && isequal(d.abstainLabels,[1;100001]));
assert(d.reopenedRecords(1,4)==2 && abs(d.reopenedRecords(1,6)-16)<1e-12);
% A gap beyond two frames invalidates all historical evidence.
[one,d]=alignObservationLmbPair(a,b,50,history,'temporal',5);
assert(numel(one)==1 && d.knownPairs==1 && d.historyPairs==0);
% Reopen two conflicting equal labels and reassign their current observations.
c=o;c.birthLocation=100002;c.mu={[5;0;0;0]};c.localDirectObservation(2)=5;
r1=a;r1.localDirectObservation(2)=5;r2=c;r2.localDirectObservation(2)=0;
[one,d]=alignObservationLmbPair([a,c],[r1,r2],50,memory,'direct',1);
assert(isequal([one.birthLocation],[100002,100001]) && d.reopenedKnown==2 && d.assignedPairs==2);
assert(d.droppedRemote==0 && isempty(d.abstainLabels));
% Exact legacy fallback, including its original assignment arithmetic.
a.localDirectObservation=zeros(1,8);b.localDirectObservation=zeros(1,8);b.birthLocation=200001;
[old,oldStats]=alignGaussianLmbPair(a,b,50);
[one,d]=alignObservationLmbPair(a,b,50,memory,'temporal',1);
assert(isequaln(one,old) && d.knownPairs==oldStats.knownPairs && d.assignedPairs==oldStats.assignedPairs);
% Common rigid-coordinate transformations preserve observation decisions.
a=o;b=o;b.localDirectObservation(2)=5;
[~,first]=alignObservationLmbPair(a,b,50,memory,'direct',1);
rotation=[0,-1;1,0];translation=[17;-8];A=blkdiag(rotation,rotation);
objects=[a,b];
for k=1:2
    objects(k).mu={A*objects(k).mu{1}+[translation;0;0]};
    objects(k).Sigma={A*objects(k).Sigma{1}*A'};
    objects(k).localDirectObservation(2:3)=(rotation*objects(k).localDirectObservation(2:3)'+translation)';
end
[~,second]=alignObservationLmbPair(objects(1),objects(2),50,memory,'direct',1);
assert(isequal(first.abstainLabels,second.abstainLabels) && max(abs(first.reopenedRecords-second.reopenedRecords),[],'all')<1e-12);
% Actual codec round trip for all eight observation statistics.
[decoded,bytes]=observationEvidencePacket(objects,model,1,2);
assert(numel(bytes)==32+416*2 && isequal([decoded.localDirectObservation],[objects.localDirectObservation]));
[decoded,bytes]=observationEvidencePacket(model.object,model,1,2);
assert(isempty(decoded) && numel(bytes)==32);
% An identity conflict abstains; it must not become a censored absence.
cfg=buildMixtureAwareKlaReferenceConfig();cfg.captureIterationRecords=true;
details=struct('eventType',[0,2],'sourceIndices',[1,2],'isStale',[false,false], ...
    'isSelf',[true,false],'currentTime',2);
details.labelSpecificWeightOverrides=struct('label',[o.birthTime;o.birthLocation], ...
    'sourceInputIndex',1,'replacedInputIndex',2,'mode','dominant-nonself-transfer');
[one,stats]=fuseGaussianEvidence({o,model.object},[.5,.5],model,details,cfg,'gaussian_evidence',2);
assert(abs(one.r-o.r)<1e-12 && norm(one.mu{1}-o.mu{1})<1e-12 && norm(one.Sigma{1}-o.Sigma{1})<1e-12);
assert(isequal(stats.records(14:15),[1,0]) && stats.observableAbsences==0);
fprintf('OBSERVATION ASSOCIATION CHECK PASSED\n');
end
