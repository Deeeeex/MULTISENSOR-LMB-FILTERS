"""Port frozen rules to one common-input runner; do not select from holdout."""
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
marked=OUT.parent/'icra_marked_iteration'
s=(marked/'runMarkedEvidenceReplayStable.m').read_text()
old=s[:s.index('\nfunction model=makeModel')]
header='''function runFusionSelectionReplay(firstIndex,lastIndex,mode)
if nargin<1,firstIndex=0;end
if nargin<2,lastIndex=0;end
if nargin<3,mode='development_check';end
assert(any(strcmp(mode,{'development_check','holdout'})));
out=fileparts(mfilename('fullpath'));root=fileparts(fileparts(out));
original=fullfile(root,'trials','icra_external_fusion');
priorIteration=fullfile(root,'trials','icra_method_iteration');
priorCeiling=fullfile(root,'trials','icra_ceiling_iteration');
priorMarked=fullfile(root,'trials','icra_marked_iteration');
addpath(priorIteration,priorCeiling,priorMarked,original,fullfile(root,'common'), ...
    fullfile(root,'lmb'),fullfile(root,'multisensorLmb'),fullfile(root,'trials','icra_reunion_fusion'));
checkStableMarkedEvidence();setupTcDependency();checkGaussianMil();
baseQuality=@evaluateSensorQuality;
assert(strcmp(functions(baseQuality).file,fullfile(root,'common','evaluateSensorQuality.m')));
qualityPath=fullfile(original,'replay_quality');addpath(qualityPath);
cleanup=onCleanup(@()rmpath(qualityPath)); %#ok<NASGU>
runtimePath=fullfile(priorIteration,'runtime');addpath(runtimePath);
runtimeCleanup=onCleanup(@()rmpath(runtimePath)); %#ok<NASGU>
assert(strcmp(which('Hungarian'),fullfile(runtimePath,'Hungarian.m')));
sourceManifest=jsondecode(fileread(fullfile(out,'source_sha256_port.json')));
if strcmp(mode,'holdout')
    assert(isfile(fullfile(out,'METHOD_FREEZE.json')),'Final method and comparisons must be frozen before any holdout output.');
    inputManifest=jsondecode(fileread(fullfile(out,'input_manifest.json')));
    selected=inputManifest.selected_sequences;
else
    inputManifest=jsondecode(fileread(fullfile(original,'v2v4real_input_manifest.json')));
    selected=0:8;
end
resultDir=fullfile(out,['results_',mode]);if ~isfolder(resultDir),mkdir(resultDir);end
arms={'local','lineage','qualified_exist','mil_support','tc_ospa2_w5','tc_ospa2_w10','ceiling_calibrated', ...
      'marked_local','marked_lineage','marked_er','marked_mil_support','marked_tc_ospa2_w5','marked_tc_ospa2_w10', ...
      'marked_ceiling_association','marked_ceiling_score','marked_ceiling_calibrated'};
for sequenceIndex=firstIndex:lastIndex
    seq=selected(sequenceIndex+1);
    if strcmp(mode,'holdout')
        data=load(fullfile(out,'data',sprintf('v2v4real_%04d.mat',seq)));
        marks=struct('rawScores',{data.rawScores},'calibratedScores',{data.calibratedScores});
        ratios=struct('likelihoodRatios',{data.likelihoodRatios});
    else
        data=load(fullfile(original,'data',sprintf('v2v4real_%04d.mat',seq)));
        marks=load(fullfile(priorCeiling,'data_marks',sprintf('marks_%04d.mat',seq)));
        ratios=load(fullfile(priorMarked,'data_likelihoods',sprintf('likelihoods_%04d.mat',seq)));
    end
    T=double(data.T);measurements=data.measurements;positions=data.positions;
    model=makeModel(positions,T,baseQuality);
    fprintf('START PORT %04d local %d frames\\n',seq,T);
    local=runDensity(model,measurements,positions,false(2,2,T),'local',marks,ratios);
    fprintf('START PORT %04d marked_local %d frames\\n',seq,T);
    markedLocal=runDensity(model,measurements,positions,false(2,2,T),'marked_local',marks,ratios);
    for condition={'reliable','intermittent'}
        radio=condition{1};delivered=repmat(logical([0,1;1,0]),1,1,T);
        if strcmp(radio,'intermittent')
            rng(8301+seq,'twister');draws=rand(2,2,T);delivered=delivered & draws>=.1;
            delivered(:,:,floor(.4*T)+1:floor(.6*T))=false;
        end
        runs=cell(size(arms));
        for a=1:numel(arms)
            arm=arms{a};fprintf('START PORT %04d %s %s %d frames\\n',seq,radio,arm,T);
            if strcmp(arm,'local'),run=local;
            elseif strcmp(arm,'marked_local'),run=markedLocal;
            elseif contains(arm,'tc_ospa2')
                window=10;if contains(arm,'_w5'),window=5;end
                if startsWith(arm,'marked_'),input=markedLocal;else,input=local;end
                run=runTc(input,positions,delivered,window);run.arm=arm;
            else,run=runDensity(model,measurements,positions,delivered,arm,marks,ratios);end
            % Truth first enters after each complete arm's trajectory.
            runs{a}=scoreOutputs(run,data.truth);
            fprintf('DONE PORT %04d %s %s OSPA %.6f runtime %.2fs\\n',seq,radio,arm,mean(runs{a}.ospa,'all'),run.runtimeSeconds);
        result=struct('protocol','remaining-v2v4real-fusion-selection-v1','implementation','common-port-v1', ...
            'cohort',mode,'sourceSha256',sourceManifest,'inputSha256',inputManifest.sequences(sequenceIndex+1).input_sha256, ...
            'sequence',sprintf('%04d',seq),'condition',radio,'smoke',false,'time',double(data.time), ...
            'positions',positions,'truth',{data.truth},'truthIds',{data.truthIds},'delivered',delivered,'runs',runs{a});
        path=fullfile(resultDir,sprintf('%04d_%s_%s.json',seq,radio,arm));
        fid=fopen(path,'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(result));fclose(fid);gzip(path);delete(path);
        runs{a}=[];
        end
    end
end
fprintf('COMPLETED PORT indices %d--%d cohort=%s\\n',firstIndex,lastIndex,mode);
end
'''
s=header+s[len(old):]
s=s.replace("'iterationRecords',zeros(0,26)", "'packetBytes',zeros(2,T),'iterationRecords',zeros(0,26)")
s=s.replace("% No truth, IDs from annotations, future detections or other-source births.\n", "% No truth, IDs from annotations, future detections or other-source births.\nmarked=startsWith(arm,'marked_');rule=erase(arm,'marked_');\n")
s=s.replace("else,[local{n},~,W]=updateMarkedLmbStable(predicted,num2cell(measurements{n,t},1),model,n,t,ratios.likelihoodRatios{n,t});end",
'''elseif marked,[local{n},~,W]=updateMarkedLmbStable(predicted,num2cell(measurements{n,t},1),model,n,t,ratios.likelihoodRatios{n,t});
        else,[local{n},~,W]=updateLmbWithAssociationWeights(predicted,num2cell(measurements{n,t},1),model,n,t);end''')
s=s.replace("strcmp(arm,'marked_ceiling_score')", "strcmp(rule,'ceiling_score')")
s=s.replace("strcmp(arm,'marked_ceiling_calibrated')", "strcmp(rule,'ceiling_calibrated')")
# Only the function body has a local rule; header arm dispatch stays unchanged.
start=s.index('function run=runDensity');body=s[start:];body=body.replace("strcmp(arm,'local')", "strcmp(rule,'local')")
body=body.replace("for n=1:2,[decoded{n},packet]=ceilingLmbPacket(local{n},model,n,t);sizes(n)=numel(packet);end",'''for n=1:2
            if startsWith(rule,'ceiling_'),[decoded{n},packet]=ceilingLmbPacket(local{n},model,n,t);
            else,[decoded{n},packet]=gaussianLmbPacket(local{n},model,n,t);end
            sizes(n)=numel(packet);
        end''')
body=body.replace('[posterior{n},stats]=fuseMarkedInputsStable(inputs,[.5,.5],model,details,cfg,arm,t);',
                  '[posterior{n},stats]=fusePortInputs(inputs,[.5,.5],model,details,cfg,rule,t);')
body=body.replace('run.rawPayloadBytes(t)=sum(sizes);', 'run.packetBytes(:,t)=sizes(:);run.rawPayloadBytes(t)=sum(sizes);')
s=s[:start]+body
tc=(OUT.parent/'icra_method_iteration/runTransferReplay.m').read_text()
tc=tc[tc.index('function run=runTc'):tc.index('function run=scoreOutputs')]
tc=tc.replace('run.rawPayloadBytes(t)=sum(sizes);','run.packetBytes(:,t)=sizes(:);run.rawPayloadBytes(t)=sum(sizes);')
s+='\n'+tc
(OUT/'runFusionSelectionReplay.m').write_text(s)
print('Common runner generated; holdout remains gated by METHOD_FREEZE.json.')
