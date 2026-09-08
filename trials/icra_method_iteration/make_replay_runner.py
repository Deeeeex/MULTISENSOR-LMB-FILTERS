"""Derive an isolated replay harness; preserve the frozen original source."""
from pathlib import Path
OUT=Path(__file__).resolve().parent
source=OUT.parent/'icra_external_fusion/runV2v4realReplay.m'
s=source.read_text()
start=s.index('function model=makeModel(')
helpers=s[start:]
helpers=helpers[:helpers.index('function run=runTc(')]+helpers[helpers.index('function run=scoreOutputs('):]
helpers=helpers.replace('template.lastDirectOpportunity=0;','template.lastDirectOpportunity=0;template.localLogOddsIncrement=0;')
helpers=helpers.replace("'maximumBernoulliCount',0,'runtimeSeconds'", "'iterationRecords',zeros(0,26),'maximumBernoulliCount',0,'runtimeSeconds'")
helpers=helpers.replace('cfg.payloadExistenceThreshold=model.existenceThreshold;',
                        'cfg.payloadExistenceThreshold=model.existenceThreshold;\ncfg.captureIterationRecords=true;')
helpers=helpers.replace('        local{n}=reduce(local{n},model);', '''        for j=1:numel(local{n})
            assert(strcmp(key(local{n}(j)),key(predicted(j))));
            before=min(max(predicted(j).r,1e-9),1-1e-9);
            after=min(max(local{n}(j).r,1e-9),1-1e-9);
            local{n}(j).localLogOddsIncrement=log(after)-log1p(-after)-log(before)+log1p(-before);
        end
        local{n}=reduce(local{n},model);''')
helpers=helpers.replace('gaussianLmbPacket(local{n},model,n,t)','innovationLmbPacket(local{n},model,n,t)')
helpers=helpers.replace('fuseValidationInputs(inputs,[.5,.5],model,details,cfg,arm,t)',
                        'fuseInnovationRecency(inputs,[.5,.5],model,details,cfg,arm,t)')
helpers=helpers.replace('            run.weightChangeL1(n,t)=stats.weightChange;',
                        '            run.iterationRecords=[run.iterationRecords;stats.records]; %#ok<AGROW>\n            run.weightChangeL1(n,t)=stats.weightChange;')
head='''function runMethodReplay(firstSequence,lastSequence,smoke,arms)
% Generated from the frozen replay by make_replay_runner.py.
if nargin<1,firstSequence=0;end
if nargin<2,lastSequence=8;end
if nargin<3,smoke=false;end
if nargin<4,arms={'qualified_exist','innovation_recency'};end
out=fileparts(mfilename('fullpath'));root=fileparts(fileparts(out));
original=fullfile(root,'trials','icra_external_fusion');
addpath(fullfile(root,'common'),fullfile(root,'lmb'),fullfile(root,'multisensorLmb'), ...
    fullfile(root,'trials','icra_reunion_fusion'),original);
baseQuality=@evaluateSensorQuality;
assert(strcmp(functions(baseQuality).file,fullfile(root,'common','evaluateSensorQuality.m')));
qualityPath=fullfile(original,'replay_quality');addpath(qualityPath);
cleanup=onCleanup(@()rmpath(qualityPath)); %#ok<NASGU>
resultDir=fullfile(out,'results_v2v');if ~isfolder(resultDir),mkdir(resultDir);end
manifest=jsondecode(fileread(fullfile(original,'v2v4real_input_manifest.json')));
sourceManifest=jsondecode(fileread(fullfile(out,'source_sha256.json')));
for seq=firstSequence:lastSequence
    data=load(fullfile(original,'data',sprintf('v2v4real_%04d.mat',seq)));
    T=double(data.T);if smoke,T=min(T,12);end
    measurements=data.measurements(:,1:T);positions=data.positions(:,:,1:T);
    model=makeModel(positions,T,baseQuality);
    for condition={'reliable','intermittent'}
        radio=condition{1};delivered=repmat(logical([0,1;1,0]),1,1,T);
        if strcmp(radio,'intermittent')
            rng(8301+seq,'twister');draws=rand(2,2,T);delivered=delivered & draws>=.1;
            delivered(:,:,floor(.4*T)+1:floor(.6*T))=false;
        end
        runs=cell(1,numel(arms));
        for a=1:numel(arms)
            fprintf('START iteration V2V %04d %s %s %d frames\\n',seq,radio,arms{a},T);
            runs{a}=runDensity(model,measurements,positions,delivered,arms{a});
            % Truth first enters here, after the complete tracking run.
            runs{a}=scoreOutputs(runs{a},data.truth(1:T));
            fprintf('DONE iteration V2V %04d %s %s OSPA %.6f count %.6f runtime %.2fs\\n', ...
                seq,radio,arms{a},mean(runs{a}.ospa,'all'),mean(runs{a}.countError,'all'),runs{a}.runtimeSeconds);
        end
        result=struct('protocol','current-innovation-recency-v1','implementation','ir-v1', ...
            'sourceSha256',sourceManifest,'inputSha256',manifest.sequences(seq+1).input_sha256, ...
            'sequence',sprintf('%04d',seq),'condition',radio,'smoke',smoke, ...
            'time',double(data.time(1:T)),'positions',positions,'truth',{data.truth(1:T)}, ...
            'truthIds',{data.truthIds(1:T)},'delivered',delivered,'runs',[runs{:}]);
        suffix='';if smoke,suffix='_smoke';end
        path=fullfile(resultDir,sprintf('%04d_%s%s.json',seq,radio,suffix));
        fid=fopen(path,'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(result));fclose(fid);gzip(path);delete(path);
    end
end
fprintf('COMPLETED iteration sequences %04d--%04d smoke=%d\\n',firstSequence,lastSequence,smoke);
end

'''
(OUT/'runMethodReplay.m').write_text(head+helpers)
print('Generated isolated replay harness from',source)
