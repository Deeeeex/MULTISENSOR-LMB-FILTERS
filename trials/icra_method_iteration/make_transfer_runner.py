"""Freeze the complete eight-arm transfer replay before tracking outcomes."""
from pathlib import Path
import hashlib,json
out=Path(__file__).resolve().parent;root=out.parents[1]
s=(out.parent/'icra_external_fusion/runV2v4realReplay.m').read_text()
s=s.replace('runV2v4realReplay','runTransferReplay')
s=s.replace('lastSequence=8','lastSequence=6')
s=s.replace("setupTcDependency(); checkGaussianMil();", "original=fullfile(root,'trials','icra_external_fusion');addpath(original);\nsetupTcDependency(); checkGaussianMil(); checkConservativeRecency(); checkConfirmedRecency();")
s=s.replace("qualityPath=fullfile(out,'replay_quality')","qualityPath=fullfile(original,'replay_quality')")
s=s.replace("'results_v2v'","'results_transfer'")
s=s.replace("'v2v4real_input_manifest.json'","'transfer_input_manifest.json'")
s=s.replace("'replay_source_sha256.json'","'source_sha256_transfer.json'")
s=s.replace('for seq=firstSequence:lastSequence',"for sequenceIndex=firstSequence:lastSequence\n    seq=inputManifest.selected_sequences(sequenceIndex+1);")
s=s.replace("'data',sprintf('v2v4real_%04d.mat',seq)","'data_transfer',sprintf('v2v4real_%04d.mat',seq)")
s=s.replace('runs=cell(1,6)','runs=cell(1,8)')
s=s.replace("arms={'lineage','qualified_exist','mil_support'}","arms={'lineage','qualified_exist','conservative_recency','confirmation_recency','mil_support'}")
s=s.replace('for a=1:3','for a=1:5')
s=s.replace('runs{5}=runTc(local,positions,delivered,5);runs{6}=runTc(local,positions,delivered,10);',
            'runs{7}=runTc(local,positions,delivered,5);runs{8}=runTc(local,positions,delivered,10);')
s=s.replace("'v2v4real-2d-real-detection-replay-v1'","'reserved-v2v4real-algorithm-transfer-v1'")
s=s.replace("'consistent-domain-and-absence-v2'","'cr-cgr-transfer-v1'")
s=s.replace('inputManifest.sequences(seq+1).input_sha256','inputManifest.sequences(sequenceIndex+1).input_sha256')
s=s.replace('            [posterior{n},stats]=fuseValidationInputs(inputs,[.5,.5],model,details,cfg,arm,t);',
'''            if strcmp(arm,'conservative_recency')
                [posterior{n},stats]=fuseConservativeRecency(inputs,[.5,.5],model,details,cfg,arm,t);
            elseif strcmp(arm,'confirmation_recency')
                [posterior{n},stats]=fuseConfirmedRecency(inputs,[.5,.5],model,details,cfg,arm,t);
            else
                [posterior{n},stats]=fuseValidationInputs(inputs,[.5,.5],model,details,cfg,arm,t);
            end''')
needle='cleanup=onCleanup(@()rmpath(qualityPath)); %#ok<NASGU>'
assert needle in s
s=s.replace(needle,needle+"\nruntimePath=fullfile(out,'runtime');addpath(runtimePath);\nruntimeCleanup=onCleanup(@()rmpath(runtimePath)); %#ok<NASGU>\nassert(strcmp(which('Hungarian'),fullfile(runtimePath,'Hungarian.m')));")
needle="direct={containers.Map('KeyType','char','ValueType','double'),containers.Map('KeyType','char','ValueType','double')};"
assert needle in s
s=s.replace(needle,needle+"\nhits={containers.Map('KeyType','char','ValueType','double'),containers.Map('KeyType','char','ValueType','double')};\nlastUpdate={containers.Map('KeyType','char','ValueType','double'),containers.Map('KeyType','char','ValueType','double')};")
s=s.replace('        local{n}=reduce(local{n},model);', '''        if strcmp(arm,'confirmation_recency')
            for j=1:numel(local{n})
                id=key(local{n}(j));mass=0;
                if ~isempty(measurements{n,t}) && stamp(direct{n},id)==t
                    mass=local{n}(j).detectionAssociationMass;
                end
                [count,confirmed]=updateLocalConfirmation(stamp(hits{n},id),stamp(lastUpdate{n},id), ...
                    t,mass,stamp(direct{n},id)==t,~isempty(measurements{n,t}));
                hits{n}(id)=count;lastUpdate{n}(id)=t;
                local{n}(j).positiveConfirmation=confirmed;
            end
        end
        local{n}=reduce(local{n},model);''')
s=s.replace('posterior{n}(j).lastDirectOpportunity=stamp(direct{n},key(posterior{n}(j)));',
            "posterior{n}(j).lastDirectOpportunity=stamp(direct{n},key(posterior{n}(j)));\n            if strcmp(arm,'confirmation_recency'),posterior{n}(j).positiveConfirmation=stamp(lastUpdate{n},key(posterior{n}(j)))==t && stamp(hits{n},key(posterior{n}(j)))>=2;end")
(out/'runTransferReplay.m').write_text(s)
base=json.loads((out/'source_sha256_cgr_fast.json').read_text())
for name,digest in base.items():assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
for p in [out/n for n in ['TRANSFER_PROTOCOL.md','transfer_input_manifest.json','transfer_transform_manifest.json','transfer_overlap_audit.json',
                           'prepare_transfer.py','fetch_transfer_transforms.py','audit_transfer_overlap.py','make_transfer_runner.py','runTransferReplay.m']]+list((out/'data_transfer').glob('*.mat')):
    base[str(p.relative_to(root))]=hashlib.sha256(p.read_bytes()).hexdigest()
for name in ['summary_conservative.json','summary_confirmed.json']:
    development=json.loads((out/name).read_text())
    assert development['sequences']==9 and development['audited_node_frames']==1993*4
    base[str((out/name).relative_to(root))]=hashlib.sha256((out/name).read_bytes()).hexdigest()
(out/'source_sha256_transfer.json').write_text(json.dumps(base,indent=2,sort_keys=True)+'\n')
print('Reserved transfer sources and data frozen',len(base),'files; development summary fixed before transfer outputs.')
