"""Generate narrowly patched native runner and existing mathematical auditors."""
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
OLD=OUT.parent/'icra_known_censor'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    receipts=[]
    def port(source,name,changes):
        target=OUT/name;assert not target.exists();content=source.read_text()
        for a,b in changes:assert content.count(a)==1,a;content=content.replace(a,b)
        target.write_text(content);receipts.append(dict(source=str(source.relative_to(ROOT)),source_sha256=sha(source),
            target=str(target.relative_to(ROOT)),target_sha256=sha(target),changes=[dict(before=a,after=b) for a,b in changes]))
    port(OLD/'runKnownCensorReplay.m','runPruneInformationReplay.m',[
        ('function runKnownCensorReplay(', 'function runPruneInformationReplay('),
        ("addpath(fullfile(root,'trials','icra_range_detection'));addpath(out);", "addpath(fullfile(root,'trials','icra_range_detection'));addpath(fullfile(root,'trials','icra_known_censor'));addpath(out);"),
        ('checkRangeDetection(baseQuality);checkKnownCensor();','checkRangeDetection(baseQuality);checkKnownCensor();checkPruneInformation();'),
        ("'protocol','icra-known-censor-v1'", "'protocol','icra-prune-information-v1'"),
        ("knownCensorEnabled=endsWith(rule,'_known_censor');", "pruneInformationEnabled=endsWith(rule,'_prune_info');\nif pruneInformationEnabled,rule=erase(rule,'_prune_info');end\nknownCensorEnabled=endsWith(rule,'_known_censor');\nassert(pruneInformationEnabled || knownCensorEnabled);"),
        ('run.knownCensorEnabled=knownCensorEnabled;run.knownCensorRecords=zeros(0,25);',
         'run.knownCensorEnabled=knownCensorEnabled;run.knownCensorRecords=zeros(0,25);\nrun.pruneInformationEnabled=pruneInformationEnabled;run.pruneInformationRecords=zeros(0,24);\nrun.pruneTrailerBytes=cell(2,T);run.basePacketBytes=zeros(2,T);'),
        ('        decoded=cell(1,2);sizes=zeros(1,2);', '        decoded=cell(1,2);sizes=zeros(1,2);sentPackets=cell(1,2);baseSizes=zeros(1,2);'),
        ('            sizes(n)=numel(packet);', '            baseSizes(n)=numel(packet);\n            if pruneInformationEnabled\n                qualified=knownLocal{n}(:,4)<=.001 & knownLocal{n}(:,5)>0;\n                trailer=encodePruneInformation(knownLocal{n}(qualified,[1,2,4,6]),n,t);\n                packet=[reshape(packet,1,[]),trailer];\n                run.pruneTrailerBytes{n,t}=packet(baseSizes(n)+1:end);\n            end\n            sentPackets{n}=packet;sizes(n)=numel(packet);'),
        ('            if knownCensorEnabled\n                [posterior{n},stats,events]=refineKnownCensor',
         '            if pruneInformationEnabled\n                own=knownLocal{n}(:,4)<=.001 & knownLocal{n}(:,5)>0;\n                reports={knownLocal{n}(own,[1,2,4,6]),decodePruneInformation(sentPackets{other}(baseSizes(other)+1:end),other,t)};\n                [posterior{n},stats,events]=refinePruneInformation(posterior{n},stats,inputs,reports,rule,t,n);\n                run.pruneInformationRecords=[run.pruneInformationRecords;events]; %#ok<AGROW>\n            elseif knownCensorEnabled\n                [posterior{n},stats,events]=refineKnownCensor'),
        ('        run.attemptedMessages(t)=2;run.deliveredMessages(t)=nnz(delivered(:,:,t));',
         '        run.basePacketBytes(:,t)=baseSizes(:);\n        run.attemptedMessages(t)=2;run.deliveredMessages(t)=nnz(delivered(:,:,t));'),
    ])
    for name in ['censor_gaussian_audit.py','censor_scalar_audit.py']:
        port(OLD/name,name,[('from event_audit import check_censor_event','from prune_event_audit import check_censor_event')])
    port(OLD/'censor_range_audit.py','censor_range_audit.py',[])
    changes=[("'known_censor_controls','known_censor_refined'","'prune_info_controls','prune_info_shared'"),
        ("    v2=json.loads((OUT/'AUDIT_FREEZE_V2.json').read_text())\n    for name,h in v2['source_sha256'].items():assert sha(ROOT/name)==h,name\n",''),
        ("if args.stage=='known_censor_refined':","if args.stage=='prune_info_shared':"),
        ("'audit_v2_known_censor_controls.json'","'audit_prune_info_controls.json'"),
        ("'RUN/ICRA_KNOWN_CENSOR'","'RUN/ICRA_PRUNE_INFORMATION'"),
        ("addpath('trials/icra_known_censor');runKnownCensorReplay(","addpath('trials/icra_prune_information');runPruneInformationReplay("),
        ('ALL KNOWN CENSOR STAGE RUNS COMPLETE','ALL PRUNE INFORMATION STAGE RUNS COMPLETE')]
    port(OLD/'run_stage_v2.py','run_stage.py',changes)
    port(OLD/'preflight_fixture.py','preflight_fixture.py',[
        ("'RUN/ICRA_KNOWN_CENSOR/fixture.log'","'RUN/ICRA_PRUNE_INFORMATION/fixture.log'"),
        ("'trials/icra_known_censor');checkKnownCensor();","'trials/icra_known_censor','trials/icra_prune_information');checkKnownCensor();checkPruneInformation();"),
        ("'KNOWN CENSOR CHECK PASSED' in content","'KNOWN CENSOR CHECK PASSED' in content and 'PRUNE INFORMATION CHECK PASSED' in content")])
    (OUT/'PORTS.json').write_text(json.dumps(receipts,indent=2)+'\n');print('PRUNE INFORMATION PORTS',len(receipts),flush=True)

if __name__=='__main__':main()
