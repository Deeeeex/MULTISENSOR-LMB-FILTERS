"""Combine the unchanged joint rule with the existing marked local update."""
from pathlib import Path

OUT = Path(__file__).resolve().parent


def replace(source, old, new, count=1):
    assert source.count(old) == count, (old, source.count(old), count)
    return source.replace(old, new)


def main():
    source = (OUT.parent / 'icra_marked_control/runMarkedControlReplay.m').read_text()
    source = replace(source, 'function runMarkedControlReplay(', 'function runMarkedJointReplay(')
    source = replace(source, "priorIteration=fullfile(root,'trials','icra_method_iteration');",
                     "joint=fullfile(root,'trials','icra_evidence_iteration');addpath(joint);\npriorIteration=fullfile(root,'trials','icra_method_iteration');")
    source = replace(source, 'checkStableMarkedEvidence();checkConservativeRecency();',
                     'checkStableMarkedEvidence();checkJointEvidence();')
    source = source.replace("'holdout'", "'seen_transfer'")
    source = replace(source, "arms={'marked_conservative'};\nif strcmp(mode,'development_check'),arms={'marked_er','marked_conservative'};end",
                     "arms={'marked_joint_evidence','marked_joint_evidence_recency'};\nif strcmp(mode,'development_check'),arms=[{'marked_er'},arms];end")
    source = replace(source, "'protocol','shared-information-conservative-control-v1','implementation','marked-conservative-v1'",
                     "'protocol','marked-current-innovation-v1','implementation','marked-joint-v1'")
    source = source.replace('COMPLETED MARKED CONTROL', 'COMPLETED MARKED JOINT')
    source = source.replace('START PORT', 'START MARKED JOINT').replace('DONE PORT', 'DONE MARKED JOINT')
    source = replace(source, 'template.lastDirectOpportunity=0;template.directEvidenceCeiling=0;template.positiveConfirmation=false;',
                     'template.lastDirectOpportunity=0;template.directEvidenceCeiling=0;template.positiveConfirmation=false;template.localLogOddsIncrement=0;')
    source = replace(source, "'packetBytes',zeros(2,T),'iterationRecords',zeros(0,26),",
                     "'packetBytes',zeros(2,T),'localIncrementRecords',zeros(0,8),'iterationRecords',zeros(0,26),")
    start = source.index('        mark=ones(')
    stop = source.index('        local{n}=reduce(', start)
    source = source[:start] + """        for j=1:numel(local{n})
            assert(strcmp(key(local{n}(j)),key(predicted(j))));
            before=min(max(predicted(j).r,1e-9),1-1e-9);
            after=min(max(local{n}(j).r,1e-9),1-1e-9);
            delta=log(after)-log1p(-after)-log(before)+log1p(-before);
            local{n}(j).localLogOddsIncrement=delta;
            local{n}(j).directEvidenceCeiling=0;
            current=local{n}(j).lastDirectOpportunity==t;
            if ~current,assert(abs(delta)<1e-10);end
            run.localIncrementRecords(end+1,:)=[t,n,local{n}(j).birthTime,local{n}(j).birthLocation, ...
                predicted(j).r,local{n}(j).r,delta,current]; %#ok<AGROW>
        end
""" + source[stop:]
    source = replace(source, "if startsWith(rule,'ceiling_'),[decoded{n},packet]=ceilingLmbPacket(local{n},model,n,t);",
                     "if startsWith(rule,'joint_evidence'),[decoded{n},packet]=innovationLmbPacket(local{n},model,n,t);")
    source = replace(source, "if strcmp(rule,'conservative')\n                [posterior{n},stats]=fuseConservativeRecency(inputs,[.5,.5],model,details,cfg,'conservative_recency',t);\n            else\n                [posterior{n},stats]=fusePortInputs(inputs,[.5,.5],model,details,cfg,rule,t);\n            end",
                     "fusionRule=rule;if strcmp(rule,'er'),fusionRule='qualified_exist';end\n            [posterior{n},stats]=fuseJointEvidence(inputs,[.5,.5],model,details,cfg,fusionRule,t);")
    (OUT / 'runMarkedJointReplay.m').write_text(source.rstrip() + '\n')
    print('Marked joint runner generated; original joint fusion and all previous sources unchanged.')


if __name__ == '__main__':
    main()
