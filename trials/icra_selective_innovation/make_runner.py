"""Extend the previous marked replay only at the current-evidence interface."""
from pathlib import Path

OUT = Path(__file__).resolve().parent


def replace(source, old, new, count=1):
    assert source.count(old) == count, (old, source.count(old), count)
    return source.replace(old, new)


def main():
    source = (OUT.parent / 'icra_marked_joint/runMarkedJointReplay.m').read_text()
    source = source.replace('runMarkedJointReplay', 'runSelectiveReplay')
    source = source.replace('checkJointEvidence();', 'checkSelectiveInnovations();')
    source = replace(source, "arms={'marked_joint_evidence','marked_joint_evidence_recency'};",
                     "arms={'marked_selective','marked_selective_no_history','marked_selective_no_mark','marked_selective_signed'};")
    source = replace(source, "arms=[{'marked_er'},arms]", "arms=[{'marked_conservative'},arms]")
    source = source.replace('marked-current-innovation-v1', 'selective-current-innovation-v1')
    source = source.replace('marked-joint-v1', 'selective-v1').replace('MARKED JOINT', 'SELECTIVE')
    source = replace(source, 'template.localLogOddsIncrement=0;',
                     'template.localLogOddsIncrement=0;template.positiveInnovationSupport=0;')
    source = replace(source, "'localIncrementRecords',zeros(0,8),'iterationRecords',zeros(0,26)",
                     "'localIncrementRecords',zeros(0,10),'iterationRecords',zeros(0,35)")
    anchor = '        for j=1:numel(local{n})\n'
    source = replace(source, anchor, """        opportunity=[local{n}.lastDirectOpportunity]==t;
        [support,associationMass]=positiveInnovationSupport(W,ratios.likelihoodRatios{n,t}, ...
            opportunity,strcmp(rule,'selective_no_mark'));
""" + anchor)
    source = replace(source, 'local{n}(j).directEvidenceCeiling=0;',
                     'local{n}(j).directEvidenceCeiling=0;local{n}(j).positiveInnovationSupport=support(j);')
    source = replace(source, 'predicted(j).r,local{n}(j).r,delta,current];',
                     'predicted(j).r,local{n}(j).r,delta,current,support(j),associationMass(j)];')
    source = replace(source, "startsWith(rule,'joint_evidence'),[decoded{n},packet]=innovationLmbPacket",
                     "startsWith(rule,'selective'),[decoded{n},packet]=selectiveInnovationPacket")
    anchor = "            fusionRule=rule;if strcmp(rule,'er'),fusionRule='qualified_exist';end\n            [posterior{n},stats]=fuseJointEvidence(inputs,[.5,.5],model,details,cfg,fusionRule,t);"
    source = replace(source, anchor, """            details.originalLabels={[[local{n}.birthTime];[local{n}.birthLocation]], ...
                [[decoded{other}.birthTime];[decoded{other}.birthLocation]]};
            if strcmp(rule,'conservative')
                [posterior{n},stats]=fuseConservativeRecency(inputs,[.5,.5],model,details,cfg,'conservative_recency',t);
                stats.records=[stats.records,zeros(size(stats.records,1),9)];
            else
                [posterior{n},stats]=fuseSelectiveInnovations(inputs,[.5,.5],model,details,cfg,rule,t);
            end""")
    (OUT / 'runSelectiveReplay.m').write_text(source)
    print('Generated selective replay; marked update and original CR are unchanged.')


if __name__ == '__main__':
    main()
