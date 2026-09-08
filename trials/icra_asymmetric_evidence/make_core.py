"""Add negative branch support without changing the preceding source files."""
from pathlib import Path

OUT = Path(__file__).resolve().parent
PREVIOUS = OUT.parent / 'icra_selective_innovation'


def replace(source, old, new, count=1):
    assert source.count(old) == count, (old, source.count(old), count)
    return source.replace(old, new)


def main():
    source = (PREVIOUS / 'fuseSelectiveInnovations.m').read_text()
    source = source.replace('fuseSelectiveInnovations', 'fuseAsymmetricEvidence')
    source = source.replace("{'selective','selective_no_history','selective_no_mark','selective_signed'}",
                            "{'asymmetric','asymmetric_no_history','asymmetric_no_mark'}")
    source = source.replace("strcmp(arm,'selective_no_history')", "strcmp(arm,'asymmetric_no_history')")
    source = source.replace('zeros(0,35)', 'zeros(0,37)')
    source = replace(source, 'increments=zeros(size(weights));gates=zeros(size(weights));stamps=zeros(size(weights));',
                     'increments=zeros(size(weights));gates=zeros(size(weights));negativeGates=zeros(size(weights));stamps=zeros(size(weights));')
    source = replace(source, 'if stamps(s)==t,gates(s)=o.positiveInnovationSupport;end',
                     'if stamps(s)==t,gates(s)=o.positiveInnovationSupport;negativeGates(s)=o.negativeInnovationSupport;end')
    source = replace(source, '        if isfield(details,',
                     '        assert(isfinite(negativeGates(s)) && negativeGates(s)>=0 && negativeGates(s)<=1);\n        if isfield(details,')
    source = replace(source, "fresh=max(increments,0);if strcmp(arm,'selective_signed'),fresh=increments;end",
                     'fresh=gates.*max(increments,0)+negativeGates.*min(increments,0);')
    source = replace(source, 'boost=sum((double(active)-beta).*gates.*fresh);',
                     'boost=sum((double(active)-beta).*fresh);')
    source = replace(source, 'one.positiveInnovationSupport=0;',
                     'one.positiveInnovationSupport=0;one.negativeInnovationSupport=0;')
    source = replace(source, "gates,beta,boost,original(:)'];", "gates,beta,boost,original(:)',negativeGates];")
    (OUT / 'fuseAsymmetricEvidence.m').write_text(source)
    source = (PREVIOUS / 'runSelectiveReplay.m').read_text()
    source = source.replace('runSelectiveReplay', 'runAsymmetricReplay').replace('SELECTIVE', 'ASYMMETRIC')
    source = replace(source, "joint=fullfile(root,'trials','icra_evidence_iteration');addpath(joint);",
                     "selective=fullfile(root,'trials','icra_selective_innovation');addpath(selective);\njoint=fullfile(root,'trials','icra_evidence_iteration');addpath(joint);")
    source = source.replace('checkSelectiveInnovations();', 'checkAsymmetricEvidence();')
    source = replace(source, "arms={'marked_selective','marked_selective_no_history','marked_selective_no_mark','marked_selective_signed'};\nif strcmp(mode,'development_check'),arms=[{'marked_conservative'},arms];end",
                     "arms={'marked_asymmetric','marked_asymmetric_no_history','marked_asymmetric_no_mark'};\nif strcmp(mode,'development_check'),arms=[{'marked_selective'},arms];end\nif strcmp(mode,'seen_transfer'),arms=[arms,{'marked_selective','marked_selective_signed'}];end")
    source = source.replace('selective-current-innovation-v1', 'asymmetric-current-evidence-v1').replace('selective-v1', 'asymmetric-v1')
    source = replace(source, 'template.positiveInnovationSupport=0;',
                     'template.positiveInnovationSupport=0;template.negativeInnovationSupport=0;')
    source = source.replace("'localIncrementRecords',zeros(0,10),'iterationRecords',zeros(0,35)",
                            "'localIncrementRecords',zeros(0,12),'iterationRecords',zeros(0,37)")
    source = replace(source, '        for j=1:numel(predicted)\n',
                     '        predictedPd=zeros(1,numel(predicted));\n        for j=1:numel(predicted)\n')
    source = replace(source, 'o=predicted(j);pd=evaluateSensorQuality(model,n,o.mu{1},t);',
                     'o=predicted(j);pd=evaluateSensorQuality(model,n,o.mu{1},t);predictedPd(j)=pd;')
    source = replace(source, "opportunity,strcmp(rule,'selective_no_mark'));",
                     "opportunity,strcmp(rule,'asymmetric_no_mark'));\n        predictedPd=reshape(predictedPd,size(opportunity));\n        assert(isequal(opportunity,predictedPd>0));\n        negativeSupport=negativeInnovationSupport(associationMass,predictedPd);\n        if startsWith(rule,'selective'),negativeSupport(:)=0;end")
    source = replace(source, 'local{n}(j).positiveInnovationSupport=support(j);',
                     'local{n}(j).positiveInnovationSupport=support(j);local{n}(j).negativeInnovationSupport=negativeSupport(j);')
    source = replace(source, 'delta,current,support(j),associationMass(j)];',
                     'delta,current,support(j),associationMass(j),predictedPd(j),negativeSupport(j)];')
    source = replace(source, "if startsWith(rule,'selective'),[decoded{n},packet]=selectiveInnovationPacket(local{n},model,n,t);",
                     "if startsWith(rule,'asymmetric'),[decoded{n},packet]=asymmetricInnovationPacket(local{n},model,n,t);\n            elseif startsWith(rule,'selective'),[decoded{n},packet]=selectiveInnovationPacket(local{n},model,n,t);")
    begin = source.index("            if strcmp(rule,'conservative')")
    end = source.index('            run.iterationRecords=', begin)
    source = source[:begin] + """            if startsWith(rule,'selective')
                [posterior{n},stats]=fuseSelectiveInnovations(inputs,[.5,.5],model,details,cfg,rule,t);
                stats.records=[stats.records,zeros(size(stats.records,1),2)];
            else
                [posterior{n},stats]=fuseAsymmetricEvidence(inputs,[.5,.5],model,details,cfg,rule,t);
            end
""" + source[end:]
    (OUT / 'runAsymmetricReplay.m').write_text(source)
    print('Generated branch-supported fusion and replay; previous selective controls retain exact code and native packets.')


if __name__ == '__main__':
    main()
