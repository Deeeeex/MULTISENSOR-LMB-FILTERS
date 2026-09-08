"""Add a coherent spatial ratio to the previous local-evidence interface."""
from pathlib import Path

OUT = Path(__file__).resolve().parent
PREVIOUS = OUT.parent / 'icra_asymmetric_evidence'


def replace(source, old, new, count=1):
    assert source.count(old) == count, (old, source.count(old), count)
    return source.replace(old, new)


def main():
    s = (PREVIOUS / 'fuseAsymmetricEvidence.m').read_text()
    s = s.replace('fuseAsymmetricEvidence', 'fuseGaussianEvidence')
    s = replace(s, "{'asymmetric','asymmetric_no_history','asymmetric_no_mark'}",
                "{'gaussian_evidence','gaussian_evidence_no_curvature','gaussian_evidence_no_history','gaussian_evidence_no_mark'}")
    s = s.replace('asymmetric_no_history', 'gaussian_evidence_no_history').replace('zeros(0,37)', 'zeros(0,60)')
    s = replace(s, '    one.r=logistic(base+correction);one.positiveConfirmation=confirmed;', """    scalarR=logistic(base+correction);oldMean=one.mu{1};
    alpha=rec.activeSpatialWeights;kept=zeros(size(weights));allowed=true(size(weights));aggregateFallback=false;
    logIntegral=rec.spatialLogNormalizer;
    rawKappa=(double(active)-beta).*(gates.*(increments>=0)+negativeGates.*(increments<0));
    if sum(active)<2 || ~all(present(active)),rawKappa(:)=0;end
    if any(rawKappa>0)
        [mu,P,logIntegral,kept,allowed,aggregateFallback]=applyGaussianEvidence( ...
            pieces,alpha,rawKappa,~strcmp(arm,'gaussian_evidence_no_curvature'));
        if any(kept>0)
            one.mu={mu};one.Sigma={P};
        else
            logIntegral=rec.spatialLogNormalizer;
        end
    end
    one.r=logistic(sum(beta.*logits)+sum(kept.*increments)+logIntegral);
    one.positiveConfirmation=confirmed;one.localSpatialLogRatio=zeros(1,15);""")
    s = replace(s, "gates,beta,boost,original(:)',negativeGates];",
                "gates,beta,boost,original(:)',negativeGates,scalarR,oldMean(1:2)', ...\n            one.mu{1}(3:4)',one.Sigma{1}(find(tril(true(4))))',kept,alpha,logIntegral,allowed,aggregateFallback];")
    (OUT / 'fuseGaussianEvidence.m').write_text(s)
    s = (PREVIOUS / 'runAsymmetricReplay.m').read_text()
    s = s.replace('runAsymmetricReplay', 'runGaussianEvidenceReplay').replace('ASYMMETRIC', 'GAUSSIAN EVIDENCE')
    s = replace(s, "selective=fullfile(root,'trials','icra_selective_innovation');addpath(selective);",
                "asymmetric=fullfile(root,'trials','icra_asymmetric_evidence');addpath(asymmetric);\nselective=fullfile(root,'trials','icra_selective_innovation');addpath(selective);")
    s = s.replace('checkAsymmetricEvidence();', 'checkGaussianEvidence();')
    s = replace(s, "arms={'marked_asymmetric','marked_asymmetric_no_history','marked_asymmetric_no_mark'};\nif strcmp(mode,'development_check'),arms=[{'marked_selective'},arms];end\nif strcmp(mode,'seen_transfer'),arms=[arms,{'marked_selective','marked_selective_signed'}];end",
                "arms={'marked_gaussian_evidence','marked_gaussian_evidence_no_curvature', ...\n    'marked_gaussian_evidence_no_history','marked_gaussian_evidence_no_mark'};\nif strcmp(mode,'development_check'),arms=[{'marked_asymmetric'},arms];end\nif strcmp(mode,'seen_transfer'),arms=[arms,{'marked_asymmetric'}];end")
    s = s.replace('asymmetric-current-evidence-v1', 'coherent-gaussian-evidence-v1').replace('asymmetric-v1', 'gaussian-evidence-v1')
    s = replace(s, 'template.negativeInnovationSupport=0;',
                'template.negativeInnovationSupport=0;template.localSpatialLogRatio=zeros(1,15);')
    s = replace(s, "'localIncrementRecords',zeros(0,12),'iterationRecords',zeros(0,37)",
                "'localIncrementRecords',zeros(0,12),'localGaussianRecords',zeros(0,32), ...\n    'packetGaussianRecords',zeros(0,19),'iterationRecords',zeros(0,60)")
    s = s.replace("strcmp(rule,'asymmetric_no_mark')", "strcmp(rule,'gaussian_evidence_no_mark')")
    s = replace(s, '        local{n}=reduce(local{n},model);', """        local{n}=reduce(local{n},model);
        if ~isempty(local{n})
            localKeys=[[local{n}.birthTime];[local{n}.birthLocation]]';
            priorKeys=[[predicted.birthTime];[predicted.birthLocation]]';
            [found,indices]=ismember(localKeys,priorKeys,'rows');assert(all(found));
            for j=1:numel(local{n})
                before=predicted(indices(j));after=local{n}(j);
                assert(before.numberOfGmComponents==1 && after.numberOfGmComponents==1);
                [encoded,packed]=gaussianLogRatio(before.mu{1},before.Sigma{1},after.mu{1},after.Sigma{1});
                local{n}(j).localSpatialLogRatio=encoded;
                run.localGaussianRecords(end+1,:)=[t,n,after.birthTime,after.birthLocation,packed]; %#ok<AGROW>
            end
        end""")
    begin = s.index("            if startsWith(rule,'asymmetric'),[decoded{n},packet]=asymmetricInnovationPacket")
    end = s.index('            sizes(n)=numel(packet);', begin)
    s = s[:begin]+"""            if startsWith(rule,'gaussian_evidence')
                [decoded{n},packet]=gaussianEvidencePacket(local{n},model,n,t);
                for j=1:numel(decoded{n})
                    o=decoded{n}(j);
                    run.packetGaussianRecords(end+1,:)=[t,n,o.birthTime,o.birthLocation,o.localSpatialLogRatio]; %#ok<AGROW>
                end
            else
                [decoded{n},packet]=asymmetricInnovationPacket(local{n},model,n,t);
            end
"""+s[end:]
    begin = s.index("            if startsWith(rule,'selective')")
    end = s.index('            run.iterationRecords=', begin)
    s = s[:begin]+"""            if strcmp(rule,'asymmetric')
                [posterior{n},stats]=fuseAsymmetricEvidence(inputs,[.5,.5],model,details,cfg,rule,t);
                stats.records=[stats.records,zeros(size(stats.records,1),23)];
            else
                [posterior{n},stats]=fuseGaussianEvidence(inputs,[.5,.5],model,details,cfg,rule,t);
            end
"""+s[end:]
    (OUT / 'runGaussianEvidenceReplay.m').write_text(s)
    print('Generated coherent Gaussian fusion and native spatial-evidence replay.')


if __name__ == '__main__':
    main()
