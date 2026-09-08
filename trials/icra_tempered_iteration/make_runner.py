"""Build isolated tempered-rule adapters from unchanged previous sources."""
from pathlib import Path

OUT = Path(__file__).resolve().parent
PRIOR = OUT.parent / 'icra_marked_control'


def replace(source, old, new, count=1):
    assert source.count(old) == count, (old, source.count(old), count)
    return source.replace(old, new)


def main():
    source = (OUT.parent / 'icra_marked_iteration/fuseMarkedInputsStable.m').read_text()
    source = replace(source, 'fuseMarkedInputsStable(', 'fuseTemperedRecency(')
    source = replace(source, '% ER scalar KL-average solution under r <= max(r_noage, direct evidence).',
                     '% Temper only the positive log-odds recency shift by current direct support.')
    source = source.replace('ceiling_association', 'tempered_association').replace('ceiling_score', 'tempered_score').replace('ceiling_calibrated', 'tempered_calibrated')
    source = replace(source, "if strcmp(arm,'lineage'),one.r=r0;elseif ~strcmp(arm,'er'),one.r=min(rEr,max(r0,cap));end",
                     "shift=sum((q-b).*logits);\n    if strcmp(arm,'lineage')\n        one.r=r0;\n    elseif ~strcmp(arm,'er') && shift>0 && cap<1\n        one.r=logistic(base+cap*shift);\n    end")
    source = replace(source, '        shift=sum((q-b).*logits);\n', '')
    source = replace(source, 'assert(one.r<=rEr && one.r>=min(r0,rEr)-1e-14);',
                     'assert(one.r<=rEr+1e-14 && one.r>=min(r0,rEr)-1e-14);')
    (OUT / 'fuseTemperedRecency.m').write_text(source.rstrip() + '\n')

    source = (PRIOR / 'runMarkedControlReplay.m').read_text()
    source = replace(source, 'function runMarkedControlReplay(', 'function runTemperedReplay(')
    source = source.replace("'holdout'", "'seen_transfer'")
    source = replace(source, 'checkStableMarkedEvidence();checkConservativeRecency();',
                     'checkStableMarkedEvidence();checkTemperedRecency();')
    source = replace(source, "arms={'marked_conservative'};\nif strcmp(mode,'development_check'),arms={'marked_er','marked_conservative'};end",
                     "arms={'marked_tempered_calibrated','marked_tempered_score','marked_tempered_association'};\nif strcmp(mode,'development_check'),arms=[{'marked_er'},arms];end")
    source = replace(source, "'protocol','shared-information-conservative-control-v1','implementation','marked-conservative-v1'",
                     "'protocol','evidence-tempered-recency-v1','implementation','tempered-v1'")
    source = source.replace('COMPLETED MARKED CONTROL', 'COMPLETED TEMPERED')
    source = replace(source, "if strcmp(rule,'conservative')\n                [posterior{n},stats]=fuseConservativeRecency(inputs,[.5,.5],model,details,cfg,'conservative_recency',t);\n            else\n                [posterior{n},stats]=fusePortInputs(inputs,[.5,.5],model,details,cfg,rule,t);\n            end",
                     '[posterior{n},stats]=fuseTemperedRecency(inputs,[.5,.5],model,details,cfg,rule,t);')
    source = source.replace("'ceiling_score'", "'tempered_score'").replace("'ceiling_calibrated'", "'tempered_calibrated'")
    source = replace(source, "startsWith(rule,'ceiling_')", "startsWith(rule,'tempered_')")
    source = source.replace('START PORT', 'START TEMPERED').replace('DONE PORT', 'DONE TEMPERED')
    (OUT / 'runTemperedReplay.m').write_text(source.rstrip() + '\n')
    print('Tempered rule and runner generated; earlier sources unchanged.')


if __name__ == '__main__':
    main()
