"""Create an isolated extra-control runner without changing the frozen port."""
from pathlib import Path

OUT = Path(__file__).resolve().parent
PORT = OUT.parent / 'icra_fusion_holdout'


def main():
    source = (PORT / 'runFusionSelectionReplay.m').read_text()
    source = source.replace('function runFusionSelectionReplay(firstIndex,lastIndex,mode)',
                            'function runMarkedControlReplay(firstIndex,lastIndex,mode)')
    source = source.replace("{'development_check','holdout'}", "{'development_check','development','holdout'}")
    source = source.replace("priorIteration=fullfile(root,'trials','icra_method_iteration');",
                            "port=fullfile(root,'trials','icra_fusion_holdout');\npriorIteration=fullfile(root,'trials','icra_method_iteration');")
    source = source.replace('addpath(priorIteration,priorCeiling,priorMarked,original,',
                            'addpath(port,priorIteration,priorCeiling,priorMarked,original,')
    source = source.replace('checkStableMarkedEvidence();setupTcDependency();checkGaussianMil();',
                            'checkStableMarkedEvidence();checkConservativeRecency();')
    source = source.replace("fullfile(out,'source_sha256_port.json')", "fullfile(out,'source_sha256.json')")
    source = source.replace("fullfile(out,'METHOD_FREEZE.json')", "fullfile(port,'METHOD_FREEZE.json')")
    source = source.replace("fullfile(out,'input_manifest.json')", "fullfile(port,'input_manifest.json')")
    source = source.replace("fullfile(out,'data',sprintf('v2v4real_%04d.mat',seq))",
                            "fullfile(port,'data',sprintf('v2v4real_%04d.mat',seq))")
    start = source.index("arms={'local'")
    stop = source.index('for sequenceIndex=', start)
    source = source[:start] + "arms={'marked_conservative'};\nif strcmp(mode,'development_check'),arms={'marked_er','marked_conservative'};end\n" + source[stop:]
    start = source.index("    fprintf('START PORT %04d local")
    stop = source.index("    for condition={'reliable'", start)
    source = source[:start] + source[stop:]
    source = source.replace("'protocol','remaining-v2v4real-fusion-selection-v1','implementation','common-port-v1'",
                            "'protocol','shared-information-conservative-control-v1','implementation','marked-conservative-v1'")
    source = source.replace("fprintf('COMPLETED PORT indices", "fprintf('COMPLETED MARKED CONTROL indices")
    source = source.replace('[posterior{n},stats]=fusePortInputs(inputs,[.5,.5],model,details,cfg,rule,t);',
                            "if strcmp(rule,'conservative')\n                [posterior{n},stats]=fuseConservativeRecency(inputs,[.5,.5],model,details,cfg,'conservative_recency',t);\n            else\n                [posterior{n},stats]=fusePortInputs(inputs,[.5,.5],model,details,cfg,rule,t);\n            end")
    assert source.count('function runMarkedControlReplay') == 1
    assert source.count('fuseConservativeRecency(inputs') == 1
    (OUT / 'runMarkedControlReplay.m').write_text(source.rstrip() + '\n')
    print('Isolated conservative marked runner generated; original port unchanged.')


if __name__ == '__main__':
    main()
