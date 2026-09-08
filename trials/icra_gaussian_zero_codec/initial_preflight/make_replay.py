"""Copy the fixed Gaussian replay, changing only codec and its diagnostics."""
from pathlib import Path

OUT = Path(__file__).resolve().parent
PARENT = OUT.parent / 'icra_gaussian_evidence'


def replace(s, old, new, count=1):
    assert s.count(old) == count, (old, s.count(old), count)
    return s.replace(old, new)


def main():
    s = (PARENT / 'runGaussianEvidenceReplay.m').read_text()
    s = s.replace('runGaussianEvidenceReplay', 'runGaussianZeroReplay')
    s = replace(s, 'checkStableMarkedEvidence();checkGaussianEvidence();', 'checkStableMarkedEvidence();checkGaussianZeroCodec();')
    s = replace(s, "if strcmp(mode,'development_check'),arms=[{'marked_asymmetric'},arms];end\nif strcmp(mode,'seen_transfer'),arms=[arms,{'marked_asymmetric'}];end",
                "if ~strcmp(mode,'development_check'),arms={'marked_gaussian_evidence'};end")
    s = s.replace('coherent-gaussian-evidence-v1', 'coherent-gaussian-zero-codec-v1').replace('gaussian-evidence-v1', 'gaussian-zero-codec-v1')
    s = replace(s, "'packetGaussianRecords',zeros(0,19),", "'packetGaussianRecords',zeros(0,19),'packetGaussianTags',zeros(0,5),")
    s = replace(s, '[decoded{n},packet]=gaussianEvidencePacket(local{n},model,n,t);',
                '[decoded{n},packet,tags]=gaussianZeroPacket(local{n},model,n,t);')
    s = replace(s, '                    o=decoded{n}(j);', '                    o=decoded{n}(j);\n                    run.packetGaussianTags(end+1,:)=[t,n,o.birthTime,o.birthLocation,tags(j)]; %#ok<AGROW>')
    s = s.replace('GAUSSIAN EVIDENCE', 'GAUSSIAN ZERO CODEC')
    (OUT / 'runGaussianZeroReplay.m').write_text(s)
    print('Generated codec-only replay: four-arm preflight; primary-only complete parity stages.')


if __name__ == '__main__':
    main()
