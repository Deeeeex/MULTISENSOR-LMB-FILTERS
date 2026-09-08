"""Vectorize only the original Hungarian row-major uncovered-zero search."""
from pathlib import Path
import hashlib,json
out=Path(__file__).resolve().parent;root=out.parents[1]
source=root/'common/Hungarian.m';s=source.read_text()
start=s.index('      row = 0; col = 0; exit_flag = 1;')
end=s.index('    % If there are no uncovered zeros go to step 6',start)
s=s[:start]+'''      % Transpose preserves the original row-major first-zero tie break.
      [col,row] = find((P_cond==0 & (r_cov(:)==0) & (c_cov(:)'==0))',1);
      if isempty(row), row=0; col=0; end

'''+s[end:]
runtime=out/'runtime';runtime.mkdir(exist_ok=True)
(runtime/'Hungarian.m').write_text(s)
for name,kind in [('Method','ir'),('Conservative','cr')]:
    source=out/f'run{name}Replay.m';s=source.read_text()
    s=s.replace(f'run{name}Replay',f'run{name}ReplayFast')
    s=s.replace("resultDir=fullfile(out,'results_v2v')","resultDir=fullfile(out,'results_v2v_fast')")
    s=s.replace("resultDir=fullfile(out,'results_v2v_conservative')","resultDir=fullfile(out,'results_v2v_conservative_fast')")
    old='source_sha256.json' if kind=='ir' else 'source_sha256_cr.json'
    s=s.replace(old,f'source_sha256_{kind}_fast.json')
    s=s.replace(f"'{kind}-v1'",f"'{kind}-v1-accelerated'")
    needle='cleanup=onCleanup(@()rmpath(qualityPath)); %#ok<NASGU>'
    assert needle in s
    s=s.replace(needle,needle+"\nruntimePath=fullfile(out,'runtime');addpath(runtimePath);\nruntimeCleanup=onCleanup(@()rmpath(runtimePath)); %#ok<NASGU>\nassert(strcmp(which('Hungarian'),fullfile(runtimePath,'Hungarian.m')));")
    target=out/f'run{name}ReplayFast.m';target.write_text(s)
    manifest=json.loads((out/old).read_text())
    for p,h in manifest.items():assert hashlib.sha256((root/p).read_bytes()).hexdigest()==h,p
    for p in [target,runtime/'Hungarian.m',out/'AMENDMENT_RUNTIME.md',out/'make_runtime_acceleration.py',out/'checkHungarianAcceleration.m']:
        manifest[str(p.relative_to(root))]=hashlib.sha256(p.read_bytes()).hexdigest()
    (out/f'source_sha256_{kind}_fast.json').write_text(json.dumps(manifest,indent=2,sort_keys=True)+'\n')
print('Runtime-only acceleration generated; original source and both candidate methods remain frozen.')
