"""Run only CR on unchanged cached synthetic inputs; keep original outputs."""
from pathlib import Path
import hashlib,json
out=Path(__file__).resolve().parent;root=out.parents[1]
source=out.parent/'icra_reunion_fusion/runRobotReunionValidation.m'
s=source.read_text();helpers=s[s.index('function run=runArm('):]
helpers=helpers.replace('fuseValidationInputs(inputs,weights,model,details,cfg,arm,t)',
                        'fuseConservativeRecency(inputs,weights,model,details,cfg,arm,t)')
head='''function runConservativeCases(firstSeed,lastSeed)
if nargin<1,firstSeed=2901;end
if nargin<2,lastSeed=2920;end
assert(firstSeed>=2901 && lastSeed<=2920);
out=fileparts(mfilename('fullpath'));root=fileparts(fileparts(out));
original=fullfile(root,'trials','icra_reunion_fusion');
addpath(fullfile(root,'common'),fullfile(root,'lmb'),fullfile(root,'multisensorLmb'),original, ...
    fullfile(root,'trials','icra_group_tracking'));
dest=fullfile(out,'results_cases');if ~isfolder(dest),mkdir(dest);end
sources=jsondecode(fileread(fullfile(out,'source_sha256_cases.json')));
for seed=firstSeed:lastSeed
    for name={'split_latebirth','churn_departure','split_no_new'}
        stem=sprintf('%s_seed%d_validation',name{1},seed);
        data=load(fullfile(original,'results',[stem,'.mat']));
        fprintf('START CR case %s seed %d\\n',name{1},seed);
        run=runArm(data.model,data.truth,data.measurements,data.uniforms,data.scene, ...
            data.adjacency,'conservative_recency',data.scene.T);
        reference=data.result.runs(strcmp({data.result.runs.arm},'qualified_exist'));
        assert(run.totalWireBytes==reference.totalWireBytes);
        assert(isequal(run.deliveredMessages,reference.deliveredMessages));
        result=data.result;result.runs=run;result.protocol='conservative-recency-case-studies-v1';
        result.sourceSha256=sources;
        path=fullfile(dest,[stem,'_cr.json']);writeJson(path,result);gzip(path);delete(path);
        fprintf('DONE CR case %s seed %d OSPA %.6f count %.6f runtime %.2fs\\n', ...
            name{1},seed,mean(run.ospa,'all'),mean(run.countError,'all'),run.runtimeSeconds);
    end
end
fprintf('COMPLETED CR cases seeds %d--%d\\n',firstSeed,lastSeed);
end

'''
(out/'runConservativeCases.m').write_text(head+helpers)
base=json.loads((out/'source_sha256_cr.json').read_text())
for name,digest in base.items():assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
for name in ['runConservativeCases.m','make_case_runner.py']:
    p=out/name;base[str(p.relative_to(root))]=hashlib.sha256(p.read_bytes()).hexdigest()
(out/'source_sha256_cases.json').write_text(json.dumps(base,indent=2,sort_keys=True)+'\n')
print('Case-study CR source frozen',len(base),'files; cached inputs reused.')
