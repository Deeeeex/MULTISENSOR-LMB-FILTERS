"""Build isolated review controls; never modify any frozen experiment source."""
from pathlib import Path

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
OLD = OUT.parent / 'icra_gaussian_evidence'


def replace(text, old, new):
    assert text.count(old) == 1, old[:120]
    return text.replace(old, new)


def main():
    targets = [OUT / 'fuseReviewerEvidence.m', OUT / 'runReviewerReplay.m']
    assert not any(p.exists() for p in targets), 'Never regenerate frozen extensions.'
    code = (OLD / 'fuseGaussianEvidence.m').read_text()
    code = replace(code, 'fuseGaussianEvidence(', 'fuseReviewerEvidence(')
    code = replace(code,
        "assert(ismember(arm,{'gaussian_evidence','gaussian_evidence_no_curvature','gaussian_evidence_no_history','gaussian_evidence_no_mark'}));",
        "assert(ismember(arm,{'gaussian_evidence_guarded_scalar','gaussian_evidence_fixed_025','gaussian_evidence_fixed_050','gaussian_evidence_fixed_100'}));\n"
        "fixed=startsWith(arm,'gaussian_evidence_fixed_');lambda=0;\n"
        "if fixed,lambda=str2double(extractAfter(arm,'gaussian_evidence_fixed_'))/100;end")
    code = replace(code,
        "if stamps(s)==t,gates(s)=o.positiveInnovationSupport;negativeGates(s)=o.negativeInnovationSupport;end",
        "if stamps(s)==t\n"
        "            gates(s)=o.positiveInnovationSupport;negativeGates(s)=o.negativeInnovationSupport;\n"
        "            if fixed,gates(s)=lambda;negativeGates(s)=lambda;end\n"
        "        end")
    code = replace(code,
        "    one.r=logistic(sum(beta.*logits)+sum(kept.*increments)+logIntegral);",
        "    if strcmp(arm,'gaussian_evidence_guarded_scalar')\n"
        "        % Test the candidate ratio exactly as GCE, then keep p0 and I0.\n"
        "        one.mu={oldMean};one.Sigma={oldCov};logIntegral=rec.spatialLogNormalizer;\n"
        "    end\n"
        "    one.r=logistic(sum(beta.*logits)+sum(kept.*increments)+logIntegral);")
    code = replace(code, 'scalarR=logistic(base+correction);oldMean=one.mu{1};',
                   'scalarR=logistic(base+correction);oldMean=one.mu{1};oldCov=one.Sigma{1};')
    targets[0].write_text(code)

    original = (OLD / 'runGaussianEvidenceReplay.m').read_text()
    tail = original[original.index('function model=makeModel'):]
    tail = replace(tail, 'makeModel(positions,T,baseQuality)', 'makeModel(positions,T,baseQuality,pd)')
    tail = replace(tail, 'generateMultisensorModel(2,[3,3],[.9,.9],[1,1]', 'generateMultisensorModel(2,[3,3],[pd,pd],[1,1]')
    tail = replace(tail,
        "            else\n                [decoded{n},packet]=asymmetricInnovationPacket(local{n},model,n,t);",
        "            elseif strcmp(rule,'lineage') || strcmp(rule,'er')\n"
        "                [decoded{n},packet]=ceilingLmbPacket(local{n},model,n,t);\n"
        "            else\n                [decoded{n},packet]=asymmetricInnovationPacket(local{n},model,n,t);")
    tail = replace(tail,
        "            else\n                [posterior{n},stats]=fuseGaussianEvidence(inputs,[.5,.5],model,details,cfg,rule,t);",
        "            elseif strcmp(rule,'lineage') || strcmp(rule,'er')\n"
        "                [posterior{n},stats]=fuseMarkedInputsStable(inputs,[.5,.5],model,details,cfg,rule,t);\n"
        "                stats.records=[stats.records,zeros(size(stats.records,1),34)];\n"
        "            elseif strcmp(rule,'gaussian_evidence_guarded_scalar') || startsWith(rule,'gaussian_evidence_fixed_')\n"
        "                [posterior{n},stats]=fuseReviewerEvidence(inputs,[.5,.5],model,details,cfg,rule,t);\n"
        "            else\n                [posterior{n},stats]=fuseGaussianEvidence(inputs,[.5,.5],model,details,cfg,rule,t);")
    header = """function runReviewerReplay(configPath,unitIndex)
% Isolated recursive controls on immutable, externally registered inputs.
out=fileparts(mfilename('fullpath'));root=fileparts(fileparts(out));
config=jsondecode(fileread(configPath));unit=config.units(unitIndex);
original=fullfile(root,'trials','icra_external_fusion');
priorIteration=fullfile(root,'trials','icra_method_iteration');
folders={'icra_fusion_holdout','icra_asymmetric_evidence','icra_selective_innovation', ...
    'icra_evidence_iteration','icra_ceiling_iteration','icra_marked_iteration', ...
    'icra_gaussian_evidence','icra_reunion_fusion'};
for k=1:numel(folders),addpath(fullfile(root,'trials',folders{k}));end
addpath(original,priorIteration,fullfile(root,'common'),fullfile(root,'lmb'),fullfile(root,'multisensorLmb'));
checkStableMarkedEvidence();checkGaussianEvidence();checkReviewerEvidence();
baseQuality=@evaluateSensorQuality;
assert(strcmp(functions(baseQuality).file,fullfile(root,'common','evaluateSensorQuality.m')));
qualityPath=fullfile(original,'replay_quality');addpath(qualityPath);
cleanup=onCleanup(@()rmpath(qualityPath)); %#ok<NASGU>
runtimePath=fullfile(priorIteration,'runtime');addpath(runtimePath);
runtimeCleanup=onCleanup(@()rmpath(runtimePath)); %#ok<NASGU>
assert(strcmp(which('Hungarian'),fullfile(runtimePath,'Hungarian.m')));
data=load(fullfile(root,unit.data_path));
if isfield(data,'likelihoodRatios')
    ratios=struct('likelihoodRatios',{data.likelihoodRatios});
    marks=struct('rawScores',{data.rawScores},'calibratedScores',{data.calibratedScores});
else
    marks=load(fullfile(root,unit.marks_path));ratios=load(fullfile(root,unit.ratios_path));
end
T=double(data.T);measurements=data.measurements;positions=data.positions;
model=makeModel(positions,T,baseQuality,config.pd);
resultDir=fullfile(out,'results',config.stage);if ~isfolder(resultDir),mkdir(resultDir);end
for condition={'reliable','intermittent'}
    radio=condition{1};delivered=repmat(logical([0,1;1,0]),1,1,T);
    if strcmp(radio,'intermittent')
        rng(unit.radio_seed,'twister');draws=rand(2,2,T);delivered=delivered & draws>=.1;
        delivered(:,:,floor(.4*T)+1:floor(.6*T))=false;
    end
    for a=1:numel(config.arms)
        arm=config.arms{a};
        path=fullfile(resultDir,sprintf('%s_%s_%s.json',unit.sequence,radio,arm));
        assert(~isfile(path) && ~isfile([path,'.gz']),'Never overwrite a registered run.');
        fprintf('START REVIEW %s %s %s %s %d frames\\n',config.stage,unit.sequence,radio,arm,T);
        run=runDensity(model,measurements,positions,delivered,arm,marks,ratios);
        % Truth first enters after this arm has completed its full trajectory.
        run=scoreOutputs(run,data.truth);
        result=struct('protocol','icra-reviewer-revision-v1','stage',config.stage, ...
            'cohort',config.cohort,'sourceSha256',config.source_sha256, ...
            'inputSha256',unit.input_sha256,'sequence',unit.sequence,'condition',radio, ...
            'pd',config.pd,'time',double(data.time),'positions',positions, ...
            'truth',{data.truth},'truthIds',{data.truthIds},'delivered',delivered,'runs',run);
        fid=fopen(path,'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(result));fclose(fid);gzip(path);delete(path);
        fprintf('DONE REVIEW %s %s %s %s OSPA %.6f runtime %.2fs\\n', ...
            config.stage,unit.sequence,radio,arm,mean(run.ospa,'all'),run.runtimeSeconds);
    end
end
fprintf('COMPLETED REVIEW %s %s\\n',config.stage,unit.sequence);
end

"""
    targets[1].write_text(header + tail)
    print('Created isolated GS/fixed-ratio extensions; original source unchanged.')


if __name__ == '__main__':
    main()
