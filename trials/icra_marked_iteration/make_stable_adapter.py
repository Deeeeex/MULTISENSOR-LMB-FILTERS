"""Equivalent mark insertion into the proven full local-update structure."""
from pathlib import Path
OUT=Path(__file__).resolve().parent
source=OUT.parent/'icra_ceiling_iteration/updateLmbWithAssociationWeights.m'
s=source.read_text()
s=s.replace('updateLmbWithAssociationWeights(', 'updateMarkedLmbStable(',1)
s=s.replace('currentTime, isScheduledSample)', 'currentTime, ratios)',1)
old='''if nargin < 6 || isempty(isScheduledSample)
    isScheduledSample = true;
end'''
new='''ratios=reshape(ratios,1,[]);
assert(numel(ratios)==numel(measurements) && all(isfinite(ratios)) && all(ratios>0));
isScheduledSample=true;'''
assert old in s;s=s.replace(old,new,1)
point='diagnostics = extractDiagnostics(associationMatrices);'
insert='''% Independent observation mark multiplies each detection association once.
% Measurement-conditioned Gaussian components have already been normalized;
% the same scalar over those components cancels within their normalization.
if ~all(ratios==1)
    associationMatrices.Psi=associationMatrices.Psi.*ratios;
    detectionTerms=associationMatrices.L(:,2:end).*ratios;
    associationMatrices.L=[associationMatrices.eta,detectionTerms];
    associationMatrices.P=detectionTerms./(detectionTerms+associationMatrices.eta);
    associationMatrices.C=-log(detectionTerms);
end

'''
assert point in s;s=s.replace(point,insert+point,1)
(OUT/'updateMarkedLmbStable.m').write_text(s)
s=(OUT.parent/'icra_ceiling_iteration/fuseEvidenceCeiling.m').read_text()
s=s.replace('function [objects,stats]=fuseEvidenceCeiling(', 'function [objects,stats]=fuseMarkedInputsStable(',1)
s=s.replace("assert(any(strcmp(arm,{'qualified_exist','ceiling_association','ceiling_score','ceiling_calibrated'})));",
            "arm=erase(arm,'marked_');\nassert(any(strcmp(arm,{'lineage','er','ceiling_association','ceiling_score','ceiling_calibrated'})));")
s=s.replace("    if ~strcmp(arm,'qualified_exist'),one.r=min(rEr,max(r0,cap));end",
            "    if strcmp(arm,'lineage'),one.r=r0;elseif ~strcmp(arm,'er'),one.r=min(rEr,max(r0,cap));end")
s=s.replace("    assert(isfinite(one.r) && one.r<=rEr && one.r>=min(r0,rEr)-1e-14);",
            "    assert(isfinite(one.r));\n    if ~strcmp(arm,'lineage'),assert(one.r<=rEr && one.r>=min(r0,rEr)-1e-14);end")
s=s.replace("rr=min(max(one.r,1e-9),1-1e-9);admitted=log(rr)-log1p(-rr)-base;",
            "rr=min(max(one.r,1e-9),1-1e-9);admitted=log(rr)-log1p(-rr)-base;\n        if strcmp(arm,'lineage'),admitted=0;end")
s=s.replace("end\nend\n\nfunction r=logistic(z)", "end\nif strcmp(arm,'lineage'),stats.weightChange=0;end\nend\n\nfunction r=logistic(z)")
(OUT/'fuseMarkedInputsStable.m').write_text(s)
s=(OUT/'runMarkedEvidenceReplay.m').read_text()
s=s.replace('function runMarkedEvidenceReplay(', 'function runMarkedEvidenceReplayStable(',1)
s=s.replace('checkMarkedEvidence();','checkStableMarkedEvidence();',1)
s=s.replace("'source_sha256.json'", "'source_sha256_stable.json'")
s=s.replace("'results_development'", "'results_stable'")
s=s.replace("'implementation','mecr-v1'", "'implementation','mecr-v1-stable-adapter'")
s=s.replace('updateMarkedLmb(', 'updateMarkedLmbStable(')
s=s.replace('fuseMarkedInputs(', 'fuseMarkedInputsStable(')
(OUT/'runMarkedEvidenceReplayStable.m').write_text(s)
print('Full-structure marked adapter generated; observation and fusion formulas unchanged.')
