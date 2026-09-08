"""Freeze a development-selected primary before any new cohort tracking."""
from pathlib import Path
import hashlib,json
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
primary='marked_ceiling_score'
arms=['local','lineage','qualified_exist','mil_support','tc_ospa2_w5','tc_ospa2_w10','ceiling_calibrated',
      'marked_local','marked_lineage','marked_er','marked_mil_support','marked_tc_ospa2_w5','marked_tc_ospa2_w10',
      'marked_ceiling_association','marked_ceiling_score','marked_ceiling_calibrated']
assert not (OUT/'METHOD_FREEZE.json').exists()
assert not list((OUT/'results_holdout').glob('*.json.gz')),'Holdout outcomes already exist.'
source=json.loads((OUT/'source_sha256_port.json').read_text())
for p,h in source.items():assert hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h,p
marked=OUT.parent/'icra_marked_iteration/summary_development.json'
development=json.loads(marked.read_text());assert development['sequences']==9 and development['audited_node_frames']==39860
port=json.loads((OUT/'port_audit.json').read_text());assert port['audited_node_frames']==9408 and port['paired_prior_node_frames']==7056
assert max(r['maximum_numeric_difference'] for r in port['parity'])<1e-9
cohort=json.loads((OUT/'input_manifest.json').read_text());assert cohort['frames']==5601 and len(cohort['sequences'])==25
evidence={}
for p in [OUT/'source_sha256_port.json',OUT/'input_manifest.json',OUT/'COHORT_PROTOCOL.md',OUT/'PORT_PROTOCOL.md',
          OUT/'port_audit.json',marked,OUT.parent/'icra_ceiling_iteration/summary_development.json',
          OUT.parent/'icra_ceiling_iteration/summary_cases.json',OUT/'register_method.py',OUT/'run_holdout.py']:
    evidence[str(p.relative_to(ROOT))]=hashlib.sha256(p.read_bytes()).hexdigest()
report=dict(protocol='remaining-v2v4real-fusion-selection-v1',primary=primary,arms=arms,
            selection='Shared calibrated mark-likelihood local update plus continuous raw-score evidence ceiling. Fixed from all nine development sequences: lowest mean OSPA of the five matched-information arms in both conditions. This is a development choice, not an independent result.',
            primary_references=['marked_lineage','marked_er'],
            external_references=['marked_mil_support','marked_tc_ospa2_w5','marked_tc_ospa2_w10'],
            conditions=['reliable','intermittent'],endpoint='Sequence-macro 2D position OSPA, cutoff 12 m, p=2.',
            units=cohort['selected_sequences'],frames=5601,
            bootstrap=dict(unit='sequence',resamples=10000,seed=8301,interval='95% percentile',multiplicity_adjusted=False),
            paper_gate='Inspect both radio conditions and all sequences. A persuasive main result must improve over same-information No-age and ER, supported by paired intervals rather than a favorable pooled mean alone; inspect missed/false cost, shared localization support and communication costs. Retain failures and do not revise the primary from this cohort. If evidence remains weak, continue method work and classify this cohort as seen.',
            source_and_selection_evidence_sha256=evidence,
            limitations='Released detector was trained on this train split; these are unused fusion-selection outcomes, not an independent detector benchmark. Routes may be related; intervals are descriptive. A raw support mark is not a proven statistical confidence bound.')
(OUT/'METHOD_FREEZE.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
print('FINAL PRIMARY FROZEN',primary,'before any 25-sequence holdout outcome.')
