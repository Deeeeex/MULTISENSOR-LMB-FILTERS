"""Extend the independent NumPy audit for GS and fixed admission exponents."""
from pathlib import Path

OUT=Path(__file__).resolve().parent
OLD=OUT.parent/'icra_gaussian_evidence'


def replace(text,old,new):
    assert text.count(old)==1,old[:100]
    return text.replace(old,new)


def main():
    paths=[OUT/'review_probability_audit.py',OUT/'review_gaussian_audit.py']
    assert not any(p.exists() for p in paths)
    text=(OLD/'analyze_gaussian.py').read_text().split('\ndef audit_file(')[0]
    text=replace(text,'from gaussian_audit import check_gaussians','from review_gaussian_audit import check_gaussians')
    text=replace(text,'ARMS = NEW_ARMS.copy()',"ARMS = NEW_ARMS + ['marked_gaussian_evidence_guarded_scalar','marked_gaussian_evidence_fixed_025','marked_gaussian_evidence_fixed_050','marked_gaussian_evidence_fixed_100']")
    text=replace(text,"    arm = run['arm']", "    arm = run['arm']\n    fixed = '_fixed_' in arm\n    strength = int(arm.rsplit('_', 1)[1])/100 if fixed else None")
    text=replace(text,'np.isin(pd, [0, .9])',"np.isin(pd, [0, data['pd']])")
    text=replace(text,"expected_gate = row[8] if stamps[i, side] == records[i, 0] else 0",
                 "expected_gate = (strength if fixed else row[8]) if stamps[i, side] == records[i, 0] else 0")
    text=replace(text,"expected_negative_gate = row[11] if stamps[i, side] == records[i, 0] else 0",
                 "expected_negative_gate = (strength if fixed else row[11]) if stamps[i, side] == records[i, 0] else 0")
    paths[0].write_text(text)
    text=(OLD/'gaussian_audit.py').read_text()
    text=replace(text,"    log_i[unchanged] = records[unchanged, 10]",
                 "    log_i[unchanged] = records[unchanged, 10]\n"
                 "    if run['arm'] == 'marked_gaussian_evidence_guarded_scalar':\n"
                 "        mean, cov, log_i = base_mean, base_cov, records[:, 10].copy()")
    paths[1].write_text(text)
    print('Created independent controls audit from frozen NumPy reconstruction.')


if __name__=='__main__':main()
