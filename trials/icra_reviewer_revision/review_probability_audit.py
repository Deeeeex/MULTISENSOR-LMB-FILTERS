"""Independently check selective trajectories and reuse complete saved controls."""
from pathlib import Path
import argparse
import json
import re
import sys
import time

import numpy as np
from scipy.io import loadmat
from scipy.special import expit

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
PORT = OUT.parent / 'icra_fusion_holdout'
CONTROL = OUT.parent / 'icra_marked_control'
sys.path.insert(0, str(PORT))
from analyze_holdout import counterfactual, radio_draws, score
sys.path.insert(0, str(CONTROL))
from analyze_control import sha, read, score_run, interval, METRICS
from analyze_full import baseline as saved_baseline

PRIMARY = 'marked_gaussian_evidence'
NEW_ARMS = [PRIMARY, 'marked_gaussian_evidence_no_curvature', 'marked_gaussian_evidence_no_history', 'marked_gaussian_evidence_no_mark']
OLD_ARMS = ['marked_asymmetric']
ARMS = NEW_ARMS + ['marked_gaussian_evidence_guarded_scalar','marked_gaussian_evidence_fixed_025','marked_gaussian_evidence_fixed_050','marked_gaussian_evidence_fixed_100']
BASE_REFERENCES = ['marked_lineage', 'marked_er', 'marked_conservative',
              'marked_ceiling_calibrated', 'marked_ceiling_score']
REFERENCES = BASE_REFERENCES + OLD_ARMS
ASYMMETRIC = OUT.parent / 'icra_asymmetric_evidence'
from review_gaussian_audit import check_gaussians


def source_check():
    source = json.loads((OUT / 'source_sha256.json').read_text())
    for name, expected in source.items():
        assert sha(ROOT / name) == expected, name
    mapped = {re.sub('[^A-Za-z0-9_]', '_', name)[:63]: value for name, value in source.items()}
    assert len(mapped) == len(source)
    return mapped


def audit_probability(run, data):
    arm = run['arm']
    fixed = '_fixed_' in arm
    strength = int(arm.rsplit('_', 1)[1])/100 if fixed else None
    assert arm in ARMS + OLD_ARMS
    local = np.asarray(run['localIncrementRecords'], float).reshape(-1, 12)
    assert len(local) and np.isfinite(local).all()
    assert np.all(local[:, 0] >= 1) and np.all(local[:, 0] <= len(data['time']))
    assert np.isin(local[:, 1], [1, 2]).all() and np.isin(local[:, 7], [0, 1]).all()
    assert np.all((local[:, 4:6] >= 0) & (local[:, 4:6] <= 1))
    assert len(np.unique(local[:, :4], axis=0)) == len(local)
    before, after = np.clip(local[:, 4], 1e-9, 1-1e-9), np.clip(local[:, 5], 1e-9, 1-1e-9)
    expected_delta = np.log(after)-np.log1p(-after)-np.log(before)+np.log1p(-before)
    assert np.allclose(local[:, 6], expected_delta, atol=2e-12, rtol=0)
    assert np.all(np.abs(local[local[:, 7] == 0, 6]) < 1e-10)
    assert np.all((local[:, 8:10] >= 0) & (local[:, 8:10] <= 1))
    assert np.all(local[:, 8] <= local[:, 9]+1e-14)
    assert np.all(local[local[:, 7] == 0, 8:10] == 0)
    if arm == 'marked_gaussian_evidence_no_mark':
        assert np.array_equal(local[:, 8], local[:, 9])
    pd, negative_local = local[:, 10], local[:, 11]
    assert np.isin(pd, [0, data['pd']]).all() and np.array_equal(pd > 0, local[:, 7].astype(bool))
    expected_negative = (1-local[:, 9])*pd/(2-pd)
    assert np.allclose(negative_local, expected_negative, atol=1e-14, rtol=0)
    records = np.asarray(run['iterationRecords'], float).reshape(-1, 60)
    assert len(records) and np.isfinite(records[:, :17]).all() and np.isfinite(records[:, 19:]).all()
    b, q = records[:, 13:15], records[:, 11:13]
    r, delta = records[:, 17:19], records[:, 19:21]
    factors, stamps = records[:, 15:17], records[:, 21:23]
    gates, beta = records[:, 26:28], records[:, 28:30]
    negative_gates = records[:, 35:37]
    active = b > 0
    assert np.isfinite(r[active]).all() and np.all((r[active] >= 0) & (r[active] <= 1))
    assert np.allclose(b.sum(1), 1, atol=1e-14) and np.isin(b, [0, .5, 1]).all()
    expected_factors = np.where(stamps > 0, .25+.75*np.exp(-(records[:, 0, None]-stamps)*.1/5), .25)
    assert np.all(stamps <= records[:, 0, None])
    assert np.allclose(factors[stamps > 0], expected_factors[stamps > 0], atol=2e-14, rtol=0)
    assert np.isin(factors[stamps == 0], [.25, 1]).all()
    expected_q = b*factors
    expected_q /= expected_q.sum(1, keepdims=True)
    assert np.allclose(q, expected_q, atol=2e-14, rtol=0)
    logits = np.zeros_like(b)
    rr = np.clip(r[active], 1e-9, 1-1e-9)
    logits[active] = np.log(rr)-np.log1p(-rr)
    base = (b*logits).sum(1)+records[:, 10]
    age = ((q-b)*logits).sum(1)
    joined = 0
    assert np.all((gates >= 0) & (gates <= 1))
    assert np.all((negative_gates >= 0) & (negative_gates <= 1))
    assert np.all(negative_gates[stamps != records[:, 0, None]] == 0)
    assert np.all(gates[stamps != records[:, 0, None]] == 0)
    original = records[:, 31:35].reshape(-1, 2, 2)
    present = original[:, :, 0] > 0
    assert np.all(original[~present] == 0) and np.all(original[present] > 0)
    assert np.all(delta[~present] == 0) and np.all(gates[~present] == 0)
    local_lookup = {tuple(row[:4].astype(int)): row for row in local}
    for i, side in np.argwhere(present):
        source = int(records[i, 1]) if side == 0 else 3-int(records[i, 1])
        key = (int(records[i, 0]), source, *original[i, side].astype(int))
        row = local_lookup[key]
        assert np.isclose(delta[i, side], row[6], atol=2e-12, rtol=0), (arm, key, 'received delta')
        expected_gate = (strength if fixed else row[8]) if stamps[i, side] == records[i, 0] else 0
        assert gates[i, side] == expected_gate, (arm, key, 'received gate')
        expected_negative_gate = (strength if fixed else row[11]) if stamps[i, side] == records[i, 0] else 0
        assert negative_gates[i, side] == expected_negative_gate, (arm, key, 'received negative gate')
        if active[i, side]:
            assert np.isclose(r[i, side], row[5], atol=1e-9, rtol=0), (arm, key, 'received existence')
        joined += 1
    joint = (active.sum(1) >= 2) & ((~active) | present).all(1)
    expected_beta = b.copy()
    if arm != 'marked_gaussian_evidence_no_history':
        expected_beta[age < -1e-12] = q[age < -1e-12]
    assert np.allclose(beta, expected_beta, atol=2e-14, rtol=0)
    fresh = gates*np.maximum(delta, 0)+negative_gates*np.minimum(delta, 0)
    boost = joint*((active-beta)*fresh).sum(1)
    assert np.allclose(records[:, 30], boost, atol=1e-10, rtol=0)
    correction = ((beta-b)*logits).sum(1)+boost
    expected, r0, er = expit(base+correction), expit(base), expit(base+age)
    columns = [(6, expected), (7, r0), (8, er), (9, expected)] if arm in OLD_ARMS else [(37, expected), (7, r0), (8, er)]
    for column, value in columns:
        assert np.allclose(records[:, column], value, atol=2e-12, rtol=0), (arm, 'selective scalar', column)
    assert np.allclose(records[:, 23], age, atol=1e-10, rtol=0)
    assert np.allclose(records[:, 24], correction, atol=1e-10, rtol=0)
    assert np.allclose(records[:, 23]-records[:, 24], records[:, 25], atol=1e-10, rtol=0)
    diagnostic = dict(arm=arm, local_update_records=len(local), labels=len(records), joined_source_records=joined,
                      joint_labels=int(joint.sum()), boosted_labels=int((boost > 1e-8).sum()),
                      negative_boost_labels=int((boost < -1e-8).sum()),
                      mean_gate=float(gates[active].mean()), mean_negative_gate=float(negative_gates[active].mean()),
                      mean_boost=float(boost.mean()))
    diagnostic.update(check_gaussians(run, records, original, present, active, joint, beta, logits, delta, gates, negative_gates))
    if arm == PRIMARY:
        delivery, poses = np.asarray(data['delivered']), np.asarray(data['positions'])
        rules = ['scalar_AS', 'new_r_old_space', 'old_r_new_space', 'candidate']
        effects = {rule: {key: [] for key in METRICS[:6]} for rule in rules}
        alternatives = {}
        for rule in rules:
            alternative = records.copy()
            if rule in ['scalar_AS', 'new_r_old_space']:
                alternative[:, 4:6] = records[:, 38:40]
            if rule in ['scalar_AS', 'old_r_new_space']:
                alternative[:, 9] = records[:, 37]
            alternatives[rule] = alternative
        for t in range(len(data['time'])):
            for n in range(2):
                mask = (records[:, 0] == t+1) & (records[:, 1] == n+1)
                if not delivery[n, 1-n, t]:
                    assert not mask.any()
                    continue
                for rule in rules:
                    value = score(data['truth'][t], counterfactual(alternatives[rule][mask], 9, poses[:, :, t]))
                    if rule == 'candidate':
                        assert np.isclose(value['ospa'], run['ospa'][n][t], atol=1e-8, rtol=1e-9)
                    for key in effects[rule]:
                        effects[rule][key].append(value[key])
        diagnostic['same_input'] = {rule: {key: float(np.mean(v)) for key, v in values.items()}
                                    for rule, values in effects.items()}
    return diagnostic

