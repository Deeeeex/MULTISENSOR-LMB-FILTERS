"""Recompute summary arithmetic without loading native trackers or study helpers."""
from collections import Counter, defaultdict
from pathlib import Path
import argparse
import hashlib
import json
import statistics

import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
METRICS=['ospa','gospa','loc2','miss2','false2','countError']
BASE='marked_gaussian_evidence'
QN=BASE+'_assoc_quality_nis'


def close(a,b):
    if a is None or b is None:assert a is b
    else:assert np.isclose(a,b,rtol=1e-11,atol=1e-11),(a,b)


def matches(row,scope):
    if scope=='v2v_all':return row['dataset']=='v2v'
    if scope=='v2v_development':return row['dataset']=='v2v' and row['cohort']=='development'
    if scope=='v2v_remaining':return row['dataset']=='v2v' and row['cohort']!='development'
    assert scope in ['v2x_val','v2x_test']
    return row['dataset']==scope


def check_identity(rows,reported):
    for cutoff in ['2','12']:
        counts=Counter()
        for row in rows:counts.update(row['cutoffs'][cutoff])
        value=reported[cutoff]
        assert counts==Counter(value['counts'])
        wrong=counts['known_wrong']+counts['assigned_wrong']
        total=wrong+counts['known_correct']+counts['assigned_correct']
        assert wrong==value['wrong_pairs'] and total==value['scored_pairs']
        close(value['wrong_pair_rate'],wrong/total if total else None)
        opportunities=counts['common_identity_opportunities']
        close(value['missed_common_pair_rate'],counts['common_identity_pairs_missed']/opportunities if opportunities else None)


def development(name,expected_rows):
    report=json.loads((OUT/name).read_text());assert report['passed'] and len(report['rows'])==expected_rows
    for item in report['aggregate']:
        part=[r for r in report['rows'] if r['arm']==item['arm'] and r['condition']==item['condition']]
        assert len(part)==9
        for metric in METRICS:close(item[metric],statistics.fmean(r[metric] for r in part))
        check_identity([r for r in report['identity_rows'] if r['arm']==item['arm'] and r['condition']==item['condition']],item['identity'])
    if name=='SCREENED_DEVELOPMENT_SELECTION.json':
        lookup={(r['arm'],r['condition']):r for r in report['aggregate']}
        base_wrong=report['baseline_pooled_identity']['2']['wrong_pair_rate']
        eligible=[]
        order=[BASE+'_assoc_quality',BASE+'_assoc_nis',QN]
        for row in report['ranking']:
            values=[lookup[row['arm'],c]['ospa'] for c in ['reliable','intermittent']]
            close(row['mean_ospa'],statistics.fmean(values))
            gain=[]
            for c in ['reliable','intermittent']:
                a,b=lookup[BASE,c]['ospa'],lookup[row['arm'],c]['ospa']
                close(row['ospa_difference'][c],b-a);close(row['ospa_relative_reduction'][c],(a-b)/a)
                gain.append((a-b)/a>=.01)
            check_identity([r for r in report['identity_rows'] if r['arm']==row['arm']],row['pooled_identity'])
            identity_ok=row['pooled_identity']['2']['wrong_pair_rate']<=base_wrong
            assert row['at_least_one_percent_both_conditions']==all(gain)
            assert row['wrong_pair_rate_nonincreasing']==identity_ok
            assert row['eligible']==(all(gain) and identity_ok)
            if row['eligible']:eligible.append(row)
        chosen=min(eligible,key=lambda r:(r['mean_ospa'],order.index(r['arm'])))
        assert report['advance'] and chosen==report['selected'] and chosen['arm']==QN
    return len(report['aggregate'])


def assessment(name,expected_rows):
    report=json.loads((OUT/name).read_text());assert report['passed'] and len(report['rows'])==expected_rows
    for item in report['aggregate']:
        part=[r for r in report['rows'] if matches(r,item['scope']) and r['arm']==item['arm'] and r['condition']==item['condition']]
        assert len(part)==item['sequences'] and sum(r['frames'] for r in part)==item['frames']
        by_group=defaultdict(list)
        for row in part:by_group[row['recording']].append(row['ospa'])
        assert len(by_group)==item['recording_groups']
        for metric in METRICS:close(item['sequence_macro'][metric],statistics.fmean(r[metric] for r in part))
        close(item['frame_weighted_ospa'],sum(r['frames']*r['ospa'] for r in part)/item['frames'])
        close(item['recording_macro_ospa'],statistics.fmean(statistics.fmean(v) for v in by_group.values()))
        if 'identity' in item:
            ids={(r['dataset'],r['sequence']) for r in part}
            identity=[r for r in report['identity_rows'] if (r['dataset'],r['sequence']) in ids and r['arm']==item['arm'] and r['condition']==item['condition']]
            check_identity(identity,item['identity'])
        if 'communication' in item:
            for key,value in item['communication'].items():assert value==sum(r[key] for r in part)
        if 'final_identity' in item:
            final=[r for r in report['final_identity_rows'] if r['arm']==item['arm'] and r['condition']==item['condition']]
            assert len(final)==len(part)
            for cutoff in ['2','12']:
                counts=Counter()
                for row in final:counts.update(row['cutoffs'][cutoff])
                assert counts==Counter(item['final_identity'][cutoff])
    for item in report['paired_recording']:
        part=[r for r in report['rows'] if matches(r,item['scope'])]
        scenes=sorted({r['scene'] for r in part}); groups=sorted({r['recording'] for r in part})
        lookup={(r['scene'],r['condition'],r['arm']):r for r in part}
        deltas={s:lookup[s,item['condition'],item['candidate']]['ospa']-lookup[s,item['condition'],item['reference']]['ospa'] for s in scenes}
        for key,value in deltas.items():close(value,item['sequence_differences'][key])
        close(item['sequence_macro_difference'],statistics.fmean(deltas.values()))
        grouped=[statistics.fmean(deltas[s] for s in scenes if lookup[s,item['condition'],item['candidate']]['recording']==g) for g in groups]
        close(item['recording_macro_difference'],statistics.fmean(grouped));assert len(groups)==item['recording_groups']
        indices=np.random.default_rng(8301).integers(0,len(groups),size=(10000,len(groups)))
        interval=np.quantile(np.asarray(grouped)[indices].mean(axis=1),[.025,.975])
        close(item['low'],interval[0]);close(item['high'],interval[1])
        assert item['improved']==sum(v < -1e-10 for v in deltas.values())
        assert item['worsened']==sum(v > 1e-10 for v in deltas.values())
        assert item['unchanged']==sum(abs(v)<=1e-10 for v in deltas.values())
    if name=='ADDITIONAL_TEST_ANALYSIS.json':
        assert len(report['identity_rows'])==112 and len(report['final_identity_rows'])==140 and len(report['geometry_rows'])==280
        for row in report['rows']:
            pieces=[r for r in report['geometry_rows'] if all(r[k]==row[k] for k in ['sequence','condition','arm'])]
            assert len(pieces)==2 and {r['stratum'] for r in pieces}=={'overlapping_disks','nonoverlapping_disks'}
            assert sum(r['robot_frames'] for r in pieces)==2*row['frames']
            for metric in METRICS:close(sum(r['metric_sums'][metric] for r in pieces)/(2*row['frames']),row[metric])
            for key in ['wire_bytes','raw_payload_bytes','split_branches']:assert sum(r[key] for r in pieces)==row[key]
        for item in report['geometry_aggregate']:
            part=[r for r in report['geometry_rows'] if all(r[k]==item[k] for k in ['stratum','condition','arm'])]
            assert len(part)==14
            frames=sum(r['robot_frames'] for r in part);assert frames==item['robot_frames']==2*item['paired_frames']
            available=[r for r in part if r['robot_frames']];assert len(available)==item['segments_with_frames']
            for metric in METRICS:
                close(item['frame_weighted'][metric],sum(r['metric_sums'][metric] for r in part)/frames if frames else None)
                close(item['sequence_macro_nonempty'][metric],statistics.fmean(r['means'][metric] for r in available) if available else None)
            for key in ['wire_bytes','raw_payload_bytes','split_branches']:assert item[key]==sum(r[key] for r in part)
        lookup={(r['arm'],r['condition']):r['sequence_macro']['ospa'] for r in report['aggregate']}
        for reference,value in report['comparisons'].items():
            differences=[lookup[QN,c]-lookup[reference,c] for c in ['reliable','intermittent']]
            for c,v in zip(['reliable','intermittent'],differences):close(v,value['ospa_difference'][c])
            assert value['improves_both']==all(v<0 for v in differences)
        assert report['transfer_improves_both_conditions']==report['comparisons'][BASE]['improves_both']
    return len(report['aggregate']),len(report['paired_recording'])


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--exposed-only',action='store_true');args=parser.parse_args()
    aggregates=development('RESTORED_DEVELOPMENT_SELECTION.json',90)+development('SCREENED_DEVELOPMENT_SELECTION.json',144)
    pairs=0
    reports=[('RESTORED_ASSESSMENT.json',874),('SCREENED_ASSESSMENT.json',1066)]
    if not args.exposed_only:reports.append(('ADDITIONAL_TEST_ANALYSIS.json',140))
    for name,count in reports:
        a,p=assessment(name,count);aggregates+=a;pairs+=p
    if not args.exposed_only:
        render=json.loads((OUT/'FINAL_RESULTS_RENDER.json').read_text())
        assert hashlib.sha256((ROOT/render['output_path']).read_bytes()).hexdigest()==render['output_sha256']
        assert json.loads((OUT/'REPLAY_ACCEPTANCE.json').read_text())['passed']
    print('PORTABLE ASSOCIATION SUMMARY VERIFIED',aggregates,'aggregates;',pairs,'paired comparisons',flush=True)


if __name__=='__main__':main()
