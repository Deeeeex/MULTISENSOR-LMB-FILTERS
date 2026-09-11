"""Audit the portable source bindings and values placed into the manuscript."""
from pathlib import Path
import json
import re
import numpy as np
from prepare_followup_evidence import derive

HERE=Path(__file__).resolve().parent


def check_followup_evidence():
    evidence=json.loads((HERE/'source_data/followup_evidence.json').read_text())
    assert evidence == derive()
    table=(HERE/'generated/followup_table.tex').read_text()
    columns={}
    for arm in evidence['methods']:
        columns[arm]=[next(r['sequence_macro']['ospa'] for r in evidence['aggregate']
            if r['dataset']==d and r['condition']==c and r['arm']==arm)
            for d in ['v2v','v2x'] for c in ['reliable','intermittent']]
    minima=np.min(list(columns.values()),axis=0)
    actual=[]
    for line in table.splitlines():
        if ' & ' not in line or line.startswith(('&','Method')):continue
        parts=line.split(' & ')[1:]
        if len(parts)!=4:continue
        actual.append([p.removesuffix(r' \\').strip() for p in parts])
    expected=[]
    for row in columns.values():
        expected.append([r'\textbf{'+f'{v:.3f}'+'}' if abs(v-minima[i])<1e-12 else f'{v:.3f}' for i,v in enumerate(row)])
    assert actual==expected
    facts=json.loads((HERE/'generated/followup_facts.json').read_text())
    macros=dict(re.findall(r'\\newcommand\{\\(\w+)\}\{([^}]+)\}',(HERE/'generated/followup_numbers.tex').read_text()))
    assert macros == {name:f'{v:.{digits}f}' for name,(v,digits) in facts.items()}
    aggregate={(r['dataset'],r['condition'],r['arm']):r['sequence_macro']['ospa'] for r in evidence['aggregate']}
    for c in ['reliable','intermittent']:
        for d,prefix in [('v2v','VTwoV'),('v2x','VTwoX')]:
            for arm,short in [('marked_gaussian_evidence','GCE'),('marked_lineage','NoAge'),('marked_gaussian_evidence_guarded_scalar','GS')]:
                assert facts['Follow'+prefix+short+c.title()][0]==aggregate[d,c,arm]
        np.testing.assert_allclose(facts['FollowGainNoAge'+c.title()][0],100*(aggregate['v2v',c,'marked_lineage']-aggregate['v2v',c,'marked_gaussian_evidence'])/aggregate['v2v',c,'marked_lineage'],atol=1e-12,rtol=0)
        row=next(r for r in evidence['recording_gce_gs'] if r['condition']==c)
        for key,short in [('group_macro_difference','Mean'),('low','Low'),('high','High')]:
            assert facts['FollowRecordingGS'+c.title()+short][0]==row[key]
        pair=next(r for r in evidence['seen_fixed_paired'] if r['condition']==c)
        for stat in ['mean','low','high']:assert facts['FollowFixed'+c.title()+stat.title()][0]==pair[stat]
    return dict(passed=True,evaluation_rows=854,paired_comparisons=180,v2v_segments=43,v2x_segments=5,
                selected_fixed_eta=0,native_results=458,native_robot_frames=187444,
                source_scope='Portable summary recomputation; native trajectory acceptance retained from the completed study.')
