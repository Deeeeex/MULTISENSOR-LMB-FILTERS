"""Normalize verified primary bibliographic records for the IEEE manuscript."""
from pathlib import Path
import html
import json
import re

out=Path(__file__).resolve().parent
records=json.loads((out/'literature/verification.json').read_text())
bibs=[]
for key,record in records.items():
    assert record['verified'], key
    b=record['bibtex']
    b=re.sub(r'(\w+)\s*=\s*',r'\1=',b)
    if key=='wu2022sharedpriors':
        b=b.replace('publisher={arXiv},','howpublished={arXiv:2212.07311},')
    if key=='fantacci2015consensus':
        b=b.replace('publisher={arXiv},','howpublished={arXiv:1501.01579},')
        b=b.replace('B. -N.', 'B.-N.').replace('B. -T.', 'B.-T.')
    if key=='julier1997ci':
        # Crossref places the proceedings volume inside its page field.
        b=b.replace('pages={2369–2373 vol.4}', 'volume={4}, pages={2369--2373}')
        b=b.replace('S.J.', 'S. J.').replace('J.K.', 'J. K.')
        b=b.replace('Proceedings of the 1997 American Control Conference (Cat. No.97CH36041)',
                    'Proc. American Control Conference')
    if key=='niculescu2005calibration':
        b=re.sub(r'\b(?:series|collection)=\{[^}]*\},?\s*', '', b)
        b=re.sub(r'booktitle=\{[^}]*\}',
                 'booktitle={Proc. 22nd International Conference on Machine Learning (ICML)}', b)
    if key=='williams2014bp':
        # Middle initials are explicit in the linked author manuscript.
        b=b.replace('Williams, Jason and Lau, Roslyn',
                    'Williams, Jason L. and Lau, Roslyn A.')
    if key=='dames2020search':
        # Publisher's issue citation is 2020; Crossref returns online-first 2019.
        b=b.replace('year={2019}','year={2020}')
    if key=='moore2016robotlocalization':
        # Springer identifies the proceedings citation as 2016 (online first 2015).
        b=b.replace('year={2015}','year={2016}').replace('@inbook{','@inproceedings{')
    if key=='xiang2024v2x':
        b=b.replace('@inbook{','@inproceedings{')
    b=re.sub(r'(@\w+\{)[^,]+,',r'\g<1>'+key+',',b,count=1)
    b=re.sub(r'</?roman>', '', html.unescape(b))
    b=b.replace('–','--').replace('\u00a0',' ')
    if record.get('kind')!='documentation':
        b=re.sub(r'\burl=\{[^}]*\},?\s*','',b,flags=re.I)
    b=re.sub(r'\bISSN=\{[^}]*\},?\s*','',b,flags=re.I)
    b=re.sub(r'\bISBN=\{[^}]*\},?\s*','',b,flags=re.I)
    b=re.sub(r'\bpublisher=\{[^}]*\},?\s*','',b,flags=re.I)
    b=re.sub(r'\bmonth=(?:\{[^}]*\}|\w+),?\s*','',b,flags=re.I)
    b=re.sub(r'\beditor=\{[^}]*\},?\s*','',b,flags=re.I)
    b=re.sub(r'\bDOI=\{[^}]*\},?\s*','',b,flags=re.I)
    match=re.search(r'title=\{([^}]+)\}',b)
    title=match.group(1)
    for acronym in ['LMB','KLA','RFS','PHD','CPHD','SLAM','LIO-SAM','Autoware','V2V4Real','V2X-Real','Where2comm','CoopTrack','3D']:
        title=re.sub(r'\b'+acronym+r'\b','{'+acronym+'}',title)
    for proper in ['Bernoulli', 'Kalman', 'Bayesian', 'Kullback', 'Leibler', 'Gaussian']:
        title=re.sub(r'\b'+proper+r'\b','{'+proper+'}',title,flags=re.I)
    b=b[:match.start(1)]+title+b[match.end(1):]
    # Preserve primary publication titles and expose verified persistent identifiers.
    # IEEEtran's supplied BST prints the note field but has no DOI formatter.
    doi = record.get('doi', '')
    if doi and not doi.lower().startswith('10.48550/'):
        assert re.fullmatch(r'10\.\d{4,9}/\S+', doi), (key, doi)
        assert not re.search(r'\bnote=\{', b), key
        end = b.rfind('}')
        b = b[:end].rstrip().rstrip(',') + ',\n  note={doi: \\url{' + doi + '}}\n' + b[end:]
    bibs.append(b.strip())
bibs.append(r'''@misc{lang2026adaptive,
  author={Lang, Hao and Chen, Jinhao and Wo, Tianyu},
  title={Communication-Aware Adaptive Weights for Consensus-Oriented Distributed {KLA}-Based {LMB} Fusion},
  howpublished={SSRN preprint 7129254}, year={2026}
}''')
(out/'references.bib').write_text('% Primary DOI, author and proceedings records; additional citations checked 2026-09-11; sources in literature/.\n'+'\n\n'.join(bibs)+'\n')
print('Wrote',len(bibs),'verified/reference-mapped records.')
