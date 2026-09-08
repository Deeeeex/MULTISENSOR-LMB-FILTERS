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
    if key=='dames2020search':
        # Publisher's issue citation is 2020; Crossref returns online-first 2019.
        b=b.replace('year={2019}','year={2020}')
    if key=='moore2016robotlocalization':
        # Springer identifies the proceedings citation as 2016 (online first 2015).
        b=b.replace('year={2015}','year={2016}').replace('@inbook{','@inproceedings{')
    b=re.sub(r'(@\w+\{)[^,]+,',r'\g<1>'+key+',',b,count=1)
    b=re.sub(r'</?roman>', '', html.unescape(b))
    b=b.replace('–','--').replace('\u00a0',' ')
    if record.get('kind')!='documentation':
        b=re.sub(r'\burl=\{[^}]*\},?\s*','',b,flags=re.I)
    b=re.sub(r'\bISSN=\{[^}]*\},?\s*','',b,flags=re.I)
    b=re.sub(r'\bISBN=\{[^}]*\},?\s*','',b,flags=re.I)
    b=re.sub(r'\bpublisher=\{[^}]*\},?\s*','',b,flags=re.I)
    b=re.sub(r'\bmonth=\w+,?\s*','',b,flags=re.I)
    b=re.sub(r'\bDOI=\{[^}]*\},?\s*','',b,flags=re.I)
    match=re.search(r'title=\{([^}]+)\}',b)
    title=match.group(1)
    for acronym in ['LMB','KLA','RFS','PHD','CPHD','SLAM','LIO-SAM','Autoware','V2V4Real','3D']:
        title=re.sub(r'\b'+acronym+r'\b','{'+acronym+'}',title)
    for proper in ['Bernoulli', 'Kalman', 'Bayesian', 'Kullback', 'Leibler', 'Gaussian']:
        title=re.sub(r'\b'+proper+r'\b','{'+proper+'}',title,flags=re.I)
    b=b[:match.start(1)]+title+b[match.end(1):]
    b=b.replace('IEEE Transactions on Signal Processing','IEEE Trans. Signal Process.')
    b=b.replace('IEEE Transactions on Aerospace and Electronic Systems','IEEE Trans. Aerosp. Electron. Syst.')
    b=b.replace('IEEE Transactions on Control of Network Systems','IEEE Trans. Control Netw. Syst.')
    b=b.replace('IEEE Transactions on Information Theory','IEEE Trans. Inf. Theory')
    bibs.append(b.strip())
bibs.append(r'''@misc{lang2026adaptive,
  author={Lang, Hao and Chen, Jinhao and Wo, Tianyu},
  title={Communication-Aware Adaptive Weights for Consensus-Oriented Distributed {KLA}-Based {LMB} Fusion},
  howpublished={SSRN preprint 7129254}, year={2026}
}''')
(out/'references.bib').write_text('% Primary DOI, author and official-documentation records checked 2026-09-08; sources in literature/.\n'+'\n\n'.join(bibs)+'\n')
print('Wrote',len(bibs),'verified/reference-mapped records.')
