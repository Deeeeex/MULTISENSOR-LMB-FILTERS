"""Normalize verified DOI metadata for the IEEE manuscript, preserving raw records."""
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
    if key=='dames2020search':
        # Publisher's issue citation is 2020; Crossref returns online-first 2019.
        b=b.replace('year={2019}','year={2020}')
    b=re.sub(r'(@\w+\{)[^,]+,',r'\g<1>'+key+',',b,count=1)
    b=re.sub(r'</?roman>', '', html.unescape(b))
    b=b.replace('–','--').replace('\u00a0',' ')
    b=re.sub(r'\burl=\{[^}]*\},?\s*','',b,flags=re.I)
    b=re.sub(r'\bISSN=\{[^}]*\},?\s*','',b,flags=re.I)
    b=re.sub(r'\bpublisher=\{[^}]*\},?\s*','',b,flags=re.I)
    b=re.sub(r'\bmonth=\w+,?\s*','',b,flags=re.I)
    b=re.sub(r'\bDOI=\{[^}]*\},?\s*','',b,flags=re.I)
    match=re.search(r'title=\{([^}]+)\}',b)
    title=match.group(1)
    for acronym in ['LMB','KLA','RFS','PHD','CPHD']:
        title=re.sub(r'\b'+acronym+r'\b','{'+acronym+'}',title)
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
(out/'references.bib').write_text('% Primary DOI records checked 2026-09-08; raw records in literature/.\n'+'\n\n'.join(bibs)+'\n')
print('Wrote',len(bibs),'verified/reference-mapped records.')
