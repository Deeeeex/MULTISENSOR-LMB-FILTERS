"""Decode current packets and verify every candidate translation before use."""
from pathlib import Path
import csv
import hashlib
import json
import time
import numpy as np
from scipy.io import savemat
from common import OUT,ROOT,sha,write_new,frozen
import alignment_math as method
import independent_math as independent

def main():
    cfg=frozen();freeze_sha=sha(OUT/'FREEZE.json');destination=OUT/'ESTIMATION.json';assert not destination.exists()
    feature_path=OUT/'FEATURES.json';features=json.loads(feature_path.read_text())
    receipt=json.loads((OUT/'FEATURE_EXECUTION.json').read_text());assert receipt['completed'] and receipt['returncode']==0
    assert features['passed'] and features['freeze_sha256']==receipt['freeze_sha256']==freeze_sha
    assert features['raw_clouds']==5224 and features['paired_frames']==2612
    for name,digest in features['artifacts'].items():assert sha(ROOT/name)==digest,name
    feature_rows={(r['sequence'],r['frame'],r['source']):r for r in features['rows']};assert len(feature_rows)==5224
    rows=[];artifacts={};counts=[]
    for unit in cfg['units']:
        name=unit['sequence'];T=unit['frames'];grids=np.load(OUT/'features'/f'{name}_grids.npz',allow_pickle=False)['packed']
        assert grids.shape==(T,2,4000)
        translations=np.zeros((2,T));grid_seconds=np.zeros((2,T));solve_seconds=np.zeros((2,T));all_counts=np.zeros((T,41,41),np.uint16)
        for t in range(T):
            decoded=[]
            for source in (1,2):
                grid=np.unpackbits(grids[t,source-1],bitorder='little').reshape(200,160).astype(bool)
                encoded=method.packet(grid,source,t+1);meta=feature_rows[name,t+1,source]
                assert hashlib.sha256(encoded).hexdigest()==meta['packet_sha256'] and len(encoded)==meta['packet_bytes']==4032
                decoded.append(method.decode(encoded,source,t+1));grid_seconds[source-1,t]=meta['grid_seconds']
            start=time.perf_counter();choice,values=method.estimate(*decoded);solve_seconds[0,t]=time.perf_counter()-start
            # Both receivers have these current packets in the reliable phase.
            start=time.perf_counter();other,values2=method.estimate(*decoded);solve_seconds[1,t]=time.perf_counter()-start
            assert choice==other and np.array_equal(values,values2)
            independent_values=independent.all_overlaps(*decoded)
            assert np.array_equal(values,independent_values),(name,t,'complete integer correlation')
            assert choice['translation']==independent.decision(independent_values)
            translations[:,t]=choice['translation'];all_counts[t]=values.astype(np.uint16)
            rows.append(dict(sequence=name,dataset=unit['dataset'],frame=t+1,dx=choice['translation'][0],dy=choice['translation'][1],
                             maximizer_x_cells=choice['maximizer_cells'][0],maximizer_y_cells=choice['maximizer_cells'][1],
                             boundary=choice['boundary'],maximizing_shifts=choice['maximizing_shifts'],
                             native_overlap=choice['native_overlap'],maximum_overlap=choice['maximum_overlap']))
        mat=OUT/'features'/f'{name}_alignment.mat';assert not mat.exists()
        savemat(mat,dict(translations=translations,gridSeconds=grid_seconds,solveSeconds=solve_seconds),do_compression=True)
        with mat.open('r+b') as stream:stream.write(b'MATLAB 5.0 MAT-file, frozen current-observation alignment input'.ljust(116,b' '))
        array=OUT/'features'/f'{name}_correlations.npz';assert not array.exists();np.savez_compressed(array,counts=all_counts)
        for p in [mat,array]:artifacts[str(p.relative_to(ROOT))]=sha(p)
        count=dict(sequence=name,dataset=unit['dataset'],frames=T,
                   nonzero_frames=int(np.any(translations!=0,axis=0).sum()),boundary_frames=sum(r['boundary'] for r in rows if r['sequence']==name),
                   maximum_translation_m=float(np.linalg.norm(translations,axis=0).max()),
                   mean_translation_m=translations.mean(axis=1).tolist(),grid_seconds=float(grid_seconds.sum()),
                   both_receivers_solve_seconds=float(solve_seconds.sum()))
        counts.append(count);print('CURRENT ALIGNMENT VERIFIED',json.dumps(count),flush=True)
    csvpath=OUT/'TRANSLATIONS.csv';assert not csvpath.exists()
    with csvpath.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]),lineterminator='\n');writer.writeheader();writer.writerows(rows)
    assert len(rows)==2612
    write_new(destination,dict(passed=True,freeze_sha256=freeze_sha,feature_sha256=sha(feature_path),
        feature_execution_sha256=sha(OUT/'FEATURE_EXECUTION.json'),paired_frames=2612,checked_overlap_counts=2612*1681,
        packets_decoded=5224,artifacts=artifacts,groups=counts,table_sha256=sha(csvpath),source_sha256=sha(Path(__file__)),
        truth_used=False,future_data_used=False,alignment_link='additional reliable phase only'))
    print('ALL RELATIVE TRANSLATIONS INDEPENDENTLY VERIFIED',2612*1681,'integer overlap counts',flush=True)

if __name__=='__main__':main()
