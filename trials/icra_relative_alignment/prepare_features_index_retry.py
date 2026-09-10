"""Keep frozen producer grids; use the separately verified exact index checker."""
import json
import prepare_features as original
from prepare_features_retry import RetryArchive
import independent_voxel_index as corrected
from common import OUT,ROOT,sha

class CachedDiagnosticArchive(RetryArchive):
    def read_range(self,start,length):
        diagnostic=json.loads((OUT/'GRID_CHECK_DIAGNOSTIC.json').read_text());job=diagnostic['range_job']
        if (self.name,start,length)==(job['archive'],job['start'],job['length']):
            path=ROOT/diagnostic['range_path'];assert sha(path)==diagnostic['range_sha256']
            value=path.read_bytes();assert len(value)==length
            return value
        return super().read_range(start,length)

if __name__=='__main__':
    report=json.loads((OUT/'GRID_VERIFIER_REPAIR.json').read_text());assert report['passed']
    for name,digest in report['artifacts'].items():assert sha(ROOT/name)==digest,name
    original.independent.occupancy=corrected.occupancy
    original.Archive=CachedDiagnosticArchive
    original.main()
