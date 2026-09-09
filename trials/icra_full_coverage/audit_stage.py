"""Use the frozen reviewer auditor in the separate coverage output directory."""
from pathlib import Path
import importlib.util
import sys

OUT = Path(__file__).resolve().parent
REVIEW = OUT.parent / 'icra_reviewer_revision'
sys.path.insert(0, str(REVIEW))
spec = importlib.util.spec_from_file_location('frozen_review_stage_auditor', REVIEW / 'audit_stage.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.OUT = OUT

if __name__ == '__main__':
    module.main()
