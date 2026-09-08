"""Compile, audit, and package the paper; --regenerate also rebuilds plots/tables."""
from pathlib import Path
import argparse
import os
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent


def python_with(modules, override):
    candidates = [os.environ.get(override), sys.executable, shutil.which('python3'),
                  str(Path.home() / 'miniconda3/bin/python3'),
                  str(Path.home() / '.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3')]
    for candidate in dict.fromkeys(p for p in candidates if p):
        if Path(candidate).exists() and subprocess.run(
                [candidate, '-c', '; '.join('import ' + m for m in modules)],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
            return candidate
    raise RuntimeError('Install ' + ', '.join(modules) + ' or set ' + override)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--regenerate', action='store_true')
    args = parser.parse_args()
    build = HERE / 'build'
    build.mkdir(exist_ok=True)
    if args.regenerate:
        plot_python = python_with(['numpy', 'matplotlib'], 'PAPER_PLOT_PYTHON')
        for name in ['build_bibliography.py', 'make_tables.py', 'make_figures.py',
                     'make_main_figure_integrated.py']:
            subprocess.run([plot_python, str(HERE / name)], cwd=HERE, check=True)
    tectonic = shutil.which('tectonic')
    if not tectonic and Path('/opt/homebrew/bin/tectonic').exists():
        tectonic = '/opt/homebrew/bin/tectonic'
    if not tectonic:
        raise RuntimeError('Install Tectonic and put it on PATH; see README_CN.md.')
    # Fixed reruns avoid an IEEEtran .bbl stability warning under Tectonic.
    with (build / 'compile.log').open('w') as log:
        subprocess.run([tectonic, '--reruns', '2', '--keep-logs', '--keep-intermediates',
                        '--outdir', str(build), 'main.tex'], cwd=HERE,
                       stdout=log, stderr=subprocess.STDOUT, check=True)
    destination = HERE / 'output/pdf/icra2027_draft.pdf'
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(build / 'main.pdf', destination)
    pdf_python = python_with(['pypdf', 'fitz'], 'PAPER_PDF_PYTHON')
    subprocess.run([pdf_python, str(HERE / 'qa.py')], cwd=HERE, check=True)
    subprocess.run([sys.executable, str(HERE / 'package_review.py')], cwd=HERE, check=True)
    print('Built and audited:', destination)


if __name__ == '__main__':
    main()
