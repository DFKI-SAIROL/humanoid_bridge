"""Export source plus editable model, excluding machine caches and build artifacts."""
import argparse
from pathlib import Path
import tarfile
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    output = (args.output or Path(tempfile.gettempdir()) / 'h1_2_viewer_bundle.tar.gz').resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    excluded = {'.git', '__pycache__', '.venv', 'dist', 'build', 'recordings'}
    with tarfile.open(output, 'w:gz') as archive:
        for path in sorted(root.rglob('*')):
            relative = path.relative_to(root)
            if any(part in excluded or part.endswith('.egg-info') for part in relative.parts):
                continue
            if relative.parts[:2] == ('models', 'upstream') or path == output or not path.is_file():
                continue
            archive.add(path, arcname=str(Path('h1_2_viewer_bundle') / relative), recursive=False)
    print(output)


if __name__ == '__main__':
    main()
