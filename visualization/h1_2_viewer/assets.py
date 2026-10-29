"""Pinned official model retrieval, integrity checks, and non-overwriting editable forks."""
import argparse
import hashlib
from importlib.resources import files
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

import yaml


def model_lock():
    return json.loads(files('h1_2_viewer').joinpath('config/model.lock.json').read_text())


def default_cache():
    base = Path(os.environ.get('XDG_CACHE_HOME', str(Path.home() / '.cache')))
    return base / 'h1_2_viewer/models' / model_lock()['commit']


def _relative(upstream_path):
    directory = model_lock()['directory']
    return Path(upstream_path.removeprefix(directory + '/')).as_posix() if upstream_path != 'LICENSE' else 'LICENSE'


def verify(root, upstream=False):
    root = Path(root)
    lock = model_lock()
    for filename, expected in lock['sha256'].items():
        target = root / filename if upstream else root / ('LICENSE' if filename == 'LICENSE' else 'h1_2_description/' + _relative(filename))
        if not target.is_file() or hashlib.sha256(target.read_bytes()).hexdigest() != expected:
            raise ValueError(f'Model integrity mismatch: {target}. Keep edits in a custom fork.')
    return root


def fetch(destination=None, source=None):
    destination = Path(destination or default_cache()).expanduser().resolve()
    if destination.exists():
        return verify(destination)
    lock = model_lock()
    destination.parent.mkdir(parents=True, exist_ok=True)
    # Temporary sibling permits an atomic publish only after all hashes match.
    with tempfile.TemporaryDirectory(prefix='.h1_2_model_', dir=destination.parent) as temp:
        temp = Path(temp)
        if source:
            checkout = Path(source).resolve()
        else:
            checkout = temp / 'upstream'
            subprocess.run(['git', 'init', str(checkout)], check=True, stdout=subprocess.DEVNULL)
            subprocess.run(['git', '-C', str(checkout), 'remote', 'add', 'origin', lock['repository']], check=True)
            subprocess.run(['git', '-C', str(checkout), 'sparse-checkout', 'init', '--cone'], check=True)
            subprocess.run(['git', '-C', str(checkout), 'sparse-checkout', 'set', lock['directory']], check=True)
            subprocess.run(['git', '-C', str(checkout), 'fetch', '--depth=1', '--filter=blob:none', 'origin', lock['commit']], check=True)
            subprocess.run(['git', '-C', str(checkout), 'checkout', '--detach', 'FETCH_HEAD'], check=True)
        verify(checkout, upstream=True)
        prepared = temp / 'prepared'
        shutil.copytree(checkout / lock['directory'], prepared / 'h1_2_description')
        shutil.copy2(checkout / 'LICENSE', prepared / 'LICENSE')
        (prepared / 'SOURCE.json').write_text(json.dumps({k: v for k, v in lock.items() if k != 'sha256'}, indent=2) + '\n')
        verify(prepared)
        prepared.rename(destination)
    return destination


def fork(destination, source=None):
    source = verify(source or default_cache())
    destination = Path(destination).expanduser().resolve()
    if destination.exists():
        raise FileExistsError(f'Refusing to overwrite custom model: {destination}')
    shutil.copytree(source, destination)
    # A minimal profile inherits default mapping/layout, and moves with the model.
    (destination / 'viewer.yaml').write_text(yaml.safe_dump({
        'schema_version': 1, 'model': {'urdf': 'h1_2_description/h1_2.urdf'}}, sort_keys=False))
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    retrieve = commands.add_parser('fetch', help='Fetch and verify the pinned official model')
    retrieve.add_argument('--dest', type=Path)
    retrieve.add_argument('--source', type=Path, help='Seed from an existing unitree_ros checkout, without network')
    validate = commands.add_parser('verify', help='Check official model hashes (custom edits intentionally fail)')
    validate.add_argument('path', nargs='?', type=Path, default=default_cache())
    edit = commands.add_parser('fork', help='Copy a verified model to an editable, self-contained folder')
    edit.add_argument('--dest', type=Path, required=True)
    edit.add_argument('--source', type=Path)
    args = parser.parse_args()
    try:
        if args.command == 'fetch':
            path = fetch(args.dest, args.source)
        elif args.command == 'verify':
            path = verify(args.path)
        else:
            path = fork(args.dest, args.source)
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        parser.exit(1, f'{exc}\n')
    print(path)


if __name__ == '__main__':
    main()
