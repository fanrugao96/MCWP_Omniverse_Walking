"""Shared source-package inventory; runtime outputs are deliberately excluded."""
import hashlib
import json
from pathlib import Path, PurePosixPath
import re

ROOT = Path(__file__).resolve().parents[1]
ARCHIVE_ROOT = 'MCWP_Omniverse_Walking'
MANIFEST = 'delivery_manifest.json'
RUNTIME_MODULES = (
    'mcwp_simulation', 'mcwp_control_menu', 'mcwp_lift', 'mcwp_worker',
    'worker_motion', 'worker_navigation', 'worker_validation', 'worker_full_validation',
    'mcwp_extension_features', 'mcwp_interfaces', 'mcwp_depth_tf', 'mcwp_ros_depth_contract',
)
REQUIRED = (
    '.gitignore', '.gitattributes', 'README.md', 'requirements.txt',
    'Launch_MCWP.cmd', 'Start MCWP Walking.cmd', 'apps/mcwp.walking.kit',
    'MCWP_Simulation.usda', 'capture_exposure_profile.json', 'runtime_control.json',
    'ros_config.json', 'depth_tf_config.json', 'recordings/.gitkeep',
    'evidence/worker_envelope_validation.json', 'assets/lift/lift_geometry_validation.json',
    'assets/lift/Platform_Lift.usda', 'assets/worker/Construction_Worker_Rigged.usdc',
    'assets/worker/walk_navigation.json', 'assets/UE_RightHand_original.csv',
    'assets/ue_export/MCWP_Omniverse_Demo.usda',
) + tuple('scripts/' + name + '.py' for name in RUNTIME_MODULES)


def sha256(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def collect_files(root=ROOT):
    paths = set(REQUIRED)
    for directory in ('assets', 'scripts', 'apps'):
        for path in (root / directory).rglob('*'):
            if path.is_symlink() or path.is_junction():
                raise ValueError('Linked source path is not supported: ' + str(path))
            if path.is_file() and '__pycache__' not in path.parts and path.suffix not in {'.pyc', '.pyo'}:
                paths.add(path.relative_to(root).as_posix())
    missing = [name for name in sorted(paths) if not (root / name).is_file()]
    if missing:
        raise FileNotFoundError('Missing package files: ' + ', '.join(missing))
    for name in paths:
        if not (root / name).resolve().is_relative_to(root.resolve()):
            raise ValueError('Source path escapes the project: ' + name)
    return sorted(paths)


def make_manifest(root=ROOT):
    return {'schema_version': 1, 'files': [
        {'path': name, 'bytes': (root / name).stat().st_size, 'sha256': sha256(root / name)}
        for name in collect_files(root)
    ]}


def write_manifests(root=ROOT):
    worker = root / 'assets/worker'
    worker_manifest = {
        path.relative_to(worker).as_posix(): {'bytes': path.stat().st_size, 'sha256': sha256(path)}
        for path in sorted(worker.rglob('*'))
        if path.is_file() and path.name != 'SHA256_Manifest.json' and '__pycache__' not in path.parts
    }
    (worker / 'SHA256_Manifest.json').write_text(json.dumps(worker_manifest, indent=2) + '\n', encoding='utf-8')
    manifest = make_manifest(root)
    (root / MANIFEST).write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    return manifest


def parse_manifest(data):
    manifest = json.loads(data)
    if not isinstance(manifest, dict) or manifest.get('schema_version') != 1 or not isinstance(manifest.get('files'), list):
        raise ValueError('Unsupported delivery manifest format')
    seen = set()
    for entry in manifest['files']:
        name = entry['path']
        path = PurePosixPath(name)
        if not name or path.is_absolute() or '..' in path.parts or '\\' in name or ':' in name or path.as_posix() != name:
            raise ValueError('Invalid manifest path: ' + name)
        if name == MANIFEST or name in seen:
            raise ValueError('Duplicate or self-referencing manifest path: ' + name)
        if type(entry['bytes']) is not int or entry['bytes'] < 0 or not re.fullmatch('[0-9a-f]{64}', entry['sha256']):
            raise ValueError('Invalid manifest file metadata: ' + name)
        seen.add(name)
    if not set(REQUIRED).issubset(seen):
        raise ValueError('Manifest omits required project files')
    return manifest
