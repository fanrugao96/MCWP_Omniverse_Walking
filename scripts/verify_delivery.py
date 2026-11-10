"""Verify current source files or a delivery ZIP using only Python standard libraries."""
import argparse
import hashlib
from pathlib import Path
import zipfile

from delivery_files import ARCHIVE_ROOT, MANIFEST, ROOT, collect_files, parse_manifest, sha256


def verify_folder(root=ROOT):
    manifest = parse_manifest((root / MANIFEST).read_bytes())
    expected = {item['path'] for item in manifest['files']}
    if expected != set(collect_files(root)):
        raise ValueError('The manifest does not match the current source inventory; refresh it with package_delivery.py.')
    for entry in manifest['files']:
        path = root / entry['path']
        if path.stat().st_size != entry['bytes'] or sha256(path) != entry['sha256']:
            raise ValueError('File integrity mismatch: ' + entry['path'])
    return len(expected)


def verify_archive(path):
    prefix = ARCHIVE_ROOT + '/'
    with zipfile.ZipFile(path) as archive:
        names = archive.namelist()
        if len(names) != len(set(names)):
            raise ValueError('ZIP contains duplicate members')
        manifest = parse_manifest(archive.read(prefix + MANIFEST))
        expected = {prefix + item['path'] for item in manifest['files']} | {prefix + MANIFEST}
        if set(names) != expected:
            raise ValueError('ZIP inventory does not match its manifest')
        for entry in manifest['files']:
            name = prefix + entry['path']
            with archive.open(name) as stream:
                digest = hashlib.file_digest(stream, 'sha256').hexdigest()
            if archive.getinfo(name).file_size != entry['bytes'] or digest != entry['sha256']:
                raise ValueError('ZIP integrity mismatch: ' + entry['path'])
    return len(manifest['files'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, help='Verify this ZIP instead of the source folder.')
    args = parser.parse_args()
    count = verify_archive(args.archive) if args.archive else verify_folder()
    print(f'Verified {count} file sizes and SHA-256 hashes.')


if __name__ == '__main__':
    main()
