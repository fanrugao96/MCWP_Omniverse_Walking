"""Build a portable source delivery from current files, without old recordings."""
import argparse
from pathlib import Path
import zipfile

from delivery_files import ARCHIVE_ROOT, MANIFEST, ROOT, collect_files, write_manifests


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest-only', action='store_true', help='Refresh the file manifests without creating a ZIP.')
    parser.add_argument('--output', type=Path, help='ZIP destination; default: dist/MCWP_Omniverse_Walking.zip.')
    args = parser.parse_args()
    output = (args.output or ROOT / 'dist' / (ARCHIVE_ROOT + '.zip')).resolve()
    if not args.manifest_only:
        protected = {(ROOT / name).resolve() for name in collect_files()} | {(ROOT / MANIFEST).resolve()}
        if output in protected or output.is_dir():
            parser.error('The ZIP destination must not overwrite a project file or directory.')
    manifest = write_manifests()
    if args.manifest_only:
        print(f'Updated manifests for {len(manifest["files"])} project files.')
        return
    output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for name in [item['path'] for item in manifest['files']] + [MANIFEST]:
            archive.write(ROOT / name, ARCHIVE_ROOT + '/' + name)
    with zipfile.ZipFile(output) as archive:
        invalid = archive.testzip()
        if invalid:
            raise ValueError('ZIP integrity failure: ' + invalid)
    print(f'Created {output} ({len(manifest["files"]) + 1} files).')


if __name__ == '__main__':
    main()
