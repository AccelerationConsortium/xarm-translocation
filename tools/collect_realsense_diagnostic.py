#!/usr/bin/env python3
"""Collect a diagnostic ZIP through the service using your existing claim token."""
import argparse
import getpass
import hashlib
import json
from pathlib import Path
import re
import threading
import urllib.error
import urllib.request
import zipfile


def verify_archive(path):
    with zipfile.ZipFile(path) as archive:
        manifests = [name for name in archive.namelist() if name.endswith('/manifest.json')]
        if len(manifests) != 1:
            raise ValueError('Expected exactly one manifest')
        manifest = json.loads(archive.read(manifests[0]))
        prefix = manifest['capture_id'] + '/'
        if manifest['frame_count'] != 20 or len(manifest['frames']) != 20:
            raise ValueError('Expected 20 framesets')
        for entry in manifest['frames']:
            for info in entry['files'].values():
                data = archive.read(prefix + info['path'])
                if len(data) != info['bytes'] or hashlib.sha256(data).hexdigest() != info['sha256']:
                    raise ValueError('Integrity check failed: ' + info['path'])
        return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8000', help='xArm service base URL')
    parser.add_argument('--camera', default='rs435i')
    parser.add_argument('--out', type=Path, default=Path('diagnostics'))
    args = parser.parse_args()
    if not re.fullmatch(r'[a-z0-9][a-z0-9_-]{0,31}', args.camera):
        parser.error('Invalid camera ID')
    token = getpass.getpass('Existing xArm claim token (hidden): ').strip()
    if not token:
        parser.error('A claim token is required; claim the device first')
    base = args.url.rstrip('/')
    headers = {'X-Claim-Token': token}

    def post(route, timeout=15):
        return urllib.request.urlopen(urllib.request.Request(
            base + route, headers=headers, method='POST'), timeout=timeout)

    # Verify ownership before requesting any camera operation. Keep the same
    # claim alive throughout export; leave it held for its original owner.
    with post('/control/heartbeat'):
        pass
    stop = threading.Event()
    heartbeat_errors = []

    def heartbeat():
        while not stop.wait(3):
            try:
                with post('/control/heartbeat'):
                    pass
            except Exception as exc:
                heartbeat_errors.append(type(exc).__name__)
                return

    thread = threading.Thread(target=heartbeat, daemon=True)
    thread.start()
    args.out.mkdir(parents=True, exist_ok=True)
    partial = None
    created_partial = False
    try:
        with post(f'/control/realsense/{args.camera}/diagnostic?start_if_idle=true', timeout=90) as response:
            capture_id = response.headers.get('X-Capture-ID', '')
            if not re.fullmatch(r'diagnostic-[a-f0-9]{32}', capture_id):
                raise ValueError('Unexpected capture ID in response')
            partial = args.out / (capture_id + '.zip.partial')
            target = args.out / (capture_id + '.zip')
            if target.exists():
                raise FileExistsError(target)
            total = 0
            with partial.open('xb') as output:
                created_partial = True
                while chunk := response.read(1024 * 1024):
                    total += len(chunk)
                    if total > 512 * 1024 * 1024:
                        raise ValueError('Response exceeds 512 MiB download limit')
                    output.write(chunk)
        manifest = verify_archive(partial)
        partial.rename(target)
        print(f'Saved and hash-verified {manifest["frame_count"]} framesets: {target.resolve()}')
        print(f'Manifest inside ZIP: {capture_id}/manifest.json')
        if heartbeat_errors:
            print('Claim heartbeat failed during export; check your claim in the UI.')
    finally:
        stop.set()
        thread.join(timeout=16)
        if created_partial and partial is not None and partial.exists():
            partial.unlink()


if __name__ == '__main__':
    try:
        main()
    except urllib.error.HTTPError as exc:
        raise SystemExit(f'HTTP {exc.code}: {exc.read(4096).decode(errors="replace")}')
