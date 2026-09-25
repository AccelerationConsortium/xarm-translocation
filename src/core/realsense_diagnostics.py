"""Bounded, read-only taps of the service's existing RealSense framesets."""
from __future__ import annotations

import hashlib
import io
import json
from pathlib import Path
import subprocess
import threading
import time
import uuid
import zipfile
from datetime import datetime, timezone


def provenance():
    root = Path(__file__).resolve().parents[2]
    def git(*args):
        return subprocess.check_output(['git', '-C', str(root), *args],
                                       timeout=5, stderr=subprocess.DEVNULL).decode().strip()
    try:
        status = git('status', '--porcelain', '--untracked-files=all')
        return {'commit': git('rev-parse', 'HEAD'), 'dirty': bool(status),
                'status': status, 'source_sha256': {
                    str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in (root / 'src').rglob('*.py')}}
    except Exception as exc:
        return {'commit': None, 'dirty': None, 'error': str(exc)}


# Snapshot at service module import, before a request can change files on disk.
SERVICE_PROVENANCE = provenance()


def profile_info(profile):
    from .realsense_camera import _intrinsics_dict
    video = profile.as_video_stream_profile()
    return {'format': str(profile.format()), 'fps': profile.fps(),
            'stream': str(profile.stream_type()), 'index': profile.stream_index(),
            'unique_id': profile.unique_id(),
            'intrinsics': _intrinsics_dict(video.get_intrinsics(), str(profile.stream_type()))}


def frame_info(frame, rs):
    metadata = {}
    # Enumerate SDK metadata, recording unavailable fields explicitly.
    for name, value in rs.frame_metadata_value.__members__.items():
        if name == 'count':
            continue
        try:
            metadata[name] = (frame.get_frame_metadata(value)
                              if frame.supports_frame_metadata(value) else None)
        except Exception as exc:
            metadata[name] = {'error': str(exc)}
    return {'frame_number': frame.get_frame_number(), 'timestamp_ms': frame.get_timestamp(),
            'timestamp_domain': str(frame.get_frame_timestamp_domain()),
            'metadata_sdk_units': metadata, 'profile': profile_info(frame.profile)}


def sensor_settings(device, rs):
    sensors = []
    for sensor in device.query_sensors():
        options = {}
        for option in sensor.get_supported_options():
            try:
                options[str(option)] = sensor.get_option(option)
            except Exception as exc:
                options[str(option)] = {'error': str(exc)}
        sensors.append({'name': sensor.get_info(rs.camera_info.name), 'options': options})
    return sensors


class DiagnosticCapture:
    count = 20
    warmup_frames = 30
    max_bytes = 256 * 1024 * 1024

    def __init__(self, camera):
        self.capture_id = 'diagnostic-' + uuid.uuid4().hex
        self.done = threading.Event()
        self.error = None
        self.frames = []
        self.bytes = 0
        self.skipped = 0
        self.started = time.monotonic()
        self.manifest = {'schema_version': 1, 'capture_id': self.capture_id,
                         'created_at': datetime.now(timezone.utc).isoformat(),
                         'camera_id': camera.camera_id,
                         'sdk_version': camera.library_version,
                         'service_at_import': SERVICE_PROVENANCE,
                         'warmup': {'minimum_frames': 30, 'minimum_seconds': 2},
                         'pairing': 'successive SDK framesets; not a hardware-sync guarantee',
                         'frames': []}

    def fail(self, exc):
        if self.done.is_set():
            return
        self.error = str(exc)
        self.done.set()

    def before_alignment(self, camera, frames):
        if self.done.is_set():
            return None
        self.skipped += 1
        depth, color = frames.get_depth_frame(), frames.get_color_frame()
        if not depth or not color:
            raise RuntimeError('Incomplete native frameset during diagnostic capture')
        if 'camera' not in self.manifest:
            profile = camera._pipeline.get_active_profile()
            device = profile.get_device()
            extr = depth.profile.get_extrinsics_to(color.profile)
            self.manifest.update({
                'camera': camera._device_dict(device),
                'depth_scale_m': device.first_depth_sensor().get_depth_scale(),
                'active_stream_profiles': [profile_info(p) for p in profile.get_streams()],
                'sensor_settings': sensor_settings(device, camera._rs),
                'depth_to_color': {'rotation_column_major': list(extr.rotation),
                                   'translation_m': list(extr.translation)},
                'warmup_frames_skipped': None})
        if self.skipped <= self.warmup_frames or time.monotonic() - self.started < 2:
            return None
        if not self.frames:
            self.manifest['warmup_frames_skipped'] = self.skipped - 1
        return {'native_depth': camera._np.asanyarray(depth.get_data()).copy(),
                'native_color': camera._np.asanyarray(color.get_data()).copy(),
                'metadata': {'depth': frame_info(depth, camera._rs),
                             'color': frame_info(color, camera._rs),
                             'host_monotonic_s': time.monotonic(),
                             'host_utc': datetime.now(timezone.utc).isoformat()}}

    def after_alignment(self, camera, sample, depth_frame):
        if sample is None or self.done.is_set():
            return
        if not depth_frame:
            raise RuntimeError('Missing aligned depth')
        sample['aligned_depth'] = camera._np.asanyarray(depth_frame.get_data()).copy()
        sample['metadata']['aligned_depth'] = frame_info(depth_frame, camera._rs)
        self.bytes += sum(sample[k].nbytes for k in ('native_depth', 'native_color', 'aligned_depth'))
        if self.bytes > self.max_bytes:
            raise RuntimeError('Diagnostic capture exceeds 256 MiB array budget')
        self.frames.append(sample)
        if len(self.frames) == self.count:
            self.done.set()

    def archive(self):
        import numpy as np
        from .realsense_camera import _encode_image
        if self.error or len(self.frames) != self.count:
            raise RuntimeError('Cannot archive an incomplete diagnostic capture')
        output = io.BytesIO()
        with zipfile.ZipFile(output, 'w', compression=zipfile.ZIP_STORED) as archive:
            def write(path, data):
                archive.writestr(self.capture_id + '/' + path, data)
                return {'path': path, 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
            for index, sample in enumerate(self.frames):
                files = {}
                for name in ('native_depth', 'native_color', 'aligned_depth'):
                    array = sample[name]
                    buffer = io.BytesIO()
                    np.save(buffer, array, allow_pickle=False)
                    files[name] = write(f'{index:03d}/{name}.npy', buffer.getvalue())
                    files[name].update({'dtype': str(array.dtype), 'shape': list(array.shape)})
                files['aligned_depth_png'] = write(
                    f'{index:03d}/aligned_depth.png',
                    _encode_image(sample['aligned_depth'], 'PNG', sixteen_bit=True))
                files['metadata'] = write(f'{index:03d}/metadata.json',
                                          json.dumps(sample['metadata'], indent=2).encode())
                self.manifest['frames'].append({'index': index, 'files': files})
            self.manifest['frame_count'] = len(self.frames)
            self.manifest['service_at_export'] = provenance()
            write('manifest.json', json.dumps(self.manifest, indent=2).encode())
        return output.getvalue()
