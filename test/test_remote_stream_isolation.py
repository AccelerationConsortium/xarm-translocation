"""A camera stream must never be able to freeze the shared camera-service client.

2026-10-27/28 (Gibbie UR3e): after an operator opened and left the panel's
live view, every camera call through robot-motion hung for ~22 h while the
camera service itself stayed healthy. The stream generator shared
``RemoteService.client``; httpcore guards that client's pool with a plain
``threading.Lock``. An abandoned stream is finalised by the cyclic garbage
collector on whatever thread triggers a collection, and closing its response
re-enters the pool lock -- so a collection that fires on a thread already
inside the pool deadlocks that thread with the lock held, and every later
request waits on it forever (status polling, snapshots, depth reads).

These tests run against a real local HTTP server: ``httpx.MockTransport``
has no connection pool and cannot show the hang.
"""
import gc
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from src.core.remote_realsense import RemoteCamera, RemoteService, StreamParts

PART = b'--camera-frame\r\nContent-Type: image/jpeg\r\n\r\njpeg\r\n'


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path.startswith('/v1/cameras/rs435i/stream.mjpg'):
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=camera-frame')
            self.end_headers()
            self.server.streams_open += 1
            try:
                while not self.server.stopping:
                    self.wfile.write(PART)
                    self.wfile.flush()
                    time.sleep(0.02)
            except OSError:
                pass
            finally:
                self.server.streams_open -= 1
            return
        body = b'{"cameras": []}' if self.path == '/v1/cameras' else b'{}'
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)


@pytest.fixture
def camera_service():
    server = ThreadingHTTPServer(('127.0.0.1', 0), _Handler)
    server.daemon_threads = True
    server.streams_open = 0
    server.stopping = False
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    service = RemoteService({'url': f'http://127.0.0.1:{server.server_port}', 'token': 'test',
                             'cameras': ['rs435i']})
    try:
        yield server, service, RemoteCamera('rs435i', {}, service)
    finally:
        server.stopping = True
        # Closing a deadlocked client blocks forever; never let a regression
        # hang the whole suite.
        closer = threading.Thread(target=service.close, daemon=True)
        closer.start()
        closer.join(5)
        server.shutdown()
        server.server_close()


def _wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


def test_abandoned_stream_collected_inside_the_shared_pool_does_not_deadlock(camera_service):
    server, service, camera = camera_service
    parts = camera.mjpeg_frames()
    assert next(parts).startswith(b'--camera-frame')
    # Abandon the stream the way a dropped viewer does: only a reference
    # cycle keeps it, so the cyclic collector is what finalises it.
    holder = [parts]
    holder.append(holder)
    del parts, holder

    # httpcore's pool lock on the shared client (private, but it is the lock
    # the deadlock happens on).
    pool_lock = service.client._transport._pool._optional_thread_lock
    finished = threading.Event()

    def poll_cycle():
        gc_was_enabled = gc.isenabled()
        gc.disable()
        try:
            with pool_lock:
                gc.collect()  # a collection triggered by an allocation inside the pool
        finally:
            if gc_was_enabled:
                gc.enable()
        service.request('GET', '/v1/cameras', timeout=5)
        finished.set()

    worker = threading.Thread(target=poll_cycle, daemon=True)
    worker.start()
    assert finished.wait(8), 'shared camera-service client deadlocked by an abandoned stream'
    assert _wait_for(lambda: server.streams_open == 0), 'abandoned stream kept its upstream connection'


def test_stream_parts_close_releases_the_upstream_connection(camera_service):
    server, _service, camera = camera_service
    parts = StreamParts(camera.mjpeg_frames())
    assert parts.next().startswith(b'--camera-frame')
    assert _wait_for(lambda: server.streams_open == 1)
    parts.close()
    assert _wait_for(lambda: server.streams_open == 0)
    assert parts.next() is None  # a closed stream reads as ended


def test_stream_parts_close_waits_for_an_in_flight_pull():
    release = threading.Event()
    closed = threading.Event()

    def frames():
        try:
            release.wait(5)
            yield b'part'
            yield b'never'
        finally:
            closed.set()

    parts = StreamParts(frames())
    pulled = []
    puller = threading.Thread(target=lambda: pulled.append(parts.next()))
    puller.start()
    time.sleep(0.05)  # the pull is now blocked inside the generator
    errors = []

    def close():
        try:
            parts.close()
        except Exception as exc:  # 'generator already executing' without the lock
            errors.append(exc)

    closer = threading.Thread(target=close)
    closer.start()
    time.sleep(0.05)
    assert not closed.is_set()
    release.set()
    puller.join(5)
    closer.join(5)
    assert errors == []
    assert pulled == [b'part']
    assert closed.is_set()
