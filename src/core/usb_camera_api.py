"""Reusable USB camera router; authentication is supplied by the host app."""
import asyncio
import math
from fastapi import APIRouter, Depends, HTTPException, Query
from fastapi.responses import Response, StreamingResponse
from .usb_camera import CameraUnavailable


def camera_urls(camera_id):
    base = f'/usb/{camera_id}'
    return {name: f'{base}/{path}' for name, path in {
        'status': 'status', 'start': 'start', 'stop': 'stop',
        'snapshot': 'snapshot.jpg', 'stream': 'stream.mjpg',
    }.items()}


async def listing(manager):
    try:
        cameras = await asyncio.to_thread(manager.discover)
    except CameraUnavailable as exc:
        return {'cameras': [], 'default': None, 'available': False, 'reason': str(exc)}
    for camera in cameras:
        camera['urls'] = camera_urls(camera['id'])
    return {'cameras': cameras, 'default': cameras[0]['id'] if len(cameras) == 1 else None,
            'available': True, 'reason': None}


def create_router(manager, require_login):
    router = APIRouter(prefix='/usb', tags=['USB cameras'])
    protected = [Depends(require_login)]

    async def call(fn, *args, **kwargs):
        try:
            return await asyncio.to_thread(fn, *args, **kwargs)
        except KeyError:
            raise HTTPException(404, detail={'error': 'camera_not_found'})
        except CameraUnavailable as exc:
            raise HTTPException(503, detail={'error': 'camera_unavailable', 'reason': str(exc)})

    @router.get('/cameras')
    async def cameras():
        return await listing(manager)

    @router.get('/{camera_id}/status')
    async def status(camera_id: str):
        camera = await call(manager.get, camera_id)
        return camera.describe()

    @router.post('/{camera_id}/start', dependencies=protected)
    async def start(camera_id: str):
        camera = await call(manager.get, camera_id)
        await call(camera.start)
        await call(camera.jpeg)  # Report open/read errors before claiming success.
        return camera.describe()

    @router.post('/{camera_id}/stop', dependencies=protected)
    async def stop(camera_id: str):
        camera = await call(manager.get, camera_id)
        await call(camera.stop)
        return camera.describe()

    @router.get('/{camera_id}/snapshot.jpg', dependencies=protected)
    async def snapshot(camera_id: str):
        camera = await call(manager.get, camera_id)
        await call(camera.start)
        data, number = await call(camera.jpeg)
        return Response(data, media_type='image/jpeg', headers={
            'Cache-Control': 'no-store', 'X-Frame-Number': str(number)})

    @router.get('/{camera_id}/stream.mjpg', dependencies=protected)
    async def stream(camera_id: str, fps: float = Query(default=10, ge=0.5, le=30)):
        if not math.isfinite(fps):
            raise HTTPException(422, detail='fps must be finite')
        camera = await call(manager.get, camera_id)
        await call(camera.start)
        first = await call(camera.jpeg)

        async def body():
            data, number = first
            while True:
                yield (b'--usb-frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                       + str(len(data)).encode() + b'\r\n\r\n' + data + b'\r\n')
                await asyncio.sleep(1 / fps)
                try:
                    data, number = await asyncio.to_thread(camera.jpeg, after=number)
                except CameraUnavailable:
                    return

        return StreamingResponse(body(), media_type='multipart/x-mixed-replace; boundary=usb-frame',
                                 headers={'Cache-Control': 'no-store'})

    return router
