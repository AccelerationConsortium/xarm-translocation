"""Compatibility facade for a separately owned SDL camera service.

Enabled only by XARM_CAMERA_SERVICE_CONFIG. Status reads use cached telemetry;
robot status never waits for a camera HTTP request. No hardware SDK imports.
"""
import json
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import httpx
from .realsense_camera import RealSenseError, RealSenseUnavailable, RealSenseNotStreaming
from .realsense_captures import CaptureNotFound


class RemoteService:
    def __init__(self, config):
        self.ids = list(config['cameras'])
        self.client = httpx.Client(base_url=config['url'].rstrip('/'),
            headers={'Authorization': 'Bearer '+config['token']}, timeout=40,trust_env=False,
            limits=httpx.Limits(keepalive_expiry=4))
        self.stop_event = threading.Event()
        self.states = {}
        self.store_info = {'enabled':True, 'count':0,'bytes':0,'last_id':None,'last_at':None}
        self.sampled = 0
        self.reason = 'Camera service has not been sampled'
        self.thread = None

    def request(self, method, path, **kwargs):
        try:
            r = self.client.request(method,path,**kwargs)
            r.raise_for_status()
            return r
        except httpx.HTTPStatusError as exc:
            if exc.response.status_code == 409:
                raise RealSenseNotStreaming(exc.response.text) from exc
            if exc.response.status_code == 404:
                raise CaptureNotFound(path) from exc
            raise RealSenseUnavailable(f'Camera service HTTP {exc.response.status_code}: {exc.response.text}') from exc
        except httpx.HTTPError as exc:
            raise RealSenseUnavailable(f'Camera service unavailable: {type(exc).__name__}') from exc

    def start_polling(self):
        if self.thread and self.thread.is_alive():
            return
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._poll,daemon=True,name='camera-service-status')
        self.thread.start()

    def _poll(self):
        while not self.stop_event.is_set():
            try:
                data = self.request('GET','/v1/cameras',timeout=5).json()
                self.states = {d['id']:d for d in data['cameras'] if d['id'] in self.ids}
                self.store_info = self.request('GET','/v1/store',timeout=5).json()
                self.sampled=time.monotonic()
                self.reason=None
            except Exception as exc:
                self.reason=str(exc)
            self.stop_event.wait(3)

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(12)
        self.client.close()


class RemoteCamera:
    remote = True
    configured = True
    autostart = False

    def __init__(self, camera_id, config, service):
        self.camera_id=camera_id
        self.label=config.get('label',camera_id)
        self.short_label=config.get('short_label',camera_id)
        self.mount=config.get('mount',{})
        self.align_depth_to_color=config.get('align_depth_to_color',True)
        self.service=service
        self.base=f'/v1/cameras/{camera_id}'

    @property
    def start_on_demand(self):
        d=self.describe()
        return bool(d.get('present') and d.get('installed') and d.get('start_on_demand',True))

    @property
    def streaming(self):
        return bool(self.describe().get('streaming'))

    def describe(self):
        d=dict(self.service.states.get(self.camera_id,{}))
        age=time.monotonic()-self.service.sampled if self.service.sampled else None
        stale=self.service.reason or age is None or age>15
        base={'camera_id':self.camera_id,'configured':True,'installed':False,'label':self.label,
              'short_label':self.short_label,'mount':self.mount,'device':None,'devices':[],
              'streams':{},'frames_captured':0,'fps_measured':None,'last_frame_age_s':None,
              'warnings':[],'present':False,'streaming':False,'state':'unavailable',
              'reason':self.service.reason,'start_on_demand':False}
        base.update(d)
        base['sample_age_s']=age
        if stale:
            base.update(state='unavailable',present=False,streaming=False,start_on_demand=False,
                        reason=self.service.reason or 'Camera telemetry stale')
        return base

    def component_status(self):
        d=self.describe()
        return {'connected':d['present'],'state':d['state'],'message':d.get('reason') or self.label}

    def status_block(self):
        return self.describe()

    def start(self, **kwargs):
        return self.service.request('POST',self.base+'/start').json()

    def stop(self):
        self.service.request('POST',self.base+'/stop')

    def ensure_started(self):
        # Snapshot endpoints themselves implement on-demand capture.
        pass

    def jpeg(self, kind='color'):
        r=self.service.request('GET',self.base+'/snapshot.jpg',params={'stream':kind})
        return r.content,self._frame(r)

    def depth_png(self):
        r=self.service.request('GET',self.base+'/depth.png')
        return r.content,self._frame(r)

    @staticmethod
    def _frame(r):
        return SimpleNamespace(frame_number=int(r.headers.get('X-Frame-Number',0)),
            timestamp_ms=float(r.headers.get('X-Frame-Timestamp-Ms',0)),
            depth_scale=float(r.headers.get('X-Depth-Scale-M',0)))

    def depth_at(self,x,y,window=5):
        return self.service.request('GET',self.base+'/depth',params=dict(x=x,y=y,window=window)).json()

    def intrinsics(self):
        return self.service.request('GET',self.base+'/intrinsics').json()

    def diagnostic_export(self,start_if_idle=False):
        r=self.service.request('POST',self.base+'/diagnostic',params={'start_if_idle':start_if_idle},timeout=90)
        return r.headers['X-Capture-ID'],r.content

    def mjpeg_frames(self,kind='color',max_fps=10):
        try:
            with self.service.client.stream('GET',self.base+'/stream.mjpg',params={'stream':kind,'fps':max_fps}) as r:
                r.raise_for_status()
                yield from r.iter_bytes()
        except httpx.HTTPError as exc:
            raise RealSenseUnavailable('Camera stream unavailable') from exc

    def mjpeg_content_type(self):
        return 'multipart/x-mixed-replace; boundary=camera-frame'

    def capture_remote(self, **body):
        return self.service.request('POST',self.base+'/captures',json=body).json()


class RemoteStore:
    remote=True
    enabled=True

    def __init__(self,service):
        self.service=service

    def describe(self):
        return dict(self.service.store_info)

    def summary(self,camera_id=None):
        d=self.describe()
        if camera_id:
            d=d.get('cameras',{}).get(camera_id,{})
        return {k:d.get(k) for k in ('count','bytes','last_id','last_at')}

    def list_captures(self,camera_id=None,**params):
        items=[]
        for cid in ([camera_id] if camera_id else self.service.ids):
            r=self.service.request('GET',f'/v1/cameras/{cid}/captures',params={k:v for k,v in params.items() if v is not None})
            items.extend(r.json()['captures'])
        items.sort(key=lambda m:m.get('captured_at',''),reverse=True)
        return items[:params.get('limit',50)]

    def get(self,camera_id,capture_id):
        return self.service.request('GET',f'/v1/cameras/{camera_id}/captures/{capture_id}').json()['meta']

    def get_file(self,camera_id,capture_id,filename):
        return self.service.request('GET',f'/v1/cameras/{camera_id}/captures/{capture_id}/{filename}').content

    def delete(self,camera_id,capture_id):
        try:
            self.service.request('DELETE',f'/v1/cameras/{camera_id}/captures/{capture_id}')
            return True
        except CaptureNotFound:
            return False


def configure(path, camera_config):
    config=json.loads(Path(path).read_text(encoding='utf-8-sig'))
    service=RemoteService(config)
    cameras={entry['id']:RemoteCamera(entry['id'],entry,service)
             for entry in camera_config.get('cameras',[]) if entry['id'] in service.ids}
    if set(cameras)!=set(service.ids):
        raise ValueError('Camera service IDs must match the existing xArm camera configuration')
    return service,cameras,RemoteStore(service)
