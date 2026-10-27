import json
import time
import httpx
from src.core.remote_realsense import RemoteService,RemoteCamera,RemoteStore


def test_cached_status_never_fetches_and_outage_invalidates_it():
    service=RemoteService({'url':'http://camera','token':'test','cameras':['rs435i']})
    try:
        service.states={'rs435i':{'installed':True,'present':True,'state':'streaming','streaming':True}}
        service.sampled=time.monotonic();service.reason=None
        camera=RemoteCamera('rs435i',{},service)
        assert camera.streaming
        service.reason='unreachable'
        assert not camera.streaming
        assert not camera.start_on_demand
        assert camera.describe()['state']=='unavailable'
    finally:service.close()


def test_structured_reason_from_the_service_becomes_text():
    service=RemoteService({'url':'http://camera','token':'test','cameras':['rs435i']})
    try:
        service.states={'rs435i':{'installed':True,'present':False,'state':'unavailable','streaming':False,
                                  'reason':{'error':'camera_error','reason':'Camera driver timed out'}}}
        service.sampled=time.monotonic();service.reason=None
        camera=RemoteCamera('rs435i',{'label':'depth cam'},service)
        assert camera.describe()['reason']=='Camera driver timed out'
        assert camera.component_status()=={'connected':False,'state':'unavailable','message':'Camera driver timed out'}
        service.states['rs435i']['reason']={'error':'camera_error','reason':''}
        assert camera.component_status()['message']=='camera_error'
    finally:service.close()


def test_remote_calibration_passes_refresh_through():
    service=RemoteService({'url':'http://camera','token':'test','cameras':['rs435i']})
    service.client.close()
    seen=[]
    def respond(request):
        seen.append((request.url.path,request.url.params.get('refresh'),request.headers.get('authorization')))
        return httpx.Response(200,json={'stereo_baseline':{'baseline_m':0.05}})
    service.client=httpx.Client(base_url='http://camera',headers={'Authorization':'Bearer test'},
                                transport=httpx.MockTransport(respond))
    try:
        camera=RemoteCamera('rs435i',{},service)
        assert camera.calibration()['stereo_baseline']['baseline_m']==0.05
        camera.calibration(refresh=True)
        assert seen==[('/v1/cameras/rs435i/calibration','false','Bearer test'),
                      ('/v1/cameras/rs435i/calibration','true','Bearer test')]
    finally:service.close()


def test_remote_snapshot_and_capture_keep_metadata():
    service=RemoteService({'url':'http://camera','token':'test','cameras':['rs435i']})
    service.client.close()
    def respond(request):
        if request.url.path.endswith('snapshot.jpg'):
            return httpx.Response(200,content=b'jpeg',headers={'X-Frame-Number':'12','X-Frame-Timestamp-Ms':'50'})
        data=json.loads(request.content)
        assert data['context']['arm']['node_id']=='home'
        return httpx.Response(200,json={'capture_id':'example','meta':data})
    service.client=httpx.Client(base_url='http://camera',transport=httpx.MockTransport(respond))
    try:
        camera=RemoteCamera('rs435i',{},service)
        jpeg,frame=camera.jpeg()
        assert jpeg==b'jpeg' and frame.frame_number==12
        assert camera.capture_remote(context={'arm':{'node_id':'home'}})['capture_id']=='example'
    finally:service.close()
