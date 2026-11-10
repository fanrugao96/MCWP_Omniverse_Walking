"""ROS1 image transport and recorded UE controller poses; no XR hardware emulation.

ROS topic, image size and bgr8 encoding come from the original Camera.cpp.
This module deliberately has no ROS joint-command or lift-control topic.
"""
import asyncio, base64, csv, hashlib, json, math, time
from pathlib import Path
from urllib.parse import urlparse

TOPIC = 'camera/image'
MESSAGE_TYPE = 'sensor_msgs/Image'

def image_message(image, seq=0, stamp_ns=None):
    rgb = image.convert('RGB')
    if rgb.size != (1280,720):
        raise ValueError('Source camera requires a real 1280 x 720 capture')
    stamp_ns = time.time_ns() if stamp_ns is None else stamp_ns
    return {'header':{'seq':seq,'stamp':{'secs':stamp_ns//1000000000,'nsecs':stamp_ns%1000000000},'frame_id':''},
            'height':720,'width':1280,'encoding':'bgr8','is_bigendian':0,'step':3840,
            'data':base64.b64encode(rgb.tobytes('raw','BGR')).decode('ascii')}

async def publish_image(url, message):
    """Send a single image, inspect server errors, and cleanly unadvertise.

    A successful socket write is NOT proof of receipt by a ROS subscriber.
    URLs must be explicitly configured; no automatic LAN endpoint discovery.
    """
    import websockets
    parts=urlparse(url)
    if parts.scheme not in ('ws','wss') or not parts.hostname or parts.username or parts.password:
        raise ValueError('Use an explicit ws:// or wss:// endpoint without embedded credentials')
    statuses=[]
    async with websockets.connect(url,open_timeout=4,close_timeout=2,max_size=8*1024*1024) as socket:
        await socket.send(json.dumps({'op':'advertise','id':'mcwp_image_advertise','topic':TOPIC,'type':MESSAGE_TYPE}))
        await socket.send(json.dumps({'op':'publish','id':'mcwp_image_publish','topic':TOPIC,'msg':message},separators=(',',':')))
        try:
            while True:
                status=json.loads(await asyncio.wait_for(socket.recv(),timeout=.5))
                statuses.append(status)
                if status.get('op')=='status' and status.get('level')=='error':
                    raise RuntimeError('rosbridge rejected image: '+status.get('msg',''))
        except asyncio.TimeoutError:
            pass
        await socket.send(json.dumps({'op':'unadvertise','id':'mcwp_image_stop','topic':TOPIC}))
    return {'transport_sent':True,'ros_subscriber_delivery_verified':False,'topic':TOPIC,'type':MESSAGE_TYPE,
            'payload_bytes':len(base64.b64decode(message['data'])),'statuses':statuses}

def load_ue_pose_log(path):
    """UE left-handed cm -> exported USD right-handed cm, reflect Y.

    Original logging only happens during Grip and records time to whole seconds.
    Consequently replay preserves sample order, not original tracking cadence.
    """
    result=[]
    with open(path,newline='') as f:
        for number,row in enumerate(csv.reader(f),1):
            if len(row)!=8: raise ValueError('Expected 8 columns at row '+str(number))
            x,y,z,qx,qy,qz,qw=map(float,row[1:])
            if not all(math.isfinite(v) for v in [x,y,z,qx,qy,qz,qw]): raise ValueError('Non-finite pose')
            norm=math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw)
            if not .95<norm<1.05: raise ValueError('Invalid quaternion norm')
            result.append({'source_timestamp':row[0],'position_cm':[x,-y,z],
                           'quaternion_xyzw':[-qx/norm,qy/norm,-qz/norm,qw/norm]})
    if not result: raise ValueError('Empty pose log')
    return result

async def verify_ros_transport(image):
    """Local WebSocket protocol fixture only. No ROS server is impersonated."""
    import websockets
    received=[]
    async def receiver(socket,path):
        async for payload in socket:
            received.append(json.loads(payload))
    async with websockets.serve(receiver,'127.0.0.1',0,max_size=8*1024*1024) as server:
        port=server.sockets[0].getsockname()[1]
        msg=image_message(image,seq=1,stamp_ns=1234567890)
        result=await publish_image('ws://127.0.0.1:'+str(port),msg)
    ops=[x['op'] for x in received]
    assert ops==['advertise','publish','unadvertise'],ops
    assert received[0]['type']=='sensor_msgs/Image'
    recovered=base64.b64decode(received[1]['msg']['data'])
    expected=image.convert('RGB').tobytes('raw','BGR')
    assert recovered==expected and len(recovered)==1280*720*3
    result.update({'scope':'loopback WebSocket fixture, NOT real ROS/rosbridge',
                   'operations':ops,'pixel_roundtrip_exact':True,'bgr_sha256':hashlib.sha256(recovered).hexdigest(),
                   'width':1280,'height':720,'encoding':'bgr8','passed':True})
    async def reject(socket,path):
        async for payload in socket:
            if json.loads(payload)['op']=='publish':
                await socket.send(json.dumps({'op':'status','level':'error','msg':'intentional test rejection'}))
    async with websockets.serve(reject,'127.0.0.1',0,max_size=8*1024*1024) as server:
        try:
            await publish_image('ws://127.0.0.1:'+str(server.sockets[0].getsockname()[1]),msg)
        except RuntimeError as exc:
            assert 'intentional test rejection' in str(exc)
            result['server_error_detected']=True
        else: raise AssertionError('Server error was ignored')
    return result
