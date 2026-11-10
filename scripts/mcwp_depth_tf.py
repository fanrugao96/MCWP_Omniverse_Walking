"""Depth/TF implementation and calibration checks. No original TF names assumed."""
import asyncio, base64, ctypes, hashlib, json, math, time, traceback
from pathlib import Path
import numpy as np
import omni.kit.app
from omni.kit.viewport.utility import add_aov_to_viewport, next_viewport_frame_async
from omni.kit.widget.viewport.capture import MultiAOVByteCapture
from pxr import Gf, Usd, UsdGeom
from PIL import Image, ImageStat
from mcwp_ros_depth_contract import depth_image, tf_message, verify_contract
from mcwp_interfaces import image_message

ROOT=Path(__file__).resolve().parents[1]
def save(name,value): (ROOT/'evidence'/name).write_text(json.dumps(value,indent=2),encoding='utf8')

async def capture_aovs(viewport,names):
    output={}
    def callback_for(name):
        def callback(capsule,size,width,height,fmt):
            getter=ctypes.pythonapi.PyCapsule_GetPointer
            getter.restype=ctypes.c_void_p;getter.argtypes=[ctypes.py_object,ctypes.c_char_p]
            raw=ctypes.string_at(getter(capsule,None),size)
            output[name]={'raw':raw,'width':width,'height':height,'format':str(fmt),'size':size}
        return callback
    delegate=MultiAOVByteCapture(names,[callback_for(name) for name in names])
    stamp_ns=time.time_ns()
    request=viewport.schedule_capture(delegate)
    completed=await request.wait_for_result()
    for _ in range(240):
        if len(output)==len(names):break
        await omni.kit.app.get_app().next_update_async()
    if len(output)!=len(names):raise RuntimeError('Missing AOV buffers; completed='+str(completed))
    return output,delegate.frame_info,stamp_ns

def decode_depth(data,projection,meters_per_unit):
    if data['format']!='TextureFormat.D32_SFLOAT':raise RuntimeError('Unverified depth format: '+data['format'])
    raw=np.frombuffer(data['raw'],dtype='<f4').reshape(data['height'],data['width']).astype(np.float64)
    p=np.asarray(projection,dtype=float).reshape(4,4)
    # Row-vector convention: ndc_z=(z_view*p22+p32)/(z_view*p23+p33).
    denominator=raw*p[2,3]-p[2,2]
    valid=np.isfinite(raw)&(raw>0)&(np.abs(denominator)>1e-15)
    depth=np.full(raw.shape,np.nan,dtype=np.float64)
    depth[valid]=-(p[3,2]-raw[valid]*p[3,3])/denominator[valid]*meters_per_unit
    depth[depth<=0]=np.nan
    return depth

def optical_pose(stage,camera_path,parent_path):
    if not parent_path:raise ValueError('Explicit USD parent prim mapping required')
    cache=UsdGeom.XformCache()
    camera=stage.GetPrimAtPath(camera_path)
    if not camera or not camera.IsA(UsdGeom.Camera):raise ValueError('Camera prim is invalid')
    parent=stage.GetPrimAtPath(parent_path)
    if not parent:raise ValueError('Parent prim is invalid')
    cam_world=cache.GetLocalToWorldTransform(camera)
    parent_world=Gf.Matrix4d(1) if parent_path=='/' else cache.GetLocalToWorldTransform(parent)
    optical_to_usd=Gf.Matrix4d(1);optical_to_usd[1,1]=-1;optical_to_usd[2,2]=-1
    relative=optical_to_usd*cam_world*parent_world.GetInverse()
    axes=[relative.TransformDir(Gf.Vec3d(*(1 if j==i else 0 for j in range(3)))) for i in range(3)]
    if abs(relative.GetDeterminant()-1)>1e-6 or any(abs(a.GetLength()-1)>1e-6 for a in axes):
        raise ValueError('Scaled or reflected camera/parent transforms are unsupported for TF')
    q=relative.ExtractRotationQuat().GetNormalized();v=q.GetImaginary()
    return list(relative.ExtractTranslation()*UsdGeom.GetStageMetersPerUnit(stage)),[v[0],v[1],v[2],q.GetReal()],relative

async def publish_messages(url,messages):
    """Explicit one-shot ROS WebSocket output; no subscriber delivery claim."""
    from urllib.parse import urlparse
    import websockets
    parts=urlparse(url)
    if parts.scheme not in ('ws','wss') or not parts.hostname or parts.username or parts.password:
        raise ValueError('Explicit ws/wss endpoint without embedded credentials required')
    statuses=[]
    async with websockets.connect(url,open_timeout=4,close_timeout=2,max_size=16*1024*1024) as socket:
        for topic,typ,message in messages:
            await socket.send(json.dumps({'op':'advertise','topic':topic,'type':typ}))
            await socket.send(json.dumps({'op':'publish','topic':topic,'msg':message},separators=(',',':')))
        try:
            while True:
                status=json.loads(await asyncio.wait_for(socket.recv(),timeout=.5))
                statuses.append(status)
                if status.get('op')=='status' and status.get('level')=='error':raise RuntimeError('ROS rejected output: '+status.get('msg',''))
        except asyncio.TimeoutError:pass
        for topic,_,_ in messages:await socket.send(json.dumps({'op':'unadvertise','topic':topic}))
    return {'socket_send_complete':True,'ros_subscriber_delivery_verified':False,'statuses':statuses}

async def publish_latest_capture(url):
    """Manually invoked adapter; frames must already be configured and captured."""
    cfg=json.loads((ROOT/'depth_tf_config.json').read_text())
    if not all(cfg.get(k) for k in ['parent_frame_id','optical_frame_id','parent_prim_path']):
        raise ValueError('No output sent: source TF names and parent mapping remain unspecified')
    depth=json.loads((ROOT/'evidence/depth_image_message.json').read_text())
    transform=json.loads((ROOT/'evidence/tf_configured_message.json').read_text())
    stamp=depth['header']['stamp'];stamp_ns=stamp['secs']*1000000000+stamp['nsecs']
    with Image.open(ROOT/'evidence/rgbd_color_same_frame.png') as im:rgb=image_message(im,depth['header']['seq'],stamp_ns)
    rgb['header']['frame_id']=cfg['optical_frame_id']
    assert rgb['header']==depth['header']
    assert transform['transforms'][0]['header']['stamp']==stamp
    assert transform['transforms'][0]['child_frame_id']==cfg['optical_frame_id']
    return await publish_messages(url,[('camera/image','sensor_msgs/Image',rgb),('camera/image2','sensor_msgs/Image',depth),('/tf','tf2_msgs/TFMessage',transform)])

async def verify_transport(messages):
    import websockets
    received=[]
    async def receiver(socket,path):
        async for payload in socket:received.append(json.loads(payload))
    async with websockets.serve(receiver,'127.0.0.1',0,max_size=16*1024*1024) as server:
        await publish_messages('ws://127.0.0.1:'+str(server.sockets[0].getsockname()[1]),messages)
    published=[m for m in received if m['op']=='publish']
    assert len(published)==len(messages)
    for received_msg,(_,_,expected) in zip(published,messages):assert received_msg['msg']==expected
    return {'passed':True,'messages':len(published),'operations_received':len(received),
            'topics':[m['topic'] for m in published],'scope':'Temporary loopback WebSocket fixture only; no ROS node or TF listener',
            'real_ros_subscriber_verified':False,'all_message_fields_roundtrip_exact':True}

async def verify(runtime):
    if runtime.busy:return
    runtime.busy=True;runtime.running=False
    viewport=runtime.viewport
    resolution=viewport.resolution
    matrix=runtime.camera_op.Get()
    camera=UsdGeom.Camera(runtime.stage.GetPrimAtPath('/Simulation/Camera'))
    focal=camera.GetFocalLengthAttr().Get()
    calibration_path='/Simulation/DepthCalibration'
    previous_replay=runtime.interfaces.playing;runtime.interfaces.playing=False
    result={'scope':'Native Kit depth + explicit corrected ROS contract + TF component fixtures; not original ROS/TF interoperability'}
    selected_exposure=None
    try:
        runtime.message('Calibrating native depth at 1 m and 2 m; validating optical-frame transforms...')
        viewport.resolution=(1280,720);camera.GetFocalLengthAttr().Set(18)
        add_aov_to_viewport(viewport,'Depth')
        parent=UsdGeom.Xform.Define(runtime.stage,calibration_path);parent.AddTranslateOp().Set(Gf.Vec3d(10000,0,0))
        plane=UsdGeom.Mesh.Define(runtime.stage,calibration_path+'/Plane')
        plane.CreatePointsAttr([(-600,-600,0),(600,-600,0),(600,600,0),(-600,600,0)])
        plane.CreateFaceVertexCountsAttr([4]);plane.CreateFaceVertexIndicesAttr([0,1,2,3]);plane.CreateSubdivisionSchemeAttr('none')
        plane.CreateDoubleSidedAttr(True)
        calibration=[];fixture_tf=None
        for distance in [100.,200.]:
            runtime.camera_op.Set(Gf.Matrix4d().SetTranslate(Gf.Vec3d(10000,0,distance)))
            await next_viewport_frame_async(viewport,n_frames=8);await asyncio.sleep(.5)
            outputs,info,stamp=await capture_aovs(viewport,['Depth'])
            d=decode_depth(outputs['Depth'],info['projection'],UsdGeom.GetStageMetersPerUnit(runtime.stage))
            pixels=[(640,360),(320,180),(960,540)]
            values=[float(d[v,u]) for u,v in pixels]
            error=max(abs(x-distance*.01) for x in values)
            assert error<.001,values
            translation,quat,relative=optical_pose(runtime.stage,'/Simulation/Camera',calibration_path)
            assert np.allclose(translation,[0,0,distance*.01],atol=1e-8)
            assert np.allclose(relative.TransformDir(Gf.Vec3d(1,0,0)),[1,0,0],atol=1e-8)
            assert np.allclose(relative.TransformDir(Gf.Vec3d(0,1,0)),[0,-1,0],atol=1e-8)
            assert np.allclose(relative.TransformDir(Gf.Vec3d(0,0,1)),[0,0,-1],atol=1e-8)
            fixture_tf=tf_message(translation,quat,'fixture_parent','fixture_camera_optical',1,stamp)
            calibration.append({'expected_depth_m':distance*.01,'pixel_coordinates':pixels,'depth_m':values,
                                'max_error_m':error,'tf_translation_m':translation,'tf_quaternion_xyzw':quat,'passed':True})
        runtime.stage.RemovePrim(calibration_path)
        runtime.camera_op.Set(matrix)
        runtime.configure_viewport();viewport.resolution=(1280,720);camera.GetFocalLengthAttr().Set(18)
        await next_viewport_frame_async(viewport,n_frames=8)
        exposure_trials=[]
        for f_stop,iso,exposure_time in [(1.4,400,.1),(5.,100,.02),(2.8,100,.02),(2.8,400,.1),(.7,800,.2)]:
            for key,value in [('exposure:fStop',f_stop),('exposure:iso',iso),('exposure:time',exposure_time)]:
                camera.GetPrim().GetAttribute(key).Set(value)
            camera.GetPrim().GetAttribute('omni:rtx:autoExposure:enabled').Set(False)
            await next_viewport_frame_async(viewport,n_frames=8);await asyncio.sleep(1)
            outputs,info,stamp=await capture_aovs(viewport,['Depth','LdrColor'])
            color=outputs['LdrColor'];fmt=color['format']
            if 'R8G8B8A8' in fmt or 'RGBA8_' in fmt:rawmode='RGBA'
            elif 'B8G8R8A8' in fmt or 'BGRA8_' in fmt:rawmode='BGRA'
            else:raise RuntimeError('Unverified color format: '+fmt)
            rgb=Image.frombytes('RGBA',(color['width'],color['height']),color['raw'],'raw',rawmode).convert('RGB')
            mean=sum(ImageStat.Stat(rgb).mean)/3
            exposure_trials.append({'f_stop':f_stop,'iso':iso,'exposure_time':exposure_time,'mean_rgb':mean})
            if 20<mean<245:
                selected_exposure={'f_stop':f_stop,'iso':iso,'exposure_time':exposure_time}
                break
        else:
            result['exposure_trials']=exposure_trials
            raise RuntimeError('RGB/D pair color exposure is invalid')
        result['exposure_trials']=exposure_trials
        depth=decode_depth(outputs['Depth'],info['projection'],UsdGeom.GetStageMetersPerUnit(runtime.stage))
        assert depth.shape==(720,1280)
        cfg=json.loads((ROOT/'depth_tf_config.json').read_text())
        optical_frame=cfg.get('optical_frame_id') or ''
        depth_msg=depth_image(depth,1,stamp,optical_frame)
        rgb_msg=image_message(rgb,seq=1,stamp_ns=stamp);rgb_msg['header']['frame_id']=optical_frame
        assert depth_msg['header']==rgb_msg['header']
        raw16=base64.b64decode(depth_msg['data']);mm=np.frombuffer(raw16,dtype='<u2').reshape(depth.shape)
        np.save(ROOT/'evidence/depth_optical_z_m.npy',depth.astype('<f4'))
        Image.fromarray(mm).save(ROOT/'evidence/depth_16UC1_mm.png')
        rgb.save(ROOT/'evidence/rgbd_color_same_frame.png')
        save('depth_image_message.json',depth_msg)
        save('tf_fixture_message.json',fixture_tf)
        # Verify actual scene depth against the existing PhysX geometry at fixed pixels.
        import carb
        p=np.asarray(info['projection']).reshape(4,4)
        runtime.sim.flush_changes()
        eye=matrix.ExtractTranslation();checks=[]
        for u,v in [(640,360),(450,450),(850,450),(450,600),(850,600)]:
            local=Gf.Vec3d((2*(u+.5)/1280-1)/p[0,0],(1-2*(v+.5)/720)/p[1,1],-1).GetNormalized()
            direction=matrix.TransformDir(local).GetNormalized()
            hit=runtime.query.raycast_closest(carb.Float3(*eye),carb.Float3(*direction),100000)
            if hit['hit'] and np.isfinite(depth[v,u]):
                expected=float(hit['distance'])*(-local[2])*.01
                error=abs(float(depth[v,u])-expected)
                checks.append({'pixel':[u,v],'renderer_depth_m':float(depth[v,u]),'physx_optical_depth_m':expected,'error_m':error,'collision':str(hit['collision']),'passed':error<.03})
        assert len(checks)>=3 and all(c['passed'] for c in checks),checks
        tf_state={'production_tf_emitted':False,'reason':'Original TF names and camera-to-parent mapping not defined; config fields remain null',
                  'minimum_needed':['parent_frame_id','optical_frame_id','parent_prim_path representing that ROS parent; camera extrinsic if it differs from current USD camera']}
        if cfg.get('parent_frame_id') and cfg.get('optical_frame_id') and cfg.get('parent_prim_path'):
            t,q,_=optical_pose(runtime.stage,cfg['camera_prim_path'],cfg['parent_prim_path'])
            save('tf_configured_message.json',tf_message(t,q,cfg['parent_frame_id'],cfg['optical_frame_id'],1,stamp))
            tf_state={'production_tf_emitted':False,'configured_tf_file_generated':True,'reason':'File export only; real ROS still requires a server and listener'}
        result.update({'contract_tests':verify_contract(),'native_depth_calibration':calibration,'actual_scene_raycast_checks':checks,
            'rgbd':{'same_render_capture':True,'frame_number':int(info['frame_number']),'swh_frame_number':int(info['swh_frame_number']),
                    'stamp':depth_msg['header']['stamp'],'timestamp_semantics':'UTC acquisition request time; scene frozen; not a recovered UE timestamp',
                    'frame_id':optical_frame,'frame_id_missing':not bool(optical_frame),'size':[1280,720],'horizontal_fov_degrees':90,
                    'depth_encoding':'16UC1','depth_units':'mm along optical Z, zero invalid','endianness':'little',
                    'valid_pixels':int((mm>0).sum()),'min_valid_depth_m':float(np.min(mm[mm>0]))/1000,'max_valid_depth_m':float(mm.max())/1000,
                    'rgb_mean':mean,'depth_payload_bytes':len(raw16),'depth_sha256':hashlib.sha256(raw16).hexdigest(),
                    'corrected_contract_differs_from_source':True},'tf':tf_state,
            'transport':await verify_transport([('camera/image','sensor_msgs/Image',rgb_msg),('camera/image2','sensor_msgs/Image',depth_msg),('/tf','tf2_msgs/TFMessage',fixture_tf)]),
            'all_implemented_checks_passed':True})
        save('depth_tf_validation.json',result)
        runtime.message('Depth + RGB same-frame checks passed. TF fixture passed; original frame mapping still missing.')
    except Exception:
        result.update(error=traceback.format_exc(),all_implemented_checks_passed=False)
        save('depth_tf_validation.json',result);print(traceback.format_exc())
        runtime.message('Depth/TF check failed; see evidence/depth_tf_validation.json.')
    finally:
        runtime.stage.RemovePrim(calibration_path)
        if selected_exposure:
            runtime.save_exposure_profile(selected_exposure)
        runtime.camera_op.Set(matrix);camera.GetFocalLengthAttr().Set(focal)
        viewport.resolution=resolution;runtime.restore_preview_exposure()
        try:
            await runtime.apply_capture_exposure()
            await runtime.capture_image(ROOT/'evidence/current_view.png')
            runtime.diagnose('current_render_state.json')
        finally:
            runtime.busy=False
            runtime.interfaces.playing=previous_replay
