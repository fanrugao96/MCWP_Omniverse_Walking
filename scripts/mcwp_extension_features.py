"""Additional verified interface work; imported by the Kit runtime."""
import asyncio, hashlib, json, math, traceback
from pathlib import Path
import omni.kit.app
from PIL import ImageStat
from pxr import Gf, Usd, UsdGeom, Sdf
from mcwp_interfaces import image_message, publish_image, load_ue_pose_log, verify_ros_transport

ROOT=Path(__file__).resolve().parents[1]

def save(name,value):
    (ROOT/'evidence'/name).write_text(json.dumps(value,indent=2),encoding='utf8')

class SourceInterfaces:
    def __init__(self,runtime):
        self.r=runtime
        self.poses=load_ue_pose_log(ROOT/'assets/UE_RightHand_original.csv')
        self.playing=False;self.elapsed=0;self.index=0;self.seq=0
        self.xform=UsdGeom.Xform.Define(runtime.stage,'/Simulation/RecordedUEController')
        self.op=self.xform.AddTransformOp()
        # An oriented RGB axis glyph; this is NOT an articulated hand or HMD.
        for axis,point,color in [('X',(12,0,0),(1,.1,.1)),('Y',(0,12,0),(.1,1,.1)),('Z',(0,0,12),(.1,.3,1))]:
            curve=UsdGeom.BasisCurves.Define(runtime.stage,'/Simulation/RecordedUEController/'+axis)
            curve.CreateTypeAttr('linear');curve.CreateCurveVertexCountsAttr([2])
            curve.CreatePointsAttr([Gf.Vec3f(0),Gf.Vec3f(*point)])
            curve.CreateWidthsAttr([1.5]);curve.SetWidthsInterpolation('constant')
            curve.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        self.xform.MakeInvisible()

    def set_sample(self,index):
        self.index=index
        pose=self.poses[index]; q=pose['quaternion_xyzw']
        transform=Gf.Matrix4d().SetRotate(Gf.Quatd(q[3],Gf.Vec3d(*q[:3])))
        transform.SetTranslateOnly(Gf.Vec3d(*pose['position_cm']))
        self.op.Set(transform)
        return transform

    def toggle_replay(self):
        if self.r.busy:return
        self.playing=not self.playing
        if self.playing:
            self.elapsed=0;self.set_sample(0);self.xform.MakeVisible()
            self.r.message('Recorded UE controller poses: 30 samples/s visualization; original timing unavailable.')
        else:
            self.xform.MakeInvisible();self.r.message('Historical pose replay stopped.')

    def update(self,dt):
        if self.playing:
            self.elapsed+=dt
            index=min(len(self.poses)-1,int(self.elapsed*30))
            self.set_sample(index)
            if index==len(self.poses)-1:
                self.playing=False
                self.r.message('UE pose samples replayed. No live XR hardware or grasp events inferred.')

    async def capture_source_rgb(self):
        camera=UsdGeom.Camera(self.r.stage.GetPrimAtPath('/Simulation/Camera'))
        focal=camera.GetFocalLengthAttr().Get();resolution=self.r.viewport.resolution
        try:
            # Source: Camera.cpp 1280x720, 90-degree horizontal FOV.
            self.r.viewport.resolution=(1280,720)
            camera.GetFocalLengthAttr().Set(18.0)
            await asyncio.sleep(1.0)
            await self.r.apply_capture_exposure()
            for attempt in range(8):
                img=await self.r.capture_image(ROOT/'evidence/ros_rgb_capture.png')
                stats=ImageStat.Stat(img)
                mean=sum(stats.mean)/3
                if 20<mean<245 and max(stats.stddev)>10:
                    save('ros_capture_visual_validation.json',{'passed':True,'size':list(img.size),'mean_rgb':mean,'stddev_rgb':stats.stddev,'attempt':attempt+1})
                    return img
                await asyncio.sleep(1.0)
                await self.r.apply_capture_exposure()
            raise RuntimeError('RGB capture remained underexposed/overexposed; refusing to publish it')
        finally:
            camera.GetFocalLengthAttr().Set(focal)
            self.r.viewport.resolution=resolution
            self.r.restore_preview_exposure()

    async def send_ros_image(self):
        if self.r.busy:return
        config=json.loads((ROOT/'ros_config.json').read_text(encoding='utf-8-sig'))
        if not config.get('url'):
            self.r.message('ROS blocked: no server configured. See ros_config.json. No connection attempted.')
            return
        self.r.busy=True
        try:
            img=await self.capture_source_rgb();self.seq+=1
            report=await publish_image(config['url'],image_message(img,self.seq))
            save('ros_latest_send.json',report)
            self.r.message('Image sent over WebSocket. Real ROS subscriber receipt still requires verification.')
        except Exception:
            save('ros_latest_error.json',{'error':traceback.format_exc()})
            self.r.message('ROS send failed; see evidence/ros_latest_error.json.')
        finally:self.r.busy=False

    async def verify(self):
        if self.r.busy:return
        self.r.busy=True;self.playing=False
        result={'scope':'Original source audit + local interface verification; no real ROS or live XR validation'}
        try:
            self.r.message('Verifying recorded UE controller samples and ROS image transport...')
            max_error=0.0
            original=[]
            import csv
            with open(ROOT/'assets/UE_RightHand_original.csv',newline='') as f: original=list(csv.reader(f))
            for i,pose in enumerate(self.poses):
                self.set_sample(i)
                # Read through USD's composed world transform, not the helper return value.
                matrix=UsdGeom.XformCache().GetLocalToWorldTransform(self.xform.GetPrim())
                measured=matrix.ExtractTranslation()
                raw=list(map(float,original[i][1:4]))
                expected=Gf.Vec3d(raw[0],-raw[1],raw[2])
                max_error=max(max_error,(measured-expected).GetLength())
                assert abs(matrix.GetDeterminant()-1)<1e-10
                if i%100==0:await omni.kit.app.get_app().next_update_async()
            assert max_error<1e-8
            # Independent basis-vector check for the reflected quaternion mapping.
            worst_axis_error=0.0
            for i in [0,len(original)//2,len(original)-1]:
                raw=list(map(float,original[i][4:]));norm=math.sqrt(sum(v*v for v in raw))
                ue_rotation=Gf.Rotation(Gf.Quatd(raw[3]/norm,Gf.Vec3d(*[v/norm for v in raw[:3]])))
                self.set_sample(i);matrix=UsdGeom.XformCache().GetLocalToWorldTransform(self.xform.GetPrim())
                for axis in [Gf.Vec3d(1,0,0),Gf.Vec3d(0,1,0),Gf.Vec3d(0,0,1)]:
                    reflected=Gf.Vec3d(axis[0],-axis[1],axis[2]);r=ue_rotation.TransformDir(reflected)
                    expected=Gf.Vec3d(r[0],-r[1],r[2])
                    worst_axis_error=max(worst_axis_error,(matrix.TransformDir(axis)-expected).GetLength())
            assert worst_axis_error<1e-8
            self.xform.MakeVisible();self.set_sample(len(self.poses)//2)
            self.r.configure_viewport();await self.r.apply_capture_exposure()
            await self.r.capture_image(ROOT/'evidence/ue_pose_replay.png')
            self.xform.MakeInvisible();self.r.restore_preview_exposure()
            result['recorded_controller_pose']={'passed':True,'samples':len(self.poses),'source_first_time':self.poses[0]['source_timestamp'],
                'source_last_time':self.poses[-1]['source_timestamp'],'max_position_error_cm':max_error,'max_orientation_axis_error':worst_axis_error,
                'source_sha256':hashlib.sha256((ROOT/'assets/UE_RightHand_original.csv').read_bytes()).hexdigest(),
                'translation':'x,-y,z centimetres','quaternion':'-qx,qy,-qz,qw normalized',
                'timing':'30 samples/s chosen for visualization; UE timestamps only resolve seconds; logging only while Grip',
                'limitations':'Historical controller log, not worker animation, not MCWP motion, not live XR; no grip transition events in log'}
            img=await self.capture_source_rgb()
            result['ros_image_transport']=await verify_ros_transport(img)
            result['ros_image_transport']['camera_scope']='Actual Kit capture, source resolution/FOV/encoding; current preview pose, original extrinsic/TF not reproduced'
            result['all_interface_checks_passed']=True
            save('interface_validation.json',result)
            self.r.message('Interface checks passed: recorded poses + local ROS image WebSocket fixture. No live ROS/XR.')
        except Exception:
            result['error']=traceback.format_exc();result['all_interface_checks_passed']=False
            save('interface_validation.json',result);print(result['error']);self.r.message('Interface check failed; see evidence/interface_validation.json.')
        finally:
            self.xform.MakeInvisible();self.r.restore_preview_exposure();self.r.busy=False
