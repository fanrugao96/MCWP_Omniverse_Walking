"""MCWP desktop runtime for Kit 110.4 with lift and a rigged worker.

Original export assets stay unchanged. Lift geometry is separately grouped;
the supplied rigged worker is animated with planted-foot controls.
"""
import asyncio, csv, datetime, io, json, math, os, struct, traceback, sys
from pathlib import Path
import carb, carb.input, carb.settings
import omni.kit.app, omni.usd, omni.ui as ui, omni.appwindow
from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file, next_viewport_frame_async
from omni.physx import get_physx_interface, get_physx_simulation_interface, get_physx_scene_query_interface
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf, UsdUtils
from PIL import Image, ImageStat

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / 'evidence'
if str(ROOT/'scripts') not in sys.path: sys.path.insert(0,str(ROOT/'scripts'))
from mcwp_extension_features import SourceInterfaces
from mcwp_lift import LiftController, WORKER, CARRIAGE
from mcwp_worker import WorkerController
from mcwp_control_menu import ControlWindowMenu

def save_json(name, data):
    (EVIDENCE / name).write_text(json.dumps(data, indent=2), encoding='utf-8')

def chunk(tag, data):
    return tag + struct.pack('<I', len(data)) + data + (b'\0' if len(data) % 2 else b'')

def write_mjpeg_avi(path, frames, width=1920, height=1080, fps=30):
    """Standard AVI 1.0 MJPG container; JPEG frames are actual Kit captures."""
    n, max_size = len(frames), max(map(len, frames))
    avih = struct.pack('<14I', int(1e6/fps), max_size*fps, 0, 0x10, n, 0, 1, max_size, width, height, 0, 0, 0, 0)
    strh = struct.pack('<4s4sIHHIIIIIIIIhhhh', b'vids', b'MJPG', 0, 0, 0, 0, 1, fps, 0, n, max_size, 0xffffffff, 0, 0, 0, width, height)
    strf = struct.pack('<IiiHH4sIiiII', 40, width, height, 1, 24, b'MJPG', width*height*3, 0, 0, 0, 0)
    hdrl = chunk(b'LIST', b'hdrl' + chunk(b'avih', avih) + chunk(b'LIST', b'strl'+chunk(b'strh', strh)+chunk(b'strf', strf)))
    index, offset = [], 4
    movi_size = 4 + sum(8+len(f)+(len(f)%2) for f in frames)
    idx_size = 16*n
    with open(path, 'wb') as out:
        out.write(b'RIFF'+struct.pack('<I', 4+len(hdrl)+8+movi_size+8+idx_size)+b'AVI ')
        out.write(hdrl)
        out.write(b'LIST'+struct.pack('<I', movi_size)+b'movi')
        for f in frames:
            out.write(chunk(b'00dc', f))
            index.append(struct.pack('<4sIII', b'00dc', 0x10, offset, len(f)))
            offset += 8+len(f)+(len(f)%2)
        out.write(chunk(b'idx1', b''.join(index)))

class MCWPSimulation:
    write_video = staticmethod(write_mjpeg_avi)
    def __init__(self):
        self.stage = None
        self.running = False
        self.recording = False
        self.busy = False
        self.time = 0.0
        self.orbit_time = 0.0
        self.physics_accumulator = 0.0
        self.orbit_enabled = False
        self.keys = set()
        self.nav_enabled = False
        self.grip = False
        self.hand_rows = []
        self.hand_pos = Gf.Vec3d(250, 0, 300)
        self.eye = Gf.Vec3d(1000, -1000, 800)
        self.yaw, self.pitch = 135.0, -25.0
        self.last_contact = False
        self.status = None
        self.events = []
        self.control_mtime = 0

    def message(self, text):
        print('MCWP: '+text)
        if self.status:
            self.status.text = text

    async def initialize(self):
        ctx = omni.usd.get_context()
        await ctx.new_stage_async()
        self.stage = ctx.get_stage()
        stage = self.stage
        stage.GetRootLayer().subLayerPaths = [str(ROOT/'assets/ue_export/MCWP_Omniverse_Demo.usda').replace('\\','/')]
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, .01)
        stage.SetTimeCodesPerSecond(30)
        stage.SetStartTimeCode(0)
        stage.SetEndTimeCode(599)
        self.lift = LiftController(self)
        self.lift.build(stage)
        self.worker = WorkerController(self)
        UsdGeom.Imageable(stage.GetPrimAtPath('/Root/MyPawn2')).MakeInvisible()
        scene = UsdPhysics.Scene.Define(stage, '/Simulation/PhysicsScene')
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        scene.CreateGravityMagnitudeAttr(981.0)
        api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        api.CreateSolverTypeAttr('TGS')
        api.CreateEnableGPUDynamicsAttr(False)
        api.CreateBroadphaseTypeAttr('MBP')
        self.colliders = []
        for prim in stage.Traverse():
            if prim.IsA(UsdGeom.Mesh) and (prim.GetPath().pathString.startswith('/Root/MCWP_') or prim.GetPath().pathString == '/Root/Floor'):
                if UsdGeom.Mesh(prim).GetPointsAttr().Get() and not str(prim.GetPath()).startswith(WORKER+'/'):
                    UsdPhysics.CollisionAPI.Apply(prim)
                    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('none')
                    self.colliders.append(str(prim.GetPath()))
        from pxr import UsdLux
        UsdLux.DomeLight.Define(stage, '/Simulation/FillLight').CreateIntensityAttr(1400)
        cam = UsdGeom.Camera.Define(stage, '/Simulation/Camera')
        cam.CreateHorizontalApertureAttr(36)
        cam.CreateVerticalApertureAttr(20.25)
        cam.CreateFocalLengthAttr(36/(2*math.tan(math.radians(30))))
        cam.CreateClippingRangeAttr(Gf.Vec2f(1, 100000))
        # Fixed exposure was visually verified in this Kit/RTX runtime.
        cam.GetPrim().SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit(['OmniRtxCameraAutoExposureAPI_1','OmniRtxCameraExposureAPI_1']))
        for name,value in [('exposure:fStop',1.4),('exposure:responsivity',1.1026709),('exposure:time',.1),('exposure:iso',400.0)]:
            cam.GetPrim().CreateAttribute(name,Sdf.ValueTypeNames.Float).Set(value)
        cam.GetPrim().CreateAttribute('omni:rtx:autoExposure:enabled',Sdf.ValueTypeNames.Bool).Set(False)
        cam.GetPrim().CreateAttribute('omni:rtx:autoExposure:histogramClamping:enabled',Sdf.ValueTypeNames.Bool).Set(False)
        self.camera_op = cam.AddTransformOp()
        bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default','render'])
        pb = bbox.ComputeWorldBound(stage.GetPrimAtPath('/Root/MCWP_Platform')).ComputeAlignedRange()
        wb = bbox.ComputeWorldBound(stage.GetPrimAtPath(WORKER)).ComputeAlignedRange()
        self.center = pb.GetMidpoint()
        self.corners = [Gf.Vec3d(x,y,z) for box in [pb,wb] for x in [box.GetMin()[0],box.GetMax()[0]] for y in [box.GetMin()[1],box.GetMax()[1]] for z in [box.GetMin()[2],box.GetMax()[2]]]
        # Keep the complete lift travel and standing worker inside orbit captures.
        self.corners += [c+Gf.Vec3d(0,0,160) for c in list(self.corners)]
        self.center += Gf.Vec3d(0,0,65)
        radius = max((c-self.center).GetLength() for c in self.corners)
        self.distance = radius/math.sin(math.atan(math.tan(math.radians(30))/(16/9)))*1.12
        self.probe = UsdGeom.Sphere.Define(stage, '/Simulation/ValidationProbe')
        self.probe.CreateRadiusAttr(10)
        self.probe.CreateDisplayColorAttr([Gf.Vec3f(1,.32,.02)])
        self.probe_op = self.probe.AddTranslateOp()
        self.probe_op.Set(Gf.Vec3d(250,0,350))
        UsdPhysics.CollisionAPI.Apply(self.probe.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(self.probe.GetPrim())
        UsdPhysics.MassAPI.Apply(self.probe.GetPrim()).CreateMassAttr(.5)
        PhysxSchema.PhysxRigidBodyAPI.Apply(self.probe.GetPrim()).CreateEnableCCDAttr(False)
        self.lift.sync_probe()
        self.hand = UsdGeom.Sphere.Define(stage, '/Simulation/DesktopHandProxy')
        self.hand.CreateRadiusAttr(6)
        self.hand.CreateDisplayColorAttr([Gf.Vec3f(.1,.65,1)])
        self.hand_op = self.hand.AddTranslateOp()
        self.hand_op.Set(self.hand_pos)
        self.hand.MakeInvisible()
        self.set_orbit_frame(0)
        self.viewport = get_active_viewport()
        self.viewport.camera_path = '/Simulation/Camera'
        self.viewport.resolution = (1920,1080)
        settings = carb.settings.get_settings()
        settings.set('/physics/updateToUsd', True)
        settings.set('/physics/updateVelocitiesToUsd', True)
        settings.set('/persistent/app/viewport/gizmo/scale', 0.0)
        settings.set('/app/viewport/grid/enabled', False)
        for item in ['guide/grid','guide/axis','hud/renderFPS','hud/renderResolution']:
            settings.set(f'/persistent/app/viewport/{self.viewport.id}/{item}/visible', False)
        for item in ['scene/lights','scene/cameras']:
            settings.set(f'/persistent/app/viewport/{self.viewport.id}/{item}/visible', True)
        output = ROOT/'MCWP_Simulation.usda'
        stage.GetRootLayer().Export(str(output))
        # Reopen the persistent root so relative assets are stable across launches.
        await ctx.open_stage_async(str(output))
        self.stage = ctx.get_stage()
        self.stage.GetRootLayer().subLayerPaths=['./assets/ue_export/MCWP_Omniverse_Demo.usda']
        self.stage.GetRootLayer().Save()
        self.bind_prims()
        self.lift.bind(self.stage)
        self.lift.rebase_assets()
        self.worker.bind(self.stage)
        self.worker.apply()
        self.stage.GetRootLayer().Save()
        self.viewport = get_active_viewport()
        self.viewport.camera_path = '/Simulation/Camera'
        self.viewport.resolution = (1920,1080)
        for item in ['guide/grid','guide/axis','hud/renderFPS','hud/renderResolution']:
            settings.set(f'/persistent/app/viewport/{self.viewport.id}/{item}/visible', False)
        settings.set('/persistent/app/viewport/gizmo/scale', 0.0)
        self.sim = get_physx_simulation_interface()
        self.sim.attach_stage(ctx.get_stage_id())
        self.query = get_physx_scene_query_interface()
        self.interfaces = SourceInterfaces(self)
        self.build_ui()
        self.lift.build_ui()
        self.worker.build_ui()
        self.control_menu = ControlWindowMenu(self)
        self.set_orbit_frame(150)
        self.stage.GetRootLayer().Save()
        self.input = carb.input.acquire_input_interface()
        self.keyboard = omni.appwindow.get_default_app_window().get_keyboard()
        self.key_subscription = self.input.subscribe_to_keyboard_events(self.keyboard, self.on_key)
        self.mouse = omni.appwindow.get_default_app_window().get_mouse()
        self.mouse_subscription = self.input.subscribe_to_mouse_events(self.mouse, self.worker.on_mouse)
        app_window=omni.appwindow.get_default_app_window()
        if hasattr(app_window,'get_window_focus_event_stream'):
            self.focus_subscription=app_window.get_window_focus_event_stream().create_subscription_to_pop(self.worker.on_focus)
        self.subscription = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self.on_update, name='MCWP desktop simulation')
        save_json('runtime_environment.json', {'kit':'110.4.0', 'physics_extension':'110.4.1', 'solver':'PhysX CPU TGS', 'meters_per_unit':.01, 'gravity_cm_s2':981, 'colliders':self.colliders, 'source_scene':'accepted kinematic carriage; fixed base and masts; supplied 23-joint worker with planted-foot controls', 'additional_tools':['orange rigid-body probe','desktop hand proxy'], 'xr':'not installed/verified', 'ros':'not active in source; no bridge verified'})
        self.message('Ready. Raise / Lower controls the platform; base and masts stay fixed.')
        await asyncio.sleep(12)
        self.configure_viewport()
        await self.apply_capture_exposure()
        await self.capture_image(EVIDENCE/'startup_view.png')
        self.diagnose('startup_render_state.json')
        self.restore_preview_exposure()
        if os.environ.get('MCWP_AUTOTEST') == '1':
            await self.verify()
        if os.environ.get('MCWP_INTERFACE_TEST') == '1':
            await self.interfaces.verify()
        if os.environ.get('MCWP_DEPTH_TF_TEST') == '1':
            await self.verify_depth_tf()
        if os.environ.get('MCWP_LIFT_AUTOTEST') == '1':
            await self.lift.verify()
        if os.environ.get('MCWP_WORKER_AUTOTEST') == '1':
            await self.worker.verify()
        if os.environ.get('MCWP_WORKER_DETAIL_PREVIEW') == '1':
            from worker_validation import record_detail
            await record_detail(self.worker)

    async def verify_depth_tf(self):
        # Kit's fast importer caches module bytecode across reload calls. Read
        # this fixed local diagnostic module so the button runs its current code.
        module_path=ROOT/'scripts/mcwp_depth_tf.py'
        namespace={'__file__':str(module_path),'__name__':'mcwp_depth_tf_diagnostic'}
        exec(compile(module_path.read_text(encoding='utf8'),str(module_path),'exec'),namespace)
        await namespace['verify'](self)

    def configure_viewport(self):
        self.viewport=get_active_viewport()
        self.viewport.camera_path='/Simulation/Camera'
        self.viewport.resolution=(1920,1080)
        settings=carb.settings.get_settings()
        for item in ['guide/grid','guide/axis','hud/renderFPS','hud/processMemory','hud/deviceMemory','hud/hostMemory','hud/renderResolution']:
            settings.set(f'/persistent/app/viewport/{self.viewport.id}/{item}/visible',False)
        for item in ['scene/lights','scene/cameras']:
            settings.set(f'/persistent/app/viewport/{self.viewport.id}/{item}/visible',True)
        settings.set('/persistent/app/viewport/gizmo/scale',0.0)
        settings.set('/persistent/app/viewport/gizmo/constantScaleEnabled',False)
        settings.set('/persistent/app/viewport/gizmo/constantScale',0.0)
        settings.set('/persistent/app/viewport/gizmo/constantScaleCamera',False)
        settings.set('/app/viewport/show/light',True)
        settings.set('/app/viewport/show/camera',True)
        settings.set('/app/viewport/grid/enabled',False)

    async def apply_capture_exposure(self):
        # Kit's camera binding writes its default exposure after camera_path is
        # assigned. Apply the verified values after that deferred binding update.
        from mcwp_depth_tf import capture_aovs
        for _ in range(3): await omni.kit.app.get_app().next_update_async()
        camera=self.stage.GetPrimAtPath('/Simulation/Camera')
        profile=self.exposure_profile()
        candidates=[(profile['f_stop'],profile['iso'],profile['exposure_time']),(5.,100.,.02),(1.4,400.,.1),(2.8,100.,.02),(.7,800.,.2)]
        for f_stop,iso,exposure_time in dict.fromkeys(candidates):
            for name,value in [('exposure:fStop',f_stop),('exposure:iso',iso),('exposure:time',exposure_time)]:
                camera.GetAttribute(name).Set(value)
            camera.GetAttribute('omni:rtx:autoExposure:enabled').Set(False)
            await next_viewport_frame_async(self.viewport,n_frames=8)
            await asyncio.sleep(1.0)
            buffers,_,_=await capture_aovs(self.viewport,['LdrColor'])
            color=buffers['LdrColor'];fmt=color['format']
            if 'R8G8B8A8' in fmt or 'RGBA8_' in fmt: rawmode='RGBA'
            elif 'B8G8R8A8' in fmt or 'BGRA8_' in fmt: rawmode='BGRA'
            else: raise RuntimeError('Unverified exposure color format: '+fmt)
            img=Image.frombytes('RGBA',(color['width'],color['height']),color['raw'],'raw',rawmode).convert('RGB')
            img.save(EVIDENCE/'exposure_check.png')
            stats=ImageStat.Stat(img);mean=sum(stats.mean)/3
            if 20<mean<225 and max(stats.stddev)>5:
                self.save_exposure_profile({'f_stop':f_stop,'iso':iso,'exposure_time':exposure_time})
                break
        else:raise RuntimeError('Unable to establish a readable MCWP capture exposure')
        self.stage.GetRootLayer().Save()

    def exposure_profile(self):
        path=ROOT/'capture_exposure_profile.json'
        profiles=json.loads(path.read_text()).get('profiles',{}) if path.exists() else {}
        key=self.exposure_key()
        if key in profiles: return profiles[key]
        # Resizing the editor changes the viewport resolution. Exposure for the
        # same lens remains valid; do not revert to an unrelated bright default.
        lens=key.split('_focal_')[-1]
        for candidate,profile in reversed(list(profiles.items())):
            if candidate.endswith('_focal_'+lens): return profile
        return {'f_stop':5.,'iso':100.,'exposure_time':.02}

    def exposure_key(self):
        resolution=self.viewport.resolution
        focal=self.stage.GetPrimAtPath('/Simulation/Camera').GetAttribute('focalLength').Get()
        return f'{resolution[0]}x{resolution[1]}_focal_{focal:.4f}'

    def save_exposure_profile(self,profile):
        path=ROOT/'capture_exposure_profile.json'
        content=json.loads(path.read_text()) if path.exists() else {}
        profiles=content.get('profiles',{})
        profiles[self.exposure_key()]=profile
        path.write_text(json.dumps({'profiles':profiles},indent=2))

    def restore_preview_exposure(self):
        # Use the same exposure for capture and preview; validated after the
        # render thread settles. Avoid reintroducing a dark transient frame.
        camera=self.stage.GetPrimAtPath('/Simulation/Camera')
        profile=self.exposure_profile()
        for key,name in [('f_stop','exposure:fStop'),('iso','exposure:iso'),('exposure_time','exposure:time')]:
            camera.GetAttribute(name).Set(profile[key])
        self.stage.GetRootLayer().Save()

    def diagnose(self,name):
        from pxr import UsdLux
        settings=carb.settings.get_settings()
        keys=[f'/persistent/app/viewport/{self.viewport.id}/scene/lights/visible',f'/persistent/app/viewport/{self.viewport.id}/scene/cameras/visible','/persistent/app/viewport/gizmo/scale','/rtx/sceneDb/ambientLightIntensity','/rtx/post/tonemap/op','/rtx/post/tonemap/filmIso']
        save_json(name,{'viewport_id':self.viewport.id,'settings':{k:settings.get(k) for k in keys},'camera':{str(a.GetName()):str(a.Get()) for a in self.stage.GetPrimAtPath('/Simulation/Camera').GetAttributes()},'lights':[{'path':str(p.GetPath()),'intensity':str(p.GetAttribute('inputs:intensity').Get()),'visibility':str(p.GetAttribute('visibility').Get())} for p in self.stage.Traverse() if p.HasAPI(UsdLux.LightAPI)]})

    async def capture_image(self,path):
        await capture_viewport_to_file(self.viewport,path.as_posix()).wait_for_result()
        # The capture future can precede Windows file-handle closure.
        for attempt in range(80):
            try:
                with Image.open(path) as im:
                    return im.convert('RGB').copy()
            except (OSError,ValueError):
                await omni.kit.app.get_app().next_update_async()
        raise RuntimeError('Capture did not become readable: '+str(path))

    def bind_prims(self):
        self.camera_op = UsdGeom.Xformable(self.stage.GetPrimAtPath('/Simulation/Camera')).GetOrderedXformOps()[0]
        self.probe = UsdGeom.Sphere(self.stage.GetPrimAtPath('/Simulation/ValidationProbe'))
        self.probe_op = UsdGeom.Xformable(self.probe).GetOrderedXformOps()[0]
        self.hand = UsdGeom.Sphere(self.stage.GetPrimAtPath('/Simulation/DesktopHandProxy'))
        self.hand_op = UsdGeom.Xformable(self.hand).GetOrderedXformOps()[0]

    def set_orbit_frame(self, frame):
        angle = 2*math.pi*frame/599
        elevation = math.radians(38)
        self.eye = self.center + self.distance*Gf.Vec3d(math.cos(elevation)*math.cos(angle), -math.cos(elevation)*math.sin(angle), math.sin(elevation))
        self.camera_op.Set(Gf.Matrix4d().SetLookAt(self.eye,self.center,Gf.Vec3d(0,0,1)).GetInverse())
        return self.eye

    def step(self, dt=1/60):
        self.lift.step(dt)
        self.sim.simulate(dt,self.time)
        self.sim.fetch_results()
        self.time += dt

    def reset(self):
        if self.recording: return
        self.running = False
        self.time = self.orbit_time = 0.0
        self.physics_accumulator = 0.0
        self.recreate_probe(Gf.Vec3d(250,0,350))
        self.lift.reset()
        if hasattr(self,'worker'): self.worker.reset()
        self.grip = False
        self.hand.MakeInvisible()
        self.set_orbit_frame(150)
        self.message('Reset to lower lift stop; carried probe restored.')

    def recreate_probe(self,position):
        self.stage.RemovePrim('/Simulation/ValidationProbe')
        self.sim.flush_changes()
        self.probe=UsdGeom.Sphere.Define(self.stage,'/Simulation/ValidationProbe')
        self.probe.CreateRadiusAttr(10)
        self.probe.CreateDisplayColorAttr([Gf.Vec3f(1,.32,.02)])
        self.probe_op=self.probe.AddTranslateOp()
        self.probe_op.Set(Gf.Vec3d(position))
        UsdPhysics.CollisionAPI.Apply(self.probe.GetPrim())
        body=UsdPhysics.RigidBodyAPI.Apply(self.probe.GetPrim())
        body.CreateVelocityAttr(Gf.Vec3f(0))
        body.CreateAngularVelocityAttr(Gf.Vec3f(0))
        UsdPhysics.MassAPI.Apply(self.probe.GetPrim()).CreateMassAttr(.5)
        self.sim.flush_changes()

    def toggle_play(self):
        if not self.busy:
            self.running = not self.running
            self.message('Running physics / orbit.' if self.running else 'Paused.')

    def on_update(self, event):
        control=ROOT/'runtime_control.json'
        if control.exists() and control.stat().st_mtime > self.control_mtime:
            self.control_mtime=control.stat().st_mtime
            try:
                c=json.loads(control.read_text(encoding='utf-8-sig'))
                camera=self.stage.GetPrimAtPath('/Simulation/Camera')
                if 'auto_exposure' in c: camera.GetAttribute('omni:rtx:autoExposure:enabled').Set(bool(c['auto_exposure']))
                for key,name in [('exposure_time','exposure:time'),('f_stop','exposure:fStop'),('iso','exposure:iso')]:
                    if key in c: camera.CreateAttribute(name,Sdf.ValueTypeNames.Float).Set(float(c[key]))
                if 'light_intensity' in c: self.stage.GetPrimAtPath('/Simulation/FillLight').GetAttribute('inputs:intensity').Set(float(c['light_intensity']))
                if c.get('action')=='capture': asyncio.ensure_future(self.capture_preview())
                if c.get('action')=='record' and not self.busy: asyncio.ensure_future(self.record_orbit())
                if c.get('action')=='verify' and not self.busy: asyncio.ensure_future(self.verify())
                if c.get('action')=='verify_interfaces' and not self.busy: asyncio.ensure_future(self.interfaces.verify())
                if c.get('action')=='replay' and not self.busy: self.interfaces.toggle_replay()
                if c.get('action')=='depth_tf' and not self.busy: asyncio.ensure_future(self.verify_depth_tf())
                if c.get('action')=='lift_raise' and not self.busy: self.lift.command(160)
                if c.get('action')=='lift_lower' and not self.busy: self.lift.command(0)
                if c.get('action')=='lift_stop': self.lift.stop()
                if c.get('action')=='lift_target' and not self.busy: self.lift.command(c.get('height_cm',0))
                if c.get('action')=='verify_lift' and not self.busy: asyncio.ensure_future(self.lift.verify())
                if c.get('action')=='verify_worker' and not self.busy: asyncio.ensure_future(self.worker.verify())
                if c.get('action')=='worker_press': self.worker.press(c['key'],source='keyboard')
                if c.get('action')=='worker_release': self.worker.release(c.get('key'),source='keyboard')
                if c.get('action')=='worker_pose': self.worker.pose_command(bool(c.get('raised')))
                if c.get('action')=='worker_reset': self.worker.reset()
                if c.get('action')=='worker_detail_preview' and not self.busy:
                    from worker_validation import record_detail
                    asyncio.ensure_future(record_detail(self.worker))
            except Exception: print(traceback.format_exc())
        if self.busy: return
        dt = min(max(float(event.payload.get('dt',1/60)),0),.05)
        self.interfaces.update(dt)
        self.worker.update(dt)
        if self.running:
            self.physics_accumulator += dt
            while self.physics_accumulator >= 1/60:
                self.step(1/60)
                self.physics_accumulator -= 1/60
            self.orbit_time = min(20,self.orbit_time+dt)
            if self.orbit_enabled and not self.nav_enabled:
                self.set_orbit_frame(min(599,self.orbit_time/20*599))
        if self.nav_enabled:
            self.navigate(dt)
        if self.grip:
            movement = Gf.Vec3d(int('L' in self.keys)-int('J' in self.keys), int('I' in self.keys)-int('K' in self.keys), int('U' in self.keys)-int('O' in self.keys))*100*dt
            self.hand_pos += movement
            self.move_hand(self.hand_pos)

    def on_key(self, event):
        name = getattr(event.input,'name',str(event.input)).split('.')[-1]
        event_type = getattr(event.type,'name',str(event.type)).split('.')[-1]
        worker_keys={'UP':'forward','DOWN':'back','LEFT':'left','RIGHT':'right'}
        if name in worker_keys and event_type=='KEY_RELEASE':
            self.worker.release(worker_keys[name],source='keyboard')
        if name in worker_keys and not self.nav_enabled:
            if event_type=='KEY_PRESS':self.worker.press(worker_keys[name],source='keyboard')
            return True
        if event_type == 'KEY_PRESS':
            self.keys.add(name)
            if name == 'T' and self.nav_enabled: self.teleport()
            if name == 'G': self.toggle_grip()
        elif event_type == 'KEY_RELEASE':
            self.keys.discard(name)
        return True

    async def capture_preview(self):
        await asyncio.sleep(3)
        await self.capture_image(EVIDENCE/'current_view.png')
        self.diagnose('current_render_state.json')

    def forward(self):
        y,p = math.radians(self.yaw), math.radians(self.pitch)
        return Gf.Vec3d(math.cos(p)*math.cos(y),math.cos(p)*math.sin(y),math.sin(p))

    def navigate(self, dt):
        self.yaw += (int('LEFT' in self.keys)-int('RIGHT' in self.keys))*60*dt
        self.pitch = max(-85,min(85,self.pitch+(int('UP' in self.keys)-int('DOWN' in self.keys))*60*dt))
        f = self.forward()
        right = Gf.Cross(f,Gf.Vec3d(0,0,1)).GetNormalized()
        self.eye += (f*(int('W' in self.keys)-int('S' in self.keys))+right*(int('D' in self.keys)-int('A' in self.keys))+Gf.Vec3d(0,0,int('E' in self.keys)-int('Q' in self.keys)))*250*dt
        self.camera_op.Set(Gf.Matrix4d().SetLookAt(self.eye,self.eye+f,Gf.Vec3d(0,0,1)).GetInverse())

    def toggle_nav(self):
        self.worker.stop()
        self.nav_enabled = not self.nav_enabled
        direction = (self.center-self.eye).GetNormalized()
        self.yaw = math.degrees(math.atan2(direction[1],direction[0]))
        self.pitch = math.degrees(math.asin(direction[2]))
        self.message('Navigation '+('ON: WASD, QE, arrows; T = 2 m teleport.' if self.nav_enabled else 'OFF.'))

    def teleport(self):
        self.eye += self.forward()*200
        self.navigate(0)

    def toggle_grip(self):
        if self.recording: return
        self.lift.probe_docked = False
        self.grip = not self.grip
        body = UsdPhysics.RigidBodyAPI(self.probe.GetPrim())
        body.CreateKinematicEnabledAttr(self.grip)
        if self.grip:
            self.hand_pos = Gf.Vec3d(self.probe_op.Get())
            self.hand_rows = []
            self.hand.MakeVisible()
            self.move_hand(self.hand_pos)
            self.message('Desktop proxy holds ORANGE TEST PROBE. IJKL/UO move; G releases.')
        else:
            # Recreate only the diagnostic body, clearing target-derived speed
            # without detaching the application's stage/fabric. No throwing.
            self.recreate_probe(Gf.Vec3d(self.probe_op.Get()))
            self.hand.MakeInvisible()
            self.write_hand_log()
            self.message('Released; gravity resumes when Play is active.')

    def move_hand(self, position):
        self.hand_pos = Gf.Vec3d(position)
        self.hand_op.Set(self.hand_pos)
        if self.grip: self.probe_op.Set(self.hand_pos)
        hits=[]
        def hit(h):
            p=str(h.collision)
            if 'MCWP_' in p: hits.append(p)
            return True
        self.query.overlap_sphere(10,carb.Float3(*self.hand_pos),hit,False)
        contact=bool(hits)
        self.hand_rows.append([self.time,*self.hand_pos,0,0,0,1,int(contact)])
        if contact and not self.last_contact:
            self.events.append({'time':self.time,'kind':'proxy_frame_contact','paths':hits})
            self.message('CONTACT: desktop probe touches MCWP geometry.')
        self.last_contact=contact
        return hits

    def write_hand_log(self):
        path=ROOT/'recordings'/('DesktopHand_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'.csv')
        with open(path,'w',newline='') as f:
            writer=csv.writer(f); writer.writerow(['time_s','x_cm','y_cm','z_cm','qx','qy','qz','qw','frame_contact']); writer.writerows(self.hand_rows)
        return str(path)

    def build_ui(self):
        self.window=ui.Window('MCWP Simulation Controls',width=420,height=530,position_x=950,position_y=100)
        with self.window.frame:
            with ui.VStack(spacing=6):
                ui.Label('MCWP | UE 4.27 behavior reconstruction',height=24)
                ui.Label('Kinematic lift + carried worker / CPU PhysX / cm, Z up',height=20)
                with ui.HStack(height=30):
                    ui.Button('Play / Pause',clicked_fn=self.toggle_play)
                    ui.Button('Reset',clicked_fn=self.reset)
                    ui.Button('Camera navigation',clicked_fn=self.toggle_nav)
                ui.Label('WASD / Q E move, arrows look, T teleports 2 m.',height=20)
                ui.Button('Record 20 s orbit (600 frames, 1080p AVI)',height=30,clicked_fn=lambda:asyncio.ensure_future(self.record_orbit()))
                ui.Button('G: Grip / release orange validation probe',height=30,clicked_fn=self.toggle_grip)
                ui.Label('I J K L / U O move the desktop hand proxy.\nReset returns the orange sphere to the platform.\nProbe / proxy are diagnostics; no tracked VR.',height=52,word_wrap=True)
                ui.Button('Run functional validation',height=28,clicked_fn=lambda:asyncio.ensure_future(self.verify()))
                ui.Button('Replay original UE controller log (30 samples/s)',height=28,clicked_fn=self.interfaces.toggle_replay)
                ui.Button('Send ROS RGB image (requires configured server)',height=28,clicked_fn=lambda:asyncio.ensure_future(self.interfaces.send_ros_image()))
                ui.Button('Validate recorded poses + local image protocol',height=28,clicked_fn=lambda:asyncio.ensure_future(self.interfaces.verify()))
                ui.Button('Validate depth / TF adapters',height=28,clicked_fn=lambda:asyncio.ensure_future(self.verify_depth_tf()))
                self.status=ui.Label('Starting...',word_wrap=True,height=58)
                ui.Label('Kinematic lift; real actuator / live ROS / VR not validated.',height=20)

    async def record_orbit(self):
        if self.busy: return
        self.reset()
        self.busy=self.recording=True
        self.configure_viewport()
        await self.apply_capture_exposure()
        self.probe.MakeInvisible()
        stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        output=ROOT/'recordings'/('MCWP_Orbit_'+stamp+'.avi')
        frames, rows = [],[]
        try:
            for _ in range(120): await omni.kit.app.get_app().next_update_async()
            for i in range(600):
                temp=ROOT/'recordings'/f'capture_{i:04}.png'
                eye=self.set_orbit_frame(i)
                for _ in range(3): await omni.kit.app.get_app().next_update_async()
                img=await self.capture_image(temp)
                stream=io.BytesIO(); img.save(stream,format='JPEG',quality=90); frames.append(stream.getvalue())
                view=Gf.Matrix4d().SetLookAt(eye,self.center,Gf.Vec3d(0,0,1))
                v=[view.Transform(c) for c in self.corners]
                tan_h=math.tan(math.radians(30))
                tan_v=tan_h/(16/9)
                corner=max(max(abs(c[0]/(-c[2]*tan_h)),abs(c[1]/(-c[2]*tan_v))) for c in v)
                rows.append([i,i/30,i*360/599,*eye,corner])
                if i in [0,150,300,450,599]:
                    img.save(EVIDENCE/f'orbit_frame_{i:04}.png')
                if i%60==0: self.message(f'Recording frame {i}/600')
            write_mjpeg_avi(output,frames)
            with open(output.with_suffix('.csv'),'w',newline='') as f:
                w=csv.writer(f);w.writerow(['frame','time_seconds','angle_degrees','camera_x_cm','camera_y_cm','camera_z_cm','max_corner_normalized']);w.writerows(rows)
            save_json('recording_validation.json',{'complete':True,'path':str(output),'frames':len(frames),'fps':30,'duration_seconds':20,'resolution':[1920,1080],'max_corner_normalized':max(r[-1] for r in rows),'full_model_in_frame':max(r[-1] for r in rows)<1,'note':'Camera orbit at the lower lift stop. Lift sweep video is generated by Verify lift.'})
            self.message('Saved 20 s AVI and camera CSV: '+output.name)
        except Exception:
            save_json('recording_error.json',{'error':traceback.format_exc()});self.message('Recording failed. See evidence/recording_error.json.')
        finally:
            self.recording=self.busy=False
            self.probe.MakeVisible()
            self.set_orbit_frame(0)
            self.restore_preview_exposure()
            self.diagnose('post_physics_render_state.json')

    async def verify(self):
        if self.busy: return
        self.reset();self.busy=True
        self.lift.probe_docked=False
        self.recreate_probe(Gf.Vec3d(250,0,350))
        results={'tests':{},'limitations':['VR headset tracking not tested','ROS bridge not present','No powered lift joint or worker skeletal animation in active source']}
        try:
            self.message('Verifying PhysX falling body and source-mesh contact...')
            before=Gf.Vec3d(self.probe_op.Get())
            trajectory=[]
            for i in range(240):
                self.step()
                trajectory.append([i,self.time,*self.probe_op.Get()])
                if i%4==0: await omni.kit.app.get_app().next_update_async()
            after=Gf.Vec3d(self.probe_op.Get())
            ray=self.query.raycast_closest(carb.Float3(250,30,340),carb.Float3(0,0,-1),500)
            results['tests']['gravity_contact']={'initial_cm':list(before),'final_cm':list(after),'fall_cm':before[2]-after[2],'settling_range_last_30_cm':max(r[-1] for r in trajectory[-30:])-min(r[-1] for r in trajectory[-30:]),'raycast':{k:str(v) for k,v in ray.items()},'passed':before[2]-after[2]>50 and after[2]>50 and max(r[-1] for r in trajectory[-30:])-min(r[-1] for r in trajectory[-30:])<.1}
            with open(EVIDENCE/'physics_trajectory.csv','w',newline='') as f:
                w=csv.writer(f);w.writerow(['step','time_s','x_cm','y_cm','z_cm']);w.writerows(trajectory)
            p=self.stage.GetPrimAtPath('/Root/MCWP_Platform')
            body=UsdPhysics.RigidBodyAPI(self.stage.GetPrimAtPath(CARRIAGE))
            results['tests']['lift_structure']={'fixed_has_rigid_body':self.stage.GetPrimAtPath('/Root/MCWP_Platform/Fixed').HasAPI(UsdPhysics.RigidBodyAPI),'carriage_is_kinematic':body.GetKinematicEnabledAttr().Get(),'passed':bool(body.GetKinematicEnabledAttr().Get())}
            self.toggle_nav(); self.keys={'W'}; start=Gf.Vec3d(self.eye);self.navigate(1);self.keys=set();dist=(self.eye-start).GetLength();start=Gf.Vec3d(self.eye);self.teleport();tele=(self.eye-start).GetLength();self.toggle_nav()
            results['tests']['navigation']={'one_second_move_cm':dist,'teleport_cm':tele,'passed':abs(dist-250)<.001 and abs(tele-200)<.001}
            self.toggle_grip(); self.move_hand(Gf.Vec3d(150,0,145)); self.step(); contacts=self.move_hand(Gf.Vec3d(150,0,145))
            results['tests']['frame_contact_query']={'hit_paths':contacts,'passed':any(path.startswith(CARRIAGE) for path in contacts),'scope':'desktop diagnostic sphere against actual platform triangles'}
            self.move_hand(Gf.Vec3d(250,0,300)); self.step(); held=Gf.Vec3d(self.probe_op.Get()); self.toggle_grip()
            for i in range(180): self.step()
            released=Gf.Vec3d(self.probe_op.Get())
            results['tests']['desktop_proxy_grip_release']={'held_cm':list(held),'released_cm':list(released),'passed':held[2]-released[2]>50,'scope':'additional probe only; not tracked VR'}
            self.set_orbit_frame(0)
            await self.apply_capture_exposure()
            for _ in range(30): await omni.kit.app.get_app().next_update_async()
            self.diagnose('post_physics_render_state.json')
            await capture_viewport_to_file(self.viewport,(EVIDENCE/'physics_contact_verified.png').as_posix()).wait_for_result()
            results['all_implemented_checks_passed']=all(t.get('passed',False) for t in results['tests'].values())
            save_json('functional_validation.json',results)
            self.message('Functional checks complete: '+str(results['all_implemented_checks_passed']))
        except Exception:
            results['error']=traceback.format_exc(); save_json('functional_validation.json',results);self.message('Validation error; see evidence report.');print(results['error'])
        finally:
            self.busy=False
            self.reset()
            self.restore_preview_exposure()
        if os.environ.get('MCWP_RECORD')=='1': await self.record_orbit()

runtime=MCWPSimulation()
async def start():
    try: await runtime.initialize()
    except Exception:
        save_json('startup_error.json',{'error':traceback.format_exc()});print(traceback.format_exc())
asyncio.ensure_future(start())
