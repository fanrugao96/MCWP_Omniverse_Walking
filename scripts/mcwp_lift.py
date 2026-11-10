"""Phase 1: bounded kinematic lift; no force-drive or walking simulation."""
import asyncio, csv, io, json, math, traceback, uuid
from pathlib import Path
import omni.kit.app, omni.ui as ui
from pxr import Usd, UsdGeom, UsdPhysics, UsdSkel, Sdf, Gf

ROOT=Path(__file__).resolve().parents[1]
PLATFORM='/Root/MCWP_Platform'
CARRIAGE=PLATFORM+'/Carriage'
WORKER=CARRIAGE+'/Payloads/Worker'
BASE_OFFSET=20.0
MAX_HEIGHT=160.0
SPEED=8.0
ACCELERATION=20.0

class LiftController:
    def __init__(self,runtime):
        self.r=runtime
        self.height=0.0
        self.target=0.0
        self.velocity=0.0
        self.probe_docked=True
        self.height_label=None
        self.slider_model=None
        self._ui_guard=False

    def build(self,stage):
        original=stage.GetPrimAtPath(PLATFORM)
        original.GetReferences().SetReferences([Sdf.Reference((ROOT/'assets/lift/Platform_Lift.usda').as_posix())])
        original.SetTypeName('Xform')
        stage.GetPrimAtPath('/Root/MCWP_Worker').SetActive(False)
        worker=UsdSkel.Root.Define(stage,WORKER)
        worker.GetPrim().GetReferences().AddReference((ROOT/'assets/worker/Construction_Worker_Rigged.usdc').as_posix())
        worker.AddTranslateOp().Set(Gf.Vec3d(0,0,.35))
        worker.AddRotateZOp().Set(90.)
        worker.AddScaleOp().Set(Gf.Vec3f(100.))
        carriage=stage.GetPrimAtPath(CARRIAGE)
        body=UsdPhysics.RigidBodyAPI.Apply(carriage)
        body.CreateKinematicEnabledAttr(True)
        UsdGeom.Xformable(carriage).AddTranslateOp().Set(Gf.Vec3d(0,0,BASE_OFFSET))
        carriage.CreateAttribute('mcwp:minimumHeightCm',Sdf.ValueTypeNames.Double).Set(0.)
        carriage.CreateAttribute('mcwp:maximumHeightCm',Sdf.ValueTypeNames.Double).Set(MAX_HEIGHT)
        carriage.CreateAttribute('mcwp:mode',Sdf.ValueTypeNames.String).Set('kinematic demonstration; uncalibrated actuator')
        self.bind(stage)

    def bind(self,stage):
        self.stage=stage
        self.op=stage.GetPrimAtPath(CARRIAGE).GetAttribute('xformOp:translate')
        self.height=float(self.op.Get()[2])-BASE_OFFSET

    def rebase_assets(self):
        self.stage.GetPrimAtPath(PLATFORM).GetReferences().SetReferences([Sdf.Reference('./assets/lift/Platform_Lift.usda')])
        self.stage.GetPrimAtPath(WORKER).GetReferences().SetReferences([Sdf.Reference('./assets/worker/Construction_Worker_Rigged.usdc')])

    def sync_probe(self):
        if self.probe_docked and getattr(self.r,'probe',None):
            # This diagnostic sphere is explicitly attached as a passenger until
            # G grips/releases it. Released mode retains the original PhysX test.
            UsdPhysics.RigidBodyAPI(self.r.probe.GetPrim()).CreateKinematicEnabledAttr(True)
            self.r.probe_op.Set(Gf.Vec3d(150,0,151.25+self.height))

    def set_height(self,value):
        self.height=max(0.0,min(MAX_HEIGHT,float(value)))
        self.op.Set(Gf.Vec3d(0,0,BASE_OFFSET+self.height))
        self.sync_probe()
        self.update_ui()

    def command(self,target):
        self.target=max(0.0,min(MAX_HEIGHT,float(target)))
        self.r.running=True

    def stop(self):
        self.target=self.height;self.velocity=0.0
        self.update_ui()

    def reset(self):
        self.probe_docked=True
        self.target=0.;self.velocity=0.;self.set_height(0.)

    def step(self,dt):
        distance=self.target-self.height
        if abs(distance)<1e-6:
            self.velocity=0.;self.sync_probe();return
        direction=1 if distance>0 else -1
        desired=direction*min(SPEED,math.sqrt(2*ACCELERATION*abs(distance)))
        delta=max(-ACCELERATION*dt,min(ACCELERATION*dt,desired-self.velocity))
        self.velocity+=delta
        move=self.velocity*dt
        if direction*move>abs(distance):move=distance;self.velocity=0.
        self.set_height(self.height+move)

    def update_ui(self):
        if self.height_label:
            state=('MOVING' if self.r.running else 'PAUSED') if abs(self.target-self.height)>.001 else 'STOPPED'
            self.height_label.text=f'{state} | Lift {self.height/100:.2f} m / {MAX_HEIGHT/100:.2f} m\nDeck height {(141.25+self.height)/100:.2f} m above ground'

    def build_ui(self):
        self.window=ui.Window('MCWP Lift',width=340,height=280,position_x=10,position_y=100)
        with self.window.frame:
            with ui.VStack(spacing=7):
                ui.Label('Platform lift | fixed base and masts',height=24)
                self.height_label=ui.Label('',height=44)
                with ui.HStack(height=30):
                    ui.Button('Raise',clicked_fn=lambda:self.command(MAX_HEIGHT))
                    ui.Button('Stop',clicked_fn=self.stop)
                    ui.Button('Lower',clicked_fn=lambda:self.command(0))
                ui.Label('Travel: 0.00 - 1.60 m | speed: 0.08 m/s',height=20)
                ui.Label('Worker and platform equipment move together.\nG grips/releases the orange diagnostic sphere.',height=38)
                ui.Button('Verify lift and capture evidence',height=28,clicked_fn=lambda:asyncio.ensure_future(self.verify()))
        self.update_ui()

    def snapshot(self):
        cache=UsdGeom.XformCache()
        paths=[p for p in self.stage.Traverse() if p.IsA(UsdGeom.Mesh) and str(p.GetPath()).startswith(PLATFORM)]
        result={}
        for p in paths:
            x=cache.GetLocalToWorldTransform(p)
            result[str(p.GetPath())]=[list(x[i]) for i in range(4)]
        result['/Simulation/ValidationProbe']=[list(cache.GetLocalToWorldTransform(self.r.probe.GetPrim())[i]) for i in range(4)]
        return result

    async def verify(self):
        if self.r.busy:return
        self.r.reset();self.r.busy=True;self.r.running=False
        report={'mode':'kinematic bounded demonstration, not calibrated force/drive simulation',
                'checks':{},'poses':[],'limits_cm':[0,MAX_HEIGHT], 'deck_height_cm':[141.25,301.25],
                'limitations':['Lift check freezes the rigged worker; use Worker Remote Verify for walking and pose tests.',
                               'No actuator, load capacity, friction calibration, mast structural strength, or safety certification.',
                               'Future loose payloads must be registered or independently validated; current orange sphere uses attached passenger mode until released.']}
        try:
            self.r.message('Checking fixed structures, all passengers, limits and deck collisions...')
            geometry=json.loads((ROOT/'assets/lift/lift_geometry_validation.json').read_text())
            report['checks']['continuous_geometry_clearance']={'passed':not geometry['swept_component_aabb_overlaps'],
                'minimum_conservative_gap_cm':geometry['conservative_min_separation_cm'],
                'method':geometry['method'],'components':geometry['connected_components']}
            worker_clearance=json.loads((ROOT/'evidence/worker_envelope_validation.json').read_text())
            report['checks']['payload_clearance']={'passed':worker_clearance['passed'] and geometry['payload_clearance']['docked_sphere']['passed'],'worker_envelope':worker_clearance,'docked_sphere':geometry['payload_clearance']['docked_sphere']}
            self.set_height(0);base=self.snapshot()
            for height in [0,40,80,120,160,120,80,40,0]:
                self.target=height;self.set_height(height)
                for _ in range(12):self.r.step(1/60)
                now=self.snapshot();fixed_err=0.;moving_err=0.;payload_count=0;fixed_count=0
                for path,m in now.items():
                    expected=[row[:] for row in base[path]]
                    moving=path.startswith(CARRIAGE) or path=='/Simulation/ValidationProbe'
                    if moving:expected[3][2]+=height;payload_count+=1
                    else:fixed_count+=1
                    error=max(abs(m[i][j]-expected[i][j]) for i in range(4) for j in range(4))
                    if moving:moving_err=max(moving_err,error)
                    else:fixed_err=max(fixed_err,error)
                bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render'])
                wb=bbox.ComputeWorldBound(self.stage.GetPrimAtPath(WORKER)).ComputeAlignedRange()
                deck=141.25+height
                feet=deck+float(self.r.worker.evaluate_points()[:,2].min())*100
                expected_sole_gap=self.r.worker.motion.FLOOR*100
                # Clear deck location used by the original diagnostic sphere.
                import carb
                ray=self.r.query.raycast_closest(carb.Float3(50,70,deck+80),carb.Float3(0,0,-1),150)
                ray_z=float(ray['position'][2]) if ray.get('hit') else None
                pose={'height_cm':height,'fixed_meshes':fixed_count,'moving_meshes_and_probe':payload_count,
                      'fixed_transform_error_cm':fixed_err,'passenger_transform_error_cm':moving_err,
                      'worker_feet_z_cm':feet,'worker_deck_gap_cm':feet-deck,
                      'deck_raycast_z_cm':ray_z,'deck_raycast_collider':str(ray.get('collision','')),
                      'passed':fixed_err<1e-5 and moving_err<1e-4 and abs(feet-deck-expected_sole_gap)<.01 and ray_z is not None and abs(ray_z-deck)<.36}
                report['poses'].append(pose)
                if height in [0,40,80,120,160] and len(report['poses'])<=5:
                    self.r.set_orbit_frame(150)
                    for _ in range(8):await omni.kit.app.get_app().next_update_async()
                    await self.r.capture_image(ROOT/f'evidence/lift_{height:03d}cm.png')
            report['checks']['sampled_poses']={'passed':all(p['passed'] for p in report['poses'])}
            self.set_height(-100);low=self.height;self.set_height(1000);high=self.height
            report['checks']['hard_clamps']={'input_cm':[-100,1000],'output_cm':[low,high],'passed':low==0 and high==MAX_HEIGHT}
            self.set_height(0);self.target=MAX_HEIGHT;self.velocity=0.;trajectory=[]
            for i in range(1500):
                self.r.step(1/60)
                if i%10==0:trajectory.append([i/60,self.height,self.velocity,float(self.r.probe_op.Get()[2])])
                if i%120==0:await omni.kit.app.get_app().next_update_async()
            up=self.height;self.target=0
            for i in range(1500):
                self.r.step(1/60)
                if i%10==0:trajectory.append([25+i/60,self.height,self.velocity,float(self.r.probe_op.Get()[2])])
                if i%120==0:await omni.kit.app.get_app().next_update_async()
            down=self.height
            report['checks']['continuous_up_down']={'passed':abs(up-MAX_HEIGHT)<1e-5 and abs(down)<1e-5 and all(0<=row[1]<=MAX_HEIGHT for row in trajectory),
                'upper_reached_cm':up,'lower_reached_cm':down,'physics_steps':3000,'max_speed_cm_s':max(abs(row[2]) for row in trajectory),
                'passenger_offset_error_cm':max(abs(row[3]-(151.25+row[1])) for row in trajectory)}
            report['checks']['continuous_up_down']['passed'] &= report['checks']['continuous_up_down']['passenger_offset_error_cm']<1e-4
            with open(ROOT/'evidence/lift_trajectory.csv','w',newline='') as f:
                w=csv.writer(f);w.writerow(['time_s','lift_cm','velocity_cm_s','probe_z_cm']);w.writerows(trajectory)
            frames=[];self.r.set_orbit_frame(150)
            frames_dir=ROOT/'evidence/lift_preview_frames'/uuid.uuid4().hex
            frames_dir.mkdir(parents=True)
            sweep=list(range(0,161,4))+list(range(156,-1,-4))
            for index,height in enumerate(sweep):
                self.target=height;self.set_height(height);self.r.step(1/60)
                for _ in range(2):await omni.kit.app.get_app().next_update_async()
                img=await self.r.capture_image(frames_dir/f'{index:04d}.png')
                stream=io.BytesIO();img.save(stream,format='JPEG',quality=88);frames.append(stream.getvalue())
                if index%20==0:self.r.message(f'Capturing lift preview {index+1}/{len(sweep)}')
            self.r.write_video(ROOT/'evidence/lift_preview.avi',frames,fps=10)
            report['preview']={'file':'evidence/lift_preview.avi','frames':len(frames),'fps':10,'duration_seconds':len(frames)/10,
                'note':'Time-compressed height sweep of actual Kit renders; real interactive lift speed is 0.08 m/s.'}
            report['all_passed']=all(c['passed'] for c in report['checks'].values())
            self.r.message('Lift checks complete: '+str(report['all_passed']))
        except Exception:
            report['error']=traceback.format_exc();report['all_passed']=False;print(report['error'])
        finally:
            (ROOT/'evidence/lift_runtime_validation.json').write_text(json.dumps(report,indent=2))
            self.r.busy=False;self.r.reset();self.r.set_orbit_frame(150)
        return report
