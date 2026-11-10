"""Rigged worker integration for the MCWP Kit application."""
import asyncio,json,math,time,uuid
from pathlib import Path
import numpy as np
import omni.ui as ui
import omni.kit.app
from pxr import Usd,UsdGeom,UsdSkel,Gf,Vt,Sdf
from worker_motion import WorkerMotion
from worker_navigation import WorkerNavigation

ROOT=Path(__file__).resolve().parents[1]
WORKER='/Root/MCWP_Platform/Carriage/Payloads/Worker'

class WorkerController:
    def __init__(self,runtime):
        self.r=runtime;self.mouse_held=set();self.keyboard_held=set();self.buttons={};self.status=None
        self.events=[];self.accumulator=0.;self.enabled=True;self.focus_camera=False
        self.bind(runtime.stage,first=True)
        self.apply()

    def bind(self,stage,first=False):
        self.stage=stage
        self.root=UsdSkel.Root.Get(stage,WORKER)
        self.skeleton=UsdSkel.Skeleton.Get(stage,WORKER+'/Skeleton')
        self.mesh=UsdGeom.Mesh.Get(stage,WORKER+'/Geometry')
        if first:
            names=list(self.skeleton.GetJointNamesAttr().Get())
            parents=list(UsdSkel.Topology(self.skeleton.GetJointsAttr().Get()).GetParentIndices())
            rest=np.array(self.skeleton.GetBindTransformsAttr().Get()).transpose(0,2,1)
            self.motion=WorkerMotion(names,parents,rest)
            self.points=np.array(self.mesh.GetPointsAttr().Get())
            binding=UsdSkel.BindingAPI(self.mesh.GetPrim())
            self.indices=np.array(binding.GetJointIndicesPrimvar().Get()).reshape(-1,4)
            self.weights=np.array(binding.GetJointWeightsPrimvar().Get()).reshape(-1,4)
            self.navigation=WorkerNavigation(ROOT/'assets/worker/walk_navigation.json',self.motion,self.points,self.indices,self.weights,np.array(self.mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1,3))
            self.motion.navigation=self.navigation
        self.pose=UsdSkel.Animation.Define(stage,WORKER+'/ControlPose')
        self.pose.CreateJointsAttr(self.skeleton.GetJointsAttr().Get())
        self.tr=self.pose.CreateTranslationsAttr();self.rot=self.pose.CreateRotationsAttr();self.sc=self.pose.CreateScalesAttr()
        UsdSkel.BindingAPI.Apply(self.skeleton.GetPrim()).CreateAnimationSourceRel().SetTargets([self.pose.GetPath()])
        self.root.GetPrim().SetCustomDataByKey('controller','MCWP planted-foot procedural humanoid; source mesh, skin and 23-joint rig retained')
        self.translation=self.root.GetPrim().GetAttribute('xformOp:translate')
        self.heading=self.root.GetPrim().GetAttribute('xformOp:rotateZ')
        self.cache=UsdSkel.Cache();self.cache.Populate(self.root,Usd.PrimDefaultPredicate)
        self.query=self.cache.GetSkelQuery(self.skeleton);self.skin_query=self.cache.GetSkinningQuery(self.mesh.GetPrim())

    def apply(self):
        local=self.motion.local_pose();trs=[];rots=[];sc=[]
        for m in local:
            xf=Gf.Transform(Gf.Matrix4d(*m.T.reshape(-1).tolist()))
            trs.append(Gf.Vec3f(xf.GetTranslation()));rots.append(Gf.Quatf(xf.GetRotation().GetQuat()));sc.append(Gf.Vec3h(xf.GetScale()))
        self.tr.Set(Vt.Vec3fArray(trs));self.rot.Set(Vt.QuatfArray(rots));self.sc.Set(Vt.Vec3hArray(sc))
        self.translation.Set(Gf.Vec3d(float(self.motion.xy[0]*100),float(self.motion.xy[1]*100),.35))
        self.heading.Set(float(math.degrees(self.motion.yaw)))
        p=self.motion.skin(self.points,self.indices,self.weights)
        self.mesh.GetExtentAttr().Clear();self.mesh.GetExtentAttr().Set([Gf.Vec3f(*p.min(0)),Gf.Vec3f(*p.max(0))])
        if self.status:
            state='Boundary / obstacle stop' if self.motion.blocked else ('Moving' if self.motion.held else 'Stopped')
            edge=self.navigation.edge_clearance(self.motion.feet)
            pose='Pose limited: obstacle' if self.motion.pose_limited else ('Walking arm swing' if self.motion.walk_blend>.5 else f'Arms {self.motion.pose_amount*100:.0f}%')
            self.status.text=f'{state} | X {self.motion.xy[0]:+.2f}  Y {self.motion.xy[1]:+.2f} m\n{pose} | Shoe-to-edge {edge:.2f} m'

    def step(self,dt):
        if getattr(self.r,'probe_op',None):
            p=np.array(self.r.probe_op.Get(),dtype=float)
            self.navigation.probe=(p-np.array([50,0,141.25+self.r.lift.height]))/100
        self.motion.step(dt);self.apply()

    def update(self,dt):
        self.accumulator+=dt
        while self.accumulator>=1/60:
            self.step(1/60);self.accumulator-=1/60
        if self.focus_camera:self.update_camera()

    def update_camera(self):
        center=Gf.Vec3d(50+float(self.motion.xy[0]*100),float(self.motion.xy[1]*100),141.25+self.r.lift.height+90)
        self.r.eye=center+Gf.Vec3d(310,-440,175)
        self.r.camera_op.Set(Gf.Matrix4d().SetLookAt(self.r.eye,center,Gf.Vec3d(0,0,1)).GetInverse())

    def view(self,close):
        self.focus_camera=bool(close);self.r.nav_enabled=False
        if close:self.update_camera()
        else:self.r.set_orbit_frame(150)

    def record(self,event,**extra):
        self.events.append({'monotonic_s':time.monotonic(),'event':event,'xy_m':self.motion.xy.tolist(),'yaw_deg':math.degrees(self.motion.yaw),**extra})
        (ROOT/'evidence/worker_input_events.json').write_text(json.dumps(self.events[-300:],indent=2))

    def press(self,key,source='mouse'):
        (self.mouse_held if source=='mouse' else self.keyboard_held).add(key)
        self.motion.press(key);self.record('press',key=key,source=source)

    def release(self,key=None,source='mouse'):
        held=self.mouse_held if source=='mouse' else self.keyboard_held
        if key is None:held.clear()
        else:held.discard(key)
        active=self.mouse_held|self.keyboard_held
        for action in ['forward','back','left','right']:
            if action not in active:self.motion.release(action)
        self.record('release',key=key,source=source)

    def stop(self):
        self.mouse_held.clear();self.keyboard_held.clear();self.motion.release()
        self.record('stop')

    def reset(self):
        self.stop();self.motion.reset();self.apply();self.focus_camera=False

    def pose_command(self,raised):
        self.motion.set_pose(raised);self.record('pose',raised=bool(raised))

    def on_mouse(self,event):
        event_type=getattr(event.type,'name',str(event.type)).split('.')[-1]
        if 'RELEASE' in event_type and self.mouse_held:self.release()
        return True

    def on_focus(self,event):
        # Any focus transition clears held input; no movement remains latched.
        self.stop()

    def build_ui(self):
        self.window=ui.Window('Worker Remote',width=340,height=286,position_x=10,position_y=390)
        def button(label,key):
            b=ui.Button(label,height=35,mouse_pressed_fn=lambda x,y,b,m:self.press(key) if b==0 else None,
                        mouse_released_fn=lambda x,y,b,m:self.release(key) if b==0 else None)
            self.buttons[key]=b
        with self.window.frame:
            with ui.VStack(spacing=5):
                ui.Label('HOLD to move / turn. RELEASE to stop.',height=20)
                self.status=ui.Label('',height=36)
                with ui.HStack(height=35):
                    ui.Spacer();button('FORWARD','forward');ui.Spacer()
                with ui.HStack(height=35):
                    button('TURN LEFT','left');ui.Button('STOP',clicked_fn=self.stop);button('TURN RIGHT','right')
                with ui.HStack(height=35):
                    ui.Spacer();button('BACK','back');ui.Spacer()
                with ui.HStack(height=32):
                    ui.Button('Arms down',clicked_fn=lambda:self.pose_command(False))
                    ui.Button('Arms forward / up',clicked_fn=lambda:self.pose_command(True))
                with ui.HStack(height=27):
                    ui.Button('Reset worker',clicked_fn=self.reset)
                    ui.Button('Close view',clicked_fn=lambda:self.view(True))
                    ui.Button('Full view',clicked_fn=lambda:self.view(False))
                    ui.Button('Verify',clicked_fn=lambda:asyncio.ensure_future(self.verify()))
        self.apply()

    def evaluate_points(self):
        p=Vt.Vec3fArray(self.mesh.GetPointsAttr().Get())
        ok=self.skin_query.ComputeSkinnedPoints(self.query.ComputeSkinningTransforms(),p)
        if not ok:raise RuntimeError('Native UsdSkel deformation failed')
        return np.array(p)

    async def verify(self):
        from worker_full_validation import verify
        return await verify(self)
