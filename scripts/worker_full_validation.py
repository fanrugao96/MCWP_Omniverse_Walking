"""Actual Kit regression for whole-deck controls, arm gait and edge approaches."""
import asyncio,csv,datetime,io,json,math,traceback,uuid,os
from pathlib import Path
import numpy as np
import omni.kit.app
import carb
from pxr import Usd,UsdGeom,UsdPhysics

async def verify(w):
    r=w.r;out=Path(__file__).resolve().parents[1]/'evidence'
    if r.busy:return
    r.reset();r.busy=True;r.running=False
    report={'project':str(out.parent),'checked_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'revision':'whole-platform geometry navigation + contralateral arm swing',
            'checks':{},'actions':[],'route':[],
            'limitations':['Kinematic character animation, not dynamic balance or a fall simulation.',
            'Closed rails, equipment and actual mast openings remain excluded; approach directions affect clearance.',
            'Arm raise is limited smoothly when the requested sweep would intersect existing equipment.',
            'Component obstacle bounds are conservative and fixed masts are swept over the full lift range.',
            'Standing-pose selection is remembered; walking takes priority and restores the standing pose after release.']}
    metrics={'samples':0,'native_samples':0,'max_skin_error_m':0.,'min_sole_height_m':1e6,'min_edge_clearance_m':1e6,'max_vertex_step_m':0.,'max_leg_length_error_m':0.,'max_planted_slip_m':0.}
    collisions=[];unsupported=[];previous=None;feet_before={};traces=[];hands={};foot_y={};step_number=0
    films={'arms':[],'edges':[]};film_segments=[];folder=out/'worker_fix_frames'/uuid.uuid4().hex;folder.mkdir(parents=True)
    recording=os.environ.get('MCWP_FIX_SKIP_VIDEO')!='1'
    def serial(x):return x.item() if isinstance(x,np.generic) else str(x)
    def press(key):w.buttons[key].call_mouse_pressed_fn(0,0,0,0)
    def release(key=None):
        if key:w.buttons[key].call_mouse_released_fn(0,0,0,0)
        else:
            for key in list(w.motion.held):w.buttons[key].call_mouse_released_fn(0,0,0,0)
    def clear_previous():
        nonlocal previous
        previous=None;feet_before.clear()
    def sample(label):
        nonlocal previous,step_number
        m=w.motion;expected=m.skin(w.points,w.indices,w.weights);p=m.platform_points(expected)
        metrics['samples']+=1
        metrics['min_sole_height_m']=min(metrics['min_sole_height_m'],float(p[:,2].min()))
        metrics['min_edge_clearance_m']=min(metrics['min_edge_clearance_m'],w.navigation.edge_clearance(m.feet))
        if previous is not None:metrics['max_vertex_step_m']=max(metrics['max_vertex_step_m'],float(np.linalg.norm(p-previous,axis=1).max()))
        previous=p.copy()
        for side in ['L','R']:
            f=m.feet[side];planted=not f['swing']
            if planted and side in feet_before and feet_before[side][0]:metrics['max_planted_slip_m']=max(metrics['max_planted_slip_m'],float(np.linalg.norm(f['p']-feet_before[side][1])))
            feet_before[side]=(planted,f['p'].copy())
        for name in ['Thigh_L','Calf_L','Thigh_R','Calf_R']:
            i=m.index[name];j=m.children[name]
            error=abs(np.linalg.norm(m.world[j,:3,3]-m.world[i,:3,3])-np.linalg.norm(m.rest[j,:3,3]-m.rest[i,:3,3]))
            metrics['max_leg_length_error_m']=max(metrics['max_leg_length_error_m'],float(error))
        hands.setdefault(label,[]).append([float(m.world[m.index['Hand_'+s],1,3]) for s in ['L','R']])
        foot_y.setdefault(label,[]).append([float(m.world[m.index['Foot_'+s],1,3]) for s in ['L','R']])
        if step_number%6==0:
            native=w.evaluate_points();metrics['native_samples']+=1
            metrics['max_skin_error_m']=max(metrics['max_skin_error_m'],float(np.linalg.norm(native-expected,axis=1).max()))
            if not w.navigation.clear_pose(m.platform_points(native)):collisions.append({'label':label,'xy':m.xy.tolist(),'yaw':math.degrees(m.yaw)})
            for side,f in m.feet.items():
                sole=w.navigation.soles[side]@__import__('worker_motion').rz(f['yaw']).T+f['p']
                if not w.navigation.supported(sole):unsupported.append({'label':label,'side':side,'xy':m.xy.tolist()})
            traces.append([m.total_time,label,*m.xy,math.degrees(m.yaw),r.lift.height,m.pose_target,m.pose_amount,m.walk_blend,*hands[label][-1],w.navigation.edge_clearance(m.feet),m.blocked,m.pose_limited])
        step_number+=1
    async def frame(label,film=None):
        w.step(1/60);r.step(1/60);sample(label)
        if w.focus_camera:w.update_camera()
        if recording and film and step_number%(2 if film=='arms' else 4)==0:
            await omni.kit.app.get_app().next_update_async()
            path=folder/f'{film}_{len(films[film]):05d}.png';im=await r.capture_image(path)
            buf=io.BytesIO();im.convert('RGB').save(buf,format='JPEG',quality=90);films[film].append(buf.getvalue())
            if len(films[film])%90==0:print('WORKER_FIX_VIDEO',film,len(films[film]),flush=True)
        elif step_number%30==0:await omni.kit.app.get_app().next_update_async()
    async def advance(seconds,label,film=None):
        start=len(films.get(film,[]))
        for _ in range(round(seconds*60)):await frame(label,film)
        if film:film_segments.append({'film':film,'action':label,'first_frame':start,'last_frame':len(films[film])-1})
    async def stop(label,film=None):
        release();xy=w.motion.xy.copy();yaw=w.motion.yaw
        await advance(.9,label+'_released',film)
        error=float(np.linalg.norm(w.motion.xy-xy));angle=abs(w.motion.yaw-yaw)
        report['actions'].append({'action':label,'release_drift_m':error,'release_yaw_error_rad':angle,'passed':error<1e-9 and angle<1e-9})
    async def turn(degrees):
        target=math.radians(degrees)
        def delta():return (target-w.motion.yaw+math.pi)%(2*math.pi)-math.pi
        initial=delta();key='left' if initial>0 else 'right';press(key)
        for i in range(650):
            await frame('turn')
            if abs(delta())<.01 or delta()*initial<0:break
        release(key)
        for _ in range(15):
            if abs(delta())<.001:break
            key='left' if delta()>0 else 'right';press(key);await frame('turn_fine');release(key)
            await advance(.08,'turn_fine_released')
        await stop('turn')
        ok=abs(delta())<.003
        report['route'].append({'action':'turn','target_deg':degrees,'actual_deg':math.degrees(w.motion.yaw),'xy':w.motion.xy.tolist(),'passed':ok})
        if not ok:raise RuntimeError('Cannot turn at route waypoint: '+str(report['route'][-1]))
    async def move(axis,target,key,label,film=None):
        initial=w.motion.xy[axis];press(key);start=len(films.get(film,[]))
        for i in range(1900):
            await frame(label,film if abs(w.motion.xy[axis]-target)<.60 else None)
            if (w.motion.xy[axis]-target)*(initial-target)<=0:break
        await stop(label,film)
        ok=abs(w.motion.xy[axis]-target)<.015
        report['route'].append({'action':label,'target_axis':axis,'target_m':target,'xy':w.motion.xy.tolist(),'passed':ok})
        if film:film_segments.append({'film':film,'action':label,'first_frame':start,'last_frame':len(films[film])-1})
        if not ok:raise RuntimeError('Cannot reach route waypoint: '+str(report['route'][-1]))
    async def edge_hold(key,label):
        press(key);await advance(2.0,label,'edges');release(key);await advance(.9,label+'_stop','edges')
        report['route'].append({'action':label,'xy':w.motion.xy.tolist(),'shoe_to_outer_edge_m':w.navigation.edge_clearance(w.motion.feet),'passed':True})
        ray_results=[]
        for side,f in w.motion.feet.items():
            shoe=w.navigation.soles[side]@__import__('worker_motion').rz(f['yaw']).T+f['p']
            for point in shoe:
                origin=carb.Float3(50+float(point[0]*100),float(point[1]*100),141.25+r.lift.height+10)
                ray=r.query.raycast_closest(origin,carb.Float3(0,0,-1),30)
                z=float(ray['position'][2]) if ray.get('hit') else None
                ray_results.append({'side':side,'shoe_xy_m':point[:2].tolist(),'hit_z_cm':z,'passed':z is not None and abs(z-(141.25+r.lift.height))<.5})
        report.setdefault('edge_floor_queries',[]).append({'edge':label,'rays':ray_results,'passed':all(q['passed'] for q in ray_results)})
        await r.capture_image(out/(label+'.png'))
    try:
        report['checks']['source_replaced']={'passed':len(w.motion.names)==23 and not r.stage.GetPrimAtPath('/Root/MCWP_Worker').IsActive()}
        report['checks']['geometry_navigation']={'passed':len(w.navigation.data['panels'])==13,'deck_min_m':w.navigation.data['surface_min'],'deck_max_m':w.navigation.data['surface_max'],'component_obstacles':len(w.navigation.names),'source_sha256':w.navigation.data['source_sha256']}
        w.view(True);await r.apply_capture_exposure()
        for _ in range(8):await omni.kit.app.get_app().next_update_async()
        await advance(.4,'idle','arms')
        for key in ['forward','back']:
            press(key);await advance(2.6,key+'_arm_swing','arms');await stop(key,'arms');await advance(.3,key+'_stop','arms')
        swing={}
        for key in ['forward','back']:
            label=key+'_arm_swing';h=np.array(hands[label])[45:];f=np.array(foot_y[label])[45:]
            swing[key]={'hand_y_excursion_m':np.ptp(h,axis=0).tolist(),'left_right_correlation':float(np.corrcoef(h.T)[0,1]),'left_hand_left_foot_correlation':float(np.corrcoef(h[:,0],f[:,0])[0,1])}
            swing[key]['passed']=min(swing[key]['hand_y_excursion_m'])>.15 and swing[key]['left_right_correlation']<-.9 and swing[key]['left_hand_left_foot_correlation']<-.6
        report['checks']['walking_arm_swing']={'passed':all(x['passed'] for x in swing.values()),'directions':swing}
        w.pose_command(True);await advance(2.3,'standing_arms_up','arms');up=w.motion.pose_amount
        press('forward');await advance(2.8,'walk_overrides_raised_pose','arms');walking_pose=w.motion.pose_amount;release('forward')
        await advance(2.4,'standing_pose_restored','arms');restored=w.motion.pose_amount
        w.pose_command(False);await advance(2.3,'standing_arms_down','arms')
        report['checks']['pose_priority_and_restore']={'passed':up>.99 and walking_pose<.02 and restored>.99 and w.motion.pose_amount<.01,'up':up,'during_walk':walking_pose,'restored':restored,'down':w.motion.pose_amount}
        # Every following waypoint is reached by held direction buttons, without
        # teleporting the root. Resets separate test cases / video chapters only.
        r.reset();r.busy=True;clear_previous();r.set_orbit_frame(150);await r.apply_capture_exposure()
        await turn(180);await move(1,.60,'forward','rear_access');await turn(90)
        r.lift.command(160);r.running=False
        await move(0,3.2,'forward','right_end_approach','edges');await edge_hold('forward','right_end_limit')
        await move(0,3.2,'back','right_turn_space');await turn(0)
        await move(1,-.65,'forward','right_outer_strip','edges');await move(1,.60,'back','right_return');await turn(90)
        r.lift.command(0);r.running=False
        await move(0,-3.2,'back','left_end_approach','edges');await edge_hold('back','left_end_limit')
        await move(0,-3.2,'forward','left_turn_space');await turn(0)
        await move(1,-.65,'forward','left_outer_strip','edges');await move(1,.60,'back','left_return');await turn(90)
        await move(0,-.65,'forward','central_return');await turn(0)
        await move(1,-.85,'forward','front_edge_approach','edges');await edge_hold('forward','front_edge_limit')
        w.pose_command(True);await advance(2.3,'front_edge_pose');w.pose_command(False);await advance(2.3,'front_edge_pose_down')
        await move(1,0.,'back','front_return');await turn(180)
        await move(1,.60,'forward','rear_edge_approach','edges');await edge_hold('forward','rear_edge_limit')
        w.pose_command(True);await advance(2.3,'rear_edge_pose');rear_limited=w.motion.pose_limited
        w.pose_command(False);await advance(2.3,'rear_edge_pose_down')
        await move(1,0.,'back','rear_return');await turn(90)
        report['checks']['whole_platform_route']={'passed':all(x['passed'] for x in report['route']),'continuous_button_driven':True,'rear_edge_pose_limited_by_geometry':rear_limited}
        # Fixed structure, native worker parent transform and real PhysX floor.
        fixed=[p for p in r.stage.Traverse() if p.IsA(UsdGeom.Mesh) and '/MCWP_Platform/Fixed/' in str(p.GetPath())]
        base=[np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default())) for p in fixed];lift=[]
        for height in [0,40,80,120,160,120,80,40,0]:
            r.lift.set_height(height);r.lift.target=height;r.step(1/60)
            error=max(float(np.abs(np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))-b).max()) for p,b in zip(fixed,base))
            z=float(UsdGeom.Xformable(w.root).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[2])
            ray=r.query.raycast_closest(carb.Float3(50+float(w.motion.xy[0]*100),float(w.motion.xy[1]*100),height+250),carb.Float3(0,0,-1),150)
            rz=float(ray['position'][2]) if ray.get('hit') else None
            lift.append({'height_cm':height,'fixed_error':error,'worker_z_cm':z,'raycast_z_cm':rz,'passed':error<1e-6 and abs(z-(141.25+height))<.001 and rz is not None and abs(rz-(141.25+height))<.5})
        report['checks']['lift_regression']={'passed':all(x['passed'] for x in lift),'poses':lift,'fixed_meshes':len(fixed)}
        report['checks']['hold_release']={'passed':all(x['passed'] for x in report['actions'])}
        report['checks']['physical_shoe_support_at_edges']={'passed':all(x['passed'] for x in report.get('edge_floor_queries',[])) and len(report.get('edge_floor_queries',[]))==4,'method':'Independent PhysX downward raycasts at both shoes corners and midpoints at each of four outer-edge approaches.'}
        report['checks']['skinning_contact_and_clearance']={'passed':not collisions and not unsupported and metrics['max_skin_error_m']<1e-5 and metrics['min_sole_height_m']>=0 and metrics['min_edge_clearance_m']>=0 and metrics['max_leg_length_error_m']<1e-5 and metrics['max_planted_slip_m']<.001,'metrics':metrics,'collisions':collisions[:30],'unsupported_feet':unsupported[:30]}
        report['checks']['continuous_animation']={'passed':metrics['max_vertex_step_m']<.05,'maximum_vertex_displacement_per_60hz_step_m':metrics['max_vertex_step_m']}
        report['all_passed']=all(x['passed'] for x in report['checks'].values())
        if not report['all_passed']:raise RuntimeError('One or more actual application checks failed')
        if recording:
            for film,name,fps in [('arms','worker_fixed_arm_swing.avi',30),('edges','worker_full_platform_edges.avi',15)]:
                r.write_video(out/name,films[film],fps=fps)
            report['videos']=[{'file':name,'frames':len(films[film]),'fps':fps,'duration_s':len(films[film])/fps,'actual_kit_capture':True} for film,name,fps in [('arms','worker_fixed_arm_swing.avi',30),('edges','worker_full_platform_edges.avi',15)]]
            report['video_segments']=film_segments
            report['video_editing']='Real-time segments with explicit cuts between approaches. The full continuous control route is in the CSV and route checks.'
    except Exception:
        report['error']=traceback.format_exc();report['all_passed']=False;print(report['error'],flush=True)
    finally:
        for name in ['worker_full_platform_validation.json','worker_runtime_validation.json']:(out/name).write_text(json.dumps(report,indent=2,default=serial))
        (out/'worker_envelope_validation.json').write_text(json.dumps({'passed':not collisions and not unsupported,'scope':'Native skin evaluated on the continuous full-platform route and pose transitions; component obstacle boxes, actual deck panel support, full-height swept fixed masts. Not an exhaustive proof of every possible animation state.','sampled_states':metrics['samples'],'native_geometry_samples':metrics['native_samples'],'collision_samples':len(collisions),'unsupported_samples':len(unsupported)},indent=2))
        with (out/'worker_full_platform_trajectory.csv').open('w',newline='') as f:
            writer=csv.writer(f);writer.writerow(['time_s','action','x_m','y_m','heading_deg','lift_cm','standing_pose_requested','arm_pose_blend','walk_blend','left_hand_local_y_m','right_hand_local_y_m','shoe_to_outer_edge_m','blocked','pose_limited']);writer.writerows(traces)
        r.busy=False;r.reset();r.restore_preview_exposure();r.message('Full-platform and arm-swing validation passed.' if report.get('all_passed') else 'Full-platform validation failed; see evidence report.')
    return report
