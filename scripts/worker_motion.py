"""Deterministic humanoid controls and planted-foot IK, in platform-local meters.

The supplied 23-joint skeleton/skin is retained. This is character animation,
not a dynamic balance or human biomechanics simulation.
"""
import math
import numpy as np

def rz(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1.]])

def unit(v):
    return v/max(float(np.linalg.norm(v)),1e-10)

def rotation_between(a,b):
    a,b=unit(a),unit(b);v=np.cross(a,b);c=float(np.dot(a,b))
    if c>0.9999999:return np.eye(3)
    if c<-.9999999:
        v=unit(np.cross(a,[1,0,0] if abs(a[0])<.8 else [0,1,0]))
        return 2*np.outer(v,v)-np.eye(3)
    k=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    return np.eye(3)+k+k@k/(1+c)

def smooth(t):
    t=max(0.,min(1.,t));return t*t*t*(10+t*(-15+6*t))

class WorkerMotion:
    SPEED=.30
    TURN=math.radians(40)
    CYCLE=1.5
    STANCE=.62
    FLOOR=.002

    def __init__(self,names,parents,rest):
        self.names=list(names);self.index={n:i for i,n in enumerate(names)}
        self.parents=list(parents);self.rest=np.asarray(rest,dtype=float)
        self.local=np.array([m if self.parents[i]<0 else np.linalg.inv(self.rest[self.parents[i]])@m for i,m in enumerate(self.rest)])
        self.children={n:self.index[ch] for n,ch in [('Thigh_L','Calf_L'),('Calf_L','Foot_L'),('Thigh_R','Calf_R'),('Calf_R','Foot_R'),('UpperArm_L','Forearm_L'),('Forearm_L','Hand_L'),('UpperArm_R','Forearm_R'),('Forearm_R','Hand_R')]}
        self.navigation=None
        self.reset()

    def reset(self):
        self.xy=np.array([-.65,0.]);self.yaw=math.pi/2
        self.held=set();self.speed=0.;self.turn=0.;self.phase=.15;self.weight=0.
        self.pose_target=0.;self.pose_amount=0.;self.pose_velocity=0.;self.blocked=False
        self.walk_blend=0.;self.arm_swing=0.;self.block_reason='';self.pose_limited=False
        self.feet={};self.total_time=0.;self.events=[]
        for side in ['L','R']:
            p=self.rest[self.index['Foot_'+side],:3,3].copy();p[2]+=self.FLOOR
            p=rz(self.yaw)@p;p[:2]+=self.xy
            self.feet[side]={'p':p,'yaw':self.yaw,'swing':False,'cycle':-1,'u':0.}
        self.world=self.solve_pose()

    def press(self,key):
        if key in ['forward','back','left','right']:self.held.add(key)

    def release(self,key=None):
        if key is None:self.held.clear()
        else:self.held.discard(key)
        # Root translation and yaw stop on release; a lifted foot finishes
        # its short landing, preserving continuous skeletal motion.
        if not ('forward' in self.held or 'back' in self.held):self.speed=0.
        if not ('left' in self.held or 'right' in self.held):self.turn=0.

    def set_pose(self,raised):
        self.pose_target=1. if raised else 0.

    def valid_root(self,xy,yaw=None):
        if self.navigation is None:return True
        return self.navigation.valid_root(xy,self.yaw if yaw is None else yaw)

    def step(self,dt):
        if dt<=0:return self.world
        forward=int('forward' in self.held)-int('back' in self.held)
        turn=int('left' in self.held)-int('right' in self.held)
        self.speed+=(forward*self.SPEED-self.speed)*(1-math.exp(-dt/0.12))
        self.turn+=(turn*self.TURN-self.turn)*(1-math.exp(-dt/0.12))
        oldxy=self.xy.copy();oldyaw=self.yaw
        newyaw=self.yaw+self.turn*dt
        velocity=rz(newyaw)@np.array([0.,-self.speed,0.])
        newxy=self.xy+velocity[:2]*dt
        self.blocked=not self.valid_root(newxy,newyaw)
        if not self.blocked:self.xy=newxy
        else:velocity[:]=0.;self.speed=0.
        if self.valid_root(self.xy,newyaw):self.yaw=newyaw
        else:self.turn=0.;self.blocked=True
        self.block_reason=self.navigation.last_reason if self.blocked and self.navigation else ''
        forced=set()
        for side in ['L','R']:
            ti,ci,fi=[self.index[n+'_'+side] for n in ['Thigh','Calf','Foot']]
            hip_local=self.rest[ti,:3,3].copy();hip_local[2]-=.041
            hip=rz(self.yaw)@hip_local;hip[:2]+=self.xy
            previous_hip=rz(oldyaw)@hip_local;previous_hip[:2]+=oldxy
            reach=np.linalg.norm(self.rest[ci,:3,3]-self.rest[ti,:3,3])+np.linalg.norm(self.rest[fi,:3,3]-self.rest[ci,:3,3])-.018
            distance=np.linalg.norm(self.feet[side]['p']-hip)
            previous_distance=np.linalg.norm(self.feet[side]['p']-previous_hip)
            # Moving back toward a planted foot improves reach. Blocking that
            # motion deadlocks reverse walking with both feet ahead of the hips.
            if distance>reach and distance>previous_distance+1e-8:forced.add(side)
        if forced:
            # A reversal may start with the wrong support foot. Plant/re-step
            # first rather than stretching a leg or snapping the character.
            self.xy=oldxy;self.yaw=oldyaw;velocity[:]=0.
        active=bool((forward and not self.blocked) or turn)
        self.weight+=(float(active)-self.weight)*(1-math.exp(-dt/.15))
        if active:self.phase+=dt/self.CYCLE
        for side,offset in [('L',0.),('R',.5)]:
            foot=self.feet[side];cycle=math.floor(self.phase+offset);u=(self.phase+offset)%1
            other=self.feet['R' if side=='L' else 'L']
            due=(u>=self.STANCE and foot['cycle']!=cycle) or side in forced
            if active and not foot['swing'] and not other['swing'] and due:
                duration=self.CYCLE*(1-self.STANCE)
                target_yaw=self.yaw+self.turn*(duration*.7)
                target=rz(target_yaw)@self.rest[self.index['Foot_'+side],:3,3]
                # Anticipate the next stance so the body passes over its support.
                planned_velocity=rz(target_yaw)@np.array([0.,-forward*self.SPEED,0.])
                if self.blocked:planned_velocity[:]=0.
                target[:2]+=self.xy+planned_velocity[:2]*(duration*.65+self.CYCLE*self.STANCE*.38)
                target[2]=self.rest[self.index['Foot_'+side],2,3]+self.FLOOR
                if self.navigation is not None:
                    target,target_yaw=self.navigation.foot_target(side,foot['p'],foot['yaw'],target,target_yaw)
                foot.update(swing=True,cycle=cycle,u=0.,start=foot['p'].copy(),target=target,start_yaw=foot['yaw'],target_yaw=target_yaw)
            if foot['swing']:
                foot['u']=min(1.,foot['u']+dt/(self.CYCLE*(1-self.STANCE)))
                t=foot['u'];a=smooth(t)
                foot['p']=foot['start']*(1-a)+foot['target']*a
                foot['p'][2]+=.065*math.sin(math.pi*t)**2
                foot['yaw']=foot['start_yaw']*(1-a)+foot['target_yaw']*a
                if t>=1:foot['swing']=False
        # Walking owns arm motion. The selected standing pose is remembered
        # and smoothly returns after release, rather than suppressing gait.
        walking=float(bool(forward))
        self.walk_blend+=(walking-self.walk_blend)*(1-math.exp(-dt/.18))
        feet_local={s:rz(-self.yaw)@(f['p']-np.r_[self.xy,0.]) for s,f in self.feet.items()}
        opposition=float(np.clip((feet_local['R'][1]-feet_local['L'][1])/.36,-1.,1.))
        self.arm_swing+=(opposition-self.arm_swing)*(1-math.exp(-dt/.09))
        # Critically damped standing-pose blend, with zero velocity at rest.
        omega=5.
        target_pose=self.pose_target*(1-self.walk_blend)
        self.pose_limited=False
        if self.navigation is not None and target_pose>.01:
            limit=self.navigation.arm_limit()
            self.pose_limited=target_pose>limit+.01
            target_pose=min(target_pose,limit)
        self.pose_velocity+=(omega*omega*(target_pose-self.pose_amount)-2*omega*self.pose_velocity)*dt
        self.pose_amount+=self.pose_velocity*dt
        self.pose_amount=max(0.,min(1.,self.pose_amount))
        self.total_time+=dt;self.world=self.solve_pose()
        return self.world

    def solve_pose(self):
        world=self.rest.copy()
        def rebuild(start):
            for i in range(start,len(world)):
                pi=self.parents[i]
                world[i]=self.local[i] if pi<0 else world[pi]@self.local[i]
        dz=-.012-.033*self.weight+.004*self.weight*math.cos(self.phase*4*math.pi)
        for side in ['L','R']:
            ti,ci,fi=[self.index[n+'_'+side] for n in ['Thigh','Calf','Foot']]
            p=self.feet[side]['p'].copy();p[:2]-=self.xy;p=rz(-self.yaw)@p
            horizontal=np.linalg.norm(p[:2]-self.rest[ti,:2,3])
            reach=np.linalg.norm(self.rest[ci,:3,3]-self.rest[ti,:3,3])+np.linalg.norm(self.rest[fi,:3,3]-self.rest[ci,:3,3])-.008
            dz=min(dz,p[2]+math.sqrt(max(.01,reach*reach-horizontal*horizontal))-self.rest[ti,2,3])
        hip=self.index['Hips'];world[hip,:3,3]+=np.array([.003*self.weight*math.sin(self.phase*2*math.pi),0,dz])
        for i in range(hip+1,len(world)):world[i]=world[self.parents[i]]@self.local[i]
        def aim(name,head,direction):
            i=self.index[name];ch=self.children[name]
            olddir=self.rest[ch,:3,3]-self.rest[i,:3,3]
            world[i,:3,:3]=rotation_between(olddir,direction)@self.rest[i,:3,:3]
            world[i,:3,3]=head
        for side,sgn in [('L',1),('R',-1)]:
            ti,ci,fi,toi=[self.index[n+'_'+side] for n in ['Thigh','Calf','Foot','Toe']]
            h=world[ti,:3,3].copy();foot=self.feet[side]
            target=foot['p'].copy();target[:2]-=self.xy;target=rz(-self.yaw)@target
            l1=np.linalg.norm(self.rest[ci,:3,3]-self.rest[ti,:3,3]);l2=np.linalg.norm(self.rest[fi,:3,3]-self.rest[ci,:3,3])
            delta=target-h;distance=float(np.linalg.norm(delta));d=min(distance,l1+l2-1e-6)
            axis=unit(delta);along=(l1*l1-l2*l2+d*d)/(2*d)
            front=np.array([sgn*.08,-1,0]);bend=unit(front-axis*np.dot(front,axis))
            knee=h+axis*along+bend*math.sqrt(max(0.,l1*l1-along*along))
            aim('Thigh_'+side,h,knee-h);aim('Calf_'+side,knee,target-knee)
            world[fi,:3,:3]=rz(foot['yaw']-self.yaw)@self.rest[fi,:3,:3];world[fi,:3,3]=target
            world[toi]=world[fi]@self.local[toi]
            # Upper-body action: forward shoulder-height arm raise by default.
            ui,ei,hi=[self.index[n+'_'+side] for n in ['UpperArm','Forearm','Hand']]
            shoulder=world[ui,:3,3].copy()
            swing=sgn*.42*self.walk_blend*self.weight*self.arm_swing
            amount=self.pose_amount
            down=unit(np.array([sgn*.16,math.sin(swing),-math.cos(swing)]));raised=unit(np.array([sgn*.12,-1.,.02]))
            direction=unit(down*(1-amount)+raised*amount)
            al=np.linalg.norm(self.rest[ei,:3,3]-self.rest[ui,:3,3]);elbow=shoulder+direction*al
            aim('UpperArm_'+side,shoulder,direction)
            elbow_bend=.16+.10*self.walk_blend
            fd=unit(np.array([sgn*.055,math.sin(swing-elbow_bend),-math.cos(swing-elbow_bend)]))*(1-amount)+unit(np.array([sgn*.03,-1.,.015]))*amount
            aim('Forearm_'+side,elbow,unit(fd));world[hi]=world[ei]@self.local[hi]
        return world

    def local_pose(self):
        return np.array([m if self.parents[i]<0 else np.linalg.inv(self.world[self.parents[i]])@m for i,m in enumerate(self.world)])

    def skin(self,points,indices,weights):
        mats=self.world@np.linalg.inv(self.rest)
        hom=np.column_stack([points,np.ones(len(points))])
        return np.einsum('vijk,vk,vi->vj',mats[indices],hom,weights)[:,:3]

    def platform_points(self,points):
        out=np.asarray(points)@rz(self.yaw).T;out[:,:2]+=self.xy
        return out
