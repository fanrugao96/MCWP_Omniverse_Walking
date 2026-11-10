"""Geometry-derived deck support and articulated-height obstacle envelopes.

No fall model: the character remains on supported deck. Closed guardrails,
mast openings and equipment retain their physical geometry.
"""
import json,math
import numpy as np
from worker_motion import WorkerMotion,rz

class WorkerNavigation:
    MARGIN=.008
    def __init__(self,path,motion,points,indices,weights,faces):
        self.data=json.loads(path.read_text());self.motion=motion
        self.panel_lo=np.array([p['min'] for p in self.data['panels']])-.0011
        self.panel_hi=np.array([p['max'] for p in self.data['panels']])+.0011
        self.lo=np.array([b['min'] for b in self.data['obstacles']]);self.hi=np.array([b['max'] for b in self.data['obstacles']])
        self.names=[b['path'] for b in self.data['obstacles']]
        self.surface_lo=np.array(self.data['surface_min']);self.surface_hi=np.array(self.data['surface_max'])
        self.last_reason='';self.probe=np.array([1.,0.,.1]);self.probe_radius=.10
        self.points=points;self.indices=indices;self.weights=weights;self.faces=faces
        self.arm_cache=None
        self.soles={}
        for side,sign in [('L',1),('R',-1)]:
            selected=points[(points[:,2]<.035)&(points[:,0]*sign>0)]
            ankle=motion.rest[motion.index['Foot_'+side],:3,3]
            lo,hi=selected.min(0)-ankle,selected.max(0)-ankle
            self.soles[side]=np.array([[x,y,z] for x in [lo[0],hi[0]] for y in [lo[1],(lo[1]+hi[1])/2,hi[1]] for z in [lo[2]]])
        # Separate height bands let legs pass the low drive enclosures while
        # keeping torso and arms clear of rails / controls. Use the actual skin.
        template=WorkerMotion(motion.names,motion.parents,motion.rest)
        samples=[];idle=[]
        for weight in [0.,1.]:
            for swing in [-1.,0.,1.]:
                template.weight=weight;template.walk_blend=weight;template.arm_swing=swing
                template.world=template.solve_pose();sample=template.skin(points,indices,weights);samples.append(sample)
                if weight==0:idle.append(sample)
        self.bands=[];self.idle_bands=[]
        levels=[0,.18,.36,.60,.90,1.12,1.35,1.55,2.1]
        for cloud,bands in [(np.concatenate(samples),self.bands),(np.concatenate(idle),self.idle_bands)]:
            for low,high in zip(levels[:-1],levels[1:]):
                subset=cloud[(cloud[:,2]>=low-.025)&(cloud[:,2]<=high+.025)]
                if len(subset):bands.append((low,high,subset[:,:2].min(0)-self.MARGIN,subset[:,:2].max(0)+self.MARGIN))

    def supported(self,points):
        p=np.asarray(points)[:,:2]
        return bool(np.all(np.any(np.all((p[:,None,:]>=self.panel_lo)&(p[:,None,:]<=self.panel_hi),axis=2),axis=1)))

    def foot_valid(self,side,p,yaw):
        sole=self.soles[side]@rz(yaw).T+p
        if not self.supported(sole):return False
        lo=sole.min(0)-[.006,.006,0];hi=sole.max(0)+[.006,.006,.14]
        mask=np.all((self.hi>lo)&(self.lo<hi),axis=1)
        if np.any(mask):return False
        if self.probe is not None:
            nearest=np.maximum(lo,np.minimum(hi,self.probe))
            if np.linalg.norm(nearest-self.probe)<self.probe_radius+self.MARGIN:return False
        return True

    def foot_target(self,side,start,start_yaw,target,target_yaw):
        # Bound the *whole shoe* and its swing route, not just the root point.
        # Shorten the step continuously before a rail, opening or obstacle.
        last=start.copy();last_yaw=start_yaw
        for amount in np.linspace(0,1,13)[1:]:
            p=start*(1-amount)+target*amount;yaw=start_yaw*(1-amount)+target_yaw*amount
            if not self.foot_valid(side,p,yaw):break
            last=p;last_yaw=yaw
        return last,last_yaw

    def valid_root(self,xy,yaw):
        self.last_reason='';R=rz(yaw)[:2,:2]
        for side in ['L','R']:
            ankle=rz(yaw)@self.motion.rest[self.motion.index['Foot_'+side],:3,3]+np.r_[xy,self.motion.FLOOR]
            if not self.foot_valid(side,ankle,yaw):self.last_reason='deck / foot clearance';return False
        # Oriented band-box against component AABBs, with separating axes in XY.
        walking=('forward' in self.motion.held)!=('back' in self.motion.held) or self.motion.walk_blend>.2
        for z0,z1,a,b in (self.bands if walking else self.idle_bands):
            center=xy+R@((a+b)/2);half=(b-a)/2
            world_half=np.abs(R)@half
            mask=(self.hi[:,2]>z0)&(self.lo[:,2]<z1)&np.all((self.hi[:,:2]>center-world_half)&(self.lo[:,:2]<center+world_half),axis=1)
            ids=np.where(mask)[0]
            if len(ids):
                oc=(self.lo[ids,:2]+self.hi[ids,:2])/2;oh=(self.hi[ids,:2]-self.lo[ids,:2])/2
                local_delta=(oc-center)@R
                overlap=np.all(np.abs(local_delta)<half+oh@np.abs(R),axis=1)
                if np.any(overlap):self.last_reason=self.names[ids[np.where(overlap)[0][0]]].split('/')[-1];return False
            if self.probe is not None and self.probe[2]+self.probe_radius>z0 and self.probe[2]-self.probe_radius<z1:
                pc=(self.probe[:2]-center)@R;near=np.clip(pc,-half,half)
                if np.linalg.norm(pc-near)<self.probe_radius+self.MARGIN:self.last_reason='diagnostic sphere';return False
        if self.motion.pose_amount>.02:
            posed=self.motion.skin(self.points,self.indices,self.weights)@rz(yaw).T+np.r_[xy,0.]
            if not self.clear_pose(posed):self.last_reason='raised-arm clearance';return False
        return True

    def clear_pose(self,points):
        # Exact triangle versus each conservative component box; unlike a
        # whole-character AABB this does not fill empty space between limbs.
        lo=points.min(0);hi=points.max(0)
        ids=np.where(np.all((self.hi>lo)&(self.lo<hi),axis=1))[0]
        if not len(ids):return True
        tri=points[self.faces];tlo=tri.min(1);thi=tri.max(1)
        for i in ids:
            a=self.lo[i]-self.MARGIN;b=self.hi[i]+self.MARGIN
            near=np.all((thi>a)&(tlo<b),axis=1)
            if not np.any(near):continue
            v=tri[near]-(a+b)/2;half=(b-a)/2
            alive=np.ones(len(v),dtype=bool)
            edges=[v[:,1]-v[:,0],v[:,2]-v[:,1],v[:,0]-v[:,2]]
            axes=[np.cross(edges[0],edges[1])]
            axes.extend(np.cross(edge,axis) for edge in edges for axis in np.eye(3))
            for axis in axes:
                proj=np.einsum('tvi,ti->tv',v,axis);radius=np.abs(axis)@half
                alive &= (proj.min(1)<=radius)&(proj.max(1)>=-radius)
                if not np.any(alive):break
            if np.any(alive):return False
        return True

    def arm_limit(self):
        m=self.motion;key=(*np.round(m.xy,3),round(m.yaw,3),round(m.weight,2))
        if self.arm_cache is not None and self.arm_cache[0]==key:return self.arm_cache[1]
        original=m.pose_amount;safe=0.
        try:
            for amount in np.linspace(0,1,21):
                m.pose_amount=float(amount);m.world=m.solve_pose()
                p=m.platform_points(m.skin(self.points,self.indices,self.weights))
                if not self.clear_pose(p):break
                safe=float(amount)
        finally:m.pose_amount=original;m.world=m.solve_pose()
        self.arm_cache=(key,safe)
        return safe

    def edge_clearance(self,feet):
        shoe=np.concatenate([self.soles[s]@rz(f['yaw']).T+f['p'] for s,f in feet.items()])
        return float(np.min(np.minimum(shoe[:,:2]-self.surface_lo,self.surface_hi-shoe[:,:2])))
